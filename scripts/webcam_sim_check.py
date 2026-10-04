#!/usr/bin/env python3
"""
웹캠 시뮬 보정 1단계: 실기 사진과 같은 자리에 RC카/dummy 를 놓고 /sim/webcam 에서 웹캠 YOLO 를 돌려 비교.
  (터미널 1) ./run_sim.sh x:=-3.0 y:=2.0        ← 로봇을 웹캠 시야 밖에 스폰
  (터미널 2) source env.sh; source ~/yolo_venv/bin/activate
             python3 scripts/webcam_sim_check.py --ros-args -p use_sim_time:=true [-p limit:=20]
입력: calib/webcam_real_ref.csv (실기 valid/test 라벨 박스로 역추정한 물체 위치·방향 + 실기 conf)
출력: calib/webcam_sim_vs_real.csv, 끝에 요약 (판정: |Δconf| ≤ 0.1, 박스 중심 오차 ≤ 20 px)
좌표: ref 의 cam_X(앞)/cam_Y(왼쪽) 는 웹캠 바닥 투영점 기준. 월드의 webcam_cam 은 (-0.5, 0) 에서 +x 를 봄.
yaw 는 0~180 도 (라벨 박스로는 앞뒤 구분 불가).
"""
import csv, math, os, subprocess, time
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAM_XY = (-0.5, 0.0)                        # worlds/*.sdf 의 webcam_cam 위치와 같아야 함
MODEL = {'car': 'rc_car', 'dummy': 'dummy_box'}
PARK = {'rc_car': (10.0, 10.0), 'dummy_box': (10.0, -10.0)}   # 쓰지 않는 물체는 펜스 밖으로


def set_pose(name, x, y, yaw):
    req = (f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: 0.0}}, '
           f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    r = subprocess.run(['gz', 'service', '-s', '/world/follow_arena/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', req],
                       capture_output=True, text=True)
    return 'true' in r.stdout


class Check(Node):
    def __init__(self):
        super().__init__('webcam_sim_check')
        self.declare_parameter('model', os.path.join(ROOT, 'weights', 'webcam_yolo26s_best.pt'))
        self.declare_parameter('limit', 0)          # 0 = 전부
        self.declare_parameter('settle_s', 0.6)     # 자리 옮긴 뒤 새 프레임까지 대기
        self.img = None; self.stamp = 0.0
        self.create_subscription(Image, '/sim/webcam', self.cb, 2)
        self.yolo = YOLO(self.get_parameter('model').value)

    def cb(self, m):
        self.img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3][:, :, ::-1].copy()
        self.stamp = time.time()

    def frame_after(self, t0, timeout=5.0):
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.img is not None and self.stamp > t0:
                return self.img
        return None


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1])); i = ix * iy
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def main():
    rclpy.init(); n = Check()
    rows = list(csv.DictReader(open(os.path.join(ROOT, 'calib', 'webcam_real_ref.csv'), encoding='utf-8')))
    lim = n.get_parameter('limit').value
    rows = rows[:lim] if lim else rows
    out = []
    for i, r in enumerate(rows):
        name = MODEL[r['cls']]
        for other, (px, py) in PARK.items():
            if other != name: set_pose(other, px, py, 0.0)
        ok = set_pose(name, CAM_XY[0] + float(r['cam_X']), CAM_XY[1] + float(r['cam_Y']), math.radians(float(r['yaw_deg'])))
        time.sleep(n.get_parameter('settle_s').value)
        img = n.frame_after(time.time()) if ok else None
        if img is None:
            n.get_logger().warn(f'{i}: set_pose={ok}, 프레임 없음 — 건너뜀'); continue
        res = n.yolo.predict(img, conf=0.05, verbose=False)[0]
        real = [float(r[k]) for k in ('u1', 'v1', 'u2', 'v2')]
        best, conf = None, 0.0
        for b in res.boxes:
            if n.yolo.names[int(b.cls)] == r['cls'] and float(b.conf) > conf:
                best, conf = b.xyxy[0].tolist(), float(b.conf)
        dc = (math.hypot((best[0] + best[2] - real[0] - real[2]) / 2, (best[1] + best[3] - real[1] - real[3]) / 2)
              if best else float('nan'))
        out.append(dict(image=r['image'], cls=r['cls'], real_conf=float(r['real_conf']), sim_conf=round(conf, 3),
                        d_conf=round(conf - float(r['real_conf']), 3), center_err_px=round(dc, 1),
                        iou=round(iou(best, real), 2) if best else 0.0))
        n.get_logger().info(f"{i + 1}/{len(rows)} {r['cls']} real {r['real_conf']} sim {conf:.2f} center {dc:.0f}px")
    path = os.path.join(ROOT, 'calib', 'webcam_sim_vs_real.csv')
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=out[0].keys()); w.writeheader(); w.writerows(out)
    for k in ('car', 'dummy'):
        s = [o for o in out if o['cls'] == k]
        if not s: continue
        det = [o for o in s if o['sim_conf'] >= 0.25]
        dcs = [abs(o['d_conf']) for o in s]; ces = [o['center_err_px'] for o in det]
        print(f"{k}: n={len(s)} 검출(≥0.25) {len(det)}/{len(s)}  |Δconf| 중앙값 {np.median(dcs):.3f}"
              f"  중심오차 중앙값 {np.median(ces) if ces else float('nan'):.1f}px"
              f"  → {'통과' if np.median(dcs) <= 0.1 and ces and np.median(ces) <= 20 else '조정 필요'}")
    print('저장:', path)
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
