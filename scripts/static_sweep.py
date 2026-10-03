#!/usr/bin/env python3
"""
정지 거리 스윕 (로봇 구동 없음): RC카를 줄자로 잰 거리에 차례로 두고, 거리별 검출·depth 품질을 측정.
근거리 추종(target_dist 축소) 가능 여부와 가즈보 센서 모델(검출률·depth 잡음·유효율·최소거리) 데이터용.
  python3 static_sweep.py [--bumper 0.8,0.6,0.5,0.4,0.3,0.2,0.1] [--settle 6] [--capture 5] [--tag rear]
절차: 단계 시작 = 로봇 비프 1 회 → settle 초 안에 RC카를 '범퍼 앞 X m' 에 두고 손을 뺌 → capture 초 측정 → 비프 2 회.
거리 기준: 범퍼(로봇 앞 둥근 테두리) 앞면 → RC카 가장 가까운 면. 카메라 기준 거리 = 범퍼 거리 + 0.231 m
(카메라 x=−0.06, 범퍼 반경 0.171 — 노드의 target_dist·min_dist 는 카메라 기준).
결과: ~/tb4_real_logs/sweep_<시각>_<tag>/ frames.csv (프레임별), summary.csv (거리별), step_<X>.jpg
"""
import argparse, csv, math, os, time
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import CompressedImage, LaserScan, CameraInfo
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAM_TO_BUMPER = 0.231
ap = argparse.ArgumentParser()
ap.add_argument('--bumper', default='0.8,0.6,0.5,0.4,0.3,0.2,0.1')
ap.add_argument('--settle', type=float, default=6.0); ap.add_argument('--capture', type=float, default=5.0)
ap.add_argument('--tag', default='rear'); ap.add_argument('--ns', default='/robot1')
ap.add_argument('--model', default=f'{ROOT}/weights/yolo12n_amr_best.pt'); ap.add_argument('--no_beep', action='store_true')
a = ap.parse_args()
steps = [float(x) for x in a.bumper.split(',')]
out = os.path.expanduser(f'~/tb4_real_logs/sweep_{time.strftime("%y%m%d-%H%M")}_{a.tag}'); os.makedirs(out, exist_ok=True)


class Sweep(Node):
    def __init__(self):
        super().__init__('static_sweep')
        q = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.sync = ApproximateTimeSynchronizer(
            [Subscriber(self, CompressedImage, f'{a.ns}/oakd/rgb/image_raw/compressed', qos_profile=q),
             Subscriber(self, CompressedImage, f'{a.ns}/oakd/stereo/image_raw/compressedDepth', qos_profile=q)], 5, 0.1)
        self.sync.registerCallback(self.cb)
        self.create_subscription(CameraInfo, f'{a.ns}/oakd/rgb/camera_info', self.cb_info, q)
        self.create_subscription(LaserScan, f'{a.ns}/scan', lambda m: setattr(self, 'scan', m), q)
        self.audio = self.create_publisher(AudioNoteVector, f'{a.ns}/cmd_audio', 1)
        self.model = YOLO(a.model); self.K = None; self.scan = None; self.last = None

    def cb_info(self, m):
        self.K = self.K or (m.k[0], m.k[4], m.k[2], m.k[5])

    def cb(self, rm, dm):
        rgb = cv2.imdecode(np.frombuffer(rm.data, np.uint8), cv2.IMREAD_COLOR)
        depth = cv2.imdecode(np.frombuffer(bytes(dm.data[12:]), np.uint8), cv2.IMREAD_UNCHANGED)
        if rgb is not None and depth is not None:
            self.last = (time.time(), rgb, depth)

    def beep(self, n, hz=880):
        if a.no_beep:
            return
        v = AudioNoteVector(append=False)
        for _ in range(n):
            v.notes += [AudioNote(frequency=hz, max_runtime=Duration(seconds=0.15).to_msg()),
                        AudioNote(frequency=0, max_runtime=Duration(seconds=0.1).to_msg())]
        self.audio.publish(v)

    def measure(self, rgb, depth):
        fx, fy, cx, cy = self.K
        res = self.model.predict(rgb, conf=0.1, verbose=False)[0]
        cars = [b for b in res.boxes if self.model.names[int(b.cls[0])] == 'car' and float(b.xyxy[0][3]) > cy]   # 바닥 물체만
        r = dict(det=0)
        if self.scan is not None:   # 정면 ±10° 라이다 최소거리 (라이다 +90° 장착 → 로봇각 = 라이다각 + 90°)
            s = self.scan; rg = np.asarray(s.ranges, np.float32)
            ang = (s.angle_min + s.angle_increment * np.arange(len(rg)) + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
            sel = (np.abs(ang) < math.radians(10)) & np.isfinite(rg) & (rg > s.range_min)
            r['lidar_front'] = round(float(rg[sel].min()) - 0.04, 3) if sel.any() else ''   # base_link 원점 기준
        if not cars:
            return r, res
        b = max(cars, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        ux, vy = (x1 + x2) // 2, y1 + (y2 - y1) * 2 // 3
        rr = max(1, min(4, min(x2 - x1, y2 - y1) // 4))
        roi = depth[max(0, vy - rr):vy + rr + 1, max(0, ux - rr):ux + rr + 1]; v = roi[roi > 0]
        box = depth[y1:y2, x1:x2]; bv = box[box > 0]
        r.update(det=1, conf=round(float(b.conf[0]), 3), x1=x1, y1=y1, x2=x2, y2=y2,
                 d_node=round(float(np.median(v)) / 1000, 3) if v.size >= max(3, roi.size * 0.3) else '',
                 patch_valid=round(v.size / max(1, roi.size), 2),
                 d_box_p20=round(float(np.percentile(bv, 20)) / 1000, 3) if bv.size > 20 else '',
                 box_valid=round(bv.size / max(1, box.size), 2),
                 bearing_deg=round(math.degrees(-math.atan((ux - cx) / fx)), 1), cut_bottom=int(y2 >= rgb.shape[0] - 2))
        return r, res


rclpy.init(); n = Sweep()
t = time.time()
while (n.K is None or n.last is None) and time.time() - t < 15:
    rclpy.spin_once(n, timeout_sec=0.1)
if n.last is None:
    raise SystemExit('영상 없음: compressed 토픽/네트워크 확인')
cols = ['bumper_m', 'cam_m', 't', 'det', 'conf', 'x1', 'y1', 'x2', 'y2', 'd_node', 'patch_valid', 'd_box_p20', 'box_valid',
        'bearing_deg', 'cut_bottom', 'lidar_front']
fw = csv.DictWriter(open(f'{out}/frames.csv', 'w', newline=''), cols); fw.writeheader()
summ = []
print(f'저장 폴더: {out}\n단계: 범퍼 앞 {steps} m  (settle {a.settle}s, capture {a.capture}s)')
for bm in steps:
    cam = round(bm + CAM_TO_BUMPER, 3)
    print(f'\n▶ 범퍼 앞 {bm:.2f} m (카메라 {cam:.2f} m) 에 RC카를 두세요 — {a.settle:.0f}s', flush=True); n.beep(1)
    t = time.time()
    while time.time() - t < a.settle:
        rclpy.spin_once(n, timeout_sec=0.05)
    rows = []; seen = None; t = time.time(); shot = None
    while time.time() - t < a.capture:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.last and n.last[0] != seen:
            seen = n.last[0]; r, res = n.measure(n.last[1], n.last[2])
            r.update(bumper_m=bm, cam_m=cam, t=round(seen, 3)); rows.append(r); fw.writerow(r); shot = res
    n.beep(2, 1320)
    if shot is not None:
        cv2.imwrite(f'{out}/step_{bm:.2f}.jpg', shot.plot())
    dets = [r for r in rows if r['det']]; dn = np.array([r['d_node'] for r in dets if r['d_node'] != ''], float)
    s = dict(bumper_m=bm, cam_m=cam, frames=len(rows), det_rate=round(len(dets) / max(1, len(rows)), 2),
             conf_med=round(float(np.median([r['conf'] for r in dets])), 2) if dets else '',
             d_valid=round(len(dn) / max(1, len(dets)), 2) if dets else '',
             d_med=round(float(np.median(dn)), 3) if dn.size else '', d_std=round(float(np.std(dn)), 3) if dn.size else '',
             d_err=round(float(np.median(dn)) - cam, 3) if dn.size else '',
             cut_bottom=round(float(np.mean([r['cut_bottom'] for r in dets])), 2) if dets else '',
             lidar_front=rows[-1].get('lidar_front', '') if rows else '')
    summ.append(s); print('  ', s, flush=True)
with open(f'{out}/summary.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, list(summ[0].keys())); w.writeheader(); w.writerows(summ)
n.beep(3, 660); print(f'\n완료 → {out}/summary.csv'); n.destroy_node(); rclpy.shutdown()
