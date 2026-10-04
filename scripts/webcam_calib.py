#!/usr/bin/env python3
"""마커 없는 웹캠 → SLAM 맵 보정. 로봇(AMCL 위치)을 움직이는 기준물로 쓴다.

  1) 수집 (ROS):  python3 webcam_calib.py collect --ros-args -p image_topic:=/sim/webcam
       - 시작 후 bg_s 초 동안 로봇·RC카가 화면 밖에 있어야 함 (배경 학습, Zivkovic 2004 MOG2)
       - 로봇을 화면 안 여러 곳(가장자리 포함)에 세우거나 천천히 몰면, 멈춰 있을 때마다
         (AMCL x, y, 로봇 전경 박스) 1 쌍 기록 → calib/webcam_robot_samples.csv
  2) 추정 (오프라인):  python3 webcam_calib.py solve [samples.csv]
       - 박스 재투영 오차 최소화로 카메라 8 파라미터(k1 포함) 추정, 5-fold 교차검증 → calib/webcam_calib.yaml
권장: 보정점 30 개 이상 (시뮬 비교: 10 개 중앙값 3.9 cm / 30 개 2.5 cm / 100 개 1.3 cm, AMCL σ 5 cm 가정).
"""
import csv, math, os, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import webcam_geom as G

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CSV = os.path.join(ROOT, 'calib', 'webcam_robot_samples.csv')
YAML = os.path.join(ROOT, 'calib', 'webcam_calib.yaml')
KEYS = ['x', 'y', 'z', 'yaw', 'pitch', 'roll', 'f', 'k1']


def collect():
    import cv2, rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry

    class Collect(Node):
        def __init__(self):
            super().__init__('webcam_calib_collect')
            P = self.declare_parameter
            P('image_topic', '/sim/webcam'); P('pose_topic', '/robot1/amcl_pose'); P('odom_topic', '/robot1/odom')
            P('bg_s', 5.0); P('still_s', 0.8); P('min_move', 0.15); P('min_area', 1500)
            g = lambda n: self.get_parameter(n).value
            self.bg = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=25, detectShadows=True)
            self.t0 = time.time(); self.pose = None; self.still_since = None; self.last_xy = None
            self.bg_s, self.still_s, self.min_move, self.min_area = g('bg_s'), g('still_s'), g('min_move'), g('min_area')
            os.makedirs(os.path.dirname(CSV), exist_ok=True)
            new = not os.path.exists(CSV)
            self.out = open(CSV, 'a', newline='', encoding='utf-8'); self.w = csv.writer(self.out)
            if new: self.w.writerow(['t', 'x', 'y', 'u1', 'v1', 'u2', 'v2']); self.out.flush()
            self.create_subscription(Image, g('image_topic'), self.cb_img, qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, g('pose_topic'), self.cb_pose, 10)
            self.create_subscription(Odometry, g('odom_topic'), self.cb_odom, 10)
            self.n = 0
            self.get_logger().info(f'배경 학습 {self.bg_s:.0f} s — 로봇·RC카를 화면 밖에 두세요')

        def cb_pose(self, m): self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

        def cb_odom(self, m):
            v = m.twist.twist
            still = abs(v.linear.x) < 0.01 and abs(v.angular.z) < 0.02
            self.still_since = (self.still_since or time.time()) if still else None

        def cb_img(self, m):
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
            learning = time.time() - self.t0 < self.bg_s
            fg = self.bg.apply(img, learningRate=-1 if learning else 0)    # 학습 후 배경 고정
            if learning: return
            fg = cv2.morphologyEx((fg > 200).astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            cnts, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)   # 그림자(127) 제외
            if not cnts or self.pose is None or self.still_since is None: return
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < self.min_area or time.time() - self.still_since < self.still_s: return
            u, v, w, h = cv2.boundingRect(c)
            if u < 3 or v < 3 or u + w > m.width - 3 or v + h > m.height - 3: return       # 잘린 박스 제외
            if self.last_xy and math.dist(self.last_xy, self.pose) < self.min_move: return
            self.last_xy = self.pose; self.n += 1
            self.w.writerow([f'{time.time():.2f}', *(f'{p:.4f}' for p in self.pose), u, v, u + w, v + h]); self.out.flush()
            self.get_logger().info(f'#{self.n} map ({self.pose[0]:.2f}, {self.pose[1]:.2f}) box {u},{v},{u + w},{v + h}')

    rclpy.init(); n = Collect()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.out.close()


def load(path):
    rows = list(csv.DictReader(open(path, encoding='utf-8')))
    return [(float(r['x']), float(r['y']), np.array([float(r[k]) for k in ('u1', 'v1', 'u2', 'v2')])) for r in rows]


def solve(path=CSV, k=5, seed=0):
    S = load(path)
    if len(S) < 8: sys.exit(f'보정점 {len(S)} 개 — 최소 8, 권장 30 개 이상')
    c, rms = G.calibrate_from_robot(S, k1_sigma=0.1)
    # 교차검증: 빼 둔 점의 로봇 위치를 박스로 추정해 AMCL 과 비교
    idx = np.random.default_rng(seed).permutation(len(S)); err = []
    for f in range(k):
        test = set(idx[f::k]); ck, _ = G.calibrate_from_robot([s for i, s in enumerate(S) if i not in test], k1_sigma=0.1)
        err += [math.dist(G.localize_robot(ck, S[i][2]), S[i][:2]) for i in test]
    err = np.array(err) * 100
    lines = ['# webcam_calib.py solve — 맵 좌표계 카메라 (주점=영상 중심, 방사 왜곡 k1)']
    lines += [f'{kk}: {v:.6f}' for kk, v in zip(KEYS, c)]
    lines += [f'image_size: [{G.IMG_W}, {G.IMG_H}]', f'samples: {len(S)}', f'reproj_rms_px: {rms:.2f}',
              f'cv_err_cm_median: {np.median(err):.1f}', f'cv_err_cm_p90: {np.percentile(err, 90):.1f}']
    open(YAML, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print('\n'.join(lines[1:]))
    print(f'저장: {YAML}')
    print('참고: cv 오차에는 AMCL 자체 오차가 섞여 있음 (정답이 아님). 중앙값이 크게 나오면 보정점을 늘리거나 가장자리 쪽을 보강.')


def load_calib(path=YAML):
    d = {}
    for line in open(path, encoding='utf-8'):
        if ':' in line and not line.startswith('#'):
            k, v = line.split(':', 1); d[k.strip()] = v.strip()
    return np.array([float(d[k]) for k in KEYS])


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else ''
    if cmd == 'collect': sys.argv.pop(1); collect()
    elif cmd == 'solve': solve(*(sys.argv[2:3] or [CSV]))
    else: print(__doc__)
