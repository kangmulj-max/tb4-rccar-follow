#!/usr/bin/env python3
"""
실기 bag(mcap) → 프레임별 RC카 검출·거리 프로파일 CSV (가즈보 센서 모델·근거리 추종 연구용, 로봇 구동 없음).
  python3 bag_depth_profile.py ~/tb4_real_logs/<시도ID>/bag [--model weights/yolo12n_amr_best.pt] [--out depth_profile.csv]
열:
  t            bag 시작 기준 [s]
  conf, x1..y2 가장 큰 car 박스 (없으면 빈칸)
  box_h, box_w 박스 크기 [px]
  d_node       노드(depth_at)와 같은 방식: 가로 중앙·세로 2/3 지점 9×9 패치 중앙값 [m], 유효 30% 미만이면 빈칸
  patch_valid  그 패치의 유효 depth 비율
  d_box_p20    박스 안 유효 depth 의 20 백분위 [m] (가까운 표면 쪽 — 근거리 대안 추정)
  box_valid    박스 안 유효 depth 비율
  bearing_deg  박스 중심 방위 (왼쪽 +)
  lidar_at_b   같은 방위 ±3° 라이다 최소거리 [m] (base_link 기준, 라이다가 RC카를 보는지 확인용)
  odom_v       같은 시각 odom 선속도
"""
import argparse, bisect, csv, math, os
import numpy as np, cv2
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ultralytics import YOLO

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument('bag'); ap.add_argument('--model', default=f'{ROOT}/weights/yolo12n_amr_best.pt')
ap.add_argument('--out', default=None); ap.add_argument('--ns', default='/robot1'); ap.add_argument('--conf', type=float, default=0.25)
a = ap.parse_args()
out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.bag)), 'depth_profile.csv')
T = {k: f'{a.ns}/{v}' for k, v in dict(rgb='oakd/rgb/image_raw/compressed', dep='oakd/stereo/image_raw/compressedDepth',
                                        info='oakd/rgb/camera_info', scan='scan', odom='odom').items()}

r = SequentialReader(); r.open(StorageOptions(uri=a.bag, storage_id='mcap'), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = set(T.values()); data = {k: [] for k in T}; inv = {v: k for k, v in T.items()}
while r.has_next():
    topic, raw, _ = r.read_next()
    if topic not in want:
        continue
    k = inv[topic]
    if k == 'info' and data['info']:
        continue
    m = deserialize_message(raw, get_message(types[topic]))
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    data[k].append((st, m))
t0 = data['rgb'][0][0]
fx, fy, cx, cy = (lambda k: (k[0], k[4], k[2], k[5]))(data['info'][0][1].k)
dep_t = [s for s, _ in data['dep']]; scan_t = [s for s, _ in data['scan']]
odom_t = [s for s, _ in data['odom']]
print(f"frames rgb={len(data['rgb'])} depth={len(dep_t)} scan={len(scan_t)}  fx={fx:.1f}")


def near(ts, t):
    i = bisect.bisect_left(ts, t)
    c = [j for j in (i - 1, i) if 0 <= j < len(ts)]
    return min(c, key=lambda j: abs(ts[j] - t)) if c else None


model = YOLO(a.model)
rows = []
for n, (st, rm) in enumerate(data['rgb']):
    j = near(dep_t, st)
    if j is None or abs(dep_t[j] - st) > 0.06:
        continue
    rgb = cv2.imdecode(np.frombuffer(rm.data, np.uint8), cv2.IMREAD_COLOR)
    depth = cv2.imdecode(np.frombuffer(bytes(data['dep'][j][1].data[12:]), np.uint8), cv2.IMREAD_UNCHANGED)
    res = model.predict(rgb, conf=a.conf, verbose=False)[0]
    cars = [b for b in res.boxes if model.names[int(b.cls[0])] == 'car']
    row = dict(t=round(st - t0, 3))
    if cars:
        b = max(cars, key=lambda b: float((b.xyxy[0][2] - b.xyxy[0][0]) * (b.xyxy[0][3] - b.xyxy[0][1])))
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        ux, vy = (x1 + x2) // 2, y1 + (y2 - y1) * 2 // 3
        rr = max(1, min(4, min(x2 - x1, y2 - y1) // 4))
        roi = depth[max(0, vy - rr):vy + rr + 1, max(0, ux - rr):ux + rr + 1]
        v = roi[roi > 0]
        box = depth[y1:y2, x1:x2]; bv = box[box > 0]
        bearing = -math.atan((ux - cx) / fx)
        row.update(conf=round(float(b.conf[0]), 3), x1=x1, y1=y1, x2=x2, y2=y2, box_h=y2 - y1, box_w=x2 - x1,
                   d_node=round(float(np.median(v)) / 1000, 3) if v.size >= max(3, roi.size * 0.3) else '',
                   patch_valid=round(v.size / max(1, roi.size), 2),
                   d_box_p20=round(float(np.percentile(bv, 20)) / 1000, 3) if bv.size > 20 else '',
                   box_valid=round(bv.size / max(1, box.size), 2), bearing_deg=round(math.degrees(bearing), 1))
        k = near(scan_t, st)
        if k is not None and abs(scan_t[k] - st) < 0.2:
            sm = data['scan'][k][1]
            rg = np.asarray(sm.ranges, np.float32)
            ang = sm.angle_min + sm.angle_increment * np.arange(len(rg)) + math.pi / 2   # 라이다 +90° 장착 → 로봇각
            ang = (ang + math.pi) % (2 * math.pi) - math.pi
            sel = (np.abs(ang - bearing) < math.radians(3)) & np.isfinite(rg) & (rg > sm.range_min)
            if sel.any():
                row['lidar_at_b'] = round(float(rg[sel].min()), 3)
    k = near(odom_t, st)
    if k is not None:
        row['odom_v'] = round(data['odom'][k][1].twist.twist.linear.x, 3)
    rows.append(row)
    if n % 1000 == 0:
        print(f'  {n}/{len(data["rgb"])}')

cols = ['t', 'conf', 'x1', 'y1', 'x2', 'y2', 'box_h', 'box_w', 'd_node', 'patch_valid', 'd_box_p20', 'box_valid',
        'bearing_deg', 'lidar_at_b', 'odom_v']
with open(out, 'w', newline='') as f:
    w = csv.DictWriter(f, cols); w.writeheader(); [w.writerow(r) for r in rows]
print('저장:', out, len(rows), 'rows')
