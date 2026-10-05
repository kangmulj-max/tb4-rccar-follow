#!/usr/bin/env python3
"""실물 RPLIDAR 잡음: bag 에서 로봇이 멈춘 구간(odom 속도 ~0, 3 초 이상)의 /scan 을 모아, 빔마다 거리 표준편차.
  python3 lidar_noise_real.py <bag_dir> [out.json]"""
import json, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

bag = sys.argv[1]; out = sys.argv[2] if len(sys.argv) > 2 else None
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/robot1/scan', '/robot1/odom']))
odom, scans = [], []
while r.has_next():
    t, data, ts = r.read_next()
    if t.endswith('odom'):
        m = deserialize_message(data, Odometry); odom.append((ts * 1e-9, abs(m.twist.twist.linear.x), abs(m.twist.twist.angular.z)))
    else:
        m = deserialize_message(data, LaserScan); scans.append((ts * 1e-9, np.array(m.ranges, dtype=float), m.range_min, m.range_max))
od = np.array(odom)
def still(t):
    i = np.searchsorted(od[:, 0], t); w = od[max(0, i - 20): i + 20]
    return len(w) > 0 and w[:, 1].max() < 0.003 and w[:, 2].max() < 0.01
# 연속 정지 구간 묶기
segs, cur = [], []
for t, rr, rmin, rmax in scans:
    if still(t): cur.append(rr)
    else:
        if len(cur) >= 25: segs.append(np.array(cur))
        cur = []
if len(cur) >= 25: segs.append(np.array(cur))
print(f'스캔 {len(scans)} 개, 정지 구간 {len(segs)} 개 (프레임 수 {[len(s) for s in segs][:10]} ...)')
bins = [(0.2, 1.0), (1.0, 2.0), (2.0, 3.5), (3.5, 6.0)]
stats = {f'{a}-{b}m': [] for a, b in bins}; drop = []
for S in segs:
    valid = np.isfinite(S) & (S > 0.15)
    drop.append(1 - valid.mean())
    for j in range(S.shape[1]):
        v = S[valid[:, j], j]
        if len(v) < 0.8 * len(S): continue                      # 자주 끊기는 빔은 제외 (따로 집계)
        m = np.median(v); sd = np.std(v)
        for a, b in bins:
            if a <= m < b: stats[f'{a}-{b}m'].append(sd)
res = {k: dict(n=len(v), median_sd_mm=round(float(np.median(v)) * 1000, 1) if v else None, p90_sd_mm=round(float(np.percentile(v, 90)) * 1000, 1) if v else None) for k, v in stats.items()}
res['invalid_ratio'] = round(float(np.mean(drop)), 3) if drop else None
res['segments'] = len(segs); res['frames'] = int(sum(len(s) for s in segs))
print(json.dumps(res, ensure_ascii=False, indent=1))
if out: json.dump(res, open(out, 'w'), ensure_ascii=False, indent=1)
