#!/usr/bin/env python3
"""시뮬 위치 추정 정확도: AMCL 자세(/robot1/amcl_pose) 를 같은 시각(sim time)의 가즈보 정답과 비교.
SLAM 월드는 지도 좌표 그대로라 world = map. AMCL 은 이동·회전 문턱을 넘을 때만 갱신하므로
'지금' 정답과 비교하면 회전 중 오차가 부풀려진다 → 메시지 타임스탬프에 가장 가까운 정답(±0.1 s)과 비교.
  python3 loc_eval.py <out_dir> [duration_s]"""
import bisect, json, math, sys, time
from collections import deque
import numpy as np, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage

out = sys.argv[1]; dur = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
rclpy.init(); n = rclpy.create_node('loc_eval')
gt = deque(maxlen=4000); rows = []
def cb_gt(m):
    for t in m.transforms:
        if t.child_frame_id == 'robot1/turtlebot4':
            gt.append((stamp(t.header), t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)))
def cb_amcl(m):
    if not gt: return
    ts = stamp(m.header); keys = [g[0] for g in gt]; i = bisect.bisect_left(keys, ts)
    cand = [gt[j] for j in (i - 1, i) if 0 <= j < len(gt)]
    g = min(cand, key=lambda c: abs(c[0] - ts))
    if abs(g[0] - ts) > 0.1: return
    p = m.pose.pose
    e = math.hypot(p.position.x - g[1], p.position.y - g[2])
    ey = abs((yaw(p.orientation) - g[3] + math.pi) % (2 * math.pi) - math.pi)
    rows.append((ts, g[1], g[2], p.position.x, p.position.y, e, math.degrees(ey)))
n.create_subscription(TFMessage, '/robot1/_internal/sim_ground_truth_pose', cb_gt, 50)
n.create_subscription(PoseWithCovarianceStamped, '/robot1/amcl_pose', cb_amcl, 10)
t0 = time.time(); last_save = 0
def save():
    if not rows: return
    R = np.array(rows)
    np.savetxt(f'{out}/loc_eval.csv', R, delimiter=',', header='t,gt_x,gt_y,amcl_x,amcl_y,err_m,err_deg', comments='', fmt='%.4f')
    s = dict(n=len(R), err_m_median=round(float(np.median(R[:, 5])), 3), err_m_p95=round(float(np.percentile(R[:, 5], 95)), 3),
             err_m_max=round(float(R[:, 5].max()), 3), err_deg_median=round(float(np.median(R[:, 6])), 2), err_deg_p95=round(float(np.percentile(R[:, 6], 95)), 2))
    json.dump(s, open(f'{out}/loc_eval.json', 'w'), indent=1); return s
try:
    while rclpy.ok() and time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - last_save > 30: save(); last_save = time.time()
except KeyboardInterrupt:
    pass
print(save())
