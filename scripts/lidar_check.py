#!/usr/bin/env python3
"""
라이다 정면 섹터 점검: v3 노드의 cb_scan 계산을 그대로 재현해 lidar_offset_deg 0 vs -90 비교 +
로봇 기준 방향별 최소거리 출력. (로봇은 정지 상태에서 실행)
  python3 lidar_check.py
"""
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def front_min(msg, offset_deg, half_deg=20):        # follow_object_node_v3.cb_scan 과 동일 로직
    n = len(msg.ranges); half = int(math.radians(half_deg) / msg.angle_increment)
    i0 = int((math.radians(offset_deg) - msg.angle_min) / msg.angle_increment) % n
    sel = [msg.ranges[(i0 + i) % n] for i in range(-half, half + 1)]
    sel = [r for r in sel if msg.range_min < r < msg.range_max]
    return min(sel) if sel else float('inf')


rclpy.init(); n = Node('lidar_check'); box = []
n.create_subscription(LaserScan, '/robot1/scan', lambda m: box.append(m), 5)
t = time.time()
while not box and time.time() - t < 10: rclpy.spin_once(n, timeout_sec=0.1)
m = box[-1]
print(f'scan: frame={m.header.frame_id} n={len(m.ranges)} angle_min={m.angle_min:.3f} inc={m.angle_increment:.4f}')
print(f'v3 front_min  offset   0deg (기본값) = {front_min(m, 0):.2f} m')
print(f'v3 front_min  offset -90deg (보정)   = {front_min(m, -90):.2f} m')
print('로봇 기준 방향별 최소거리 (라이다는 base 대비 +90° 회전 장착 → 로봇각 = 라이다각 + 90°):')
for name, robot_deg in [('정면', 0), ('좌측', 90), ('후방', 180), ('우측', -90)]:
    print(f'  {name:>3} ({robot_deg:+4d}°): {front_min(m, robot_deg - 90):.2f} m')
n.destroy_node(); rclpy.shutdown()
