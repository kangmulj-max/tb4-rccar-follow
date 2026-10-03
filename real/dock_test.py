#!/usr/bin/env python3
"""robot1 undock → (dock 보이는지 확인) → dock 반복 시험. 타임라인을 CSV 로 남김. 실패하면 중단 (로봇이 움직임 — 후방 0.5 m 이상 비우고 E-Stop 대기).
  python3 real/dock_test.py [반복수=3] [출력.csv=~/tb4_real_logs/dock_cycle_<시각>.csv]
  실기 261003: 3/3 성공 (undock 5.6 s: 0.13 m 후진 + 179° 회전 / dock 20.5 s: 178° 회전 + 0.12 m 전진).
  dock_visible 은 도크를 향해 적당히 떨어졌을 때만 True (도크 위·반대 방향이면 False) — 너무 가까이(라이다 정면 ~0.26 m) 두면 dock 이 ABORTED."""
import csv, math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import Odometry

NS = '/robot1'; CYCLES = int(sys.argv[1]) if len(sys.argv) > 1 else 3
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser(time.strftime('~/tb4_real_logs/dock_cycle_%y%m%d-%H%M.csv'))


class T(Node):
    def __init__(self):
        super().__init__('dock_cycle_test')
        self.ds = None; self.od = None; self.fb = ''
        self.create_subscription(DockStatus, f'{NS}/dock_status', lambda m: setattr(self, 'ds', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, f'{NS}/odom', lambda m: setattr(self, 'od', m), qos_profile_sensor_data)
        self.undock = ActionClient(self, Undock, f'{NS}/undock'); self.dock = ActionClient(self, Dock, f'{NS}/dock')
        self.w = csv.writer(open(OUT, 'w', newline='')); self.w.writerow(['t', 'phase', 'x', 'y', 'yaw_deg', 'v', 'w', 'is_docked', 'dock_visible', 'feedback'])
        self.t0 = time.time()

    def row(self, phase):
        o = self.od; d = self.ds
        if o is None or d is None:
            return
        p = o.pose.pose; q = p.orientation
        self.w.writerow([f'{time.time() - self.t0:.2f}', phase, f'{p.position.x:.3f}', f'{p.position.y:.3f}',
                         f'{math.degrees(2 * math.atan2(q.z, q.w)):.1f}', f'{o.twist.twist.linear.x:.3f}',
                         f'{o.twist.twist.angular.z:.3f}', int(d.is_docked), int(d.dock_visible), self.fb])

    def spin_for(self, s, phase):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(self, timeout_sec=0.05); self.row(phase)

    def run(self, client, goal, phase, timeout):
        if not client.wait_for_server(timeout_sec=10):
            return 'no-server', None
        t = time.time(); self.fb = ''
        fut = client.send_goal_async(goal, feedback_callback=lambda f: setattr(self, 'fb', str(f.feedback).replace(',', ';')))
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05); self.row(phase)
        gh = fut.result()
        if not gh.accepted:
            return 'rejected', time.time() - t
        res = gh.get_result_async()
        while not res.done() and time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.05); self.row(phase)
        if not res.done():
            gh.cancel_goal_async(); self.spin_for(1.0, phase + '-cancel')
            return 'timeout', time.time() - t
        r = res.result()
        return {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(r.status, str(r.status)) + f' is_docked={r.result.is_docked}', time.time() - t


rclpy.init(); n = T()
n.spin_for(2.0, 'start')
print(f'시작: is_docked={n.ds.is_docked} dock_visible={n.ds.dock_visible}', flush=True)
for c in range(1, CYCLES + 1):
    if n.ds.is_docked:
        r, dt = n.run(n.undock, Undock.Goal(), f'undock{c}', 60)
        p = n.od.pose.pose.position
        print(f'[{c}] undock: {r} ({dt:.1f}s) → odom x={p.x:.2f} y={p.y:.2f}', flush=True)
        if 'SUCCEEDED' not in r or n.ds.is_docked:
            print('undock 실패 → 중단'); break
    n.spin_for(3.0, f'look{c}')
    print(f'[{c}] 도크 보임: {n.ds.dock_visible}', flush=True)
    r, dt = n.run(n.dock, Dock.Goal(), f'dock{c}', 120)
    print(f'[{c}] dock: {r} ({dt:.1f}s)', flush=True)
    n.spin_for(2.0, f'after{c}')
    if not n.ds.is_docked:
        print('dock 실패 → 중단 (로봇은 도크 밖)'); break
print('기록:', OUT); n.destroy_node(); rclpy.shutdown()
