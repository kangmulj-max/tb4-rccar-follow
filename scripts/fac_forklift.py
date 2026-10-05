#!/usr/bin/env python3
"""가상 지게차: spec.json 의 경로를 왕복하며 /forklift/pose (TFMessage) 발행 → sim_rplidar 가 라이다에 반영.
물리 충돌체는 월드 밖(70, 70)에 있으므로 로봇과 실제로 부딪히지는 않음 — 근접 거리는 궤적 로그로 평가.
  python3 fac_forklift.py <spec.json>"""
import json, math, sys, time
import rclpy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

spec = json.load(open(sys.argv[1], encoding='utf-8'))['forklift']
(x0, y0), (x1, y1) = spec['path']; v, pause = spec['speed'], spec['pause']
L = math.hypot(x1 - x0, y1 - y0); T = L / v; cycle = 2 * (T + pause)
rclpy.init(); n = rclpy.create_node('fac_forklift'); pub = n.create_publisher(TFMessage, '/forklift/pose', 10)
t0 = time.time()
try:
    while rclpy.ok():
        t = (time.time() - t0) % cycle
        if t < pause: f = 0.0
        elif t < pause + T: f = (t - pause) / T
        elif t < 2 * pause + T: f = 1.0
        else: f = 1.0 - (t - 2 * pause - T) / T
        m = TransformStamped(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = 'world'; m.child_frame_id = 'forklift'
        m.transform.translation.x = x0 + f * (x1 - x0); m.transform.translation.y = y0 + f * (y1 - y0); m.transform.rotation.w = 1.0
        pub.publish(TFMessage(transforms=[m])); rclpy.spin_once(n, timeout_sec=0.1)
except KeyboardInterrupt:
    pass
