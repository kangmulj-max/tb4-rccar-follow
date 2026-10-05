#!/usr/bin/env python3
"""가즈보 정답 궤적 로그 (5 Hz CSV): 로봇 정답 자세, 지게차 위치, 바퀴 주행거리계(odom), AMCL 자세 — 주행·회피·미끄러짐 평가용.
  python3 fac_traj.py <out.csv>"""
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage

out = open(sys.argv[1], 'w'); out.write('t,x,y,yaw,fx,fy,ox,oy,ax,ay\n')
NAN2 = (float('nan'), float('nan'))
st = dict(r=None, f=NAN2, o=NAN2, a=NAN2)
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def cb_r(m):
    for t in m.transforms:
        if t.child_frame_id == 'robot1/turtlebot4': st['r'] = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
def cb_f(m):
    for t in m.transforms: st['f'] = (t.transform.translation.x, t.transform.translation.y)
rclpy.init(); n = rclpy.create_node('fac_traj')
n.create_subscription(TFMessage, '/robot1/_internal/sim_ground_truth_pose', cb_r, 20)
n.create_subscription(TFMessage, '/forklift/pose', cb_f, 10)
n.create_subscription(Odometry, '/robot1/odom', lambda m: st.__setitem__('o', (m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)
n.create_subscription(PoseWithCovarianceStamped, '/robot1/amcl_pose', lambda m: st.__setitem__('a', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
last = 0
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        if st['r'] and time.time() - last >= 0.2:
            last = time.time()
            out.write(('%.2f' + ',%.3f' * 9 + '\n') % ((last,) + st['r'] + st['f'] + st['o'] + st['a'])); out.flush()
except KeyboardInterrupt:
    pass
