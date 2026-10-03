#!/usr/bin/env python3
"""시뮬 인터페이스 점검: 실기 스냅샷과 같은 토픽/형식/주기로 나오는지 확인.  python3 check_iface.py"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from tf2_msgs.msg import TFMessage

CHECK = [('/robot1/oakd/rgb/image_raw', Image), ('/robot1/oakd/stereo/image_raw', Image),
         ('/robot1/oakd/rgb/camera_info', CameraInfo), ('/robot1/oakd/stereo/camera_info', CameraInfo),
         ('/robot1/scan', LaserScan), ('/rc_car/pose', TFMessage), ('/robot1/_internal/sim_ground_truth_pose', TFMessage)]


def main():
    rclpy.init(); n = Node('check_iface'); got = {t: [] for t, _ in CHECK}
    for t, ty in CHECK:
        n.create_subscription(ty, t, lambda m, t=t: got[t].append((time.time(), m)), 10)
    t0 = time.time()
    while time.time() - t0 < 8.0: rclpy.spin_once(n, timeout_sec=0.05)
    for t, _ in CHECK:
        L = got[t]
        if not L: print(f'[--] {t}: no data'); continue
        hz = (len(L) - 1) / (L[-1][0] - L[0][0]) if len(L) > 1 else 0; m = L[-1][1]
        extra = ''
        if isinstance(m, Image): extra = f'{m.width}x{m.height} {m.encoding} frame={m.header.frame_id}'
        if isinstance(m, CameraInfo): extra = f'fx={m.k[0]:.2f} cx={m.k[2]:.2f} cy={m.k[5]:.2f} {m.width}x{m.height} frame={m.header.frame_id}'
        if isinstance(m, LaserScan): extra = f'n={len(m.ranges)} amin={m.angle_min:.3f}'
        if isinstance(m, TFMessage): extra = 'children=' + ','.join(x.child_frame_id for x in m.transforms[:8])
        print(f'[ok] {t}: {hz:.1f} Hz {extra}')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
