#!/usr/bin/env python3
"""
시뮬 TurtleBot4 → 실기(robot1 스냅샷 2026-10-02) 인터페이스 어댑터.
follow_object_node_v3.py 를 수정 없이 실기와 같은 토픽으로 돌리기 위한 브리지.

  시뮬(gz)                                   실기와 동일한 토픽
  {ns}/oakd/rgb/preview/image_raw  rgb8   →  {ns}/oakd/rgb/image_raw      bgr8
  {ns}/oakd/rgb/preview/depth      32FC1  →  {ns}/oakd/stereo/image_raw   16UC1 [mm], 무효=0
  {ns}/oakd/rgb/preview/camera_info       →  {ns}/oakd/rgb/camera_info, {ns}/oakd/stereo/camera_info
  {ns}/cmd_vel_unstamped           Twist  →  {ns}/cmd_vel                 TwistStamped  (실기 create3_repub 역할)
"""
import math, time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

FRAME = 'oakd_rgb_camera_optical_frame'


class Adapter(Node):
    def __init__(self):
        super().__init__('tb4_real_iface_adapter')
        ns = self.declare_parameter('ns', '/robot1').value
        # 실험용: false 로 바꾸면 depth 를 전부 0(무효)으로 내보냄 — 검은 무광 차체에서 스테레오 depth 가
        # 안 나오는 실기 상황 재현.  ros2 param set /tb4_real_iface_adapter depth_valid false
        self.declare_parameter('depth_valid', True)
        # 실기 드라이버가 best-effort 로 낼 경우를 재현하기 위한 옵션 (기본 reliable)
        q = self.declare_parameter('pub_qos', 'reliable').value
        # 실기 지연 재현 (10/3 실측, docs/REAL_TRIAL_261003.md 5.3·5.5): 인지 지연 ~0.4 s, 명령→바퀴 0.2 s,
        # Create3 제자리 회전 최소 ~0.26 rad/s. 기본 0 = 기존 동작. 영상 header.stamp 는 촬영 시각 그대로 두고 늦게 발행.
        self.img_delay = self.declare_parameter('img_delay', 0.0).value
        self.cmd_delay = self.declare_parameter('cmd_delay', 0.0).value
        self.min_rot = self.declare_parameter('min_rot', 0.0).value
        self.q = deque()
        self.create_timer(0.005, self.flush)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT if q == 'best_effort' else ReliabilityPolicy.RELIABLE)
        self.pub_rgb = self.create_publisher(Image, f'{ns}/oakd/rgb/image_raw', qos)
        self.pub_dep = self.create_publisher(Image, f'{ns}/oakd/stereo/image_raw', qos)
        self.pub_ci_rgb = self.create_publisher(CameraInfo, f'{ns}/oakd/rgb/camera_info', 5)
        self.pub_ci_st = self.create_publisher(CameraInfo, f'{ns}/oakd/stereo/camera_info', 5)
        self.pub_cmd = self.create_publisher(TwistStamped, f'{ns}/cmd_vel', 5)
        self.create_subscription(Image, f'{ns}/oakd/rgb/preview/image_raw', self.cb_rgb, 5)
        self.create_subscription(Image, f'{ns}/oakd/rgb/preview/depth', self.cb_dep, 5)
        self.create_subscription(CameraInfo, f'{ns}/oakd/rgb/preview/camera_info', self.cb_ci, 5)
        self.create_subscription(Twist, f'{ns}/cmd_vel_unstamped', self.cb_cmd, 5)
        self.n_rgb = self.n_dep = 0
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'adapter up (ns={ns})')

    def cb_rgb(self, m):
        if m.encoding == 'rgb8':
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
            a = a.reshape(m.height, m.width, 3)[:, :, ::-1]
            m.data = np.ascontiguousarray(a).tobytes(); m.encoding = 'bgr8'; m.step = m.width * 3
        m.header.frame_id = FRAME
        self.later(self.get_parameter('img_delay').value, self.pub_rgb, m); self.n_rgb += 1

    def cb_dep(self, m):
        if m.encoding == '32FC1':
            a = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
            mm = np.where(np.isfinite(a) & (a > 0), a * 1000.0, 0.0)
            if not self.get_parameter('depth_valid').value: mm[:] = 0.0
            out = Image(header=m.header, height=m.height, width=m.width, encoding='16UC1',
                        is_bigendian=0, step=m.width * 2)
            out.data = np.clip(mm, 0, 65535).astype(np.uint16).tobytes()
            m = out
        m.header.frame_id = FRAME
        self.later(self.get_parameter('img_delay').value, self.pub_dep, m); self.n_dep += 1

    def cb_ci(self, m):
        m.header.frame_id = FRAME
        self.pub_ci_rgb.publish(m); self.pub_ci_st.publish(m)

    def cb_cmd(self, t):
        s = TwistStamped(); s.header.stamp = self.get_clock().now().to_msg(); s.header.frame_id = 'base_link'
        if 0.0 < abs(t.angular.z) < self.get_parameter('min_rot').value and abs(t.linear.x) < 0.02:   # 제자리 회전 최소 속도 (Create3)
            t.angular.z = math.copysign(self.get_parameter('min_rot').value, t.angular.z)
        s.twist = t; self.later(self.get_parameter('cmd_delay').value, self.pub_cmd, s)

    def later(self, delay, pub, msg):
        if delay <= 0: pub.publish(msg); return
        self.q.append((time.monotonic() + delay, pub, msg))

    def flush(self):
        now = time.monotonic()
        while self.q and self.q[0][0] <= now:
            _, pub, msg = self.q.popleft(); pub.publish(msg)

    def report(self):
        self.get_logger().info(f'rgb {self.n_rgb / 5:.1f} Hz, depth {self.n_dep / 5:.1f} Hz')
        self.n_rgb = self.n_dep = 0


def main():
    rclpy.init(); n = Adapter()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
