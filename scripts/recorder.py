#!/usr/bin/env python3
"""
가즈보 실험 녹화: [천장 탑뷰 | 로봇 디버그 화면(/robot1/follow/image)] 을 나란히 붙여 mp4 저장.
  python3 recorder.py --ros-args -p tag:=square_y11 -p duration:=60.0
결과: ~/tb4_follow_sim/logs/videos/<tag>.mp4 (10 fps, 1680x720, H.264 변환은 experiment.sh 에서 ffmpeg)
"""
import math, os, time
import numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

HFOV, W, H, CAM_Z = 1.2, 960, 720, 7.0      # world 의 overhead_cam 과 동일


def to_bgr(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    return a[:, :, ::-1].copy() if m.encoding == 'rgb8' else a.copy()


def yaw_q(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self):
        super().__init__('recorder')
        self.tag = self.declare_parameter('tag', 'run').value
        self.duration = self.declare_parameter('duration', 0.0).value
        self.label = self.declare_parameter('label', '').value
        d = os.path.expanduser('~/tb4_follow_sim/logs/videos'); os.makedirs(d, exist_ok=True)
        self.path = f'{d}/{self.tag}_raw.mp4'
        self.vw = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'mp4v'), 10, (W + 720, H + 40))
        self.top = None; self.dbg = None; self.robot = None; self.car = None; self.trail = []; self.t0 = None; self.n = 0
        self.create_subscription(Image, '/sim/overhead', lambda m: setattr(self, 'top', m), 2)
        self.create_subscription(Image, '/robot1/follow/image', lambda m: setattr(self, 'dbg', m), 2)
        self.create_subscription(TFMessage, '/robot1/_internal/sim_ground_truth_pose', self.cb_robot, 5)
        self.create_subscription(TFMessage, '/rc_car/pose', self.cb_car, 5)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'recording → {self.path}')

    def cb_robot(self, m):
        for t in m.transforms:
            if t.child_frame_id == 'robot1/turtlebot4':
                self.robot = (t.transform.translation.x, t.transform.translation.y, yaw_q(t.transform.rotation))

    def cb_car(self, m):
        for t in m.transforms: self.car = (t.transform.translation.x, t.transform.translation.y)

    def px(self, x, y):
        """월드 (x,y) → 탑뷰 픽셀 (카메라 pitch +90°: 영상 위쪽 = 월드 +x? 실측 보정 없이 기하로 계산)"""
        f = (W / 2) / math.tan(HFOV / 2)
        return int(W / 2 - f * y / CAM_Z), int(H / 2 - f * x / CAM_Z)

    def tick(self):
        if self.top is None and self.dbg is None: return
        if self.t0 is None: self.t0 = time.time()
        t = time.time() - self.t0
        canvas = np.zeros((H + 40, W + 720, 3), np.uint8)
        if self.top is not None:
            top = cv2.resize(to_bgr(self.top), (W, H))
            if self.robot:
                self.trail.append(self.px(*self.robot[:2]))
                for a, b in zip(self.trail[-300:-1], self.trail[-299:]): cv2.line(top, a, b, (255, 160, 0), 2)
            canvas[40:, :W] = top
        if self.dbg is not None:
            canvas[40:, W:] = cv2.resize(to_bgr(self.dbg), (720, 720))
        txt = f'{self.label or self.tag}   t={t:5.1f}s'
        if self.robot and self.car:
            txt += f'   center dist={math.dist(self.robot[:2], self.car):.2f} m'
        cv2.putText(canvas, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(canvas, 'top view (gazebo)', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        self.vw.write(canvas); self.n += 1
        if self.duration and t > self.duration: raise SystemExit


def main():
    rclpy.init(); n = Rec()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit): pass
    finally:
        n.vw.release(); print(f'frames={n.n} → {n.path}'); n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
