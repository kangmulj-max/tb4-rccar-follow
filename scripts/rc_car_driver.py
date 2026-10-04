#!/usr/bin/env python3
"""
가즈보 타깃(검은 RC카) 주행 패턴 생성기.  /rc_car/cmd_vel(Twist) → gz /model/rc_car/cmd_vel

  pattern: line | square | circle | figure8 | stopgo | zigzag | random | approach | dash
  예) python3 rc_car_driver.py --ros-args -p pattern:=square -p speed:=0.15

펜스(±4 x ±3 m) 0.7 m 이내로 다가가면 안쪽으로 회전해 빠져나옵니다 (/rc_car/pose 사용).
"""
import math, random, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage


class Driver(Node):
    def __init__(self):
        super().__init__('rc_car_driver')
        p = self.declare_parameter
        p('pattern', 'square'); p('speed', 0.15); p('turn_rate', 0.6); p('start_delay', 3.0)
        p('arena_x', 4.0); p('arena_y', 3.0); p('margin', 0.7)
        self.pub = self.create_publisher(Twist, '/rc_car/cmd_vel', 1)
        self.create_subscription(TFMessage, '/rc_car/pose', self.cb_pose, 5)
        self.pose = None; self.t0 = time.time(); self.rnd = (0.0, 0.0, 0.0)
        self.escape_until = 0.0; self.escape_w = 0.0
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"pattern={self.P('pattern')} speed={self.P('speed')}")

    def P(self, n): return self.get_parameter(n).value

    def cb_pose(self, m):
        for t in m.transforms:
            if t.child_frame_id.endswith('rc_car') or len(m.transforms) == 1:
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.pose = (t.transform.translation.x, t.transform.translation.y, yaw)

    def pattern(self, t):
        v, w, pat = self.P('speed'), self.P('turn_rate'), self.P('pattern')
        if pat == 'line':                     # 3 m 직진 → 2 s 정지 → 180° 회전
            per = 3.0 / v; T = per + 2.0 + math.pi / w; k = t % T
            return (v, 0.0) if k < per else (0.0, 0.0) if k < per + 2.0 else (0.0, w)
        if pat == 'square':                   # 1.5 m 변 정사각형 (좌회전)
            per = 1.5 / v; T = per + (math.pi / 2) / w
            return (v, 0.0) if t % T < per else (0.0, w)
        if pat == 'circle':                   # 반경 1 m
            return v, v / 1.0
        if pat == 'figure8':                  # 반경 0.8 m 원 두 개
            T = 2 * math.pi * 0.8 / v
            return v, (v / 0.8) * (1 if (t // T) % 2 == 0 else -1)
        if pat == 'stopgo':                   # 3 s 주행 / 2 s 정지, 정지 후 방향 약간 변경
            k = t % 5.0
            return (v, 0.0) if k < 3.0 else (0.0, 0.35 * math.sin(t))
        if pat == 'zigzag':                   # 사인 조향
            return v, 0.8 * math.sin(2 * math.pi * t / 6.0)
        if pat == 'approach':                 # 로봇 쪽으로 후진 접근 4 s → 정지 4 s 반복 (로봇 후진 유발)
            k = t % 8.0
            return (-v, 0.0) if k < 4.0 else (0.0, 0.0)
        if pat == 'dash':                     # 2 s 직진 후 고속 급선회로 시야 이탈 → 6 s 정지 반복
            k = t % 10.0
            return (v, 0.0) if k < 2.0 else (2.5 * v, 2.5) if k < 4.0 else (0.0, 0.0)
        if pat == 'random':                   # 2~4 s마다 무작위 속도·조향
            if t >= self.rnd[2]:
                self.rnd = (random.uniform(0.3, 1.0) * v, random.uniform(-w, w), t + random.uniform(2, 4))
            return self.rnd[0], self.rnd[1]
        return 0.0, 0.0

    def tick(self):
        t = time.time() - self.t0 - self.P('start_delay')
        lin, ang = (0.0, 0.0) if t < 0 else self.pattern(t)
        now = time.time()
        if self.pose and t >= 0:              # 펜스 회피: 경계 근처에서 바깥을 향하면 중심 쪽으로 회전
            x, y, yaw = self.pose
            ax, ay, m = self.P('arena_x'), self.P('arena_y'), self.P('margin')
            near = abs(x) > ax - m or abs(y) > ay - m
            to_c = math.atan2(-y, -x); err = math.atan2(math.sin(to_c - yaw), math.cos(to_c - yaw))
            if near and abs(err) > math.pi / 3 and now > self.escape_until:
                self.escape_until = now + 0.5; self.escape_w = math.copysign(self.P('turn_rate'), err)
            if now < self.escape_until or (near and abs(err) > math.pi / 3):
                lin, ang = 0.0, self.escape_w or math.copysign(self.P('turn_rate'), err)
        cmd = Twist(); cmd.linear.x = float(lin); cmd.angular.z = float(ang); self.pub.publish(cmd)

    def stop(self):
        for _ in range(3): self.pub.publish(Twist()); time.sleep(0.05)


def main():
    rclpy.init(); n = Driver()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.stop(); n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
