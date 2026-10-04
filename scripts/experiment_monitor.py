#!/usr/bin/env python3
"""
주행실험 모니터 (가즈보 ground truth 기반) — 추종 품질 + 안전 이벤트.
  python3 experiment_monitor.py --ros-args -p tag:=y11_square -p duration:=60
결과: ~/tb4_follow_sim/logs/<tag>.csv (10 Hz 시계열), <tag>_summary.json, 콘솔 요약

측정 항목
 - gap_car     : 로봇 외곽(반경 0.171 m) ↔ RC카 박스 최근접 거리 (물리적 여유, ≤0.01 → 충돌)
 - cam_car     : 카메라 광학중심 ↔ RC카 최근접 거리 (노드 depth 거리와 같은 의미, target_dist 비교용)
 - clear_obs   : 로봇 외곽 ↔ 타깃 제외 모든 장애물(펜스·기둥·dummy) 최근접 거리
 - in_fov      : RC카가 카메라 HFOV 안에 있는가
안전 이벤트 (실기 사고로 이어질 수 있는 상황)
 - COLLISION_CAR / COLLISION_OBS : 여유 ≤ 0.01 m
 - BUMPER                        : create3 hazard_detection BUMP
 - FWD_NEAR_OBS                  : 정면 장애물 < 0.35 m 인데 전진 명령(lin>0)
 - REVERSE                       : 후진 명령 (TB4 후방 센서 없음) — 후진 거리 누적
 - MOVE_WITHOUT_CAMERA           : 카메라 프레임 0.5 s 이상 끊겼는데 로봇이 움직임
"""
import csv, json, math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_rplidar import load_world, yaw_q   # noqa: E402

STATES = ('IDLE', 'TRACKING', 'REID', 'SEARCH')
R_ROBOT = 0.170; CAM_FWD = -0.06; HFOV = 0.7403   # 10/4 실측 반경, 실측 fx 909 화각. 카메라 광학중심 base_link x=-0.06 (실기 tf_static, 줄자로 범퍼 뒤 0.231 m 확인)
try:
    from irobot_create_msgs.msg import HazardDetectionVector
except ImportError:
    HazardDetectionVector = None


def box_corners_dist(px, py, cx, cy, yaw, sx, sy):
    """점(px,py) ↔ 회전 박스 최근접 거리 (내부면 0)"""
    c, s = math.cos(-yaw), math.sin(-yaw); x, y = c * (px - cx) - s * (py - cy), s * (px - cx) + c * (py - cy)
    return math.hypot(max(abs(x) - sx / 2, 0.0), max(abs(y) - sy / 2, 0.0))


class Monitor(Node):
    def __init__(self):
        super().__init__('experiment_monitor')
        p = self.declare_parameter
        self.tag = p('tag', 'run').value; self.duration = p('duration', 0.0).value
        self.target = p('target_dist', 1.0).value
        self.models = load_world(self.declare_parameter('world', os.path.join(os.environ.get('TB4SIM', os.path.expanduser('~/tb4_follow_sim')), 'worlds', 'follow_arena.sdf')).value)   # experiment.sh 가 WORLD 로 넘김
        d = os.path.expanduser('~/tb4_follow_sim/logs'); os.makedirs(d, exist_ok=True)
        self.base = f'{d}/{self.tag}'
        self.f = open(self.base + '.csv', 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['t', 'wall', 'rx', 'ry', 'ryaw', 'carx', 'cary', 'gap_car', 'cam_car', 'clear_obs', 'bearing_deg',
                         'in_fov', 'cmd_lin', 'cmd_ang', 'gt_speed', 'img_age', 'state'])
        self.robot = None; self.prev_robot = None; self.cmd = (0.0, 0.0); self.cmd_t = 0.0
        self.img_t = None; self.follow_img_n = 0; self.state = 'IDLE'; self.state_time = {}
        self.events = []; self.last_ev = {}; self.rows = []; self.t0 = None; self.rev_dist = 0.0; self.bumps = 0
        self.create_subscription(TFMessage, '/robot1/_internal/sim_ground_truth_pose', self.cb_robot, 10)
        for n, m in self.models.items():
            if not m['static']:
                self.create_subscription(TFMessage, f'/{n}/pose', lambda msg, n=n: self.cb_model(n, msg), 10)
        self.create_subscription(TwistStamped, '/robot1/cmd_vel', self.cb_cmd, 10)
        self.create_subscription(Image, '/robot1/oakd/rgb/image_raw', self.cb_img, 2)
        self.create_subscription(Image, '/robot1/follow/image', self.cb_follow_img, 2)
        # /rosout 은 transient_local 로 퍼블리시됨 → 같은 내구성으로 구독해야 늦게 연결돼도 지난 상태 전환 로그를 받음
        rosout_qos = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Log, '/rosout', self.cb_log, rosout_qos)
        if HazardDetectionVector:
            self.create_subscription(HazardDetectionVector, '/robot1/hazard_detection', self.cb_haz, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'monitor → {self.base}.csv')

    # ---- callbacks
    def cb_model(self, n, msg):
        for t in msg.transforms:
            if t.child_frame_id in (n, ''):
                tr = t.transform.translation; self.models[n]['pose'] = (tr.x, tr.y, yaw_q(t.transform.rotation))

    def cb_robot(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'robot1/turtlebot4':
                tr = t.transform.translation; self.robot = (tr.x, tr.y, yaw_q(t.transform.rotation))

    def cb_cmd(self, m): self.cmd = (m.twist.linear.x, m.twist.angular.z); self.cmd_t = time.time()
    def cb_img(self, m): self.img_t = time.time()
    def cb_follow_img(self, m): self.follow_img_n += 1

    def cb_log(self, m):
        # 'A -> B' 중 A, B 가 모두 상태명일 때만 상태 전환 (v3_safe 의 'camera timeout -> stop' 같은 로그 제외)
        if m.name.endswith('follow_object_node') and '->' in m.msg:
            a, b = (x.strip() for x in m.msg.split('->', 1))
            if a in STATES and b in STATES:
                self.state = b; self.event('STATE', m.msg, cool=0)

    def cb_haz(self, m):
        for h in m.detections:
            if h.type == 1: self.bumps += 1; self.event('BUMPER', h.header.frame_id, cool=1.0)

    def event(self, kind, detail, cool=2.0):
        now = time.time()
        if now - self.last_ev.get(kind, -1e9) < cool: return
        self.last_ev[kind] = now; t = round(now - (self.t0 or now), 1)
        self.events.append({'t': t, 'type': kind, 'detail': detail})
        if kind != 'STATE': self.get_logger().warn(f'[{t:6.1f}s] {kind}: {detail}')

    # ---- geometry
    def nearest(self, px, py, names):
        best = (1e9, None)
        for n in names:
            m = self.models[n]; mx, my, myaw = m['pose']; cm, sm = math.cos(myaw), math.sin(myaw)
            for bx, by, byaw, sx, sy, z0, z1 in m['boxes']:
                d = box_corners_dist(px, py, mx + cm * bx - sm * by, my + sm * bx + cm * by, myaw + byaw, sx, sy)
                if d < best[0]: best = (d, n)
        return best

    def tick(self):
        if self.robot is None: return
        now = time.time()
        if self.t0 is None: self.t0 = now
        t = now - self.t0
        rx, ry, ryaw = self.robot
        speed = 0.0
        if self.prev_robot:
            px, py, _, pt = self.prev_robot
            vx, vy = (rx - px) / max(now - pt, 1e-3), (ry - py) / max(now - pt, 1e-3)
            speed = vx * math.cos(ryaw) + vy * math.sin(ryaw)          # 전진 +, 후진 -
            if speed < -0.02: self.rev_dist += -speed * (now - pt)
        self.prev_robot = (rx, ry, ryaw, now)
        cmd = self.cmd if now - self.cmd_t < 0.6 else (0.0, 0.0)

        d_car, _ = self.nearest(rx, ry, ['rc_car']); gap_car = d_car - R_ROBOT
        cx, cy = rx + CAM_FWD * math.cos(ryaw), ry + CAM_FWD * math.sin(ryaw)
        cam_car, _ = self.nearest(cx, cy, ['rc_car'])
        others = [n for n in self.models if n not in ('rc_car', 'floor')]
        d_obs, obs_name = self.nearest(rx, ry, others); clear = d_obs - R_ROBOT
        carx, cary, _ = self.models['rc_car']['pose']
        b = math.atan2(cary - ry, carx - rx) - ryaw; b = math.atan2(math.sin(b), math.cos(b))
        fov = int(abs(b) < HFOV / 2)
        # 정면(±30°) 장애물 여유 — 높이 무관(낮은 물체 포함), ground truth
        fwd_clear = 1e9
        for k in range(-30, 31, 5):
            a = ryaw + math.radians(k)
            for r in np.arange(R_ROBOT, 0.8, 0.02):
                qx, qy = rx + r * math.cos(a), ry + r * math.sin(a)
                if self.nearest(qx, qy, others)[0] < 0.005: fwd_clear = min(fwd_clear, r - R_ROBOT); break
        img_age = (now - self.img_t) if self.img_t else 99.0

        if gap_car <= 0.01: self.event('COLLISION_CAR', f'gap={gap_car:.3f}')
        if clear <= 0.01: self.event('COLLISION_OBS', f'{obs_name} clear={clear:.3f}')
        if fwd_clear < 0.35 and cmd[0] > 0.01: self.event('FWD_NEAR_OBS', f'front={fwd_clear:.2f} m lin={cmd[0]:.2f}')
        if cmd[0] < -0.01: self.event('REVERSE', f'lin={cmd[0]:.2f}', cool=3.0)
        if img_age > 0.5 and abs(speed) > 0.03: self.event('MOVE_WITHOUT_CAMERA', f'img_age={img_age:.1f}s v={speed:.2f}')
        self.state_time[self.state] = self.state_time.get(self.state, 0.0) + 0.1

        row = [t, now, rx, ry, ryaw, carx, cary, gap_car, cam_car, clear, math.degrees(b), fov, cmd[0], cmd[1], speed,
               min(img_age, 99.0), self.state]
        self.w.writerow([f'{v:.3f}' if isinstance(v, float) else v for v in row]); self.rows.append(row)
        if self.duration and t > self.duration: raise SystemExit

    def summary(self):
        self.f.close(); r = self.rows
        if not r: print('no data'); return
        T = r[-1][0]; cam = np.array([x[8] for x in r]); gap = np.array([x[7] for x in r])
        trk = np.array([x[16] == 'TRACKING' for x in r])
        ev = {}
        for e in self.events:
            if e['type'] != 'STATE': ev[e['type']] = ev.get(e['type'], 0) + 1
        s = {'tag': self.tag, 'duration_s': round(T, 1), 'samples': len(r),
             'cam_car_mean': round(float(cam.mean()), 3), 'cam_car_min': round(float(cam.min()), 3),
             'cam_car_max': round(float(cam.max()), 3),
             'abs_err_mean_tracking': round(float(np.abs(cam[trk] - self.target).mean()), 3) if trk.any() else None,
             'within_0.2m_pct_tracking': round(100 * float((np.abs(cam[trk] - self.target) < 0.2).mean()), 1) if trk.any() else None,
             'gap_car_min': round(float(gap.min()), 3),
             'clear_obs_min': round(min(x[9] for x in r), 3),
             'in_fov_pct': round(100 * sum(x[11] for x in r) / len(r), 1),
             'state_time_s': {k: round(v, 1) for k, v in self.state_time.items()},
             'state_transitions': sum(1 for e in self.events if e['type'] == 'STATE'),
             'reverse_distance_m': round(self.rev_dist, 3), 'bumper_hits': self.bumps,
             'max_cmd_lin': round(max(x[12] for x in r), 3), 'min_cmd_lin': round(min(x[12] for x in r), 3),
             'max_cmd_ang': round(max(abs(x[13]) for x in r), 3),
             'control_rate_hz': round(self.follow_img_n / max(T, 1e-3), 1),
             'events': ev, 'event_log': self.events}
        json.dump(s, open(self.base + '_summary.json', 'w'), ensure_ascii=False, indent=1)
        print(f'\n===== {self.tag}  ({T:.0f} s)')
        for k, v in s.items():
            if k != 'event_log': print(f'  {k}: {v}')


def main():
    rclpy.init(); n = Monitor()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit): pass
    finally: n.summary(); n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
