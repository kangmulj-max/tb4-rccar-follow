#!/usr/bin/env python3
"""
실기 추종 실험 녹화기 — 시뮬 녹화와 같은 형식으로 남기기 위한 도구 (ground truth 없음).
  python3 real_recorder.py --ros-args -p trial:=261003-1430 -p out:=~/tb4_real_logs/261003-1430 [-p ns:=/robot1]

남기는 것 (out 폴더)
  video.mp4      : [로봇 디버그 화면(/robot1/follow/image, 없으면 RGB) | 텔레메트리 패널(상태·명령·속도·통로 여유·라이다 탑뷰·이벤트)]
  telemetry.csv  : 10 Hz — t, state, cmd_lin, cmd_ang, odom_v, odom_w, front_clear, rear_clear, img_age, scan_age, bump, battery
  events.csv     : 상태 전환, 노드 경고(camera timeout 등), 범퍼, 카메라·스캔 끊김
  summary.json   : 시간, 상태별 시간, 놓침 횟수/시간, 범퍼, 후진, 기록지에 옮겨 적을 값
노드 상태는 /rosout 의 'A -> B' 로그로 읽음 (v3, v3_safe 공통).
"""
import csv, json, math, os, time
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image, CompressedImage, LaserScan, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
try:
    from irobot_create_msgs.msg import HazardDetectionVector
except ImportError:
    HazardDetectionVector = None

R_ROBOT, HALF = 0.171, 0.221               # 로봇 반경, 통로 반폭(반경 + 0.05)
LIDAR_X, LIDAR_YAW = -0.04, math.pi / 2    # 실기 tf_static (base_link → rplidar_link)
W, H, PW = 720, 720, 560                   # 디버그 영역, 패널 폭
STATE_COL = {'TRACKING': (60, 170, 60), 'REID': (0, 150, 230), 'SEARCH': (0, 110, 230), 'IDLE': (140, 140, 140)}


def img_bgr(m):
    if m.encoding in ('bgr8', 'rgb8'):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        return a[:, :, ::-1].copy() if m.encoding == 'rgb8' else a.copy()
    return None


class Recorder(Node):
    def __init__(self):
        super().__init__('real_recorder')
        p = self.declare_parameter
        self.ns = p('ns', '/robot1').value; self.trial = p('trial', time.strftime('%y%m%d-%H%M')).value
        self.out = os.path.expanduser(p('out', f'~/tb4_real_logs/{self.trial}').value); os.makedirs(self.out, exist_ok=True)
        self.vw = cv2.VideoWriter(f'{self.out}/video_raw.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 10, (W + PW, H))
        self.tf = open(f'{self.out}/telemetry.csv', 'w', newline=''); self.tw = csv.writer(self.tf)
        self.tw.writerow(['t', 'state', 'cmd_lin', 'cmd_ang', 'odom_v', 'odom_w', 'front_clear', 'rear_clear', 'img_age', 'scan_age', 'bump', 'battery'])
        self.ef = open(f'{self.out}/events.csv', 'w', newline=''); self.ew = csv.writer(self.ef); self.ew.writerow(['t', 'type', 'detail'])
        self.t0 = time.time(); self.dbg = self.dbg_t = None; self.rgb = None; self.scan = None; self.scan_t = 0.0
        self.cmd = (0.0, 0.0); self.cmd_t = 0.0; self.odom = (0.0, 0.0); self.batt = None; self.bump_t = -9.0
        self.state = 'IDLE'; self.state_t = {}; self.events = []; self.lost_n = 0; self.started = False
        self.rev = 0.0; self.bumps = 0; self.front = self.rear = float('inf'); self.min_front = float('inf'); self.n = 0
        self.cam_lost = self.scan_lost = False; self.trail = []
        ns = self.ns
        self.create_subscription(Image, f'{ns}/follow/image', self.cb_dbg, 2)
        self.create_subscription(CompressedImage, f'{ns}/oakd/rgb/image_raw/compressed', self.cb_rgb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, f'{ns}/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, f'{ns}/odom', self.cb_odom, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, f'{ns}/cmd_vel', lambda m: self.cb_cmd(m.twist), 10)
        self.create_subscription(Twist, f'{ns}/cmd_vel_unstamped', self.cb_cmd, 10)
        self.create_subscription(BatteryState, f'{ns}/battery_state', lambda m: setattr(self, 'batt', m.percentage), qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.cb_log, 50)
        if HazardDetectionVector:
            self.create_subscription(HazardDetectionVector, f'{ns}/hazard_detection', self.cb_haz, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'trial {self.trial} → {self.out}')

    def t(self): return time.time() - self.t0

    def event(self, kind, detail):
        e = (round(self.t(), 1), kind, detail); self.events.append(e); self.ew.writerow(e); self.ef.flush()

    # ---- callbacks
    def cb_dbg(self, m): self.dbg = m; self.dbg_t = time.time()
    def cb_rgb(self, m): self.rgb = m
    def cb_cmd(self, tw): self.cmd = (tw.linear.x, tw.angular.z); self.cmd_t = time.time()

    def cb_odom(self, m):
        now = time.time(); dt = min(0.2, now - getattr(self, 'odom_t', now)); self.odom_t = now
        self.odom = (m.twist.twist.linear.x, m.twist.twist.angular.z)
        if self.odom[0] < -0.02: self.rev += -self.odom[0] * dt

    def cb_scan(self, m):
        r = np.asarray(m.ranges, np.float32); a = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        lx, ly = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]); c, s = math.cos(LIDAR_YAW), math.sin(LIDAR_YAW)
        x, y = LIDAR_X + c * lx - s * ly, s * lx + c * ly
        self.scan = (x, y); self.scan_t = time.time()
        inside = np.abs(y) < HALF; f = x[inside & (x > 0)] - R_ROBOT; b = -x[inside & (x < 0)] - R_ROBOT
        self.front = float(f[f > -0.05].min()) if np.any(f > -0.05) else float('inf')
        self.rear = float(b[b > -0.05].min()) if np.any(b > -0.05) else float('inf')

    def cb_log(self, m):
        if not m.name.endswith('follow_object_node'): return
        if '->' in m.msg and m.msg.split('->')[0].strip() in ('IDLE', 'TRACKING', 'REID', 'SEARCH'):
            new = m.msg.split('->')[1].strip()
            if new == 'TRACKING': self.started = True
            if self.started and new in ('SEARCH', 'IDLE') and self.state not in ('SEARCH', 'IDLE'): self.lost_n += 1
            self.state = new; self.event('STATE', m.msg)
        elif m.level >= 30 or 'timeout' in m.msg or 'stop' in m.msg:
            self.event('NODE', m.msg[:120])

    def cb_haz(self, m):
        for h in m.detections:
            if h.type == 1 and time.time() - self.bump_t > 1.0:
                self.bump_t = time.time(); self.bumps += 1; self.event('BUMP', h.header.frame_id)

    # ---- drawing
    def panel(self, now, img_age, scan_age):
        P = np.full((H, PW, 3), 24, np.uint8); y = 34
        put = lambda txt, yy, sc=0.6, col=(235, 235, 235), th=1, x=16: cv2.putText(P, txt, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, sc, col, th, cv2.LINE_AA)
        put(f'trial {self.trial}', y, 0.75, th=2); put(time.strftime('%H:%M:%S'), y, 0.6, (170, 170, 170), x=PW - 110)
        y += 30; put(f't = {self.t():6.1f} s   battery {"-" if self.batt is None else f"{self.batt * 100:.0f}%"}', y, 0.55, (190, 190, 190))
        y += 36; col = STATE_COL.get(self.state, (140, 140, 140))
        cv2.rectangle(P, (16, y - 24), (200, y + 8), col, -1); put(self.state, y, 0.75, (255, 255, 255), 2, x=26)
        put(f'lost {self.lost_n}   bump {self.bumps}   rev {self.rev:.2f} m', y, 0.55, (200, 200, 200), x=214)
        cmd = self.cmd if now - self.cmd_t < 0.6 else (0.0, 0.0)
        for i, (lab, v, vmax, unit) in enumerate([('cmd lin', cmd[0], 0.3, 'm/s'), ('cmd ang', cmd[1], 1.0, 'rad/s'),
                                                  ('odom v', self.odom[0], 0.3, 'm/s'), ('odom w', self.odom[1], 1.0, 'rad/s')]):
            yy = y + 40 + i * 30; put(f'{lab}', yy, 0.5, (190, 190, 190)); cx = 260
            cv2.rectangle(P, (110, yy - 12), (410, yy + 2), (60, 60, 60), 1); cv2.line(P, (cx, yy - 14), (cx, yy + 4), (120, 120, 120), 1)
            w = int(np.clip(v / vmax, -1, 1) * 150); cv2.rectangle(P, (cx, yy - 10), (cx + w, yy), (0, 140, 255) if v < 0 else (90, 200, 90), -1)
            put(f'{v:+.2f} {unit}', yy, 0.5, x=420)
        y += 40 + 4 * 30 + 6
        fc = (60, 60, 230) if self.front < 0.3 else (0, 170, 255) if self.front < 0.5 else (90, 200, 90)
        put(f'front clear {self.front:5.2f} m   rear {self.rear:5.2f} m', y, 0.55, fc, 1)
        y += 26; warn = []
        if img_age > 0.5: warn.append(f'CAMERA {img_age:.1f}s')
        if scan_age > 1.0: warn.append(f'SCAN {scan_age:.1f}s')
        put('  '.join(warn) or 'sensors ok', y, 0.55, (60, 60, 230) if warn else (150, 150, 150))
        # 라이다 탑뷰 (로봇 기준 3 x 3 m, 위쪽 = 전방)
        top, S = y + 14, 260; cx0, cy0 = PW // 2, top + S // 2
        cv2.rectangle(P, (cx0 - S // 2, top), (cx0 + S // 2, top + S), (55, 55, 55), 1)
        k = S / 3.0
        tp = lambda xx, yy: (int(cx0 - yy * k), int(cy0 - xx * k))
        cv2.rectangle(P, tp(R_ROBOT + 0.6, HALF), tp(-R_ROBOT - 0.35, -HALF), (70, 70, 110), 1)
        cv2.circle(P, (cx0, cy0), int(R_ROBOT * k), (200, 200, 200), 1); cv2.line(P, (cx0, cy0), tp(0.25, 0), (200, 200, 200), 1)
        if self.scan is not None:
            for xx, yy in zip(*self.scan):
                if abs(xx) < 1.5 and abs(yy) < 1.5: cv2.circle(P, tp(xx, yy), 1, (0, 200, 255), -1)
        put('lidar (front = up, 3 m)', top + S + 18, 0.45, (150, 150, 150))
        yy = top + S + 44
        for e in self.events[-5:]:
            put(f'{e[0]:6.1f} {e[1]:5s} {e[2][:44]}', yy, 0.45, (60, 60, 230) if e[1] in ('BUMP', 'CAMLOST') else (200, 200, 200)); yy += 20
        return P

    def tick(self):
        now = time.time(); img_age = now - self.dbg_t if self.dbg_t else 99.0; scan_age = now - self.scan_t if self.scan_t else 99.0
        if img_age > 0.5 and not self.cam_lost and self.dbg_t: self.cam_lost = True; self.event('CAMLOST', f'debug image {img_age:.1f}s')
        if img_age < 0.5: self.cam_lost = False
        if scan_age > 1.0 and not self.scan_lost and self.scan_t: self.scan_lost = True; self.event('SCANLOST', f'{scan_age:.1f}s')
        if scan_age < 1.0: self.scan_lost = False
        self.state_t[self.state] = self.state_t.get(self.state, 0.0) + 0.1
        self.min_front = min(self.min_front, self.front)
        cmd = self.cmd if now - self.cmd_t < 0.6 else (0.0, 0.0)
        self.tw.writerow([f'{self.t():.2f}', self.state, f'{cmd[0]:.3f}', f'{cmd[1]:.3f}', f'{self.odom[0]:.3f}', f'{self.odom[1]:.3f}',
                          f'{self.front:.3f}', f'{self.rear:.3f}', f'{min(img_age, 99):.2f}', f'{min(scan_age, 99):.2f}',
                          int(now - self.bump_t < 0.2), '' if self.batt is None else f'{self.batt:.3f}'])
        left = None
        if self.dbg is not None: left = img_bgr(self.dbg)
        elif self.rgb is not None: left = cv2.imdecode(np.frombuffer(self.rgb.data, np.uint8), cv2.IMREAD_COLOR)
        frame = np.zeros((H, W + PW, 3), np.uint8)
        if left is not None: frame[:, :W] = cv2.resize(left, (W, H))
        else: cv2.putText(frame, 'no image', (W // 2 - 60, H // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2)
        frame[:, W:] = self.panel(now, img_age, scan_age); self.vw.write(frame); self.n += 1

    @staticmethod
    def warn_labels(e):
        t, kind, d = e
        if kind == 'CAMLOST': return ['camera drop']
        if kind == 'SCANLOST': return ['scan drop']
        if kind == 'BUMP': return ['bumper']
        if kind == 'NODE':
            for key, lab in (('camera timeout', 'camera timeout'), ('lidar TF', 'lidar TF fallback'), ('scan', 'scan warning')):
                if key in d: return [lab]
        return []

    def close(self):
        self.vw.release(); self.tf.close(); self.ef.close()
        T = self.t(); lost_s = sum(v for k, v in self.state_t.items() if k in ('SEARCH', 'IDLE')) if self.started else 0.0
        s = {'trial': self.trial, 'duration_s': round(T, 1), 'frames': self.n, 'state_time_s': {k: round(v, 1) for k, v in self.state_t.items()},
             'lost_events': self.lost_n, 'bumps': self.bumps, 'reverse_m': round(self.rev, 2),
             'min_front_clear_m': None if math.isinf(self.min_front) else round(self.min_front, 2),
             'camera_drops': sum(1 for e in self.events if e[1] == 'CAMLOST'), 'scan_drops': sum(1 for e in self.events if e[1] == 'SCANLOST'),
             'for_trial_log': {'lostN': self.lost_n, 'lostS_approx': round(lost_s, 1), 'reverse': 'yes' if self.rev > 0.05 else 'no',
                               'warnings': sorted({lab for e in self.events for lab in self.warn_labels(e)})}}
        json.dump(s, open(f'{self.out}/summary.json', 'w'), ensure_ascii=False, indent=1)
        print(json.dumps(s, ensure_ascii=False, indent=1))


def main():
    rclpy.init(); n = Recorder()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally: n.close(); n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
