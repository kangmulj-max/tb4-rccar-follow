#!/usr/bin/env python3
"""
특정 물체 따라가기 v3 — 검증된 연구 기반 재설계. 노트북 실행, 로봇 설치 없음.

근거
 [CH06] Chaumette & Hutchinson, "Visual Servo Control I/II", IEEE RAM 2006/2007
        → 2.5D 서보잉: 회전=IBVS(방위각 오차), 전진=PBVS(거리 오차), 제어법칙 v = -λ e
 [BT22] Zhang et al., "ByteTrack", ECCV 2022 → 이미지 평면 추적·칼만 예측은 트래커에 위임
 [YE23] Ye et al., Robot Person Following (ICRA 2023 계열) → 검출→추적→Re-ID 상태기계→제어기
 [AC20] Algabri & Choi, Sensors 2020 → 색 특징 재식별, 상실 시 마지막 관측 위치(LOP)로 복귀

실행: python3 follow_object_node_v3.py --ros-args -p model:=best.pt -p ns:=/robot1 -p target:=rc_car
      (tf 토픽이 네임스페이스 안이면 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static 추가)
"""
import math, time
from collections import deque
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Twist, PointStamped
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from ultralytics import YOLO

IDLE, TRACKING, REID, SEARCH = 'IDLE', 'TRACKING', 'REID', 'SEARCH'


class FollowObjectNode(Node):
    def __init__(self):
        super().__init__('follow_object_node')
        p = self.declare_parameter
        p('model', 'best.pt'); p('ns', '/robot1'); p('target', 'rc_car'); p('conf', 0.4)
        p('enable', False); p('debug_image', True)
        # [CH06] 제어
        p('target_dist', 1.0); p('lambda_ang', 1.2); p('lambda_lin', 0.6)
        p('bearing_dead_deg', 4.0); p('dist_dead', 0.1)
        p('max_lin', 0.3); p('max_ang', 1.0); p('acc_lin', 0.4); p('acc_ang', 1.5)
        # [YE23][AC20] 재식별
        p('reid_frames', 15)            # REID 상태 유지 프레임 수 (초과 시 SEARCH)
        p('reid_pos_w', 0.5); p('reid_app_w', 0.5); p('reid_accept', 0.6)
        p('reid_pos_radius', 0.3)       # 마지막 위치 반경(화면 비율) — 위치 점수 0이 되는 거리
        # [AC20] 상실 복구
        p('search_max_s', 6.0); p('search_ang', 0.5); p('odom_frame', 'odom')
        # 안전 (공학적 장치, 논문 외)
        p('min_dist', 0.45); p('lidar_stop', 0.35); p('lidar_offset_deg', 0.0); p('lidar_stale', 1.0)
        p('patch', 9)

        self.ns = self.get_parameter('ns').value
        self.model = YOLO(self.get_parameter('model').value)
        self.bridge = CvBridge()
        self.tf_buf = Buffer(); self.tf_lis = TransformListener(self.tf_buf, self)
        self.K = None                               # camera_info → fx, cx
        self.state = IDLE; self.state_since = time.time()
        self.lock_id = None; self.lock_center = None; self.lock_hist = None; self.reid_count = 0
        self.dist_hist = deque(maxlen=5)
        self.lop = None                             # [AC20] 마지막 관측 위치 (odom frame PointStamped)
        self.front_min = float('inf'); self.scan_time = 0.0
        self.cmd_prev = Twist(); self.t_prev = time.time(); self.enable_prev = False
        self.last_bearing = 0.0

        rgb = Subscriber(self, Image, f'{self.ns}/oakd/rgb/image_raw')
        dep = Subscriber(self, Image, f'{self.ns}/oakd/stereo/image_raw')
        self.sync = ApproximateTimeSynchronizer([rgb, dep], 5, 0.1); self.sync.registerCallback(self.cb)
        self.create_subscription(CameraInfo, f'{self.ns}/oakd/rgb/camera_info', self.cb_info, 1)
        self.create_subscription(LaserScan, f'{self.ns}/scan', self.cb_scan, 5)
        self.pub_cmd = self.create_publisher(Twist, f'{self.ns}/cmd_vel', 1)
        self.pub_img = self.create_publisher(Image, f'{self.ns}/follow/image', 1)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(f"v3 target='{self.get_parameter('target').value}' enable=False")

    def P(self, n): return self.get_parameter(n).value

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}'); self.state = s; self.state_since = time.time()

    # ---------- 센서 보조 ----------
    def cb_info(self, m):
        if self.K is None: self.K = (m.k[0], m.k[4], m.k[2], m.k[5]); self.get_logger().info('camera_info ok')

    def cb_scan(self, msg):
        n = len(msg.ranges); half = int(math.radians(20) / msg.angle_increment)
        i0 = int((math.radians(self.P('lidar_offset_deg')) - msg.angle_min) / msg.angle_increment) % n
        sel = [msg.ranges[(i0 + i) % n] for i in range(-half, half + 1)]
        sel = [r for r in sel if msg.range_min < r < msg.range_max]
        self.front_min = min(sel) if sel else float('inf'); self.scan_time = time.time()

    def lidar_blocks(self):
        return time.time() - self.scan_time > self.P('lidar_stale') or self.front_min < self.P('lidar_stop')

    def depth_at(self, depth, x1, y1, x2, y2):
        cx, cy = (x1 + x2) // 2, y1 + (y2 - y1) * 2 // 3
        r = max(1, min(self.P('patch') // 2, min(x2 - x1, y2 - y1) // 4))
        roi = depth[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1]
        v = roi[(roi > 0) & np.isfinite(roi)]
        if v.size < max(3, roi.size * 0.3): return None, (cx, cy)
        d = float(np.median(v)); return (d / 1000.0 if depth.dtype == np.uint16 else d), (cx, cy)

    # ---------- [AC20] 색 히스토그램 (HSV H·S, 상단 2/3 영역) ----------
    @staticmethod
    def appearance(rgb, x1, y1, x2, y2):
        crop = rgb[y1:y1 + max(1, (y2 - y1) * 2 // 3), x1:x2]
        if crop.size == 0: return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]); cv2.normalize(h, h); return h

    # ---------- [YE23] 후보 점수: 위치 근접 + 외형 유사 ----------
    def reid_score(self, cand, rgb):
        pos = 1.0 - min(1.0, math.dist(cand['center'], self.lock_center) / self.P('reid_pos_radius'))
        app = 0.0
        if self.lock_hist is not None:
            h = self.appearance(rgb, *cand['box'])
            if h is not None: app = max(0.0, cv2.compareHist(self.lock_hist, h, cv2.HISTCMP_CORREL))
        return self.P('reid_pos_w') * pos + self.P('reid_app_w') * app

    # ---------- [AC20] LOP 저장 / 방위 ----------
    def update_lop(self, u, v, d, header):
        if self.K is None or d is None: return
        fx, fy, cx, cy = self.K
        pt = PointStamped(); pt.header = header
        pt.point.x, pt.point.y, pt.point.z = (u - cx) * d / fx, (v - cy) * d / fy, d
        try:
            tf = self.tf_buf.lookup_transform(self.P('odom_frame'), header.frame_id, header.stamp, Duration(seconds=0.2))
            self.lop = do_transform_point(pt, tf)
        except Exception: pass

    def bearing_to_lop(self):
        """로봇 base_link 기준 LOP 방위각(rad). 못 구하면 None"""
        if self.lop is None: return None
        try:
            tf = self.tf_buf.lookup_transform('base_link', self.P('odom_frame'), rclpy.time.Time(), Duration(seconds=0.2))
            p = do_transform_point(self.lop, tf); return math.atan2(p.point.y, p.point.x)
        except Exception: return None

    # ---------- [CH06] 제어 + 가감속 제한 ----------
    def send(self, lin, ang):
        now = time.time(); dt = max(1e-3, now - self.t_prev); self.t_prev = now
        lin = float(np.clip(lin, self.cmd_prev.linear.x - self.P('acc_lin') * dt, self.cmd_prev.linear.x + self.P('acc_lin') * dt))
        ang = float(np.clip(ang, self.cmd_prev.angular.z - self.P('acc_ang') * dt, self.cmd_prev.angular.z + self.P('acc_ang') * dt))
        cmd = Twist(); cmd.linear.x = float(np.clip(lin, -self.P('max_lin'), self.P('max_lin')))
        cmd.angular.z = float(np.clip(ang, -self.P('max_ang'), self.P('max_ang'))); self.cmd_prev = cmd
        if self.P('enable'): self.pub_cmd.publish(cmd)
        return cmd

    def servo(self, bearing, d):
        """[CH06] 2.5D: ang = -λ_ang·e_bearing (IBVS, 특징=방위각), lin = λ_lin·e_dist (PBVS)"""
        dead = math.radians(self.P('bearing_dead_deg'))
        e_b = 0.0 if abs(bearing) < dead else bearing - math.copysign(dead, bearing)
        ang = -self.P('lambda_ang') * e_b
        lin = 0.0
        if d is not None:
            e_d = d - self.P('target_dist')
            if abs(e_d) > self.P('dist_dead'): lin = self.P('lambda_lin') * e_d
            if d < self.P('min_dist'): lin = min(lin, 0.0)
        if self.lidar_blocks(): lin = min(lin, 0.0)
        return self.send(lin, ang)

    # ---------- 메인 ----------
    def cb(self, rgb_msg, dep_msg):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')
        if depth.shape[:2] != rgb.shape[:2]:
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        H, W = rgb.shape[:2]
        res = self.model.track(rgb, conf=self.P('conf'), persist=True, verbose=False, tracker='bytetrack.yaml')[0]  # [BT22]

        cands = []
        if res.boxes is not None and res.boxes.id is not None:
            for b in res.boxes:
                if self.model.names[int(b.cls[0])] != self.P('target'): continue
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cands.append({'id': int(b.id[0]), 'box': (x1, y1, x2, y2), 'area': (x2 - x1) * (y2 - y1),
                              'center': ((x1 + x2) / 2 / W, (y1 + y2) / 2 / H)})

        chosen = None
        if self.state in (IDLE, SEARCH):                       # 새 대상: 가장 큰 박스
            if cands:
                chosen = max(cands, key=lambda c: c['area']); self.dist_hist.clear()
                self.lock_hist = self.appearance(rgb, *chosen['box']); self.set_state(TRACKING)
        elif self.state == TRACKING:
            chosen = next((c for c in cands if c['id'] == self.lock_id), None)
            if chosen is None: self.reid_count = 0; self.set_state(REID)
        if self.state == REID and chosen is None:               # [YE23] 점수 기반 복귀
            self.reid_count += 1
            if cands and self.lock_center is not None:
                best = max(cands, key=lambda c: self.reid_score(c, rgb)); s = self.reid_score(best, rgb)
                if s >= self.P('reid_accept'):
                    chosen = best; self.get_logger().info(f'reid ok id={best["id"]} score={s:.2f}'); self.set_state(TRACKING)
            if chosen is None and self.reid_count > self.P('reid_frames'): self.set_state(SEARCH)

        txt = f'[{self.state}] '
        if chosen is not None:
            x1, y1, x2, y2 = chosen['box']; self.lock_id = chosen['id']; self.lock_center = chosen['center']
            if self.state == TRACKING and self.lock_hist is not None:   # 외형 모델 완만 갱신
                h = self.appearance(rgb, x1, y1, x2, y2)
                if h is not None: self.lock_hist = 0.9 * self.lock_hist + 0.1 * h
            dist, (u, v) = self.depth_at(depth, x1, y1, x2, y2)
            if dist is not None: self.dist_hist.append(dist)
            d = float(np.median(self.dist_hist)) if self.dist_hist else None
            fx, cx = (self.K[0], self.K[2]) if self.K else (W / 2 / math.tan(math.radians(32)), W / 2)
            bearing = -math.atan((u - cx) / fx)                 # 왼쪽 +(ROS 각도 규약)
            self.last_bearing = bearing; self.update_lop(u, v, d, rgb_msg.header)
            cmd = self.servo(-bearing, d)                       # servo는 "오른쪽 +" 오차를 받음
            txt += f'id{self.lock_id} d={d:.2f}m b={math.degrees(bearing):+.1f}deg ' if d else f'id{self.lock_id} d=n/a '
            if self.P('debug_image'): cv2.circle(rgb, (u, v), 5, (0, 0, 255), -1)
        elif self.state == REID:
            cmd = self.send(self.cmd_prev.linear.x * 0.5, self.cmd_prev.angular.z)   # 선속도 감쇠, 회전 유지
            txt += f'reid {self.reid_count} '
        else:
            cmd = self.cmd_prev                                 # SEARCH/IDLE 명령은 watchdog이 담당

        if self.P('debug_image'):
            out = res.plot(img=rgb)
            lid = 'STALE' if time.time() - self.scan_time > self.P('lidar_stale') else f'{self.front_min:.2f}'
            cv2.putText(out, txt + f'lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f} lidar={lid} en={self.P("enable")}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            m = self.bridge.cv2_to_imgmsg(out, 'bgr8'); m.header = rgb_msg.header; self.pub_img.publish(m)

    # ---------- [AC20] SEARCH: LOP 방향으로 회전, 시간 초과 시 IDLE ----------
    def watchdog(self):
        en = self.P('enable')
        if self.enable_prev and not en: self.stop(); self.get_logger().info('enable=false -> stop')
        self.enable_prev = en
        if not en or self.state not in (SEARCH, IDLE): return
        if self.state == SEARCH:
            if time.time() - self.state_since > self.P('search_max_s'):
                self.lock_id = None; self.lock_hist = None; self.lop = None; self.set_state(IDLE); self.send(0.0, 0.0); return
            b = self.bearing_to_lop()
            if b is None: b = self.last_bearing                 # TF 실패 시 마지막 방위로
            if abs(b) < math.radians(8): self.send(0.0, 0.0)    # LOP를 바라보고 있으면 정지 대기
            else: self.send(0.0, math.copysign(self.P('search_ang'), b))
        else:
            self.send(0.0, 0.0)

    def stop(self):
        self.cmd_prev = Twist()
        for _ in range(3): self.pub_cmd.publish(Twist()); time.sleep(0.05)


def main():
    rclpy.init(); node = FollowObjectNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.stop(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
