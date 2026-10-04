#!/usr/bin/env python3
# follow_object_node_v3_safe — 실행본 (자동 생성: scripts/make_lean.py, 원본 주석본: follow_object_node_v3_safe_annotated.py)
# 이 파일을 직접 고치지 말고 주석본을 고친 뒤 make_lean.py 로 다시 만들 것. 설명·근거는 주석본과 docs/ 참고.

import json
import math
import signal
import time
from collections import deque

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from ultralytics import YOLO

IDLE, TRACKING, REID, SEARCH = 'IDLE', 'TRACKING', 'REID', 'SEARCH'


def quat_mat(q):
    """「기초」 쿼터니언(x,y,z,w) → 3×3 회전행렬. TF 는 회전을 쿼터니언으로 주기 때문에 행렬로 바꿔 점에 곱한다."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FollowObjectNode(Node):
    """「기초」 Node 를 상속한 클래스 하나 = ROS 2 노드 하나. __init__ 에서 파라미터·구독·발행·타이머를 등록하고 …"""

    def __init__(self):
        super().__init__('follow_object_node')
        p = self.declare_parameter

        p('model', 'best.pt')
        p('ns', '/robot1')
        p('target', 'car')
        p('conf', 0.4)
        p('track_conf', 0.1)
        p('enable', False)
        p('debug_image', True)
        p('cmd_topic', '')

        p('target_dist', 1.0)
        p('lambda_ang', 1.2)
        p('lambda_lin', 0.6)
        p('bearing_dead_deg', 4.0)
        p('dist_dead', 0.1)
        p('max_lin', 0.3)
        p('max_ang', 1.0)
        p('acc_lin', 0.4)
        p('acc_ang', 1.5)

        p('reid_frames', 15)
        p('reid_pos_w', 0.5)
        p('reid_app_w', 0.5)
        p('reid_accept', 0.6)
        p('reid_pos_radius', 0.3)
        p('search_max_s', 6.0)
        p('search_ang', 0.5)
        p('odom_frame', 'odom')

        p('min_dist', 0.45)
        p('patch', 9)
        p('lidar_stale', 1.0)
        p('base_frame', 'base_link')
        p('robot_radius', 0.171)
        p('corridor_margin', 0.05)
        p('stop_base', 0.12)
        p('stop_gain', 0.6)
        p('rear_stop', 0.35)
        p('max_rev', 0.10)
        p('depth_miss_max', 3)
        p('depth_max_age', 0.6)
        p('obs_min_h', 0.03)
        p('obs_max_h', 0.60)
        p('obs_min_pts', 12)
        p('obs_step', 4)
        p('cam_timeout', 1.0)
        p('sensor_qos', 'best_effort')
        p('image_queue', 2)
        p('lidar_x', -0.04); p('lidar_y', 0.0); p('lidar_yaw_deg', 90.0)

        self.ns = self.get_parameter('ns').value
        self.model = YOLO(self.get_parameter('model').value)
        self.bridge = CvBridge()
        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)
        self.K = None
        self.state = IDLE; self.state_since = time.time()
        self.lock_id = None
        self.lock_center = None
        self.lock_hist = None
        self.reid_count = 0
        self.dist_hist = deque(maxlen=5)
        self.depth_miss = 0
        self.lop = None
        self.scan_time = 0.0
        self.front_clear = float('inf'); self.rear_clear = float('inf')
        self.depth_obs = float('inf')
        self.d_now = None
        self.cmd_prev = Twist()
        self.t_prev = time.time()
        self.enable_prev = False
        self.last_bearing = 0.0
        self.last_frame = 0.0
        self.cam_lost_logged = False
        self.sensor_cache = {}
        self.block_reason = ''

        rel = ReliabilityPolicy.BEST_EFFORT if self.P('sensor_qos') == 'best_effort' else ReliabilityPolicy.RELIABLE
        img_qos = QoSProfile(depth=int(self.P('image_queue')), reliability=rel, history=HistoryPolicy.KEEP_LAST)
        scan_qos = QoSProfile(depth=5, reliability=rel, history=HistoryPolicy.KEEP_LAST)
        rgb = Subscriber(self, Image, f'{self.ns}/oakd/rgb/image_raw', qos_profile=img_qos)
        dep = Subscriber(self, Image, f'{self.ns}/oakd/stereo/image_raw', qos_profile=img_qos)
        self.sync = ApproximateTimeSynchronizer([rgb, dep], 5, 0.1)
        self.sync.registerCallback(self.cb)
        self.create_subscription(CameraInfo, f'{self.ns}/oakd/rgb/camera_info', self.cb_info, scan_qos)
        self.create_subscription(LaserScan, f'{self.ns}/scan', self.cb_scan, scan_qos)

        topic = self.get_parameter('cmd_topic').value or f'{self.ns}/cmd_vel_unstamped'
        self.pub_cmd = self.create_publisher(Twist, topic, 1)
        self.pub_img = self.create_publisher(Image, f'{self.ns}/follow/image', 1)
        self.pub_state = self.create_publisher(String, f'{self.ns}/follow/state', 5)

        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(f"v3_safe target='{self.P('target')}' cmd→{topic} enable={self.P('enable')} "
                               f"sensor_qos={self.P('sensor_qos')} image_queue={self.P('image_queue')}")

    def P(self, n):
        """파라미터 현재값 읽기 (ros2 param set 으로 바뀐 값이 즉시 반영됨)"""
        return self.get_parameter(n).value

    def set_state(self, s):
        """상태 변경 + 로그. 로그 형식 'A -> B' 는 기록 도구(experiment_monitor, real_recorder)가 파싱하므로 바꾸지 말 것"""
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}')
            self.state = s; self.state_since = time.time()

    def _tf(self, target, source, stamp=None, timeout=0.2):
        """TF 조회 공통 함수. 실패하면 None (예외를 호출부마다 따로 처리하지 않도록 한 곳에 모음)."""
        try:
            return self.tf_buf.lookup_transform(target, source, stamp or Time(), Duration(seconds=timeout))
        except Exception:
            return None

    def sensor_T(self, key, frame, fallback):
        """센서 좌표계 frame → base_link 변환 (회전행렬 R, 이동벡터 t)."""
        if key not in self.sensor_cache:
            tf = self._tf(self.P('base_frame'), frame, timeout=0.05)
            if tf is None:
                self.get_logger().warn(f'{key} TF({frame}) 없음 → 기본값 사용', throttle_duration_sec=5.0)
                return fallback
            t = tf.transform.translation
            self.sensor_cache[key] = (quat_mat(tf.transform.rotation), np.array([t.x, t.y, t.z]))
            self.get_logger().info(f'{key} TF ok: {frame}')
        return self.sensor_cache[key]

    def corridor(self):
        """S3 통로: (로봇 반지름, 통로 반폭 = 반지름 + 여유). 라이다·depth 장애물 판정이 같은 통로를 쓴다."""
        rr = self.P('robot_radius')
        return rr, rr + self.P('corridor_margin')

    def backproject(self, u, v, Z):
        """「기초」 역투영: 픽셀 (u,v) 와 깊이 Z → 카메라 3D 점 (X=(u−cx)Z/fx, Y=(v−cy)Z/fy, Z). 배열도 그대로 됨."""
        fx, fy, cx, cy = self.K
        return (u - cx) * Z / fx, (v - cy) * Z / fy, Z

    def cb_info(self, m):
        """camera_info 를 처음 한 번만 저장. K 행렬 = [[fx,0,cx],[0,fy,cy],[0,0,1]] 을 1차원으로 펼친 9 개 값."""
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])
            self.get_logger().info('camera_info ok')

    def stop_dist(self, v=None):
        """S3 속도 v 에서의 정지거리 [m] (범퍼 앞 기준, 기본은 직전 명령 속도). 빠를수록 멀리서 멈춰야 함."""
        v = self.cmd_prev.linear.x if v is None else v
        return self.P('stop_base') + self.P('stop_gain') * max(0.0, v)

    def cb_scan(self, msg):
        """S3 라이다 콜백 (약 10 Hz)."""
        yaw = math.radians(self.P('lidar_yaw_deg')); c, s = math.cos(yaw), math.sin(yaw)
        R, t = self.sensor_T('lidar', msg.header.frame_id,
                             (np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), np.array([self.P('lidar_x'), self.P('lidar_y'), 0.19])))
        r = np.asarray(msg.ranges, np.float32)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        x, y = R[:2, :2] @ np.vstack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]) + t[:2, None]
        rr, half = self.corridor()
        inside = np.abs(y) < half
        f = x[inside & (x > 0)] - rr
        b = -x[inside & (x < 0)] - rr
        self.front_clear = float(f[f > -0.05].min()) if np.any(f > -0.05) else float('inf')
        self.rear_clear = float(b[b > -0.05].min()) if np.any(b > -0.05) else float('inf')
        self.scan_time = time.time()
        if self.P('enable') and self.cmd_prev.linear.x > 0 and self.front_clear < self.stop_dist():
            self.send(0.0, self.cmd_prev.angular.z)

    def depth_at(self, depth, x1, y1, x2, y2):
        """박스 안에서 RC카까지 거리 [m] 를 읽음 (원본과 동일)."""
        cx, cy = (x1 + x2) // 2, y1 + (y2 - y1) * 2 // 3
        r = max(1, min(self.P('patch') // 2, min(x2 - x1, y2 - y1) // 4))
        roi = depth[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1]
        v = roi[(roi > 0) & np.isfinite(roi)]
        if v.size < max(3, roi.size * 0.3):
            return None, (cx, cy)
        d = float(np.median(v))
        return (d / 1000.0 if depth.dtype == np.uint16 else d), (cx, cy)

    def depth_obstacle(self, depth, skip_box):
        """S4 depth 영상으로 '로봇 통로 안, 바닥 위' 물체까지 범퍼 기준 거리 [m]."""
        if self.K is None:
            return float('inf')
        st = self.P('obs_step')
        Z = depth[::st, ::st].astype(np.float32)
        if depth.dtype == np.uint16: Z = Z / 1000.0
        H, W = Z.shape
        v, u = np.mgrid[0:H, 0:W]; u = u * st; v = v * st
        m = np.isfinite(Z) & (Z > 0.1)
        if skip_box is not None:
            x1, y1, x2, y2 = skip_box; px, py = (x2 - x1) * 0.1, (y2 - y1) * 0.1
            m &= ~((u > x1 - px) & (u < x2 + px) & (v > y1 - py) & (v < y2 + py))
        R, t = self.sensor_T('camera', 'oakd_rgb_camera_optical_frame',
                             (np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float), np.array([-0.06, 0.0, 0.244])))
        x, y, z = R @ np.vstack(self.backproject(u[m], v[m], Z[m])) + t[:, None]
        rr, half = self.corridor()
        sel = (np.abs(y) < half) & (z > self.P('obs_min_h')) & (z < self.P('obs_max_h')) & (x > 0)
        f = np.sort(x[sel] - rr)
        n = self.P('obs_min_pts')
        return float(f[n - 1]) if f.size >= n else float('inf')

    @staticmethod
    def appearance(rgb, x1, y1, x2, y2):
        """「이론」 [6] 색 히스토그램 재식별. 박스 위쪽 2/3 을 HSV 로 바꿔 H(색상 16칸) × S(채도 8칸) 2D 히스토그램."""
        crop = rgb[y1:y1 + max(1, (y2 - y1) * 2 // 3), x1:x2]
        if crop.size == 0:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])
        cv2.normalize(h, h)
        return h

    def reid_score(self, cand, rgb):
        """재식별 점수 = 0.5 × 위치점수 + 0.5 × 외형점수 (0~1)."""
        pos = 1.0 - min(1.0, math.dist(cand['center'], self.lock_center) / self.P('reid_pos_radius'))
        app = 0.0
        if self.lock_hist is not None:
            h = self.appearance(rgb, *cand['box'])
            if h is not None:
                app = max(0.0, cv2.compareHist(self.lock_hist, h, cv2.HISTCMP_CORREL))
        return self.P('reid_pos_w') * pos + self.P('reid_app_w') * app

    def update_lop(self, u, v, d, header):
        """「이론」 [6] LOP(Last Observed Position): 지금 본 RC카 위치를 odom(고정) 좌표로 저장."""
        if self.K is None or d is None:
            return
        pt = PointStamped(); pt.header = header
        pt.point.x, pt.point.y, pt.point.z = self.backproject(u, v, d)
        tf = self._tf(self.P('odom_frame'), header.frame_id, header.stamp)
        if tf is not None:
            self.lop = do_transform_point(pt, tf)

    def bearing_to_lop(self):
        """LOP 가 지금 로봇 기준 몇 rad 방향인지 (왼쪽 +). 못 구하면 None."""
        tf = None if self.lop is None else self._tf(self.P('base_frame'), self.P('odom_frame'))
        if tf is None:
            return None
        p = do_transform_point(self.lop, tf)
        return math.atan2(p.point.y, p.point.x)

    def send(self, lin, ang):
        """모든 속도 명령이 반드시 지나는 곳."""
        now = time.time(); dt = max(1e-3, now - self.t_prev); self.t_prev = now
        lin = float(np.clip(lin, self.cmd_prev.linear.x - self.P('acc_lin') * dt, self.cmd_prev.linear.x + self.P('acc_lin') * dt))
        ang = float(np.clip(ang, self.cmd_prev.angular.z - self.P('acc_ang') * dt, self.cmd_prev.angular.z + self.P('acc_ang') * dt))
        lin = float(np.clip(lin, -self.P('max_rev'), self.P('max_lin')))
        ang = float(np.clip(ang, -self.P('max_ang'), self.P('max_ang')))
        why = []
        scan_ok = time.time() - self.scan_time < self.P('lidar_stale')
        if lin > 0:
            sd = self.stop_dist(max(lin, self.cmd_prev.linear.x))
            if not scan_ok: why.append('scan-stale')
            elif self.front_clear < sd: why.append(f'lidar{self.front_clear:.2f}')
            if self.depth_obs < sd: why.append(f'depthobs{self.depth_obs:.2f}')
            if self.d_now is not None and self.d_now < self.P('min_dist'): why.append('min_dist')
            if why: lin = 0.0
        elif lin < 0:
            if not scan_ok or self.rear_clear < self.P('rear_stop'):
                why.append('rear'); lin = 0.0
        self.block_reason = ','.join(why)
        cmd = Twist(); cmd.linear.x = lin; cmd.angular.z = ang
        self.cmd_prev = cmd
        if self.P('enable'):
            self.pub_cmd.publish(cmd)
        return cmd

    def servo(self, bearing, d):
        """「이론」 [1][2] 시각 서보: 오차 e 를 지수적으로 0 으로 보내는 v = −λe 를 두 축에 따로 적용."""
        dead = math.radians(self.P('bearing_dead_deg'))
        e_b = 0.0 if abs(bearing) < dead else bearing - math.copysign(dead, bearing)
        ang = -self.P('lambda_ang') * e_b
        lin = 0.0
        if d is not None:
            e_d = d - self.P('target_dist')
            if abs(e_d) > self.P('dist_dead'):
                lin = self.P('lambda_lin') * e_d
        return self.send(lin, ang)

    def cb(self, rgb_msg, dep_msg):
        self.last_frame = time.time(); self.cam_lost_logged = False
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')
        if depth.shape[:2] != rgb.shape[:2]:
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        H, W = rgb.shape[:2]

        res = self.model.track(rgb, conf=self.P('track_conf'), persist=True, verbose=False, tracker='bytetrack.yaml')[0]

        cands = []
        if res.boxes is not None and res.boxes.id is not None:
            for b in res.boxes:
                if self.model.names[int(b.cls[0])] != self.P('target'):
                    continue
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cands.append({'id': int(b.id[0]), 'box': (x1, y1, x2, y2), 'area': (x2 - x1) * (y2 - y1),
                              'conf': float(b.conf[0]),
                              'center': ((x1 + x2) / 2 / W, (y1 + y2) / 2 / H)})
        strong = [c for c in cands if c['conf'] >= self.P('conf')]

        chosen = None
        if self.state in (IDLE, SEARCH):
            if strong:
                chosen = max(strong, key=lambda c: c['area'])
                self.dist_hist.clear()
                self.lock_hist = self.appearance(rgb, *chosen['box'])
                self.set_state(TRACKING)
        elif self.state == TRACKING:
            chosen = next((c for c in cands if c['id'] == self.lock_id), None)
            if chosen is None:
                self.reid_count = 0; self.set_state(REID)
        if self.state == REID and chosen is None:
            self.reid_count += 1
            if strong and self.lock_center is not None:
                s, best = max(((self.reid_score(c, rgb), c) for c in strong), key=lambda sc: sc[0])
                if s >= self.P('reid_accept'):
                    chosen = best
                    self.get_logger().info(f'reid ok id={best["id"]} score={s:.2f}')
                    self.set_state(TRACKING)
            if chosen is None and self.reid_count > self.P('reid_frames'):
                self.set_state(SEARCH)

        self.depth_obs = self.depth_obstacle(depth, chosen['box'] if chosen else None)

        txt = f'[{self.state}] '
        if chosen is not None:
            x1, y1, x2, y2 = chosen['box']
            self.lock_id = chosen['id']; self.lock_center = chosen['center']
            if self.state == TRACKING and self.lock_hist is not None:
                h = self.appearance(rgb, x1, y1, x2, y2)
                if h is not None:
                    self.lock_hist = 0.9 * self.lock_hist + 0.1 * h
            dist, (u, v) = self.depth_at(depth, x1, y1, x2, y2)
            now = time.time()
            if dist is not None:
                self.dist_hist.append((now, dist)); self.depth_miss = 0
            else:
                self.depth_miss += 1
                if self.depth_miss >= self.P('depth_miss_max'):
                    self.dist_hist.clear()
            fresh = [dd for tt, dd in self.dist_hist if now - tt < self.P('depth_max_age')]
            d = float(np.median(fresh)) if fresh else None
            self.d_now = d
            fx, cx = (self.K[0], self.K[2]) if self.K else (W / 2 / math.tan(math.radians(32)), W / 2)
            bearing = -math.atan((u - cx) / fx)
            self.last_bearing = bearing
            self.update_lop(u, v, d, rgb_msg.header)
            cmd = self.servo(-bearing, d)
            txt += f'id{self.lock_id} d={d:.2f}m b={math.degrees(bearing):+.1f}deg ' if d is not None else f'id{self.lock_id} d=n/a({self.depth_miss}) '
            if self.P('debug_image'):
                cv2.circle(rgb, (u, v), 5, (0, 0, 255), -1)
        elif self.state == REID:
            self.d_now = None
            cmd = self.send(self.cmd_prev.linear.x * 0.5, self.cmd_prev.angular.z)
            txt += f'reid {self.reid_count} '
        else:
            self.d_now = None; cmd = self.cmd_prev

        if self.P('debug_image'):
            out = res.plot(img=rgb)
            lid = 'STALE' if time.time() - self.scan_time > self.P('lidar_stale') else f'{self.front_clear:.2f}/{self.rear_clear:.2f}'
            cv2.putText(out, txt + f'lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f} en={self.P("enable")}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(out, f'v3_safe F/R={lid} depthobs={self.depth_obs:.2f} stop={self.stop_dist():.2f} {self.block_reason}',
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255) if self.block_reason else (0, 255, 0), 2)
            m = self.bridge.cv2_to_imgmsg(out, 'bgr8'); m.header = rgb_msg.header
            self.pub_img.publish(m)
        self.publish_state(cmd)

    def publish_state(self, cmd):
        """S11 기록·디버깅용 상태 (JSON 문자열). 예: ros2 topic echo /robot1/follow/state"""
        r3 = lambda x: None if x is None or math.isinf(x) else round(x, 3)
        s = {'state': self.state, 'id': self.lock_id, 'd': r3(self.d_now), 'lin': r3(cmd.linear.x), 'ang': r3(cmd.angular.z),
             'block': self.block_reason, 'front': r3(self.front_clear), 'rear': r3(self.rear_clear),
             'depth_obs': r3(self.depth_obs), 'enable': bool(self.P('enable'))}
        self.pub_state.publish(String(data=json.dumps(s)))

    def watchdog(self):
        en = self.P('enable')
        if self.enable_prev and not en:
            self.stop(); self.get_logger().info('enable=false -> stop')
        self.enable_prev = en
        if not en:
            return
        if self.state in (TRACKING, REID) and time.time() - self.last_frame > self.P('cam_timeout'):
            self.stop(1)
            if not self.cam_lost_logged:
                self.get_logger().warn('camera timeout: stop')
                self.cam_lost_logged = True
            return
        if self.state not in (SEARCH, IDLE):
            return
        if self.state == SEARCH:
            if time.time() - self.state_since > self.P('search_max_s'):
                self.lock_id = None; self.lock_hist = None; self.lop = None
                self.set_state(IDLE); self.send(0.0, 0.0); return
            b = self.bearing_to_lop()
            b = self.last_bearing if b is None else b
            self.send(0.0, 0.0 if abs(b) < math.radians(8) else math.copysign(self.P('search_ang'), b))
        else:
            self.send(0.0, 0.0)

    def stop(self, n=3):
        """정지 명령 n 번 (기본 3 번 — 한 번은 유실될 수 있어서). 가감속 제한 없이 바로 0."""
        self.cmd_prev = Twist()
        for i in range(n):
            if i: time.sleep(0.05)
            self.pub_cmd.publish(Twist())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = FollowObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('shutdown: stop')
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
