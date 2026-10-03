#!/usr/bin/env python3
# =====================================================================================================
#  [리뷰용 주석본] follow_object_node_v3.py  — 원본 코드는 한 글자도 바꾸지 않고 주석만 추가
#  (scripts/check_annotated_identical.py 로 원본과 AST(코드 구조)가 같은지 검증)
#
#  표기:  「기초」 개념 설명 / 「이론」 근거 이론 / ⚠ Bx 가즈보에서 재현된 결함 (docs/CODE_REVIEW_GUIDE.md 의 B 번호)
#        「정정」 원본 주석·인용 중 사실과 다른 부분
#  안전 수정본은 follow_object_node_v3_safe.py (S1~S11)
# =====================================================================================================
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
# 「정정」 위 docstring 에 대해
#  - [CH06]: v = −λe 는 맞음. '2.5D 서보잉' 이라는 이름의 원전은 Malis·Chaumette·Boudet (IEEE T-RA 1999).
#            이 코드는 상호작용 행렬 없이 방위·거리 두 축을 따로 비례제어하는 단순화.
#  - [BT22]: ByteTrack 은 '저점수 박스까지 연결' 이 핵심인데 168행에서 conf=0.4 를 넘겨 그 기능이 꺼짐 (⚠ B11).
#  - [YE23]: ICRA 2023 Ye et al. 은 '부분 가림 시 관절로 사람 위치 추정' 논문. 재식별은 같은 그룹 RA-L 2024 가 더 가깝고,
#            111행의 '위치 0.5 + 색 0.5' 점수식은 어느 논문에도 없는 이 코드의 자체 설계.
#  - [AC20]: 색(H·S) 히스토그램·LOP·상태기계 개념은 맞음. 단 논문은 SLAM 으로 LOP 까지 '이동' + PI 제어,
#            이 코드는 LOP 방향으로 '제자리 회전' 만 + P 제어.
#  - 실행 예시의 target:=rc_car 는 이 프로젝트 모델(car, dummy)에 없는 클래스 → target:=car (⚠ B2)
#  - 실기/시뮬 모두 cmd_vel 은 TwistStamped → -r /robot1/cmd_vel:=/robot1/cmd_vel_unstamped 필요 (⚠ B1)
import math, time                       # 「기초」 math: 삼각함수 / time: 벽시계 시간(타임아웃·가감속 계산)
from collections import deque           # 「기초」 길이 제한 큐 — 최근 5 개 거리만 보관
import numpy as np, cv2, rclpy          # numpy 배열 연산, OpenCV 영상처리, rclpy = ROS 2 파이썬 라이브러리
from rclpy.node import Node             # 모든 ROS 2 노드의 부모 클래스
from rclpy.duration import Duration     # TF 조회 대기시간
from sensor_msgs.msg import Image, LaserScan, CameraInfo   # 「기초」 메시지 타입: 영상, 라이다 스캔, 카메라 내부 파라미터
from geometry_msgs.msg import Twist, PointStamped          # 속도명령(선속도 x, 각속도 z), 좌표계 붙은 3D 점
from cv_bridge import CvBridge          # ROS Image ↔ numpy 영상 변환
from message_filters import Subscriber, ApproximateTimeSynchronizer   # RGB·depth 를 시간으로 짝짓기
from tf2_ros import Buffer, TransformListener          # 「기초」 TF: 좌표계 변환 트리 수신·보관
from tf2_geometry_msgs import do_transform_point       # 점을 다른 좌표계로 변환
from ultralytics import YOLO            # YOLO 검출 + 내장 ByteTrack 추적

IDLE, TRACKING, REID, SEARCH = 'IDLE', 'TRACKING', 'REID', 'SEARCH'   # 상태기계의 4 상태


class FollowObjectNode(Node):
    def __init__(self):
        super().__init__('follow_object_node')          # 노드 이름 (ros2 param set /follow_object_node ...)
        p = self.declare_parameter                       # 「기초」 실행 중 바꿀 수 있는 설정값 선언
        p('model', 'best.pt'); p('ns', '/robot1'); p('target', 'rc_car'); p('conf', 0.4)   # ⚠ B2 target 기본값이 모델에 없는 클래스
        p('enable', False); p('debug_image', True)      # enable=false 면 명령을 계산만 하고 보내지 않음 (안전한 시작)
        # [CH06] 제어
        p('target_dist', 1.0); p('lambda_ang', 1.2); p('lambda_lin', 0.6)   # 목표거리 1 m, 비례 이득
        p('bearing_dead_deg', 4.0); p('dist_dead', 0.1)                      # 데드밴드 (작은 오차 무시 → 떨림 방지)
        p('max_lin', 0.3); p('max_ang', 1.0); p('acc_lin', 0.4); p('acc_ang', 1.5)   # 속도·가속도 상한
        # [YE23][AC20] 재식별
        p('reid_frames', 15)            # REID 상태 유지 프레임 수 (초과 시 SEARCH)  — 약 7 Hz 에서 ~2 s
        p('reid_pos_w', 0.5); p('reid_app_w', 0.5); p('reid_accept', 0.6)   # 재식별 점수 가중치·합격선
        p('reid_pos_radius', 0.3)       # 마지막 위치 반경(화면 비율) — 위치 점수 0이 되는 거리
        # [AC20] 상실 복구
        p('search_max_s', 6.0); p('search_ang', 0.5); p('odom_frame', 'odom')   # 탐색 6 s, 0.5 rad/s 회전, LOP 저장 좌표계
        # 안전 (공학적 장치, 논문 외)
        p('min_dist', 0.45); p('lidar_stop', 0.35); p('lidar_offset_deg', 0.0); p('lidar_stale', 1.0)
        # ⚠ B3 lidar_offset_deg=0: TB4 의 RPLIDAR 는 base 대비 +90° 장착 (실기 tf_static) → 0° 는 로봇 '왼쪽' 을 감시. −90 이어야 정면
        # ⚠ B5 lidar_stop 0.35 m 는 '라이다 중심' 기준 → 범퍼(반경 0.171 + 라이다 x −0.04)까지 실제 여유 ≈ 0.14 m
        p('patch', 9)                   # depth 패치 크기 [px]

        self.ns = self.get_parameter('ns').value
        self.model = YOLO(self.get_parameter('model').value)   # 가중치 로드
        self.bridge = CvBridge()
        self.tf_buf = Buffer(); self.tf_lis = TransformListener(self.tf_buf, self)   # TF 수신 시작
        self.K = None                               # camera_info → fx, cx
        self.state = IDLE; self.state_since = time.time()
        self.lock_id = None; self.lock_center = None; self.lock_hist = None; self.reid_count = 0   # 잠근 타깃의 id·중심·색
        self.dist_hist = deque(maxlen=5)            # 최근 거리 5 개 (중앙값용)
        self.lop = None                             # [AC20] 마지막 관측 위치 (odom frame PointStamped)
        self.front_min = float('inf'); self.scan_time = 0.0   # 라이다 정면 최소거리, 수신 시각
        self.cmd_prev = Twist(); self.t_prev = time.time(); self.enable_prev = False   # 가감속 계산용 직전 명령
        self.last_bearing = 0.0

        # 「기초」 RGB 와 depth 는 별도 토픽 → 타임스탬프 차 0.1 s 이내 쌍을 찾아 cb() 호출 (큐 5)
        rgb = Subscriber(self, Image, f'{self.ns}/oakd/rgb/image_raw')
        dep = Subscriber(self, Image, f'{self.ns}/oakd/stereo/image_raw')
        self.sync = ApproximateTimeSynchronizer([rgb, dep], 5, 0.1); self.sync.registerCallback(self.cb)
        self.create_subscription(CameraInfo, f'{self.ns}/oakd/rgb/camera_info', self.cb_info, 1)
        self.create_subscription(LaserScan, f'{self.ns}/scan', self.cb_scan, 5)
        self.pub_cmd = self.create_publisher(Twist, f'{self.ns}/cmd_vel', 1)
        # ⚠ B1 실기·시뮬의 {ns}/cmd_vel 은 TwistStamped. 여기서 Twist 로 퍼블리시하면 타입이 달라 연결 안 됨 → 로봇이 안 움직임
        #       실행 시 -r /robot1/cmd_vel:=/robot1/cmd_vel_unstamped 로 리맵 (실기에는 Twist 용 cmd_vel_unstamped 가 있음)
        self.pub_img = self.create_publisher(Image, f'{self.ns}/follow/image', 1)   # 디버그 영상
        self.create_timer(0.1, self.watchdog)       # 「기초」 0.1 s 마다 watchdog() — 영상이 없어도 도는 주기 작업
        self.get_logger().info(f"v3 target='{self.get_parameter('target').value}' enable=False")
        # (참고) enable=False 를 문자열로 고정 출력 — 실제 enable 파라미터 값과 무관

    def P(self, n): return self.get_parameter(n).value   # 파라미터 현재값 (ros2 param set 즉시 반영)

    def set_state(self, s):
        # 로그 형식 'A -> B' — 기록 도구들이 이 형식을 파싱함
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}'); self.state = s; self.state_since = time.time()

    # ---------- 센서 보조 ----------
    def cb_info(self, m):
        # 「기초」 K = [fx 0 cx; 0 fy cy; 0 0 1]. 핀홀 모델 u = fx·X/Z + cx. 처음 한 번만 저장
        if self.K is None: self.K = (m.k[0], m.k[4], m.k[2], m.k[5]); self.get_logger().info('camera_info ok')

    def cb_scan(self, msg):
        # 라이다 각도 기준으로 lidar_offset_deg 중심 ±20° 섹터의 최소거리
        n = len(msg.ranges); half = int(math.radians(20) / msg.angle_increment)   # ±20° 에 해당하는 인덱스 수
        i0 = int((math.radians(self.P('lidar_offset_deg')) - msg.angle_min) / msg.angle_increment) % n   # 중심 인덱스
        sel = [msg.ranges[(i0 + i) % n] for i in range(-half, half + 1)]          # 섹터의 거리들 (배열 끝에서 감김 처리)
        sel = [r for r in sel if msg.range_min < r < msg.range_max]               # 유효 측정만
        self.front_min = min(sel) if sel else float('inf'); self.scan_time = time.time()
        # ⚠ B3 오프셋 0° → 실제로는 로봇 왼쪽 섹터 (시뮬 실측: 정면 0.7 m 기둥을 0° 는 2.97 m(좌측 벽), −90° 는 0.60 m 로 봄)
        # ⚠ B6 고정 ±20° 섹터는 가까운 거리에서 로봇 폭(반경 0.171 m)을 못 덮음 (0.47 m 안쪽) → 옆으로 걸친 장애물 놓침
        # ⚠ 라이다 평면 높이 0.193 m — 그보다 낮은 물체(RC카 0.14 m, 상자, 문턱)는 보이지 않음

    def lidar_blocks(self):
        # 스캔이 1 s 이상 끊겼거나 정면이 lidar_stop 보다 가까우면 True (→ 전진 금지)
        return time.time() - self.scan_time > self.P('lidar_stale') or self.front_min < self.P('lidar_stop')

    def depth_at(self, depth, x1, y1, x2, y2):
        # 박스 가로 중앙, 세로 위에서 2/3 지점 주변 패치의 depth 중앙값 [m]. 유효 픽셀 30 % 미만이면 None
        cx, cy = (x1 + x2) // 2, y1 + (y2 - y1) * 2 // 3
        r = max(1, min(self.P('patch') // 2, min(x2 - x1, y2 - y1) // 4))
        roi = depth[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1]
        v = roi[(roi > 0) & np.isfinite(roi)]
        if v.size < max(3, roi.size * 0.3): return None, (cx, cy)
        d = float(np.median(v)); return (d / 1000.0 if depth.dtype == np.uint16 else d), (cx, cy)   # uint16 = mm

    # ---------- [AC20] 색 히스토그램 (HSV H·S, 상단 2/3 영역) ----------
    @staticmethod
    def appearance(rgb, x1, y1, x2, y2):
        # 「이론」 H(색상) 16 칸 × S(채도) 8 칸 2D 히스토그램, V(밝기) 제외 → 조명에 덜 민감. bin 수는 이 코드의 선택(논문 미명시)
        crop = rgb[y1:y1 + max(1, (y2 - y1) * 2 // 3), x1:x2]
        if crop.size == 0: return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]); cv2.normalize(h, h); return h

    # ---------- [YE23] 후보 점수: 위치 근접 + 외형 유사 ----------
    def reid_score(self, cand, rgb):
        # 점수 = 0.5·(1 − 화면거리/0.3) + 0.5·히스토그램 상관계수. 검은 RC카는 채도가 낮아 색 점수의 변별력이 약함
        pos = 1.0 - min(1.0, math.dist(cand['center'], self.lock_center) / self.P('reid_pos_radius'))
        app = 0.0
        if self.lock_hist is not None:
            h = self.appearance(rgb, *cand['box'])
            if h is not None: app = max(0.0, cv2.compareHist(self.lock_hist, h, cv2.HISTCMP_CORREL))
        return self.P('reid_pos_w') * pos + self.P('reid_app_w') * app

    # ---------- [AC20] LOP 저장 / 방위 ----------
    def update_lop(self, u, v, d, header):
        # 픽셀 (u,v) + 깊이 d → 카메라 3D 점 (역투영) → odom 좌표로 저장. 로봇이 돌아도 odom 의 점은 고정
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
        # 직전 명령 ± 가속도×dt 로 제한 → 최대속도로 자름 → enable 이면 퍼블리시
        now = time.time(); dt = max(1e-3, now - self.t_prev); self.t_prev = now
        lin = float(np.clip(lin, self.cmd_prev.linear.x - self.P('acc_lin') * dt, self.cmd_prev.linear.x + self.P('acc_lin') * dt))
        ang = float(np.clip(ang, self.cmd_prev.angular.z - self.P('acc_ang') * dt, self.cmd_prev.angular.z + self.P('acc_ang') * dt))
        cmd = Twist(); cmd.linear.x = float(np.clip(lin, -self.P('max_lin'), self.P('max_lin')))
        cmd.angular.z = float(np.clip(ang, -self.P('max_ang'), self.P('max_ang'))); self.cmd_prev = cmd
        if self.P('enable'): self.pub_cmd.publish(cmd)
        return cmd
        # ⚠ B5 안전 정지(라이다·min_dist 로 lin 을 0 으로 만든 경우)도 이 가감속 제한을 거침
        #       → 0.3 m/s 에서 0 까지 0.75 s, 약 0.11 m 더 진행 (시뮬 pole 시나리오: 보정 오프셋에서도 1 회 충돌 후 정지)

    def servo(self, bearing, d):
        """[CH06] 2.5D: ang = -λ_ang·e_bearing (IBVS, 특징=방위각), lin = λ_lin·e_dist (PBVS)"""
        # 「이론」 비례(P) 제어 — RC카가 속도 v 로 멀어지면 정상상태 거리 = 1.0 + v/0.6 (0.15 m/s → 1.25 m, 뒤처짐)
        dead = math.radians(self.P('bearing_dead_deg'))
        e_b = 0.0 if abs(bearing) < dead else bearing - math.copysign(dead, bearing)
        ang = -self.P('lambda_ang') * e_b
        lin = 0.0
        if d is not None:
            e_d = d - self.P('target_dist')
            if abs(e_d) > self.P('dist_dead'): lin = self.P('lambda_lin') * e_d
            if d < self.P('min_dist'): lin = min(lin, 0.0)       # 너무 가까우면 전진 금지 (후진은 허용)
        if self.lidar_blocks(): lin = min(lin, 0.0)              # 라이다가 막히면 전진 금지 (후진은 허용)
        return self.send(lin, ang)
        # ⚠ B7 후진(lin<0)에는 아무 검사도 없음 — 후방 감시 없음 (시뮬 approach: 뒤 기둥에 충돌)

    # ---------- 메인 ----------
    def cb(self, rgb_msg, dep_msg):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')          # ROS Image → numpy (BGR)
        depth = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')  # depth 원본 그대로 (uint16 mm)
        if depth.shape[:2] != rgb.shape[:2]:
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        H, W = rgb.shape[:2]
        res = self.model.track(rgb, conf=self.P('conf'), persist=True, verbose=False, tracker='bytetrack.yaml')[0]  # [BT22]
        # ⚠ B11 track() 에 conf=0.4 를 넘기면 0.4 미만 박스가 추적기 전에 버려짐. ultralytics 기본은 0.1
        #       ("trackers need low-confidence input" — engine/model.py). ByteTrack 의 저점수(0.1~0.25) 2차 연결이 꺼져
        #       점수가 잠깐 떨어질 때마다 id 가 끊기고 TRACKING→REID 가 잦아짐

        cands = []
        if res.boxes is not None and res.boxes.id is not None:
            for b in res.boxes:
                if self.model.names[int(b.cls[0])] != self.P('target'): continue   # 타깃 클래스만
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cands.append({'id': int(b.id[0]), 'box': (x1, y1, x2, y2), 'area': (x2 - x1) * (y2 - y1),
                              'center': ((x1 + x2) / 2 / W, (y1 + y2) / 2 / H)})

        chosen = None
        if self.state in (IDLE, SEARCH):                       # 새 대상: 가장 큰 박스
            if cands:
                chosen = max(cands, key=lambda c: c['area']); self.dist_hist.clear()
                self.lock_hist = self.appearance(rgb, *chosen['box']); self.set_state(TRACKING)
        elif self.state == TRACKING:
            chosen = next((c for c in cands if c['id'] == self.lock_id), None)   # 잠근 id 가 있나
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
            # ⚠ B4 depth 가 무효(None)여도 이력을 지우지 않음 → 옛 거리의 중앙값으로 계속 전진/후진
            #       min_dist 검사도 같은 옛 값을 씀 (시뮬 depthstop: 0.78 m 옛 값으로 −0.13 m/s 후진이 끝없이 지속)
            fx, cx = (self.K[0], self.K[2]) if self.K else (W / 2 / math.tan(math.radians(32)), W / 2)
            bearing = -math.atan((u - cx) / fx)                 # 왼쪽 +(ROS 각도 규약)
            # 「주의」 실기 RGB 의 렌즈 왜곡(rational_polynomial)을 보정하지 않음 → 화면 가장자리에서 방위 오차
            self.last_bearing = bearing; self.update_lop(u, v, d, rgb_msg.header)
            cmd = self.servo(-bearing, d)                       # servo는 "오른쪽 +" 오차를 받음
            txt += f'id{self.lock_id} d={d:.2f}m b={math.degrees(bearing):+.1f}deg ' if d else f'id{self.lock_id} d=n/a '
            if self.P('debug_image'): cv2.circle(rgb, (u, v), 5, (0, 0, 255), -1)
        elif self.state == REID:
            cmd = self.send(self.cmd_prev.linear.x * 0.5, self.cmd_prev.angular.z)   # 선속도 감쇠, 회전 유지
            # ⚠ B9 이 경로는 servo() 를 거치지 않아 라이다·min_dist 검사 없이 전진 (몇 프레임, 감쇠하며)
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
        # ⚠ B10 TRACKING/REID 에서 카메라가 끊기면 이 함수는 아무것도 안 함 → 노드는 정지 명령을 내지 않음
        #        (로봇이 멈추는 건 베이스의 cmd_vel 타임아웃 덕분)
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
    # ⚠ B8 Ctrl+C 시 rclpy 의 신호 처리기가 컨텍스트를 먼저 종료 → finally 의 stop() 퍼블리시가
    #       'RCLError: publisher's context is invalid' 로 실패 → 정지 명령이 나가지 않음 (베이스 타임아웃에 의존)


if __name__ == '__main__':
    main()
