#!/usr/bin/env python3
# ===== 2안 (모션 개선) — follow_object_node_v3_safe.py (10/3 실기 시험본) 복사 + 아래 3 가지만 추가 =====
#  M1 lat_comp       : 지연 보상 (기본은 방위만, lat_comp_dist 로 거리도). 영상이 찍힌 뒤 처리·명령까지 ~0.6 s (인지 ~0.4 + 구동 0.2, 10/3 실측) 동안
#                      로봇 자신이 돌고 움직인 만큼을 TF(odom) 로 빼서 '지금' 기준 방위·거리로 제어 → 좌우 진동 감소 목적
#  M2 slow_bearing_deg: 차가 화면 가장자리로 갈수록 전진을 줄여 회전 먼저 (화각 ±21° 로 좁음, 10/3: 코너에서 차가 옆으로 빠짐)
#  M3 ff_gain        : (기존 S16 코드, 기본 0) 차가 멀어지는 속도를 미리 더해 뒤처짐 감소 — run_plan2.sh 의 2b 에서만 켬
#  나머지(검출·상태기계·안전 게이트)는 v3_safe 와 같다. 끄면(lat_comp false, slow_bearing_deg 0, ff_gain 0) v3_safe 와 같은 동작.
# =====================================================================================================
#  follow_object_node_v3_safe.py — TurtleBot4 가 움직이는 RC카를 카메라로 찾아 일정 거리로 따라가는 ROS 2 노드
# -----------------------------------------------------------------------------------------------------
#  원본 follow_object_node_v3.py 의 추종 로직(검출 → 추적 → 상태기계 → 서보 제어)은 그대로 두고,
#  가즈보 실험(tb4_follow_sim, docs/CODE_REVIEW_GUIDE.md)에서 재현된 안전 결함만 고친 버전.
#
#  [읽는 법]  이 파일은 코드 리뷰용으로 "한 줄씩" 주석을 달았다.
#    - 「기초」 : ROS 2 / Python / 영상처리 기본 개념 설명
#    - 「이론」 : 제어·추적 이론과 근거 문헌 (번호는 docs/FOLLOW_NODE_EXPLAINED.md 7절 참고문헌)
#    - 「수정 Sx」: 원본 v3 대비 바뀐 곳 (S1~S11, 표는 CODE_REVIEW_GUIDE.md)
#    - 「주의」 : 실기에서 튜닝·확인이 필요한 곳
#
#  [수정 요약]
#   S1  기본값을 실기 robot1 에 맞춤 (Twist→cmd_vel_unstamped, target=car, 라이다 장착각은 TF 로 읽음)
#   S2  depth 가 연속으로 무효이면 거리 이력을 버림 → 거리를 모르면 전·후진하지 않음 (원본: 옛 거리로 계속 주행)
#   S3  라이다를 로봇 좌표로 바꿔 '로봇 폭 + 여유' 직사각형 통로로 판정, 정지거리를 속도에 비례, 스캔마다 즉시 판정
#   S4  depth 영상으로 라이다 평면(높이 0.19 m)보다 낮은 장애물 감지
#   S5  안전 정지는 가감속 제한을 거치지 않음 (즉시 0)
#   S6  후진은 후방 통로가 비었을 때만, 최대 0.1 m/s
#   S7  REID 상태의 관성 주행도 같은 안전 게이트 통과
#   S8  카메라가 cam_timeout 초 이상 끊기면 정지 (기본 1.0 s — 0.5 s 는 시뮬 부하만으로도 자주 걸려 1.0 으로 올림)
#   S9  Ctrl+C / SIGTERM 시 ROS 종료 "전에" 정지 명령 전송 (원본: 종료 후 전송 시도 → RCLError, 정지 명령 안 나감)
#   S10 ByteTrack 에는 저신뢰(≥0.1) 박스까지 넘기고, '새 타깃 선택·재식별'에만 conf(0.4) 적용
#       (원본: conf=0.4 를 track() 에 넘겨 ByteTrack 의 저점수 2차 연결이 꺼짐 → 잦은 id 끊김)
#   S11 상태/거리/정지사유를 {ns}/follow/state (JSON 문자열)로 퍼블리시 — 실험 기록·디버깅용
#   S12 센서 구독을 best-effort + 짧은 큐로: 로봇이 best-effort 로 보내도 받고(reliable 구독은 못 받음),
#       노트북이 느려도 오래된 프레임이 쌓이지 않게 (영상 큐 2, 원본은 기본 10 → 최대 ~1 s 지연 가능)
#
#  실행: python3 follow_object_node_v3_safe.py --ros-args -p model:=best.pt -p ns:=/robot1
#        (TF 가 네임스페이스 안이면 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static 추가)
#        시작은 enable=false. 주행: ros2 param set /follow_object_node enable true
# =====================================================================================================

# ---- 표준 라이브러리 -------------------------------------------------------------------------------
import json        # S11 상태를 JSON 문자열로 만들 때
import math        # atan, cos, sin, radians 등 수학 함수
import signal      # S9 운영체제 신호(SIGINT=Ctrl+C, SIGTERM=kill) 처리
import time        # time.time(): 벽시계 초 — 주기·타임아웃 계산에 사용 (시뮬 시간과 무관하게 실제 경과시간)
from collections import deque   # 길이 제한 큐: 최근 N 개만 보관 (거리 이력)

# ---- 외부 라이브러리 -------------------------------------------------------------------------------
import numpy as np  # 배열 연산 (영상·라이다 데이터는 큰 배열이라 반복문 대신 numpy 로 한꺼번에 계산)
import cv2          # OpenCV: 색공간 변환, 히스토그램, 그리기
import rclpy        # 「기초」 ROS 2 의 파이썬 클라이언트 라이브러리. 노드·토픽·타이머를 만든다
from rclpy.node import Node                     # 모든 ROS 2 노드의 부모 클래스
from rclpy.duration import Duration             # ROS 시간 간격 (TF 조회 대기시간에 사용)
from rclpy.signals import SignalHandlerOptions  # S9 rclpy 의 기본 Ctrl+C 처리기를 끄기 위해
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy   # S12 센서 구독 QoS
# 「기초」 메시지 타입 = 토픽으로 주고받는 데이터 구조 (C 의 struct 와 비슷)
from sensor_msgs.msg import Image, CompressedImage, LaserScan, CameraInfo   # 카메라 영상(원본·압축), 라이다 스캔, 카메라 내부 파라미터
from nav_msgs.msg import Odometry                         # S16 로봇 실제 속도 (차 속도 추정)
from geometry_msgs.msg import Twist, PointStamped          # 속도 명령(선속도·각속도), 좌표계가 붙은 3D 점
from std_msgs.msg import String                             # S11 상태 퍼블리시용 문자열
from cv_bridge import CvBridge                  # ROS Image ↔ OpenCV(numpy) 영상 변환기
# 「기초」 message_filters: 여러 토픽을 시간 기준으로 짝지어 한 번에 콜백. RGB 와 depth 는 따로 오므로 묶어야 함
from message_filters import Subscriber, ApproximateTimeSynchronizer
# 「기초」 TF = 좌표계(프레임) 사이의 변환 트리. 예) camera → base_link → odom. Buffer 가 수신·보관, Listener 가 구독
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point   # PointStamped 를 다른 좌표계로 변환
from ultralytics import YOLO                        # YOLO 검출기 + 내장 추적기(ByteTrack)

# 상태기계의 상태 이름 (문자열 상수로 두면 로그·디버그 화면에 그대로 찍을 수 있음)
IDLE, TRACKING, REID, SEARCH = 'IDLE', 'TRACKING', 'REID', 'SEARCH'


def quat_mat(q):
    """「기초」 쿼터니언(x,y,z,w) → 3×3 회전행렬. TF 는 회전을 쿼터니언으로 주기 때문에 행렬로 바꿔 점에 곱한다."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FollowObjectNode(Node):
    """「기초」 Node 를 상속한 클래스 하나 = ROS 2 노드 하나. __init__ 에서 파라미터·구독·발행·타이머를 등록하고,
    이후에는 메시지가 올 때마다 콜백 함수가 호출되는 '이벤트 구동' 방식으로 동작한다."""

    def __init__(self):
        super().__init__('follow_object_node')   # 노드 이름. ros2 node list / ros2 param set 에서 이 이름을 씀
        p = self.declare_parameter               # 「기초」 파라미터 = 실행 중에도 바꿀 수 있는 설정값 (ros2 param set)

        # ---- 검출·일반 ------------------------------------------------------------------------------
        p('model', 'best.pt')        # YOLO 가중치 파일 경로
        p('ns', '/robot1')           # 로봇 네임스페이스. 모든 토픽 앞에 붙음 (/robot1/scan ...)
        p('target', 'car')           # 수정 S1: 따라갈 클래스 이름 (원본 기본값 rc_car 는 이 모델 클래스에 없음)
        p('conf', 0.4)               # 새 타깃 선택·재식별에 쓰는 최소 신뢰도
        p('track_conf', 0.1)         # 수정 S10: 추적기에 넘길 최소 신뢰도 (ByteTrack 의 저점수 2차 연결용, ultralytics 기본값)
        p('enable', False)           # true 일 때만 속도 명령을 실제로 내보냄. 시작은 항상 false (안전)
        p('debug_image', True)       # 디버그 영상 퍼블리시 여부
        p('cmd_topic', '')           # 수정 S1: 비우면 {ns}/cmd_vel_unstamped (Twist). 실기 cmd_vel 은 TwistStamped 라 직접 못 씀

        # ---- 제어 (원본과 동일) — 「이론」 비례 제어 v = −λe [1][2] --------------------------------------
        p('target_dist', 1.0)        # 유지할 거리 [m] (카메라 → RC카 표면, depth 측정값 기준)
        p('lambda_ang', 1.2)         # 회전 이득 λ_ang: 방위 오차 1 rad 당 1.2 rad/s
        p('lambda_lin', 0.6)         # 전진 이득 λ_lin: 거리 오차 1 m 당 0.6 m/s
        p('bearing_dead_deg', 5.0)   # 방위 데드밴드: 회전 중 이 안으로 들어오면 회전 멈춤 (원본 4°)
        # S17 회전 히스테리시스: 실기 Create3 는 제자리 회전 명령 0.02~0.25 rad/s 를 모두 ~0.26 rad/s 로 실행 (T085·A 공통)
        #     → 비례 제어가 계전기(relay)처럼 동작해 정지한 차 앞에서 ±15° 좌우 진동 (261003-A 115~172 s, 부호 반전 36 회/52 s).
        #     0.26 rad/s × 지연 ~0.3 s ≈ 4.5° 넘침 → 5° 에서 멈추고 8° 를 넘어야 다시 돌기 시작.
        p('bearing_start_deg', 8.0)  # 정지 중 이 각도를 넘으면 회전 시작
        p('ang_min', 0.0)            # 회전할 때 최소 각속도 [rad/s] (0 = 끔. 0.25 로 강제하면 시뮬상 오히려 나빠짐)
        # S18 카메라 초점거리 덮어쓰기 [px] (0 = camera_info 사용). 실기 robot1 의 camera_info fx=fy=564.65 는 704×704 영상과 안 맞음:
        #     차 정지 + 로봇 회전 구간에서 방위 = −1.71·yaw (기대 −1.06, 잔차 0.2°), 라이다 스캔 매칭·IMU 로 odom 회전은 정확 확인
        #     → 실제 fx ≈ 564.65·1.71/1.06 ≈ 909 (수평 FOV ~42°, 1080p 정사각 크롭과 일치). 방위·depth 장애물·LOP 역투영에 모두 쓰임.
        p('fx_override', 0.0)
        p('dist_dead', 0.1)          # 거리 데드밴드: ±0.1 m 이내 오차는 무시
        p('max_lin', 0.3)            # 최대 전진 속도 [m/s] (Create3 상한 ~0.306 m/s 와 비슷)
        p('max_ang', 1.0)            # 최대 회전 속도 [rad/s]
        p('acc_lin', 0.4)            # 선가속도 제한 [m/s²] — 출발·감속을 부드럽게 (안전 정지에는 적용 안 함, S5)
        p('acc_ang', 1.5)            # 각가속도 제한 [rad/s²]

        # ---- 재식별·탐색 (원본과 동일) — 「이론」 [6] Algabri & Choi 의 색 특징 재식별·LOP 개념 ------------
        p('reid_frames', 15)         # REID 상태에서 버티는 프레임 수 (약 7 Hz → ~2 s). 넘으면 SEARCH
        p('reid_pos_w', 0.5)         # 재식별 점수 중 '위치 가까움' 가중치
        p('reid_app_w', 0.5)         # 재식별 점수 중 '색 비슷함' 가중치
        p('reid_accept', 0.6)        # 이 점수 이상이면 같은 RC카로 인정
        p('reid_pos_radius', 0.3)    # 화면 크기 대비 이 거리 이상 떨어지면 위치 점수 0
        # S15 실기 261003: 원본 0.5 rad/s × 6 s = 172° 라 뒤(180°)를 보기 전에 포기 (포기한 탐색 27 회 중 13 회가 165~174°),
        #     LOP 방향에서 멈춰 기다리다 시간 초과도 다수 → 0.8 rad/s × 10 s (≈ 1 바퀴 + LOP 확인) + LOP 확인 후 계속 회전
        p('search_max_s', 10.0)      # SEARCH 최대 시간 [s]. 넘으면 포기하고 IDLE (원본 6.0)
        p('search_ang', 0.8)         # SEARCH 회전 속도 [rad/s] (원본 0.5, max_ang 1.0 이하)
        p('search_wait_s', 0.7)      # S15 LOP 방향을 보고 이 시간 동안 없으면 같은 방향으로 계속 회전 (한 바퀴 훑기)
        p('odom_frame', 'odom')      # 마지막 관측 위치(LOP)를 저장할 고정 좌표계

        # ---- 안전 -----------------------------------------------------------------------------------
        p('min_dist', 0.45)          # RC카와 이 거리[m]보다 가까우면 전진 금지 (원본과 같음, 단 '신선한' 거리로만 판단 S2)
        p('patch', 9)                # depth 를 읽을 정사각 패치 한 변 [px]
        p('lidar_stale', 1.0)        # 라이다가 이 시간[s] 이상 안 오면 '모름' → 전·후진 금지
        p('base_frame', 'base_link') # 로봇 몸체 좌표계 (x 앞, y 왼쪽, z 위, 원점은 바닥)
        p('robot_radius', 0.171)     # TB4 반지름 [m] (지름 0.342 m)
        p('corridor_margin', 0.02)   # S3/S17 원형 몸체 여유 [m] (원본 직선 앞면 + 0.05 → 원형 + 0.02)
        p('stop_base', 0.12)         # S3 정지거리 = stop_base + stop_gain × 속도 (범퍼 앞 기준) 「주의」 실기에서 실측 보정
        p('stop_gain', 0.6)          #    0.3 m/s → 0.30 m, 0.1 m/s → 0.18 m
        p('rear_stop', 0.35)         # S6 후방 통로가 이 거리[m]보다 가까우면 후진 금지
        p('max_rev', 0.10)           # S6 최대 후진 속도 [m/s]
        p('depth_miss_max', 3)       # S2 depth 무효가 이 프레임 수 이상 연속이면 거리 이력 폐기
        p('depth_max_age', 0.6)      # S2 이 시간[s]보다 오래된 거리값은 버림
        p('obs_min_h', 0.03)         # S4 바닥에서 이 높이[m] 이상인 점만 장애물 (바닥 잡음 제거) 「주의」 실기 depth 잡음 보고 조정
        p('obs_max_h', 0.60)         # S4 이 높이 이하만 (로봇이 부딪힐 높이)
        p('obs_min_pts', 12)         # S4 이만큼 점이 모여야 장애물로 인정 (잡음 1~2 점 무시)
        p('obs_step', 4)             # S4 depth 를 4 픽셀 간격으로 띄엄띄엄 사용 (계산량 1/16)
        p('cam_timeout', 1.0)        # S8 카메라가 이 시간[s] 이상 끊기면 정지
        p('sensor_qos', 'best_effort')   # S12 'best_effort'(기본: 어떤 발행자와도 연결) 또는 'reliable'
        p('image_queue', 2)          # S12 영상 구독 큐 깊이 (작을수록 최신 프레임만 처리)
        # S13 'compressed'(기본): RGB 는 .../compressed (jpeg), depth 는 .../compressedDepth (png) 를 받아 노트북에서 디코드.
        #     실기 261003: Wi-Fi 로 raw(RGB 1.4 MB + depth 1 MB/프레임)를 구독하면 로봇 oakd 가 밀려 RGB 가 15~20 s 늦게
        #     도착 → RGB·depth 시각이 안 맞아 동기화 콜백이 한 번도 안 불림. compressed 는 10 Hz, 지연 0.05~0.14 s.
        p('image_transport', 'compressed')   # 'compressed' 또는 'raw'
        # S14 근거리·오검출 보호 (실기 261003 줄자 실측: 카메라 0.70 m 정상 (+4 cm), 0.45 m 는 차 아래가 잘려 지붕만 보이고
        #     depth 가 지붕을 읽어 +23 cm, 클래스도 dummy 로 바뀜 → min_dist 가 걸리지 않음. 벽 TV 를 car 로 오검출 → 7 m)
        p('ground_only', True)       # 박스 아래 끝이 지평선(cy) 위인 검출은 버림 — 바닥 위 RC카만 (벽 TV 등 제외)
        p('cut_bottom_px', 4)        # 박스 아래 끝이 화면 하단에서 이 픽셀 이내면 '잘림' → 거리 불신 → 전·후진 0
        p('min_fresh', 3)            # 유효 거리가 이 개수 이상 쌓여야 중앙값 사용 (재획득 직후 튐 1 프레임 무시)
        # S16 거리 제어 안정화 (실기 T085: 멈춤-출발 0.23 회/s, 후진 20 % — 목표가 가까울수록 증가)
        p('dist_dead_cont', True)    # 거리 데드밴드를 방위처럼 연속으로 (False = 원본 계단)
        p('lat_comp', True)         # M1 지연 보상 — 방위 (TF 실패 프레임은 보상 없이)
        p('lat_comp_dist', False)   # M1 거리까지 보상. 끔: 따라가는 중엔 차도 같이 멀어져 로봇 전진만 빼면 거리를 작게 봐 뒤처짐 (시뮬 확인)
        p('slow_bearing_deg', 21.0) # M2 방위가 이 각도면 전진 0 (그 사이는 직선으로 줄임). 0 = 끔
        p('ff_gain', 0.0)            # 차 속도 피드포워드 이득 (0 = 끔, 실험값 0.5~1.0)
        # TF 를 못 읽을 때 쓰는 라이다 장착값 — 실기 스냅샷 tf_static: base_link → rplidar_link (x=−0.04, yaw=+90°)
        p('lidar_x', -0.04); p('lidar_y', 0.0); p('lidar_yaw_deg', 90.0)

        # ---- 내부 상태 변수 -----------------------------------------------------------------------------
        self.ns = self.get_parameter('ns').value
        self.model = YOLO(self.get_parameter('model').value)   # 가중치 로드 (수 초 걸림)
        self.bridge = CvBridge()
        self.tf_buf = Buffer()                                  # TF 저장소 (기본 10 s 보관)
        self.tf_lis = TransformListener(self.tf_buf, self)      # /tf, /tf_static 을 구독해 tf_buf 를 채움
        self.K = None                                           # 카메라 내부 파라미터 (fx, fy, cx, cy). camera_info 수신 후 채움
        self.state = IDLE; self.state_since = time.time()       # 현재 상태와 진입 시각
        self.lock_id = None          # 따라가는 RC카의 추적 id (ByteTrack 이 부여)
        self.lock_center = None      # 마지막으로 본 박스 중심 (화면 정규화 좌표 0~1)
        self.lock_hist = None        # 타깃의 색 히스토그램 (재식별용)
        self.reid_count = 0          # REID 상태에서 지난 프레임 수
        self.dist_hist = deque(maxlen=5)   # 최근 5 개의 (시각, 거리) — 중앙값으로 잡음 제거
        self.depth_miss = 0          # S2 depth 무효 연속 횟수
        self.lop = None              # 마지막 관측 위치 (odom 좌표의 PointStamped)
        self.scan_time = 0.0         # 마지막 라이다 수신 시각
        self.front_clear = float('inf'); self.rear_clear = float('inf')   # S3 범퍼 기준 전·후방 통로 여유 [m]
        self.depth_obs = float('inf')    # S4 depth 로 본 통로 안 장애물까지 여유 [m]
        self.d_now = None                # S2 지금 쓸 수 있는 '신선한' 거리 (없으면 None)
        self.v_car = None                # S16 추정한 차의 지름 방향 속도 (없으면 None)
        self.turning = False             # S17 방위 히스테리시스 상태
        self.cmd_prev = Twist()          # 직전에 보낸 명령 (가감속 제한 계산용)
        self.t_prev = time.time()        # 직전 명령 시각
        self.enable_prev = False         # enable 이 true→false 로 바뀌는 순간을 잡기 위해
        self.last_bearing = 0.0          # 마지막 방위 (LOP 를 못 구할 때 SEARCH 방향)
        self.search_phase = 'turn'       # S15 SEARCH 단계: turn(LOP 로 회전) → wait(LOP 확인) → sweep(같은 방향 계속 회전)
        self.search_dir = 1.0; self.search_t = 0.0
        self.last_frame = 0.0            # S8 마지막 카메라 프레임 처리 시각
        self.cam_lost_logged = False     # S8 같은 경고를 반복해서 찍지 않기 위해
        self.lidar_T = None; self.cam_T = None   # 센서 → base_link 변환 (회전행렬, 이동벡터) 캐시
        self.block_reason = ''           # 마지막으로 전진/후진을 막은 이유 (디버그 화면·상태 토픽)
        self.lop_base_img = None         # M1 영상 시각의 로봇 기준 RC카 점
        self.comp = None                 # M1 이번 프레임 보상량 (Δ방위, Δ거리) — 상태 토픽에 기록

        # ---- 구독 ------------------------------------------------------------------------------------
        # 「기초」 RGB 와 depth 는 별도 토픽 → ApproximateTimeSynchronizer 가 타임스탬프 차이 0.1 s 이내인 쌍을 찾아 cb() 호출
        # S12 「기초」 QoS: 발행자가 reliable 이면 best-effort 구독도 연결되지만, 발행자가 best-effort 인데 구독이 reliable 이면
        #     연결이 안 돼 메시지가 0 개 온다. 센서는 최신값이 중요하므로 best-effort + 짧은 큐가 표준 (ROS 'sensor data' QoS)
        rel = ReliabilityPolicy.BEST_EFFORT if self.P('sensor_qos') == 'best_effort' else ReliabilityPolicy.RELIABLE
        img_qos = QoSProfile(depth=int(self.P('image_queue')), reliability=rel, history=HistoryPolicy.KEEP_LAST)
        scan_qos = QoSProfile(depth=5, reliability=rel, history=HistoryPolicy.KEEP_LAST)
        self.compressed = self.P('image_transport') == 'compressed'   # S13
        if self.compressed:
            rgb = Subscriber(self, CompressedImage, f'{self.ns}/oakd/rgb/image_raw/compressed', qos_profile=img_qos)
            dep = Subscriber(self, CompressedImage, f'{self.ns}/oakd/stereo/image_raw/compressedDepth', qos_profile=img_qos)
        else:
            rgb = Subscriber(self, Image, f'{self.ns}/oakd/rgb/image_raw', qos_profile=img_qos)
            dep = Subscriber(self, Image, f'{self.ns}/oakd/stereo/image_raw', qos_profile=img_qos)
        self.sync = ApproximateTimeSynchronizer([rgb, dep], 5, 0.1)   # 쌍 맞추기 대기열 5, 허용 오차 0.1 s
        self.sync.registerCallback(self.cb)
        # 「기초」 create_subscription(타입, 토픽, 콜백, QoS): 메시지가 오면 콜백 실행
        self.create_subscription(CameraInfo, f'{self.ns}/oakd/rgb/camera_info', self.cb_info, scan_qos)
        self.create_subscription(LaserScan, f'{self.ns}/scan', self.cb_scan, scan_qos)
        self.odom_v = None; self.odom_t = 0.0   # S16 명령이 아닌 실제 속도 (가감속·오버슈트 반영)
        self.create_subscription(Odometry, f'{self.ns}/odom', self.cb_odom, scan_qos)

        # ---- 발행 ------------------------------------------------------------------------------------
        topic = self.get_parameter('cmd_topic').value or f'{self.ns}/cmd_vel_unstamped'
        self.pub_cmd = self.create_publisher(Twist, topic, 1)                       # 속도 명령
        self.pub_img = self.create_publisher(Image, f'{self.ns}/follow/image', 1)   # 디버그 영상
        self.pub_state = self.create_publisher(String, f'{self.ns}/follow/state', 5)  # S11 상태 JSON

        # 「기초」 타이머: 0.1 s 마다 watchdog() 호출 (영상이 안 와도 돌아가야 하는 일: 정지, SEARCH 회전)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(f"v3_motion(2안) lat_comp={self.P('lat_comp')} slow_bearing={self.P('slow_bearing_deg')} ff={self.P('ff_gain')} target='{self.P('target')}' cmd→{topic} enable={self.P('enable')} "
                               f"sensor_qos={self.P('sensor_qos')} image_queue={self.P('image_queue')} "
                               f"image_transport={self.P('image_transport')}")

    def P(self, n):
        """파라미터 현재값 읽기 (ros2 param set 으로 바뀐 값이 즉시 반영됨)"""
        return self.get_parameter(n).value

    def set_state(self, s):
        """상태 변경 + 로그. 로그 형식 'A -> B' 는 기록 도구(experiment_monitor, real_recorder)가 파싱하므로 바꾸지 말 것"""
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}')
            self.state = s; self.state_since = time.time()
            if s == SEARCH:              # S15 탐색 시작: 차가 사라진 쪽(마지막 방위 부호)으로 돈다
                self.search_phase = 'turn'; self.search_dir = math.copysign(1.0, self.last_bearing or 1.0)

    # =================================================================================================
    #  센서 보조
    # =================================================================================================
    def sensor_T(self, frame):
        """센서 좌표계 frame → base_link 변환을 TF 에서 읽음. 실패하면 None.
        「기초」 lookup_transform(target, source, 시각) 은 source 좌표의 점을 target 좌표로 옮기는 변환을 준다.
        rclpy.time.Time() (= 0) 은 '가장 최근 값' 이라는 뜻."""
        try:
            tf = self.tf_buf.lookup_transform(self.P('base_frame'), frame, rclpy.time.Time(), Duration(seconds=0.05))
            t = tf.transform.translation
            return quat_mat(tf.transform.rotation), np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def cb_info(self, m):
        """camera_info 를 처음 한 번만 저장. K 행렬 = [[fx,0,cx],[0,fy,cy],[0,0,1]] 을 1차원으로 펼친 9 개 값.
        「기초」 핀홀 카메라: 3D 점 (X,Y,Z) 가 화면 u = fx·X/Z + cx, v = fy·Y/Z + cy 에 찍힌다."""
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])   # fx, fy, cx, cy
            fo = self.P('fx_override')
            if fo > 0:                                   # S18 cx, cy 는 그대로 (중앙 크롭이라 주점 동일)
                self.K = (fo, fo, m.k[2], m.k[5])
            self.get_logger().info(f'camera_info ok fx={self.K[0]:.1f} (camera_info {m.k[0]:.1f})')

    def cb_odom(self, m):
        self.odom_v = m.twist.twist.linear.x; self.odom_t = time.time()

    def stop_dist(self):
        """S3 현재 속도에서의 정지거리 [m] (범퍼 앞 기준). 빠를수록 멀리서 멈춰야 함."""
        return self.P('stop_base') + self.P('stop_gain') * max(0.0, self.cmd_prev.linear.x)

    def cb_scan(self, msg):
        """S3 라이다 콜백 (약 10 Hz).
        「기초」 LaserScan 은 각도별 거리 배열: i 번째 거리 ranges[i] 의 각도 = angle_min + i·angle_increment (라이다 좌표).
        원본은 '정면 ±20° 섹터' 를 라이다 각도로 직접 골랐는데, 라이다가 90° 돌아 달려 있어 기본값이 옆을 봤다(B3).
        여기서는 모든 점을 base_link 좌표 (x 앞, y 왼쪽) 로 옮긴 뒤 '로봇 폭 통로' 안의 점만 본다 → 장착각 실수에 강함."""
        if self.lidar_T is None:                               # 처음 한 번 라이다 장착 위치를 TF 에서 읽음
            T = self.sensor_T(msg.header.frame_id)
            if T is None:                                      # TF 를 못 읽으면 파라미터 기본값 (실기 tf_static 값)
                yaw = math.radians(self.P('lidar_yaw_deg')); c, s = math.cos(yaw), math.sin(yaw)
                T = (np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), np.array([self.P('lidar_x'), self.P('lidar_y'), 0.19]))
                self.get_logger().warn(f'lidar TF({msg.header.frame_id}) 실패 → 기본값 사용')
            else:
                self.get_logger().info(f'lidar TF ok: x={T[1][0]:.3f} yaw={math.degrees(math.atan2(T[0][1, 0], T[0][0, 0])):.0f}deg')
            self.lidar_T = T
        r = np.asarray(msg.ranges, np.float32)                          # 거리 배열
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))     # 각 거리의 각도
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)  # 유효한 측정만 (inf, 측정범위 밖 제외)
        R, t = self.lidar_T
        # 극좌표(r, a) → 라이다 직교좌표 (r cos a, r sin a) → 회전 R·이동 t 로 base_link 좌표
        pts = R[:2, :2] @ np.vstack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]) + t[:2, None]
        x, y = pts
        rr = self.P('robot_radius')
        ri = rr + self.P('corridor_margin')           # S17 여유를 더한 원형 몸체 반지름
        # S17 원형 몸체 기하: 점 (x, y) 에 닿기까지 앞(뒤)으로 갈 수 있는 거리 = |x| − √(ri² − y²)  (|y| < ri 일 때만 닿음)
        #     원본 S3 은 앞면을 x = rr 직선으로 봐서, 로봇 '옆' 벽(x 0.12~0.17, |y| 0.19~0.22)을 앞쪽 −0.02 로 계산해 전진을 막았다
        #     (실기 261003-A: 벽 옆에서 전진 막힘 96 % → 원형 + 2 cm 60 %). 몸체 반지름 −3 cm 안쪽 점은 자기 부품 반사로 제외.
        inside = (np.abs(y) < ri) & (np.hypot(x, y) > rr - 0.03)
        reach = np.sqrt(np.maximum(0.0, ri * ri - y * y))
        f = x[inside & (x > 0)] - reach[inside & (x > 0)]    # 앞쪽 점
        b = -x[inside & (x < 0)] - reach[inside & (x < 0)]   # 뒤쪽 점
        self.front_clear = float(f.min()) if f.size else float('inf')
        self.rear_clear = float(b.min()) if b.size else float('inf')
        self.scan_time = time.time()
        # 카메라 콜백(~7 Hz)을 기다리지 않고 스캔이 올 때마다 바로 정지 판정 (원본은 카메라 주기에서만 판정)
        if self.P('enable') and self.cmd_prev.linear.x > 0 and self.front_clear < self.stop_dist():
            self.send(0.0, self.cmd_prev.angular.z)

    def depth_at(self, depth, x1, y1, x2, y2):
        """박스 안에서 RC카까지 거리 [m] 를 읽음 (원본과 동일).
        표시점: 가로 중앙, 세로는 위에서 2/3 지점 (차체 몸통 — 위쪽은 배경이 섞이기 쉬움).
        그 주변 최대 9×9 패치에서 유효(0 아님) 픽셀이 30 % 이상이면 중앙값. 아니면 None (= 이번 프레임 거리 없음)."""
        cx, cy = (x1 + x2) // 2, y1 + (y2 - y1) * 2 // 3
        r = max(1, min(self.P('patch') // 2, min(x2 - x1, y2 - y1) // 4))   # 박스가 작으면 패치도 작게
        roi = depth[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1]
        v = roi[(roi > 0) & np.isfinite(roi)]
        if v.size < max(3, roi.size * 0.3):
            return None, (cx, cy)
        d = float(np.median(v))                  # 중앙값: 튀는 값(가장자리 배경)에 강함
        return (d / 1000.0 if depth.dtype == np.uint16 else d), (cx, cy)   # OAK-D depth 는 uint16 [mm]

    def depth_obstacle(self, depth, skip_box):
        """S4 depth 영상으로 '로봇 통로 안, 바닥 위' 물체까지 범퍼 기준 거리 [m].
        라이다는 높이 0.19 m 평면만 보므로 그보다 낮은 상자·문턱·RC카 자체를 못 본다 → 카메라로 보완.
        「주의」 카메라는 base_link 기준 x=−0.06 (중심보다 뒤), 근거리 한계 ~0.3 m → 범퍼 앞 ~0.07 m 이내는 못 봄."""
        if self.K is None:
            return float('inf')
        if self.cam_T is None:   # 카메라 광학좌표 → base_link 변환. TF 실패 시 실기 tf_static 값
            self.cam_T = self.sensor_T('oakd_rgb_camera_optical_frame') or \
                (np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float), np.array([-0.06, 0.0, 0.244]))
        st = self.P('obs_step'); fx, fy, cx, cy = self.K
        Z = depth[::st, ::st].astype(np.float32)          # 4 픽셀 간격으로 샘플링
        if depth.dtype == np.uint16: Z = Z / 1000.0       # mm → m
        H, W = Z.shape
        v, u = np.mgrid[0:H, 0:W]; u = u * st; v = v * st  # 각 샘플의 원래 픽셀 좌표
        m = np.isfinite(Z) & (Z > 0.1)                     # 유효 depth 만
        if skip_box is not None:                           # 따라가는 RC카 박스(+10 %)는 장애물에서 제외 (그건 min_dist 가 담당)
            x1, y1, x2, y2 = skip_box; px, py = (x2 - x1) * 0.1, (y2 - y1) * 0.1
            m &= ~((u > x1 - px) & (u < x2 + px) & (v > y1 - py) & (v < y2 + py))
        Z = Z[m]; u = u[m]; v = v[m]
        # 「기초」 역투영: 픽셀 (u,v) 와 깊이 Z → 카메라 3D 점 (X=(u−cx)Z/fx, Y=(v−cy)Z/fy, Z)
        P = np.vstack([(u - cx) * Z / fx, (v - cy) * Z / fy, Z])
        R, t = self.cam_T
        x, y, z = R @ P + t[:, None]                       # base_link 좌표 (z = 바닥 위 높이)
        ri = self.P('robot_radius') + self.P('corridor_margin')
        sel = (np.abs(y) < ri) & (z > self.P('obs_min_h')) & (z < self.P('obs_max_h')) & (x > 0)
        f = np.sort(x[sel] - np.sqrt(ri * ri - y[sel] ** 2))   # S17 원형 몸체까지 갈 수 있는 거리, 가까운 순
        n = self.P('obs_min_pts')
        return float(f[n - 1]) if f.size >= n else float('inf')   # n 번째로 가까운 점 = 잡음 몇 점은 무시

    # =================================================================================================
    #  외형(색) 특징, 재식별, 마지막 관측 위치 — 원본과 동일
    # =================================================================================================
    @staticmethod
    def appearance(rgb, x1, y1, x2, y2):
        """「이론」 [6] 색 히스토그램 재식별. 박스 위쪽 2/3 을 HSV 로 바꿔 H(색상 16칸) × S(채도 8칸) 2D 히스토그램.
        V(밝기)는 버림 → 조명 변화에 덜 민감. 정규화해서 크기가 달라도 비교 가능.
        「주의」 검은 RC카는 채도가 낮아 색 정보가 적다 → 외형 점수가 비슷한 검은 물체(dummy)와 잘 구분 못 할 수 있음."""
        crop = rgb[y1:y1 + max(1, (y2 - y1) * 2 // 3), x1:x2]
        if crop.size == 0:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])   # OpenCV H 범위는 0~180
        cv2.normalize(h, h)
        return h

    def reid_score(self, cand, rgb):
        """재식별 점수 = 0.5 × 위치점수 + 0.5 × 외형점수 (0~1).
        위치점수: 마지막 중심과 화면상 거리가 0 이면 1, reid_pos_radius(화면의 30 %) 이상이면 0.
        외형점수: 히스토그램 상관계수 (1 = 같음, 음수는 0 처리).
        이 가중합 식은 v3 자체 설계 (문헌 [7][8] 은 파이프라인 개념만 해당 — CODE_REVIEW_GUIDE 의 문헌 대조 참고)."""
        pos = 1.0 - min(1.0, math.dist(cand['center'], self.lock_center) / self.P('reid_pos_radius'))
        app = 0.0
        if self.lock_hist is not None:
            h = self.appearance(rgb, *cand['box'])
            if h is not None:
                app = max(0.0, cv2.compareHist(self.lock_hist, h, cv2.HISTCMP_CORREL))
        return self.P('reid_pos_w') * pos + self.P('reid_app_w') * app

    def update_lop(self, u, v, d, header):
        """「이론」 [6] LOP(Last Observed Position): 지금 본 RC카 위치를 odom(고정) 좌표로 저장.
        로봇이 돌아도 odom 좌표의 점은 그대로라, 나중에 '그 점이 지금 어느 방향인지' 다시 계산할 수 있다."""
        if self.K is None or d is None:
            return False
        fx, fy, cx, cy = self.K
        pt = PointStamped(); pt.header = header                  # header.frame_id = 카메라 광학 좌표계
        pt.point.x, pt.point.y, pt.point.z = (u - cx) * d / fx, (v - cy) * d / fy, d   # 역투영
        try:
            tf = self.tf_buf.lookup_transform(self.P('odom_frame'), header.frame_id, header.stamp, Duration(seconds=0.2))
            self.lop = do_transform_point(pt, tf)
            if self.P('lat_comp'):   # M1 같은 점을 '영상 시각'의 base_link 로 (보상 전 기준)
                tb = self.tf_buf.lookup_transform(self.P('base_frame'), header.frame_id, header.stamp, Duration(seconds=0.02))   # 막히지 않게 짧게
                self.lop_base_img = do_transform_point(pt, tb)
            return True
        except Exception:
            return False   # TF 가 아직 없으면 이번엔 저장 안 함 (SEARCH 는 last_bearing 으로 대체)

    def ego_comp(self):
        """M1 「이론」 지연 보상 (Smith 예측기의 단순형: 아는 지연 동안의 '자기 움직임' 만 미리 반영).
        같은 RC카 점을 영상 시각의 로봇 기준(p_img)과 지금 로봇 기준(p_now)으로 표현해 차이를 돌려줌.
        → (Δ방위 rad 왼쪽 +, Δ거리 m). 차 자신의 움직임은 보상하지 않음 (그건 M3 피드포워드 몫)."""
        if self.lop is None or self.lop_base_img is None:
            return None
        try:
            tf = self.tf_buf.lookup_transform(self.P('base_frame'), self.P('odom_frame'), rclpy.time.Time(), Duration(seconds=0.05))
            p = do_transform_point(self.lop, tf).point; q = self.lop_base_img.point
            db = math.atan2(p.y, p.x) - math.atan2(q.y, q.x)
            db = (db + math.pi) % (2 * math.pi) - math.pi
            return db, math.hypot(p.x, p.y) - math.hypot(q.x, q.y)
        except Exception:
            return None

    def bearing_to_lop(self):
        """LOP 가 지금 로봇 기준 몇 rad 방향인지 (왼쪽 +). 못 구하면 None."""
        if self.lop is None:
            return None
        try:
            tf = self.tf_buf.lookup_transform(self.P('base_frame'), self.P('odom_frame'), rclpy.time.Time(), Duration(seconds=0.2))
            p = do_transform_point(self.lop, tf)
            return math.atan2(p.point.y, p.point.x)
        except Exception:
            return None

    # =================================================================================================
    #  안전 게이트 + 제어 — S5 S6 S7
    # =================================================================================================
    def send(self, lin, ang):
        """모든 속도 명령이 반드시 지나는 곳.
        1) 가감속 제한으로 부드럽게 → 2) 최대 속도로 자르기 → 3) 안전 게이트(막히면 즉시 0, S5) → 4) 퍼블리시."""
        now = time.time(); dt = max(1e-3, now - self.t_prev); self.t_prev = now
        # 1) 직전 명령 ± (가속도 × 경과시간) 범위로 제한 → 급출발·급회전 방지
        lin = float(np.clip(lin, self.cmd_prev.linear.x - self.P('acc_lin') * dt, self.cmd_prev.linear.x + self.P('acc_lin') * dt))
        ang = float(np.clip(ang, self.cmd_prev.angular.z - self.P('acc_ang') * dt, self.cmd_prev.angular.z + self.P('acc_ang') * dt))
        # 2) 후진은 max_rev 까지만 (S6), 전진 max_lin, 회전 ±max_ang
        lin = float(np.clip(lin, -self.P('max_rev'), self.P('max_lin')))
        ang = float(np.clip(ang, -self.P('max_ang'), self.P('max_ang')))
        # 3) 안전 게이트 — 가감속 제한을 '거친 뒤' 0 으로 덮어쓰므로 정지는 즉시 (원본은 여기도 0.4 m/s² 로 서서히 감속)
        why = []
        scan_ok = time.time() - self.scan_time < self.P('lidar_stale')
        if lin > 0:
            sd = self.P('stop_base') + self.P('stop_gain') * max(lin, self.cmd_prev.linear.x)   # 현재·직전 중 빠른 속도 기준
            if not scan_ok: why.append('scan-stale')                                   # 라이다를 모르면 전진 금지
            elif self.front_clear < sd: why.append(f'lidar{self.front_clear:.2f}')     # 라이다 통로 막힘
            if self.depth_obs < sd: why.append(f'depthobs{self.depth_obs:.2f}')        # 낮은 장애물 (S4)
            if self.d_now is not None and self.d_now < self.P('min_dist'): why.append('min_dist')   # RC카 너무 가까움
            if why: lin = 0.0
        elif lin < 0:
            if not scan_ok or self.rear_clear < self.P('rear_stop'):                   # 후방을 모르거나 막힘
                why.append('rear'); lin = 0.0
        self.block_reason = ','.join(why)
        cmd = Twist(); cmd.linear.x = lin; cmd.angular.z = ang
        self.cmd_prev = cmd                      # 다음 가감속 계산의 기준 (막혔으면 0 부터 다시 가속)
        if self.P('enable'):                     # enable=false 면 계산만 하고 로봇에는 안 보냄
            self.pub_cmd.publish(cmd)
        return cmd

    def servo(self, bearing, d):
        """「이론」 [1][2] 시각 서보: 오차 e 를 지수적으로 0 으로 보내는 v = −λe 를 두 축에 따로 적용.
          - 회전: 특징 = 화면상 방위각(영상 기반, IBVS 성격)  → ang = −λ_ang · e_b
          - 전진: 특징 = depth 로 잰 거리(3D, PBVS 성격)     → lin =  λ_lin · (d − target_dist)
          ('2.5D' 라는 이름의 원전은 [3] Malis 1999. 여기서는 상호작용 행렬 없이 축을 분리한 단순 비례 제어.)
        bearing 인자는 '오른쪽 +' 부호로 받음 (호출부 cb() 에서 −bearing 을 넘김).
        「이론」 비례 제어의 한계: RC카가 일정 속도 v 로 멀어지면 정상상태 거리 = target_dist + v/λ_lin
                  (0.15 m/s → 1.25 m). 0.3 m/s(max_lin) 보다 빠르면 거리가 계속 벌어진다."""
        dead = math.radians(self.P('bearing_dead_deg'))
        # S17 히스테리시스: 돌고 있으면 dead 안에서 멈춤, 멈춰 있으면 bearing_start 를 넘어야 시작
        if self.turning and abs(bearing) < dead:
            self.turning = False
        elif not self.turning and abs(bearing) > math.radians(self.P('bearing_start_deg')):
            self.turning = True
        e_b = bearing - math.copysign(dead, bearing) if self.turning else 0.0   # 데드밴드만큼 빼서 연속적으로
        ang = -self.P('lambda_ang') * e_b
        if self.turning and 0.0 < abs(ang) < self.P('ang_min'):
            ang = math.copysign(self.P('ang_min'), ang)   # 어차피 Create3 가 ~0.26 으로 돌므로 명시 (예측 가능하게)
        lin = 0.0
        if d is not None:                        # S2 거리를 모르면 전·후진 0 (회전은 계속해서 타깃을 화면 중앙에)
            e_d = d - self.P('target_dist')
            dd = self.P('dist_dead')
            if abs(e_d) > dd:
                if self.P('dist_dead_cont'):     # S16 방위와 같이 데드밴드만큼 빼서 연속 (원본: 경계에서 0 → λ·0.1 = 0.06 m/s 계단)
                    e_d -= math.copysign(dd, e_d)
                lin = self.P('lambda_lin') * e_d     # 멀면 +(전진), 가까우면 −(후진, send() 에서 후방 확인)
            # S16 피드포워드: 차의 지름 방향 속도를 미리 더해 비례 제어의 정상상태 지연(v/λ)과 멈춤-출발 반복을 줄임
            if self.P('ff_gain') > 0 and self.v_car is not None:
                lin += self.P('ff_gain') * self.v_car
            sb = self.P('slow_bearing_deg')                # M2 화면 가장자리면 회전 먼저 (전진만 줄임, 후진은 그대로)
            if sb > 0 and lin > 0:
                lin *= max(0.0, 1.0 - abs(bearing) / math.radians(sb))
        return self.send(lin, ang)

    def estimate_v_car(self, now):
        """S16 차의 지름 방향 속도 [m/s] (멀어지면 +). 최근 신선한 거리들의 직선 기울기 d' + 로봇 자신의 전진 속도.
        「이론」 등속 모델 — 칼만 예측 [R10] 의 가장 단순한 형태. 표본 3 개·0.2 s 미만이면 None.
        로봇 속도는 odom 실측 (0.5 s 이상 끊기면 직전 명령으로 대체)."""
        pts = [(tt, dd) for tt, dd in self.dist_hist if now - tt < self.P('depth_max_age')]
        if len(pts) < 3 or pts[-1][0] - pts[0][0] < 0.2:
            return None
        tt = np.array([p[0] for p in pts]); dd = np.array([p[1] for p in pts])
        slope = float(np.polyfit(tt - tt[0], dd, 1)[0])
        v_robot = self.odom_v if (self.odom_v is not None and now - self.odom_t < 0.5) else self.cmd_prev.linear.x
        return float(np.clip(slope + v_robot, -0.5, 0.5))   # 추정 범위 (명령은 send() 에서 max_lin 으로 다시 자름)

    # =================================================================================================
    #  메인 콜백 — RGB·depth 한 쌍이 올 때마다 (약 7~10 Hz)
    # =================================================================================================
    def cb(self, rgb_msg, dep_msg):
        self.last_frame = time.time(); self.cam_lost_logged = False      # S8 카메라 살아 있음
        if self.compressed:   # S13 jpeg → BGR, compressedDepth = 12 바이트 헤더 + 16 비트 PNG (uint16 mm)
            rgb = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)
            depth = cv2.imdecode(np.frombuffer(bytes(dep_msg.data[12:]), np.uint8), cv2.IMREAD_UNCHANGED)
            if rgb is None or depth is None:
                return                                                    # 디코드 실패 프레임은 건너뜀 (S8 타임아웃이 감시)
        else:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')              # ROS Image → numpy (H×W×3, BGR 순서)
            depth = self.bridge.imgmsg_to_cv2(dep_msg, 'passthrough')     # depth 는 변환 없이 (uint16 mm 또는 float32 m)
        if depth.shape[:2] != rgb.shape[:2]:                              # 해상도가 다르면 depth 를 RGB 크기에 맞춤
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)  # 깊이값 섞이지 않게 최근접
        H, W = rgb.shape[:2]

        # ---- 1) 검출 + 추적 「이론」 [4] ByteTrack -----------------------------------------------------
        # persist=True: 이전 프레임의 추적 상태를 유지 (같은 물체에 같은 id).
        # S10: track_conf(0.1) 로 낮은 점수 박스까지 추적기에 넘겨야 ByteTrack 의 2차 연결이 동작한다.
        #      (원본은 conf=0.4 를 넘겨 0.4 미만 박스가 추적기 전에 버려짐 → 점수가 잠깐 떨어지면 id 가 끊김)
        res = self.model.track(rgb, conf=self.P('track_conf'), persist=True, verbose=False, tracker='bytetrack.yaml')[0]

        # ---- 2) 후보 만들기 -------------------------------------------------------------------------
        cands = []
        if res.boxes is not None and res.boxes.id is not None:      # id 가 하나도 없으면(추적 시작 전) 후보 없음
            for b in res.boxes:
                if self.model.names[int(b.cls[0])] != self.P('target'):
                    continue                                         # 타깃 클래스(car)만
                x1, y1, x2, y2 = map(int, b.xyxy[0])                # 박스 좌상단·우하단 픽셀
                if self.P('ground_only') and y2 < (self.K[3] if self.K else H / 2):
                    continue                                         # S14 지평선 위 = 바닥 물체가 아님 (벽 TV 오검출)
                cands.append({'id': int(b.id[0]), 'box': (x1, y1, x2, y2), 'area': (x2 - x1) * (y2 - y1),
                              'conf': float(b.conf[0]),
                              'center': ((x1 + x2) / 2 / W, (y1 + y2) / 2 / H)})   # 화면 크기로 나눠 0~1
        strong = [c for c in cands if c['conf'] >= self.P('conf')]   # S10 새로 잡거나 재식별할 때는 확실한 것만

        # ---- 3) 상태기계 -----------------------------------------------------------------------------
        chosen = None
        if self.state in (IDLE, SEARCH):                     # 타깃이 없음 → 가장 큰(=가까운) 확실한 박스를 새 타깃으로
            if strong:
                chosen = max(strong, key=lambda c: c['area'])
                self.dist_hist.clear()
                self.lock_hist = self.appearance(rgb, *chosen['box'])
                self.set_state(TRACKING)
        elif self.state == TRACKING:                          # 잠근 id 가 이번 프레임에도 있나? (S10: 점수가 낮아도 추적기가 이어준 박스면 유지)
            chosen = next((c for c in cands if c['id'] == self.lock_id), None)
            if chosen is None:
                self.reid_count = 0; self.set_state(REID)
        if self.state == REID and chosen is None:             # id 가 끊김 → 위치+색 점수로 같은 RC카 찾기
            self.reid_count += 1
            if strong and self.lock_center is not None:
                best = max(strong, key=lambda c: self.reid_score(c, rgb)); s = self.reid_score(best, rgb)
                if s >= self.P('reid_accept'):
                    chosen = best
                    self.get_logger().info(f'reid ok id={best["id"]} score={s:.2f}')
                    self.set_state(TRACKING)
            if chosen is None and self.reid_count > self.P('reid_frames'):
                self.set_state(SEARCH)                        # 너무 오래 못 찾음 → 탐색 (회전은 watchdog 가 담당)

        # ---- 4) 낮은 장애물 검사 (S4) — 따라가는 박스는 제외 -----------------------------------------------
        self.depth_obs = self.depth_obstacle(depth, chosen['box'] if chosen else None)

        # ---- 5) 제어 ---------------------------------------------------------------------------------
        txt = f'[{self.state}] '
        if chosen is not None:
            x1, y1, x2, y2 = chosen['box']
            self.lock_id = chosen['id']; self.lock_center = chosen['center']
            if self.state == TRACKING and self.lock_hist is not None:   # 외형 모델을 천천히 갱신 (조명·각도 변화 반영)
                h = self.appearance(rgb, x1, y1, x2, y2)
                if h is not None:
                    self.lock_hist = 0.9 * self.lock_hist + 0.1 * h        # 지수이동평균
            dist, (u, v) = self.depth_at(depth, x1, y1, x2, y2)
            now = time.time()
            if dist is not None:                                         # S2 유효 거리 → 이력에 추가
                self.dist_hist.append((now, dist)); self.depth_miss = 0
            else:                                                        # S2 무효 → 연속 횟수가 쌓이면 이력 폐기
                self.depth_miss += 1
                if self.depth_miss >= self.P('depth_miss_max'):
                    self.dist_hist.clear()
            fresh = [dd for tt, dd in self.dist_hist if now - tt < self.P('depth_max_age')]   # 오래된 값 제외
            d = float(np.median(fresh)) if len(fresh) >= self.P('min_fresh') else None      # S14 1~2 개로는 안 믿음
            cut = y2 >= H - self.P('cut_bottom_px')                      # S14 차 아래가 화면 밖 → depth 가 지붕을 읽음
            if cut:
                d = None
            self.d_now = d
            self.v_car = self.estimate_v_car(now) if d is not None else None   # S16
            # 방위: 핀홀 모델 b = −atan((u − cx)/fx). 화면 중앙보다 왼쪽(u<cx) → b>0 (ROS 규약: 왼쪽 회전 +)
            # 「주의」 실기 RGB 는 렌즈 왜곡이 강하지만 보정하지 않음 → 화면 가장자리일수록 방위 오차
            fx, cx = (self.K[0], self.K[2]) if self.K else (W / 2 / math.tan(math.radians(32)), W / 2)   # camera_info 전엔 FOV 64° 가정
            bearing = -math.atan((u - cx) / fx)
            ok_lop = self.update_lop(u, v, d, rgb_msg.header)
            self.comp = None
            if self.P('lat_comp') and ok_lop:              # M1 지난 ~0.6 s 동안 로봇이 돈·움직인 만큼 반영
                self.comp = self.ego_comp()
                if self.comp is not None:
                    bearing += self.comp[0]
                    if d is not None and self.P('lat_comp_dist'):
                        d = max(0.0, d + self.comp[1]); self.d_now = d
            self.last_bearing = bearing
            cmd = self.servo(-bearing, d)                                # servo 는 '오른쪽 +' 오차를 받음
            if cut:
                self.block_reason = (self.block_reason + ',cut').strip(',')   # 상태 토픽·디버그 화면에 표시
            txt += f'id{self.lock_id} d={d:.2f}m b={math.degrees(bearing):+.1f}deg ' if d else \
                f'id{self.lock_id} d=n/a({"cut" if cut else self.depth_miss}) '
            if self.P('debug_image'):
                cv2.circle(rgb, (u, v), 5, (0, 0, 255), -1)              # 거리 읽은 점 표시
        elif self.state == REID:
            self.d_now = None; self.v_car = None
            # REID 중에는 직전 선속도를 반으로 줄이며 회전은 유지 (관성처럼 잠깐 이어감) — S7: send() 의 안전 게이트 통과
            cmd = self.send(self.cmd_prev.linear.x * 0.5, self.cmd_prev.angular.z)
            txt += f'reid {self.reid_count} '
        else:
            self.d_now = None; self.v_car = None; cmd = self.cmd_prev                       # IDLE/SEARCH 명령은 watchdog 담당

        # ---- 6) 디버그 영상 + 상태 토픽 ----------------------------------------------------------------
        if self.P('debug_image'):
            out = res.plot(img=rgb)                                       # YOLO 박스·id·점수 그리기
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
        s = {'state': self.state, 'id': self.lock_id, 'd': None if self.d_now is None else round(self.d_now, 3),
             'lin': round(cmd.linear.x, 3), 'ang': round(cmd.angular.z, 3), 'block': self.block_reason,
             'front': None if math.isinf(self.front_clear) else round(self.front_clear, 3),
             'rear': None if math.isinf(self.rear_clear) else round(self.rear_clear, 3),
             'depth_obs': None if math.isinf(self.depth_obs) else round(self.depth_obs, 3), 'enable': bool(self.P('enable')),
             'vcar': None if self.v_car is None else round(self.v_car, 3),
             'comp': None if self.comp is None else [round(math.degrees(self.comp[0]), 2), round(self.comp[1], 3)]}   # M1 [Δ방위°, Δ거리 m]
        self.pub_state.publish(String(data=json.dumps(s)))

    # =================================================================================================
    #  watchdog — 0.1 s 마다 (영상이 안 와도 실행)
    # =================================================================================================
    def watchdog(self):
        en = self.P('enable')
        if self.enable_prev and not en:                 # enable 을 끈 순간 → 정지 명령 3 번
            self.stop(); self.get_logger().info('enable=false -> stop')
        self.enable_prev = en
        if not en:
            return
        # S8 따라가는 중인데 카메라가 cam_timeout 이상 안 옴 → 정지 (원본: 아무것도 안 해서 베이스 타임아웃에 의존)
        if self.state in (TRACKING, REID) and time.time() - self.last_frame > self.P('cam_timeout'):
            self.cmd_prev = Twist(); self.pub_cmd.publish(Twist())
            if not self.cam_lost_logged:
                self.get_logger().warn('camera timeout: stop')   # 'A -> B' 형식이 아님 (상태 전환으로 오해되지 않게)
                self.cam_lost_logged = True
            return
        if self.state not in (SEARCH, IDLE):
            return
        if self.state == SEARCH:
            if time.time() - self.state_since > self.P('search_max_s'):   # 시간 초과 → 포기
                self.lock_id = None; self.lock_hist = None; self.lop = None
                self.set_state(IDLE); self.send(0.0, 0.0); return
            ang = self.P('search_ang')
            if self.search_phase == 'turn':                             # 1) LOP 쪽으로 제자리 회전
                b = self.bearing_to_lop()
                if b is None:
                    b = self.last_bearing
                if abs(b) < math.radians(8):
                    self.search_phase = 'wait'; self.search_t = time.time(); self.send(0.0, 0.0)
                else:
                    self.search_dir = math.copysign(1.0, b); self.send(0.0, self.search_dir * ang)
            elif self.search_phase == 'wait':                           # 2) LOP 를 잠깐 확인 (원본은 여기서 시간 초과까지 대기)
                self.send(0.0, 0.0)
                if time.time() - self.search_t > self.P('search_wait_s'):
                    self.search_phase = 'sweep'
            else:                                                       # 3) S15 없으면 같은 방향으로 계속 → 뒤까지 한 바퀴
                self.send(0.0, self.search_dir * ang)
        else:
            self.send(0.0, 0.0)                                         # IDLE: 정지

    def stop(self):
        """정지 명령 3 번 (한 번은 유실될 수 있어서)"""
        self.cmd_prev = Twist()
        for _ in range(3):
            self.pub_cmd.publish(Twist()); time.sleep(0.05)


def main():
    # S9 rclpy 의 기본 신호 처리기는 Ctrl+C 를 받으면 ROS 컨텍스트를 먼저 내려버려 이후 stop() 퍼블리시가 실패한다.
    #    기본 처리기를 끄고 Ctrl+C / SIGTERM 을 파이썬 KeyboardInterrupt 로 받아, 컨텍스트가 살아 있을 때 정지를 보낸다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)                         # Ctrl+C → KeyboardInterrupt
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))   # kill → KeyboardInterrupt
    node = FollowObjectNode()
    try:
        rclpy.spin(node)               # 「기초」 콜백을 계속 처리하는 이벤트 루프 (여기서 프로그램이 머묾)
    except KeyboardInterrupt:
        node.get_logger().info('shutdown: stop')
    finally:
        try:
            node.stop()                # 컨텍스트가 살아 있으므로 정지 명령이 실제로 나감
        finally:
            node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
