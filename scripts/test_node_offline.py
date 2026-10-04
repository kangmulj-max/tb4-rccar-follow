#!/usr/bin/env python3
"""Gazebo 없이 v3_safe 노드 기능 시험 (정답을 아는 입력을 직접 넣어 확인).
  ROS_DOMAIN_ID=77 python3 scripts/test_node_offline.py [가중치]
실제 노드를 만들고(YOLO 로드 포함) 발행기만 가짜로 바꿔, 각 기능이 기대한 명령을 내는지 본다.
시뮬·로봇과 통신하지 않도록 다른 ROS_DOMAIN_ID 에서 실행할 것."""
import glob, json, math, os, sys, time
import numpy as np, cv2, rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follow_object_node_v3_safe as F

ROOT = os.path.expanduser('~/tb4_follow_sim')
W8 = sys.argv[1] if len(sys.argv) > 1 else f'{ROOT}/weights/yolo12n_amr_best.pt'


class Pub:                                     # 가짜 발행기: 보낸 메시지를 모아 둠
    def __init__(self): self.msgs = []
    def publish(self, m): self.msgs.append(m)


results = []
def check(name, ok, detail=''):
    results.append((name, bool(ok), detail)); print(f"[{'PASS' if ok else 'FAIL'}] {name}  {detail}")


rclpy.init(args=['--ros-args', '-p', f'model:={W8}', '-p', 'enable:=true'])
n = F.FollowObjectNode()
n.pub_cmd, n.pub_img, n.pub_state = Pub(), Pub(), Pub()
n.K = (300.0, 300.0, 352.0, 352.0)            # 704×704, 시뮬 camera_info 와 비슷한 값
P = n.P


def reset(lin=0.0, ang=0.0):
    n.cmd_prev = F.Twist(); n.cmd_prev.linear.x = lin; n.cmd_prev.angular.z = ang
    n.t_prev = time.time(); n.scan_time = time.time()
    n.front_clear = n.rear_clear = n.depth_obs = float('inf'); n.d_now = None


def scan(ranges_by_deg):
    """라이다 좌표 각도(도)→거리 를 넣은 360 점 스캔"""
    m = LaserScan(); m.header = Header(frame_id='rplidar_link')
    m.angle_min, m.angle_increment, m.range_min, m.range_max = -math.pi, math.radians(1), 0.15, 12.0
    r = [float('inf')] * 360
    for deg, d in ranges_by_deg.items(): r[int(deg) + 180] = d
    m.ranges = r; return m


# ---- 1. 라이다 통로 (S3) + TF 없을 때 기본값 (R1/B15) ----------------------------------------------
reset()
n.cb_scan(scan({-90: 0.5}))       # 라이다는 +90° 돌아 달림 → 라이다 −90° 방향 = 로봇 정면
check('S3 정면 장애물: 라이다 장착각 보정 후 범퍼까지 거리', abs(n.front_clear - (0.5 - 0.04 - 0.171)) < 0.01, f'front_clear={n.front_clear:.3f} (기대 0.289)')
check('B15 TF 없음 → 기본값 사용, 캐시하지 않음', 'lidar' not in n.sensor_cache, f'cache={list(n.sensor_cache)}')
reset(); n.cb_scan(scan({0: 0.5}))  # 라이다 0° = 로봇 왼쪽 → 통로 밖
check('S3 옆(통로 밖) 물체는 무시', math.isinf(n.front_clear), f'front_clear={n.front_clear}')
reset(); n.cb_scan(scan({90: 0.5}))  # 라이다 +90° = 로봇 뒤
check('S3 후방 물체는 rear_clear 로', abs(n.rear_clear - (0.5 + 0.04 - 0.171)) < 0.01, f'rear_clear={n.rear_clear:.3f}')

# ---- 2. 안전 게이트 send() (S5 S6 S7) ----------------------------------------------------------------
reset(0.3); n.front_clear = 0.2
c = n.send(0.3, 0.0)
check('S5 정면 막힘 → 감속 없이 즉시 0', c.linear.x == 0.0 and 'lidar' in n.block_reason, f'lin={c.linear.x} why={n.block_reason}')
reset(0.3); n.scan_time = 0.0
c = n.send(0.3, 0.0)
check('라이다 끊김(1 s↑) → 전진 금지', c.linear.x == 0.0 and 'scan-stale' in n.block_reason, n.block_reason)
reset(0.3); n.depth_obs = 0.15
c = n.send(0.3, 0.0)
check('S4 depth 낮은 장애물 → 전진 금지', c.linear.x == 0.0 and 'depthobs' in n.block_reason, n.block_reason)
reset(0.2); n.d_now = 0.4
c = n.send(0.2, 0.0)
check('min_dist: RC카 0.45 m 이내 → 전진 금지', c.linear.x == 0.0 and 'min_dist' in n.block_reason, n.block_reason)
reset(-0.1); n.rear_clear = 0.2
c = n.send(-0.3, 0.0)
check('S6 후방 막힘 → 후진 금지', c.linear.x == 0.0 and n.block_reason == 'rear', n.block_reason)
reset(-0.1)
c = n.send(-0.3, 0.0)
check('S6 후진 속도 상한 0.1 m/s', abs(c.linear.x + 0.1) < 1e-6, f'lin={c.linear.x}')
reset(0.0); time.sleep(0.1)
c = n.send(0.3, 0.0)
check('가감속 제한: 0 에서 0.1 s 뒤 ≤ 0.4×0.1 m/s', 0 < c.linear.x <= 0.4 * 0.11 + 1e-6, f'lin={c.linear.x:.3f}')
check('정지거리 = 0.12 + 0.6·v', abs(n.stop_dist(0.3) - 0.30) < 1e-9 and abs(n.stop_dist(0.1) - 0.18) < 1e-9)

# ---- 3. 서보 제어 servo() -------------------------------------------------------------------------
reset(0.3); c = n.servo(0.0, 1.5)
check('servo: 1.5 m → 전진 (0.6×0.5=0.3)', abs(c.linear.x - 0.3) < 1e-6, f'lin={c.linear.x}')
reset(0.0); c = n.servo(0.0, 1.05)
check('servo: 거리 데드밴드 ±0.1 m → 0', c.linear.x == 0.0)
reset(0.0); c = n.servo(0.0, None)
check('S2 servo: 거리 모름 → 전·후진 0', c.linear.x == 0.0)
reset(0.0); n.t_prev = time.time() - 10        # 가감속 제한이 걸리지 않게 직전 명령을 오래전으로
c = n.servo(math.radians(20), 1.0)
check('servo: 오른쪽 20° → 오른쪽(−) 회전 1.2×16°', abs(c.angular.z + 1.2 * math.radians(16)) < 1e-6, f'ang={c.angular.z:.3f}')
reset(0.0); c = n.servo(math.radians(3), 1.0)
check('servo: 방위 데드밴드 4° 이내 → 회전 0', c.angular.z == 0.0)

# ---- 4. depth 읽기 depth_at() (S2) / 낮은 장애물 depth_obstacle() (S4) ---------------------------------
dep = np.zeros((704, 704), np.uint16)
d, _ = n.depth_at(dep, 300, 300, 400, 400)
check('depth_at: 유효 픽셀 부족 → None', d is None)
dep[:] = 1500
d, (u, v) = n.depth_at(dep, 300, 300, 400, 400)
check('depth_at: mm → m, 박스 세로 2/3 지점', abs(d - 1.5) < 1e-6 and v == 300 + 100 * 2 // 3, f'd={d} v={v}')
plane = np.full((704, 704), 600, np.uint16)         # 정면 0.6 m 벽
o = n.depth_obstacle(plane, None)
check('S4 정면 0.6 m 벽 → 범퍼까지 ≈ 0.6−0.06−0.171', abs(o - (0.6 - 0.06 - 0.171)) < 0.02, f'depth_obs={o:.3f} (기대 0.369)')
o2 = n.depth_obstacle(plane, (0, 0, 704, 704))
check('S4 따라가는 RC카 박스 안은 장애물에서 제외', math.isinf(o2), f'{o2}')
check('B15 카메라 TF 없음 → 캐시 안 함', 'camera' not in n.sensor_cache)

# ---- 5. 재식별 점수 (자체 설계) ------------------------------------------------------------------------
rgb = np.zeros((704, 704, 3), np.uint8); rgb[300:400, 300:400] = (30, 30, 200)
n.lock_center = (0.5, 0.5); n.lock_hist = n.appearance(rgb, 300, 300, 400, 400)
s_same = n.reid_score({'center': (0.5, 0.5), 'box': (300, 300, 400, 400)}, rgb)
rgb2 = rgb.copy(); rgb2[300:400, 300:400] = (200, 120, 20)
s_diff = n.reid_score({'center': (0.9, 0.5), 'box': (300, 300, 400, 400)}, rgb2)
check('재식별: 같은 자리·같은 색 = 1.0', abs(s_same - 1.0) < 1e-6, f'{s_same:.3f}')
check('재식별: 먼 자리·다른 색 < 합격선 0.6', s_diff < 0.6, f'{s_diff:.3f}')

# ---- 6. watchdog (S8) / stop (S9) / 상태 토픽 (S11) / enable --------------------------------------------
n.pub_cmd.msgs.clear(); n.state = F.TRACKING; n.last_frame = time.time() - 5; n.enable_prev = True; reset(0.2)
n.last_frame = time.time() - 5
n.watchdog()
check('S8 카메라 1 s↑ 끊김 → 정지 명령', n.pub_cmd.msgs and n.pub_cmd.msgs[-1].linear.x == 0.0 and n.cmd_prev.linear.x == 0.0)
n.pub_cmd.msgs.clear(); n.stop(3)
check('S9 stop(): 정지 3 번', len(n.pub_cmd.msgs) == 3)
n.state = F.SEARCH; n.state_since = time.time(); n.lop = None; n.last_bearing = math.radians(30); reset(); n.pub_cmd.msgs.clear()
n.watchdog()
check('SEARCH: LOP 없으면 마지막 방위(왼쪽) 쪽으로 회전', n.pub_cmd.msgs and n.pub_cmd.msgs[-1].angular.z > 0, f'ang={n.pub_cmd.msgs[-1].angular.z if n.pub_cmd.msgs else None}')
n.state_since = time.time() - 10; n.watchdog()
check('SEARCH 6 s 초과 → IDLE, 타깃 정보 삭제', n.state == F.IDLE and n.lock_id is None and n.lock_hist is None)
n.publish_state(n.cmd_prev); js = json.loads(n.pub_state.msgs[-1].data)
check('S11 상태 JSON: 키·무한대→null', set(js) >= {'state', 'd', 'lin', 'block', 'front', 'depth_obs', 'enable'} and js['front'] is None, str(js))
n.set_parameters([rclpy.parameter.Parameter('enable', value=False)]); n.pub_cmd.msgs.clear(); reset(); n.send(0.2, 0.0)
check('enable=false → 계산만 하고 발행 안 함', not n.pub_cmd.msgs)
n.set_parameters([rclpy.parameter.Parameter('enable', value=True)])

# ---- 7. 통합: 실물 데이터셋 사진 + 가짜 depth 로 cb() 한 번 (YOLO·ByteTrack·상태기계·서보) ----------------
def label_has_car(p):                          # YOLO 라벨 첫 숫자 0 = car
    lbl = p.replace('images', 'labels').rsplit('.', 1)[0] + '.txt'
    return os.path.exists(lbl) and any(l.split()[0] == '0' for l in open(lbl) if l.strip())
img = next(p for p in sorted(glob.glob(f'{ROOT}/dataset/test/images/*')) if label_has_car(p))
has_car = True
from cv_bridge import CvBridge
br = CvBridge(); bgr = cv2.imread(img); dep = np.full(bgr.shape[:2], 1500, np.uint16)
rm, dm = br.cv2_to_imgmsg(bgr, 'bgr8'), br.cv2_to_imgmsg(dep, 'mono16')
rm.header.frame_id = dm.header.frame_id = 'oakd_rgb_camera_optical_frame'
n.K = (bgr.shape[1] / 2 / math.tan(math.radians(32)), 0, bgr.shape[1] / 2, bgr.shape[0] / 2)
n.K = (n.K[0], n.K[0], n.K[2], n.K[3])
n.state = F.IDLE; reset(); n.pub_cmd.msgs.clear(); n.dist_hist.clear(); n.depth_miss = 0
n.cb(rm, dm)                                   # 첫 추론은 모델 준비로 느림 (1 s 넘으면 라이다 'stale' 로 전진이 막힘 — 그것도 의도된 동작)
for _ in range(3):
    n.cb_scan(scan({}))                        # 라이다가 살아 있다고 알림 (빈 통로)
    n.cb(rm, dm); time.sleep(0.05)
st = json.loads(n.pub_state.msgs[-1].data)
check(f'통합 cb(): 데이터셋 사진({os.path.basename(img)[:12]}…) → RC카 잡고 1.5 m 이니 전진', st['state'] == F.TRACKING and st['d'] == 1.5 and st['lin'] > 0,
      f"state={st['state']} d={st['d']} lin={st['lin']} ang={st['ang']} block={st['block']!r}")
n.scan_time = 0.0; n.cb(rm, dm); st = json.loads(n.pub_state.msgs[-1].data)
check('통합 cb(): 라이다 끊기면 같은 장면에서도 전진 0', st['lin'] == 0.0 and 'scan-stale' in st['block'], st['block'])
check('통합 cb(): 디버그 영상 발행', len(n.pub_img.msgs) >= 3)

n.destroy_node(); rclpy.try_shutdown()
bad = [r for r in results if not r[1]]
print(f'\n{len(results) - len(bad)}/{len(results)} PASS')
sys.exit(1 if bad else 0)
