# follow_object_node_v3 — 터틀봇이 RC카를 따라가는 원리

대상 파일: `scripts/follow_object_node_v3.py` (원본, 252줄). 줄 번호는 이 파일 기준.
안전 수정본 `follow_object_node_v3_safe.py` 의 차이는 6절.

## 0. 한눈에 보기

```
OAK-D RGB ─┐ ApproximateTime   ┌────────── 매 프레임 cb() ──────────────────────────────┐
OAK-D Depth┘ (5, 0.1 s) ──────▶│ YOLO.track(ByteTrack) → 'car' 후보 → 상태기계(IDLE/     │
                               │ TRACKING/REID/SEARCH) → 거리 d(depth) · 방위 b(픽셀)   │──▶ cmd_vel (Twist)
camera_info ── fx,cx ─────────▶│ → servo(): ang = λ_ang·b, lin = λ_lin·(d − 1.0)        │    가감속·속도 제한
/scan ── 정면 ±20° 최소거리 ───▶│   (lidar 가 가까우면 lin ≤ 0)                          │
TF(odom) ── 마지막 관측 위치 ───▶└──────────────────────────────────────────────────────┘
watchdog 10 Hz: enable 해제 시 정지, SEARCH 에서 마지막 관측 방향으로 제자리 회전, IDLE 정지
```

## 1. 입력·출력 (64–71행)

| 구분 | 토픽 | 용도 |
|---|---|---|
| 입력 | `{ns}/oakd/rgb/image_raw` + `{ns}/oakd/stereo/image_raw` | 두 영상을 `ApproximateTimeSynchronizer(queue 5, slop 0.1 s)` 로 묶어 `cb()` 호출 |
| 입력 | `{ns}/oakd/rgb/camera_info` | 첫 메시지에서 fx, fy, cx, cy 저장 (방위·3D 변환) |
| 입력 | `{ns}/scan` | 정면 섹터 최소거리 (`cb_scan`) |
| 입력 | TF `odom ↔ camera`, `base_link ↔ odom` | 마지막 관측 위치(LOP) 저장·방향 계산 |
| 출력 | `{ns}/cmd_vel` (`geometry_msgs/Twist`) | 속도 명령. **실기·시뮬의 `cmd_vel` 은 TwistStamped** → `cmd_vel_unstamped` 로 리맵 필요 |
| 출력 | `{ns}/follow/image` | 박스·상태·명령이 그려진 디버그 영상 |
| 파라미터 | `enable` (기본 false) | true 일 때만 명령 퍼블리시. 계산은 항상 수행 |

## 2. 매 프레임 처리 순서 — `cb()` (162–220행)

### 2-1. 검출 + 추적 (168행)
`self.model.track(rgb, conf=0.4, persist=True, tracker='bytetrack.yaml')`
- YOLO 가 박스를 내고, Ultralytics 내장 ByteTrack 이 프레임 사이에 같은 물체에 같은 `id` 를 붙인다 (`persist=True` 로 상태 유지).
- **확인된 문제**: 설치된 ultralytics 8.4.171 은 `track()` 의 기본 conf 를 0.1 로 두고 주석에 "trackers need low-confidence input" 이라고 적고 있다 (`engine/model.py`). ByteTrack 의 핵심은 점수 0.1~0.25 의 낮은 박스로 2차 연결을 하는 것인데 (`track_low_thresh 0.1`, `track_high_thresh 0.25`), v3 는 `conf=0.4` 를 넘겨 0.4 미만 박스를 추적기 전에 버린다. 그래서 2차 연결이 사실상 꺼진다. 처음에는 이것이 잦은 TRACKING→REID 전환의 원인이라고 추정했으나, v3_safe(S10: 추적 0.1, 타깃 선택 0.4)로 매트릭스를 돌려 보니 전환 횟수가 줄지 않았다 → **원인이 아니었거나 시뮬에서는 드러나지 않음**. ultralytics 권장 방식에 맞춘 수정으로 유지하고, 실기 bag 으로 다시 확인할 것.
- 그 밖의 ByteTrack 설정 (ultralytics 기본): `new_track_thresh 0.25`, `track_buffer 30` 프레임 (약 7 Hz 에서 ~4 s 동안 잃은 트랙 보관), `match_thresh 0.8`, `fuse_score True`.

### 2-2. 후보 만들기 (170–176행)
id 가 붙은 박스 중 클래스 이름이 `target`(기본 `rc_car` → 이 모델은 `car` 로 지정해야 함) 인 것만 후보로. 각 후보: id, 박스, 넓이, 화면 정규화 중심.

### 2-3. 상태기계 (178–192행)

| 상태 | 들어오는 조건 | 하는 일 | 나가는 조건 |
|---|---|---|---|
| IDLE | 시작 / SEARCH 시간 초과 | watchdog 이 정지 명령 | 후보가 생기면 **가장 큰 박스**를 잠금 → TRACKING (외형 히스토그램 저장, 거리 이력 초기화) |
| TRACKING | 잠금 성공 / REID 성공 | 잠근 id 의 박스로 거리·방위 계산 → 서보 제어, 외형 모델 완만 갱신 (`0.9·old + 0.1·new`) | 잠근 id 가 이번 프레임에 없으면 → REID |
| REID | TRACKING 에서 id 사라짐 | 매 프레임 후보마다 점수 계산, 선속도는 직전의 0.5 배로 감쇠·회전은 유지 | 점수 ≥ 0.6 인 후보 → TRACKING (새 id 로 잠금). 15 프레임(≈2 s) 넘으면 → SEARCH |
| SEARCH | REID 실패 | watchdog 이 마지막 관측 위치(LOP) 방향으로 0.5 rad/s 제자리 회전, 그 방향 ±8° 안이면 정지 대기 | 후보가 생기면 가장 큰 박스 → TRACKING. 6 s 넘으면 잠금·LOP 지우고 → IDLE |

REID 점수 (111–117행): `0.5 × 위치점수 + 0.5 × 외형점수`
- 위치점수 = `1 − min(1, 마지막 중심과의 화면거리 / 0.3)` (화면 폭·높이 정규화 좌표)
- 외형점수 = HSV 의 H·S 2D 히스토그램(16×8 bin, 박스 위쪽 2/3) 상관계수 (`cv2.HISTCMP_CORREL`, 음수는 0)
- ByteTrack 이 같은 RC카에 새 id 를 붙여도 위치·외형이 비슷하면 바로 다시 잡는 구조. 시뮬 로그의 `reid ok id=.. score=0.9x` 반복이 이것.

### 2-4. 거리 d — `depth_at()` (94–100행) + 이력 (201–202행)
- 박스 가로 중앙, 세로 **위에서 2/3 지점**을 중심으로 최대 9×9 패치 (박스가 작으면 더 작게).
- 0 이 아닌 유효 픽셀이 30 % 이상(최소 3개)일 때만 중앙값, `uint16` 이면 mm → m.
- 최근 5 개 값의 중앙값을 제어에 사용 (잡음 제거).
- **결함 (B4)**: depth 가 무효면 이력에 아무것도 추가하지 않지만 지우지도 않아, 옛 중앙값이 계속 쓰인다. 시뮬 depthstop/depthloss 에서 끝없는 전진·후진으로 재현.

### 2-5. 방위 b (203–204행)
핀홀 모델: `b = −atan((u − cx) / fx)` — 표시점 u 가 화면 중앙보다 왼쪽이면 + (ROS 좌회전 +). 실기 RGB 의 강한 rational_polynomial 왜곡은 보정하지 않으므로 화면 가장자리에서 오차.

### 2-6. 제어 — `servo()` (148–159행), `send()` (139–146행)
Chaumette & Hutchinson 의 지수 수렴 제어 `v = −λe` 를 방위·거리 두 축에 따로 적용한 비례(P) 제어.
- 회전: 방위 오차에 데드밴드 4° 를 뺀 값 × `λ_ang 1.2` → 타깃 쪽으로 회전.
- 전진: `lin = λ_lin 0.6 × (d − target_dist 1.0)`, |오차| < 0.1 m 면 0. d < `min_dist 0.45` 면 전진 금지. 라이다가 막히면 전진 금지 (후진은 허용).
- `send()`: 가감속 제한 `acc_lin 0.4 m/s²`, `acc_ang 1.5 rad/s²` → 최대 `max_lin 0.3`, `max_ang 1.0` 으로 자름 → `enable` 이면 퍼블리시.

P 제어라서 생기는 특성 (식으로 바로 나오는 결과)
- RC카가 일정 속도 v_c 로 멀어지면 정상상태에서 `λ_lin (d − 1.0) = v_c` → **d = 1.0 + v_c / 0.6**. 0.15 m/s 면 약 1.25 m, 0.2 m/s 면 약 1.33 m 로 **목표보다 뒤처져 따라감**. Algabri & Choi 는 같은 문제에 적분항이 있는 PI 제어를 썼다.
- `max_lin 0.3 m/s` 보다 빠른 RC카는 원리적으로 따라잡을 수 없다 (거리가 계속 벌어져 결국 시야 밖).
- 가감속 제한 때문에 0.3 m/s 에서 정지까지 ≥ 0.75 s, 약 0.11 m 더 진행.

### 2-7. 마지막 관측 위치 (LOP) — `update_lop()` / `bearing_to_lop()` (120–136행)
TRACKING 중 매 프레임 표시점 (u, v, d) 를 카메라 3D 점으로 바꿔 `odom` 좌표로 저장. SEARCH 에서 `base_link` 기준 방향을 다시 계산해 그쪽으로 회전 (로봇이 그동안 돌았어도 맞는 방향). TF 실패 시 마지막 방위각 사용.

### 2-8. 라이다 정면 감시 — `cb_scan()` / `lidar_blocks()` (84–92행)
`lidar_offset_deg` 를 중심으로 ±20° 섹터의 최소거리. 1 s 이상 스캔이 없거나 0.35 m 미만이면 전진 금지.
**결함 (B3)**: RPLIDAR 가 +90° 돌아 장착돼 있어 기본값 0° 는 정면이 아니라 **좌측**을 본다 (실기 tf_static 로 확인, 시뮬 폴 시나리오에서 반복 충돌 재현).

### 2-9. watchdog (223–236행), 종료 (238–251행)
10 Hz 타이머: enable 이 true→false 로 바뀌면 정지 3회 퍼블리시. SEARCH·IDLE 의 명령 담당.
**결함 (B8)**: Ctrl+C 시 rclpy 컨텍스트가 먼저 내려가 `stop()` 퍼블리시가 실패 (`RCLError`). 로봇은 베이스의 cmd_vel 타임아웃으로만 멈춘다.

## 3. 파라미터 요약 (34–49행)

| 묶음 | 파라미터 (기본값) |
|---|---|
| 검출 | model, target `rc_car`, conf 0.4 |
| 제어 | target_dist 1.0, lambda_ang 1.2, lambda_lin 0.6, bearing_dead_deg 4, dist_dead 0.1, max_lin 0.3, max_ang 1.0, acc_lin 0.4, acc_ang 1.5 |
| 재식별 | reid_frames 15, reid_pos_w 0.5, reid_app_w 0.5, reid_accept 0.6, reid_pos_radius 0.3 |
| 탐색 | search_max_s 6.0, search_ang 0.5, odom_frame `odom` |
| 안전 | min_dist 0.45, lidar_stop 0.35, lidar_offset_deg 0, lidar_stale 1.0, patch 9 |

## 4. 타이밍
카메라 10 Hz → 동기화 → YOLO 추론(RTX 3060 기준 yolo11n ~7 ms, yolo12n ~11 ms, 노트북 CPU 22~28 ms) → 시뮬 실측 제어 주기 6~8 Hz. 명령은 영상이 올 때만 갱신 (TRACKING/REID), SEARCH/IDLE 은 10 Hz watchdog.

## 5. 문헌과 코드의 대응 — 사실 확인 결과

| 코드 주석 | 실제 문헌 | 코드가 가져온 것 | 차이·주의 |
|---|---|---|---|
| [CH06] Chaumette & Hutchinson 2006/2007 | IEEE RAM 13(4):82–90 (2006), 14(1):109–118 (2007) | 오차 e 에 대해 `v = −λe` 로 지수 수렴시키는 제어법칙, IBVS(영상 특징)·PBVS(3D 특징) 구분 | "2.5D" 라는 이름의 원전은 Malis·Chaumette·Boudet (1999). v3 는 상호작용 행렬 없이 방위·거리를 분리한 단순 P 제어 (비홀로노믹 차동구동에 맞춘 근사) |
| [BT22] ByteTrack, ECCV 2022 | Zhang et al., ECCV 2022 | Ultralytics 의 ByteTrack 구현을 그대로 사용 | 위 2-1: conf 0.4 를 넘겨 저점수 박스 2차 연결(논문의 핵심)이 꺼져 있음 |
| [YE23] "Ye et al., Robot Person Following (ICRA 2023 계열)" | ICRA 2023 Ye et al. 은 *부분 가림 시 보이는 관절로 사람 위치를 추정* 하는 논문 | 검출→추적→재식별→제어 파이프라인이라는 일반 구조 | 인용이 모호함. 재식별 자체는 Ye et al. RA-L 2024 (온라인 지속학습 ReID) 가 더 가깝다. v3 의 "위치 0.5 + 색 0.5" 점수식은 두 논문 어디에도 없는 v3 자체 설계 |
| [AC20] Algabri & Choi, Sensors 2020 | Sensors 20(9):2699 | H·S 히스토그램으로 타깃 재식별, Tracking → LOP → Searching 상태기계, "Last Observed Position" 용어 | 논문은 SLAM 지도로 LOP 까지 **이동**한 뒤 탐색, PI 제어. v3 는 LOP 방향으로 **제자리 회전만** 하고 P 제어. 히스토그램 bin 수·비교 척도는 논문에 명시 없음 → v3 의 16×8, 상관계수는 v3 선택 |

## 6. v3_safe 에서 바뀐 것 (추종 로직은 동일)

| 번호 | 내용 |
|---|---|
| S1 | 기본값을 실기에 맞춤: Twist 를 `cmd_vel_unstamped` 로, target `car`, 라이다 장착각은 TF 로 |
| S2 | depth 무효 3 프레임 이상이면 거리 이력 폐기 → 전·후진 0, 0.6 s 넘은 거리값은 버림 |
| S3 | 라이다 점을 base 좌표로 바꿔 **로봇 폭+5 cm 통로** 로 판정, 정지거리 = 0.12 + 0.6·v (범퍼 기준), 스캔 콜백에서도 즉시 정지 |
| S4 | depth 영상으로 통로 안 바닥 위 3~60 cm 물체 감지 (라이다 평면 아래 낮은 물체), 타깃 박스는 제외 |
| S5 | 안전 정지는 가감속 제한 없이 즉시 0 |
| S6 | 후진은 후방 통로가 비었을 때만, 최대 0.1 m/s |
| S7 | REID 포함 모든 명령이 같은 안전 검사 통과 |
| S8 | 카메라 1.0 s 이상 끊기면 정지 (처음 0.5 s 는 시뮬 부하만으로 실행당 7~29 회 걸려 1.0 s 로 변경) |
| S9 | Ctrl+C·SIGTERM 에서 rclpy 종료 전에 정지 퍼블리시 |
| S10 | ByteTrack 에는 conf ≥ 0.1 박스까지 넘기고, 새 타깃 선택·재식별에만 conf 0.4 적용 (2-1 의 문제 수정). 이미 잠근 트랙은 점수가 낮아도 추적기가 이어주면 유지 |
| S11 | `{ns}/follow/state` 에 상태·거리·명령·정지사유를 JSON 으로 발행 (기록·디버깅용) |

## 7. 참고문헌

추종 제어
1. F. Chaumette, S. Hutchinson, "Visual servo control, Part I: Basic approaches," *IEEE Robotics & Automation Magazine*, 13(4):82–90, Dec. 2006. https://www.irisa.fr/lagadic/publi/publi/Chaumette06a-eng.html
2. F. Chaumette, S. Hutchinson, "Visual servo control, Part II: Advanced approaches," *IEEE Robotics & Automation Magazine*, 14(1):109–118, Mar. 2007. https://www.irisa.fr/lagadic/publi/publi/Chaumette07a-eng.html
3. E. Malis, F. Chaumette, S. Boudet, "2 1/2 D visual servoing," *IEEE Trans. Robotics and Automation*, 15(2):238–250, Apr. 1999. (2.5D 서보잉 원전)

추적
4. Y. Zhang, P. Sun, Y. Jiang, D. Yu, F. Weng, Z. Yuan, P. Luo, W. Liu, X. Wang, "ByteTrack: Multi-Object Tracking by Associating Every Detection Box," *ECCV 2022*. arXiv:2110.06864. https://github.com/ifzhang/ByteTrack
5. Ultralytics, "Multi-Object Tracking" 문서 및 `cfg/trackers/bytetrack.yaml` (8.4.171). https://docs.ultralytics.com/modes/track/

사람(물체) 추종 로봇
6. R. Algabri, M.-T. Choi, "Deep-Learning-Based Indoor Human Following of Mobile Robot Using Color Feature," *Sensors*, 20(9):2699, 2020. doi:10.3390/s20092699. https://pmc.ncbi.nlm.nih.gov/articles/PMC7273221/
7. H. Ye, J. Zhao, Y. Pan, W. Chen, L. He, H. Zhang, "Robot Person Following Under Partial Occlusion," *ICRA 2023*. arXiv:2302.02121
8. H. Ye, J. Zhao, Y. Zhan, W. Chen, L. He, H. Zhang, "Person Re-Identification for Robot Person Following with Online Continual Learning," *IEEE RA-L*, 2024. doi:10.1109/LRA.2024.3438042. arXiv:2309.11727
9. M. J. Islam, J. Hong, J. Sattar, "Person-following by autonomous robots: A categorical overview," *IJRR*, 38(14), 2019. doi:10.1177/0278364919881683. arXiv:1803.08202 (분야 개관)

검출기
10. Ultralytics YOLO11 문서 (2024-09). https://docs.ultralytics.com/models/yolo11/
11. Y. Tian, Q. Ye, D. Doermann, "YOLOv12: Attention-Centric Real-Time Object Detectors," *NeurIPS 2025*. arXiv:2502.12524. https://github.com/sunsmarterjie/yolov12
