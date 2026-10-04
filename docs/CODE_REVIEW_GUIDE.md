# 코드 리뷰 가이드 — TurtleBot4 RC카 추종 (v3 / v3_safe) + 가즈보 실험 환경

이 문서 하나로 팀원이 기초부터 리뷰할 수 있게 정리했다. 읽는 순서: 1 → 2 → 3 → 코드 → 5 체크리스트.
코드 주석본: `scripts/follow_object_node_v3_annotated.py` (원본과 코드 동일, 주석만), `scripts/follow_object_node_v3_safe_annotated.py` (수정본, 한 줄씩 주석 — **리뷰는 이 파일로**).
실행본: `scripts/follow_object_node_v3_safe.py` (주석본에서 주석만 뺀 자동 생성 파일, 2026-10-04 R1). 수정은 주석본에서 하고 `python3 scripts/make_lean.py` 로 다시 생성,
`python3 scripts/check_annotated_identical.py --ignore-docstrings scripts/follow_object_node_v3_safe.py scripts/follow_object_node_v3_safe_annotated.py` 로 코드 동일 확인.
동작 원리 요약·참고문헌: `docs/FOLLOW_NODE_EXPLAINED.md`.

---

## 1. 기초 개념 (처음 보는 사람용)

### 1-1. ROS 2
| 용어 | 뜻 | 이 프로젝트에서 |
|---|---|---|
| 노드(node) | 하나의 프로그램 단위 | `follow_object_node` 하나가 인식·제어 전부 |
| 토픽(topic) | 노드끼리 메시지를 주고받는 이름 있는 통로 (발행/구독) | `/robot1/oakd/rgb/image_raw` 를 구독, `/robot1/cmd_vel_unstamped` 로 발행 |
| 메시지 타입 | 토픽 데이터의 구조 | `Image`, `LaserScan`, `Twist`(선속도·각속도), `TwistStamped`(Twist + 시각·좌표계) |
| 콜백 | 메시지가 오면 자동 호출되는 함수 | `cb()`(영상 쌍), `cb_scan()`(라이다), `watchdog()`(타이머) |
| 파라미터 | 실행 중에도 바꿀 수 있는 설정값 | `ros2 param set /follow_object_node enable true` |
| 네임스페이스 | 토픽 앞에 붙는 접두어 | 로봇 여러 대 구분: `/robot1/...` |
| TF | 좌표계 사이 변환 트리 (`odom → base_link → oakd_..._optical_frame`) | 라이다·카메라 장착 위치, 마지막 관측 위치 저장 |
| QoS | 전달 방식 설정 (신뢰성, 보관 깊이, 늦게 온 구독자에게 과거값 전달 여부) | `/rosout` 은 transient_local — 늦게 연결돼도 지난 로그 받으려면 같은 설정 필요 |
| DDS / ROS_DOMAIN_ID | 메시지를 실어 나르는 통신 계층 / 같은 번호끼리만 통신 | 실기 1, 시뮬 30 |

### 1-2. 카메라와 거리
- **핀홀 모델**: 3D 점 (X, Y, Z) 가 화면 `u = fx·X/Z + cx`, `v = fy·Y/Z + cy` 에 찍힌다. fx, fy, cx, cy 는 `camera_info` 의 K 행렬.
  - 반대로 픽셀 (u, v) 와 깊이 Z 를 알면 3D 점을 복원 (역투영) → 마지막 관측 위치, 낮은 장애물 감지에 사용.
  - 방위각 `b = −atan((u − cx)/fx)`: 화면 중앙에서 얼마나 옆에 있는지를 각도로.
- **depth 영상**: 픽셀마다 거리. OAK-D 는 `16UC1` (mm, 0 = 측정 실패). 스테레오 방식이라 **검은 무광·무늬 없는 표면에서 실패가 잦다** — 검은 RC카가 바로 그 경우.
- **렌즈 왜곡**: 실기 RGB 는 `rational_polynomial` 왜곡이 강한데 코드에서 보정하지 않는다 → 화면 가장자리 방위 오차.

### 1-3. 검출과 추적
- **YOLO**: 한 장의 영상에서 박스·클래스·신뢰도(conf)를 낸다. 이 프로젝트 모델 클래스는 `car`, `dummy` 2 개 (81 장 학습).
- **추적(ByteTrack)**: 프레임 사이에서 같은 물체에 같은 id 를 붙인다. 핵심은 **점수 낮은 박스(0.1~0.25)도 기존 트랙과 연결** 하는 2 단계 연결 → 가림·흐림으로 점수가 잠깐 떨어져도 id 유지.
- **재식별(Re-ID)**: id 가 끊겼을 때 "새로 보이는 것이 아까 그 물체인가" 를 위치·색으로 판단.

### 1-4. 제어
- **비례(P) 제어**: 명령 = 이득 × 오차. 오차가 0 으로 지수적으로 줄어든다 (`v = −λe`, 시각 서보의 기본 법칙).
  - 회전 = 1.2 × 방위오차, 전진 = 0.6 × (거리 − 1.0 m).
  - **한계**: 타깃이 일정 속도 v 로 움직이면 오차가 0 이 되지 않고 `v/λ` 만큼 남는다 → 0.15 m/s 면 1.25 m 거리로 뒤처져 따라감. 적분(I)항을 넣으면 줄일 수 있음.
- **데드밴드**: 작은 오차는 무시 (떨림 방지). **가감속 제한**: 명령이 한 번에 크게 바뀌지 않게.
- **차동구동**: TB4(Create3)는 바퀴 두 개 — 선속도(앞뒤)와 각속도(회전)만 줄 수 있고 옆으로는 못 감.

### 1-5. 상태기계
IDLE(대기) → TRACKING(추종) → REID(잠깐 놓침, 재식별 시도) → SEARCH(마지막 본 방향으로 회전) → (6 s 후) IDLE.
상세 표: `FOLLOW_NODE_EXPLAINED.md` 2-3.

---

## 2. 저장소 구조

| 경로 | 역할 | 실기에서도 쓰나 |
|---|---|---|
| `scripts/follow_object_node_v3.py` | 원본 노드 (수정 금지) | ✓ |
| `scripts/follow_object_node_v3_annotated.py` | 원본 + 리뷰 주석 (코드 동일, `check_annotated_identical.py` 로 검증) | 읽기용 |
| `scripts/follow_object_node_v3_safe.py` | 안전 수정본 S1~S12 + R1 **실행본** (주석 없음, 자동 생성) | ✓ |
| `scripts/follow_object_node_v3_safe_annotated.py` | 같은 코드의 **주석본** (리뷰·수정은 여기서) | 읽기용 |
| `scripts/make_lean.py` | 주석본 → 실행본 생성 (주석 제거, docstring 첫 줄만) | 둘 다 |
| `scripts/collect_runs.py` | 모든 실험 회차를 `results/all_runs.csv` 한 표로 모음 | 시뮬 |
| `real/preflight.sh`, `real/run_real.sh` | 실기 출발 전 점검, 실기 노드 실행 | ✓ |
| `real/record_real.sh`, `real/real_recorder.py` | 실기 시도 기록 (bag + 영상 + 텔레메트리) | ✓ |
| `scripts/check_iface.py`, `lidar_check.py`, `detect_test.py` | 토픽·라이다 방향·YOLO 1프레임 점검 | ✓ |
| `launch/follow_sim.launch.py`, `run_sim.sh`, `env.sh` | 가즈보 시뮬 (robot1, 스냅샷 카메라 사양) | 시뮬 |
| `worlds/follow_arena.sdf` (`_bright`) | 실습장 월드, 검은 RC카, 장애물, 천장 카메라 | 시뮬 |
| `scripts/tb4_real_iface_adapter.py` | 시뮬 토픽 → 실기 토픽/형식, Twist→TwistStamped | 시뮬 |
| `scripts/sim_rplidar.py` | 기하 레이캐스트 라이다 (WSL 에서 gz 라이다 고장 대체) | 시뮬 |
| `scripts/rc_car_driver.py` | RC카 주행 패턴 | 시뮬 |
| `experiment.sh`, `run_batch2.sh`, `run_all_v2.sh` | 시나리오 자동 실험 | 시뮬 |
| `scripts/experiment_monitor.py`, `recorder.py` | ground truth 지표·안전 이벤트, 녹화 | 시뮬 |
| `scripts/follow_metrics.py`, `summarize2.py`, `make_compare_videos.sh` | 추종 지표, 비교표, 비교 영상 | 시뮬 |
| `scripts/model_bench.py` | 모델 벤치 (다른 모델 추가 비교용) | 둘 다 |

---

## 3. 발견 사항과 정정 목록

### 3-1. 노드 결함 (B) — 원본 v3

| # | 내용 | 근거 | v3_safe |
|---|---|---|---|
| B1 | `cmd_vel` 을 Twist 로 발행, 실기·시뮬은 TwistStamped → 로봇 안 움직임 | 스냅샷 topics.txt | S1 (`cmd_vel_unstamped`) |
| B2 | target 기본값 `rc_car` 가 모델 클래스에 없음 | data.yaml | S1 (`car`) |
| B3 | `lidar_offset_deg=0` 이 실제로는 왼쪽 감시 (RPLIDAR +90° 장착) | 실기 tf_static, 시뮬 lidar_check | S3 (TF 기반 통로) |
| B4 | depth 무효 시 옛 거리로 계속 주행·후진, min_dist 도 무력화 | 시뮬 depthstop/depthloss | S2 |
| B5 | 정지거리 부족: 라이다 중심 기준 0.35 m(범퍼까지 0.14 m) + 정지도 가감속 제한 + 카메라 주기 판정 | 시뮬 pole −90° 에서도 1회 충돌 | S3, S5 |
| B6 | ±20° 섹터가 근거리에서 로봇 폭을 못 덮음 | 시뮬 pillar | S3 |
| B7 | 후진 시 후방 검사 없음 | 시뮬 approach | S6 |
| B8 | Ctrl+C 시 정지 명령 전송 실패 (`RCLError`) | 노드 로그 | S9 |
| B9 | REID 관성 주행이 안전 검사 우회 | 코드 | S7 |
| B10 | 카메라 끊겨도 노드가 정지 안 함 (베이스 타임아웃 의존) | 시뮬 camdrop | S8 |
| B11 | `track(conf=0.4)` 로 ByteTrack 저점수 연결이 꺼짐 (ultralytics 권장과 다름). id 끊김의 원인이라는 가설은 **시뮬에서 확인 안 됨** — S10 적용 후에도 TRACKING→REID 횟수 비슷 (79~117 vs 86~100) | ultralytics 코드 `engine/model.py`, 매트릭스 | S10 (권장 방식에 맞춤, 효과는 실기 bag 으로 재확인) |
| B12 | 라이다 평면(0.193 m)보다 낮은 물체 미검출 (RC카 자체 포함) | 실기 TF 높이, 시뮬 lowbox | S4 (depth 장애물) |
| B13 | 비례 제어라 이동 타깃을 `v/0.6` 만큼 뒤처져 따라감, 0.3 m/s 초과 타깃은 추종 불가 | 식 | 미수정 (설계 선택 — 6장 제안) |
| B14 | 센서 구독이 reliable·큐 10 (기본값): 로봇이 best-effort 로 내면 영상 0 개, 노트북이 느리면 최대 ~1 s 묵은 프레임 | QoS 규칙, 시뮬 `scripts/test_qos.sh` | S12 (best-effort, 영상 큐 2) |
| B15 | (v3_safe 자체) 센서 장착 TF 를 첫 메시지에서 못 읽으면 기본값을 **영구 캐시** → 나중에 TF 가 와도 무시. 기본값이 실기와 같아 지금까지는 영향 없었음 | 코드 검토 (2026-10-04) | R1 (TF 성공 시에만 캐시) |

### 3-1-1. R1 정리 (2026-10-04) — 뼈대·동작 유지, 중복 제거
| 정리 | 내용 |
|---|---|
| `_tf()` | TF 조회 `try/except` 3 곳 → 공통 함수 (실패 시 None) |
| `sensor_T(key, frame, fallback)` | 라이다·카메라 장착값 읽기 통일 + B15 수정 |
| `backproject()`, `corridor()`, `stop_dist(v)` | 역투영·통로 폭·정지거리 식 중복 제거 |
| 재식별 | 후보마다 점수를 한 번만 계산 (전에는 최고점 후보를 한 번 더 계산) |
| `watchdog` / `stop(n)` / `publish_state` | 분기·정지·JSON 변환 단순화 |
| 표시 | 거리가 0.0 일 때 `d=n/a` 로 찍히던 것 수정 |
| 파일 | 주석본(585 줄) / 실행본(468 줄, 원래 579 줄) 분리. 순수 코드 404 → 397 줄 |

### 3-2. v3_safe 자체의 한계·남은 위험
- S4 depth 장애물: 시뮬 depth 는 잡음이 없다. **실기에서 바닥 반사·잡음으로 헛정지 가능** → `obs_min_h`, `obs_min_pts` 실측 튜닝.
- S3 정지거리 `0.12 + 0.6·v` 는 시뮬 기준. **실기 Create3 감속·Wi-Fi 지연으로 다시 측정**해야 함.
- 카메라 근거리 한계(~0.3 m) + 카메라가 중심보다 0.06 m 뒤 → 범퍼 앞 약 7 cm 이내 물체는 depth 로 못 봄.
- S8 `cam_timeout` 0.5 s 는 시뮬 부하만으로 실행당 7~29 회 정지 → 1.0 s 로 올림. Wi-Fi 에서는 실측 필요.
- RC카가 로봇 쪽으로 달려오면 (approach) 후방이 막혀 있을 때 로봇은 피할 방법이 없음 → 접촉 발생 (조종자 책임 영역).

### 3-3. 환경 오류 (E) — 이 세션에서 겪고 해결한 것

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| E1 | 카메라 ~1 Hz | WSL 기본 OpenGL 이 CPU 렌더링(llvmpipe) | `GALLIUM_DRIVER=d3d12` (env.sh) |
| E2 | 라이다 640 개가 전부 0.164 m | WSL d3d12 + ogre1 gpu_lidar 고장. ogre2 는 카메라 ~2 Hz | 기하 레이캐스트 라이다 `sim_rplidar.py` |
| E3 | 노드 일부가 서로 못 찾음 (카메라 0 Hz) | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` + 프로세스 ~60 개 | 해제, DOMAIN_ID 로 격리 |
| E4 | 런치한 스크립트가 안 뜸 | Windows 쪽 편집으로 실행권한(+x) 상실 | 런치에서 `python3 script` 로 실행, 셸은 `bash x.sh` |
| E5 | 종료 후 재시작하면 이상 동작 | 백그라운드 프로세스가 SIGINT 무시 → 시뮬 2 개 동시 실행 | `stop_all.sh` (자식 프로세스까지 종료), 기록 스크립트 `set -m` |
| E6 | numpy 2.x 로 바뀌어 cv_bridge 깨짐 | ultralytics 가 ONNX 내보내기 때 패키지 자동 설치 | `numpy<2` 재설치, `YOLO_AUTOINSTALL=False` (env.sh) |
| E7 | gz `light_config` 호출 후 장면이 검게 | 실행 중 조명 변경이 태양광을 꺼버림 | 조명은 월드 파일로 (`follow_arena_bright.sdf`) |
| E8 | `ros2 topic list` 가 일부만 보임 | CLI 디스커버리 시간 부족 | `check_iface.py` 로 직접 구독해 확인 |

### 3-4. 측정·보고 정정 (M) — 앞서 보고한 수치 중 틀렸던 것

| # | 틀린 내용 | 원인 | 정정 |
|---|---|---|---|
| M1 | 1차 보고의 '카메라↔RC카 거리'(오버슈트 0.59 m 등) 가 약 0.18 m 작게 계산됨 | 카메라가 중심 **앞 0.12 m** 라고 가정, 실제는 **뒤 0.06 m** (실기 tf_static) | 모니터 `CAM_FWD = −0.06`. 범퍼 기준 여유(gap/clear)와 충돌 판정은 영향 없음 |
| M2 | 1차 배치의 상태별 시간 (IDLE 과대) | 모니터가 노드보다 늦게 떠서 첫 전환 놓침 | 노드 로그로 재계산, 이후 모니터 먼저 실행 + `/rosout` transient_local 구독 |
| M3 | 2차 매트릭스에서 v3_safe 추종률이 낮게 나옴 (47/57 %) | 모니터가 `camera timeout -> stop` 을 상태 전환으로 파싱 | 상태명끼리만 전환 인정 → 재집계 67/80 % (v3 61/74 %). 노드 로그 문구도 `camera timeout: stop` 으로 변경 |
| M4 | yolo12n 실행에서만 '카메라 없이 주행' 경고 다수 | CPU 부하로 모니터 수신 지연 (어댑터는 10 Hz 유지) | 안전 결론에서 제외 |
| M5 | 감지율 순위: 1차 시험 12n 85 % > 11n 75 %, 2차 벤치 11n 50 % > 12n 19 % | 사이에 월드 조명·바닥색을 바꿈 (조건 다름) | 조명 2 종 월드로 같은 조건 재측정 (YOLO 비교 문서) |
| M6 | 벤치 지연 CPU 63/92 ms | 시뮬이 같이 돌던 중 측정 | 시뮬 끄고 재측정한 값 사용 |
| M7 | 1차 보고 "pole offset −90 도 정지 실패" 에 시나리오 설계 문제 | 첫 pole 배치는 폴이 RC카를 가려 추종 자체가 안 됨 | 시선 밖 배치로 재설계 후 재실험 |
| M8 | "S10 으로 id 끊김이 줄 것" (중간 보고) | 가설이었음 | 매트릭스에서 효과 없음 확인 → B11 행 정정 |
| M9 | 시나리오별 추종률 차이를 그대로 비교 | 같은 v3 코드가 회차에 따라 평균 61→74 %, 74→81 % | **±10 %p 이내 차이는 우연일 수 있음**. 결론은 두 회차에서 같은 방향인 것만 채택 |

### 3-5. 문헌 인용 정정 — `FOLLOW_NODE_EXPLAINED.md` 5절
[CH06] 2.5D 원전은 Malis 1999 / [BT22] conf 0.4 로 핵심 기능 꺼짐 / [YE23] 인용 대상 불일치 / [AC20] LOP '이동' 이 아니라 '회전' 만, PI 가 아니라 P.

---

## 4. 실행 방법 (리뷰어가 직접 돌려보기)
```bash
bash ~/tb4_follow_sim/setup.sh                 # 최초 1회
bash ~/tb4_follow_sim/run_sim.sh               # 터미널 1 (GUI). gui:=false 헤드리스
NODE=v3safe bash ~/tb4_follow_sim/run_follow.sh 12   # 터미널 2 (NODE 생략 = 원본 v3)
source ~/tb4_follow_sim/env.sh; ros2 param set /follow_object_node enable true
python3 ~/tb4_follow_sim/scripts/rc_car_driver.py --ros-args -p pattern:=square
NODE=v3safe bash ~/tb4_follow_sim/experiment.sh pillar 12   # 시나리오 자동 실험 (+ 녹화)
python3 ~/tb4_follow_sim/scripts/check_annotated_identical.py   # 주석본 = 원본 확인
```

## 5. 리뷰 체크리스트
- [ ] 토픽 이름·메시지 타입이 실기(`topics.txt`)와 맞는가 (특히 cmd_vel 타입)
- [ ] 좌표계: 라이다·카메라 장착각을 TF 로 읽는가, 실패 시 기본값이 실기 tf_static 과 같은가
- [ ] 단위: depth mm↔m, 각도 deg↔rad, 부호(ROS 왼쪽 회전 +)
- [ ] 센서가 끊기거나(카메라·라이다·depth) 값이 무효일 때 로봇이 **멈추는가**
- [ ] 모든 속도 명령이 하나의 안전 게이트를 지나는가 (REID, SEARCH 포함)
- [ ] 정지는 즉시인가 (가감속 제한 우회)
- [ ] 후진 시 후방을 보는가
- [ ] 종료(Ctrl+C·kill) 시 정지 명령이 실제로 나가는가
- [ ] 파라미터 기본값이 안전한가 (enable=false, 저속)
- [ ] 로그 문구 형식이 기록 도구 파싱(`A -> B`)과 충돌하지 않는가
- [ ] 시뮬에서만 맞는 값(정지거리, 장애물 임계값, cam_timeout)을 실기에서 재측정할 계획이 있는가

## 6. 남은 개선 제안 (미적용)
1. 전진에 적분항(PI) 또는 RC카 속도 앞먹임(feed-forward) → 이동 타깃 뒤처짐(B13) 감소
2. 렌즈 왜곡 보정 후 방위 계산 (실기 `rgb/image_rect` 또는 `cv2.undistortPoints`)
3. 학습 데이터: 다른 장소·조명·RC카 옆모습 추가 (옆모습이 dummy 로 오분류됨), valid/test 를 train 과 다른 원본으로 분리
4. ByteTrack 파라미터(`track_buffer`, `match_thresh`)를 실기 bag 으로 튜닝
