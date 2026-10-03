# 실기용 파이썬 코드 구조

실기에서 쓰는 코드는 5 개. 핵심은 `scripts/follow_object_node_v3_safe.py` 하나이고 나머지는 점검·기록용.
실행 순서: `real/preflight.sh` (점검) → `real/run_real.sh` (노드) → `real/record_real.sh` (기록) — 3 절.

```
[로봇 robot1 : RPi + Create3 + OAK-D + RPLIDAR]                 [노트북]
  /robot1/oakd/rgb/image_raw ──┐                          ┌─ follow_object_node_v3_safe.py  ← 핵심 (인식+판단+제어)
  /robot1/oakd/stereo/image_raw├──── Wi-Fi (DDS) ────────▶│
  /robot1/oakd/rgb/camera_info │                          ├─ real_recorder.py   (녹화·텔레메트리)
  /robot1/scan                 │                          └─ check_iface / lidar_check / detect_test (점검)
  /robot1/tf, tf_static       ─┘
  /robot1/cmd_vel_unstamped ◀────────────── 속도 명령 (Twist)
     └▶ create3_repub → /robot1/cmd_vel (TwistStamped) → 바퀴
```

## 1. `follow_object_node_v3_safe.py` — 클래스 하나, 콜백 4 개

### 1-1. 뼈대
| 구성 | 하는 일 |
|---|---|
| `main()` | ROS 초기화 (Ctrl+C 처리를 직접 맡아 종료 시 정지 명령 보장, S9) → `rclpy.spin()` 으로 콜백 처리 |
| `__init__` | ① 파라미터 ~50 개 ② YOLO 가중치 로드 ③ TF 수신 ④ 구독·발행·타이머 등록 (센서는 best-effort QoS, S12) |
| 상태 변수 | 따라가는 대상: `state, lock_id, lock_center, lock_hist, reid_count, lop` · 거리: `dist_hist, depth_miss, d_now` · 안전: `front_clear, rear_clear, depth_obs, scan_time, last_frame` · 직전 명령: `cmd_prev, t_prev` |

### 1-2. 이벤트 4 개
| 콜백 | 언제 | 하는 일 |
|---|---|---|
| `cb_info` | camera_info 첫 수신 1 회 | fx, fy, cx, cy 저장 |
| `cb_scan` | 라이다 ~10 Hz | 스캔 점 → 로봇 좌표 → 전·후방 통로 여유. 전진 중 막히면 즉시 정지 |
| `cb` (메인) | RGB+depth 쌍 ~7–10 Hz | 검출 → 상태기계 → 거리·방위 → 제어 → 디버그 영상·상태 발행 |
| `watchdog` | 타이머 10 Hz | enable 해제 시 정지, 카메라 1 s 끊김 시 정지, SEARCH 회전, IDLE 정지 |

### 1-3. 메인 콜백 `cb()` 순서
1. 영상 변환 (ROS Image → numpy, depth 는 mm 그대로)
2. 검출+추적 `model.track(conf=0.1, persist=True)` — YOLO 박스에 ByteTrack id
3. 후보: `car` 만. `cands` 전체(잠근 트랙 유지용) / `strong` conf ≥ 0.4 (새로 잡기·재식별용)
4. 상태기계: IDLE/SEARCH → 가장 큰 strong 잠금 → TRACKING / 잠근 id 없음 → REID / 점수(위치 0.5 + 색 0.5) ≥ 0.6 → TRACKING, 15 프레임 초과 → SEARCH
5. `depth_obstacle()` 낮은 장애물: depth → 3D → 통로 안 바닥 위 3~60 cm 점까지 거리 (RC카 박스 제외)
6. `depth_at()` 거리: 박스 중앙 하단 패치 중앙값. 3 프레임 연속 무효 → 거리 이력 폐기 ("거리 모름")
7. 방위 `−atan((u − cx)/fx)`
8. `servo()` 회전 = −1.2 × 방위오차, 전진 = 0.6 × (거리 − 1.0) → `send()`
9. 디버그 영상 `/robot1/follow/image`, 상태 JSON `/robot1/follow/state`

### 1-4. 안전 게이트 `send()` — 모든 명령이 지나는 한 곳
```
목표 (lin, ang)
 → 가감속 제한 (0.4 m/s², 1.5 rad/s²)
 → 범위 자르기 (전진 ≤ max_lin, 후진 ≤ 0.1, 회전 ≤ 1.0)
 → 전진이면: 스캔 1 s 없음 / 라이다 통로 < 정지거리 / depth 장애물 < 정지거리 / RC카 < 0.45 m → 즉시 0
 → 후진이면: 스캔 없음 / 후방 통로 < 0.35 m → 즉시 0
 → enable 일 때만 cmd_vel_unstamped 발행
정지거리 = 0.12 + 0.6 × 속도 (범퍼 기준) — 실기에서 재측정
```
REID 관성 주행, SEARCH 회전, 스캔 콜백의 즉시 정지까지 전부 이 함수를 거친다.

### 1-5. 실행 모델 — 리뷰 포인트
- `rclpy.spin()` 은 **단일 스레드**: 콜백이 동시에 돌지 않아 잠금 불필요.
- 대신 메인 콜백(YOLO GPU ~8–11 ms, CPU 22–35 ms + depth 처리)이 도는 동안 스캔 콜백도 기다림 → 노트북 CPU 에서는 라이다 즉시 정지가 그만큼 늦어짐.
- **S12 (적용됨)**: 센서 구독을 best-effort + 영상 큐 2 로. 원본처럼 reliable·큐 10 이면 ① 로봇이 best-effort 로 내면 영상 0 개 수신 ② 노트북이 느리면 최대 ~1 s 묵은 프레임 처리.

## 2. `real/real_recorder.py` — 기록 전용 (로봇을 움직이지 않음)
| 구독 | 용도 |
|---|---|
| `/robot1/follow/image` (없으면 압축 RGB) | 영상 왼쪽 |
| `/robot1/scan` | 전·후방 여유, 라이다 탑뷰 그림 |
| `/robot1/odom` | 실제 속도, 후진 거리 |
| `/robot1/cmd_vel`, `cmd_vel_unstamped` | 명령값 |
| `/robot1/hazard_detection` | 범퍼 |
| `/robot1/battery_state` | 배터리 |
| `/rosout` | 노드 상태 전환 `A -> B`, 경고 |

10 Hz 로 영상 1 프레임 [디버그 화면 | 텔레메트리 패널] + CSV 1 행. 종료 시 `summary.json` (실험일지에 옮길 값 포함).
`real/record_real.sh` 가 녹화기 + `ros2 bag record` + 메타 저장을 묶어 실행·정리.

## 3. 실기 실행 순서
```bash
# 0) 로봇과 같은 ROS 환경을 source (ROS_DOMAIN_ID=1, 디스커버리 서버 …)
bash real/preflight.sh          # 1) 점검: 토픽·QoS·cmd 구독자·라이다 방향·YOLO 1 프레임 → ~/tb4_real_logs/preflight_*.txt
bash real/run_real.sh 12        # 2) 노드 (enable=false, max_lin 0.1)
bash real/record_real.sh 261003-1430 "12n square 첫 시도"   # 3) 기록 (다른 터미널)
ros2 param set /follow_object_node enable true            # 4) 주행 — E-Stop 대기
```
| 스크립트 | 하는 일 |
|---|---|
| `real/preflight.sh` | `check_iface.py`(주기·형식), `ros2 topic info -v`(QoS·cmd 구독자), `lidar_check.py`(정면 방향), `detect_test.py`(YOLO) 결과를 파일로 |
| `real/run_real.sh` | 실기 환경 확인(시뮬 DOMAIN 30 이면 거부) → venv → 노드 실행 (시뮬 시간 안 씀, TF 리맵) |
| `real/record_real.sh` | 시도 ID 폴더에 bag + 영상 + 텔레메트리 + 노드 파라미터 |

## 4. 점검 스크립트 (몇 초 실행 후 종료, 모두 best-effort 구독)
| 파일 | 출력 |
|---|---|
| `scripts/check_iface.py` | 8 s 동안 영상·depth·camera_info·scan 주기, 해상도, 인코딩, frame. `no data` = 네트워크 또는 토픽 이름 문제 |
| `scripts/lidar_check.py` | 오프셋 0° vs −90° 정면 거리, 4 방향 최소거리 |
| `scripts/detect_test.py` | 현재 카메라 1 프레임에 두 모델 → `logs/detect_y11.jpg`, `detect_y12.jpg` |

## 5. 실기에서만 확인 가능한 것
1. `cmd_vel_unstamped` 를 실제로 받는 노드(create3_repub)가 있는가 — preflight 3)
2. 검은 RC카 위 depth 유효 비율 (`d=n/a` 빈도)
3. 정지거리·depth 장애물 임계값·`cam_timeout` 재측정
4. Wi-Fi 대역폭 (원본 RGB+depth ~30 MB/s) — 끊김이 잦으면 압축 토픽을 노트북에서 원본으로 되돌리는 방식으로 전환
5. 렌즈 왜곡 미보정 (화면 가장자리 방위 오차)
