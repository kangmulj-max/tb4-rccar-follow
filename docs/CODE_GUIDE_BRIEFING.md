# 코드 설명서 — 팀 브리핑용 (2026-10-04)

> 이 문서만 읽고 "오늘 어떤 코드가 어디서 무엇을 하는지" 설명할 수 있게 쓴 안내서다. 코드를 처음 보는 사람 기준.
> 시험 순서·이유는 `REAL_TEST_PLAN_RATIONALE`(계획과 근거), 명령어는 `REAL_TEST_SEQUENCE`(시험 순서) 를 같이 본다.

## 0. 전체 그림 — 컴퓨터 세 대, 할 일 세 가지
```
[로봇 robot1]  카메라(OAK-D) · 라이다 · 바퀴              ← 오늘 아무것도 설치·수정하지 않음
      │ Wi-Fi (ROS 2)
      ▼
[AMR PC 노트북]  ① 추종: 카메라 영상 → YOLO → 거리·방향 → 속도 명령 → 로봇 바퀴
                 ② 기록: 시도마다 bag + 영상 + 요약
[웹캠 PC]        ③ 위에서 녹화: 시험장 영상 + 프레임 시각 (평가용 정답 자료)
                 ↓ 시험 뒤
[아무 PC]        ④ 분석: 웹캠 영상 → RC카·로봇 실제 위치, 간격, 시계 맞추기
```
- **추종은 ① 하나뿐**이다. ③④ 는 "잘 따라갔는지"를 바깥에서 재는 도구라, 꺼져 있어도 로봇은 똑같이 움직인다.

## 1. 먼저 알아 둘 말 10 개
| 말 | 뜻 | 오늘 예 |
|---|---|---|
| 노드 | ROS 2 에서 돌아가는 프로그램 하나 | `follow_object_node` (추종) |
| 토픽 | 노드끼리 데이터를 주고받는 이름 붙은 통로 | `/robot1/scan` (라이다), `/robot1/cmd_vel_unstamped` (속도 명령) |
| 파라미터 | 노드를 끄지 않고 바꿀 수 있는 설정값 | `enable` (주행 켜기), `max_lin` (최대 속도) |
| bag | 토픽을 시간 순서대로 녹화한 파일 — 나중에 다시 재생 가능 | `~/tb4_real_logs/<시도ID>/bag` |
| QoS (best-effort) | 전송 방식. best-effort = 놓친 건 다시 안 보냄 (영상처럼 최신 것만 중요할 때) | 센서 영상 구독 |
| 압축 영상 | 원본 대신 JPEG 로 보내는 영상 — Wi-Fi 부하가 1/10 이하 | `.../image_raw/compressed` |
| YOLO | 사진에서 물체 상자(박스)와 이름·점수를 찾는 인공지능 모델 | `car` 박스 |
| ByteTrack | 프레임마다 나온 박스에 "같은 물체면 같은 번호(id)"를 붙이는 추적기 | id 3 을 계속 따라감 |
| depth | 카메라에서 각 픽셀까지 거리 (스테레오 카메라가 잼) | RC카까지 0.95 m |
| fx (초점거리, 픽셀 단위) | 화면 위치를 "몇 도 옆인가"로 바꾸는 값 | 로봇이 보내는 564.65 는 틀린 값, 실측 909 로 덮어씀 |

## 2. AMR PC 코드 (`tb4_follow_sim/`)

### 2-1. `real/preflight.sh` — 출발 전 점검 (로봇은 안 움직임)
| 단계 | 확인 | 정상 | 문제면 |
|---|---|---|---|
| 0 | 도크 상태, 카메라 해상도·fx, 압축 영상 주기 | 도크에서 내려옴, 704×704, fx 564.65, 약 10 Hz | 도크 위면 언도킹 명령을 알려 줌. 704 가 아니면 로봇 카메라 설정이 바뀐 것 |
| 1 | 토픽 주기·형식 (`scripts/check_iface.py`) | no data 없음 | 네트워크·QoS |
| 2 | 발행자의 전송 방식 | — | (참고) |
| 3 | 속도 명령을 받는 쪽이 있는지 | create3 쪽 노드 보임 | 명령이 바퀴까지 안 감 |
| 4 | 라이다 정면 (`scripts/lidar_check.py`) | 정면 0.5 m 상자 → 0.5 | 장착 방향 확인 |
| 5 | YOLO 1 장 (`scripts/detect_test.py`) | `car` 박스 그림 | 가중치·라이브러리 |
결과는 `~/tb4_real_logs/preflight_<시각>.txt` 에 저장.

### 2-2. `real/run_real.sh` — 추종 노드 실행
```bash
bash real/run_real.sh 12                                      # yolo12n
bash real/run_real.sh 11                                      # yolo11n
MODEL=$PWD/models/AMRCam_yolov8n_baseline.pt bash real/run_real.sh v8n   # 다른 가중치
```
- **오늘 쓰는 두 가지 실행** (`docs/PLAN1_PLAN2.md`):
  - `bash real/run_plan1.sh 12` — **1안**: 10/3 노드 그대로 + `target_dist 0.85` (1부 주행 시험 기본)
  - `bash real/run_plan2.sh 12` — **2안**: 움직임 개선 노드 `follow_object_node_v3_motion.py` (지연 보상·가장자리에서 회전 먼저). `FF=0.5` 를 앞에 붙이면 차 속도 피드포워드까지
- 이 스크립트가 정해 주는 것: 가중치 경로, 로봇 이름 `/robot1`, **최대 속도 0.1 m/s**(처음 시험용), **fx 909**, TF 토픽 이름.
- **항상 `enable false` 로 시작** → 계산·디버그 영상만 나오고 바퀴는 안 움직임. 주행: `ros2 param set /follow_object_node enable true`, 멈춤: `... enable false`.
- 실행 중 바꾸기 예: `ros2 param set /follow_object_node max_lin 0.2`

### 2-3. `scripts/follow_object_node_v3_safe.py` — 핵심 (676 줄)
**한 문장**: 카메라 영상이 올 때마다 RC카를 찾아, 정해진 거리(1.0 m)와 정면 방향을 유지하도록 속도를 내고, 모든 명령은 안전 검사를 통과해야 나간다.

**매 프레임(초당 약 7~10 번) 하는 일**
1. 영상 받기 — 압축 RGB 와 depth 를 시각이 맞는 쌍으로 묶음
2. **YOLO + ByteTrack** — `car` 박스에 번호(id)
3. **누구를 따라갈지** (상태기계, 아래 표)
4. 낮은 장애물 찾기 — depth 로 로봇 앞 통로 안 바닥 위 3~60 cm 물체 (라이다가 못 보는 낮은 상자)
5. **거리 d** — 박스 안 depth 패치의 중앙값, 최근 값 여러 개의 중앙값 (튀는 값 제거)
6. **방향 b** — 박스 위치가 화면 가운데에서 몇 도 옆인지 (`−atan((u − cx)/fx)`)
7. **속도 계산** — 회전 = 1.2 × 방향 오차, 전진 = 0.6 × (d − 1.0)
8. **안전 검사 `send()`** — 아래
9. 디버그 영상 `/robot1/follow/image`, 상태 `/robot1/follow/state` 발행

**상태기계 — 지금 무엇을 하는 중인가**
| 상태 | 뜻 | 하는 일 | 다음 |
|---|---|---|---|
| IDLE | 대기 | 멈춤 | `car` 가 보이면 가장 큰 것을 잠금 → TRACKING |
| TRACKING | 따라가는 중 | 잠근 id 로 거리·방향 계산 → 속도 | 그 id 가 사라지면 → REID |
| REID | 방금 놓침, 다시 찾는 중 | 속도를 줄이며 후보마다 "위치가 가까운가 + 색이 비슷한가" 점수 | 점수 ≥ 0.6 → TRACKING / 약 2 초 넘으면 → SEARCH |
| SEARCH | 완전히 놓침 | 마지막으로 본 방향(LOP)으로 돌고, 없으면 한 바퀴 훑기 | 찾으면 TRACKING / 10 초 넘으면 IDLE |

**속도 식을 쉽게**: "목표와 얼마나 어긋났나 × 비율". RC카가 1.5 m 에 있으면 전진 = 0.6 × (1.5 − 1.0) = 0.3 m/s → 단, 오늘은 최대 0.1 로 잘림. 가까워질수록 느려지고 1.0 m 근처(±0.1)에서 멈춘다. 근거: 시각 서보 제어 (Chaumette & Hutchinson 2006).
→ 이 방식은 RC카가 계속 멀어지면 목표보다 조금 뒤처져 따라간다 (0.15 m/s 로 멀어지면 약 1.25 m 에서 균형).

**안전 검사 `send()` — 모든 명령이 지나는 한 곳**
```
계산한 속도 → 부드럽게(가속 제한) → 최대값 자르기 (전진 ≤ max_lin, 후진 ≤ 0.1)
→ 전진이면 즉시 0: 라이다 1 초 끊김 / 앞 통로 < 정지거리 / 낮은 장애물 < 정지거리 / RC카 < 0.45 m
→ 후진이면 즉시 0: 라이다 끊김 / 뒤 통로 < 0.35 m
→ enable 일 때만 실제로 보냄
정지거리 = 0.12 + 0.6 × 속도 (0.1 m/s → 0.18 m)  ← 오늘 C3 로 실측 보정
```
- 카메라가 1 초 넘게 끊기면 정지 (S8), Ctrl+C 로 꺼도 정지 명령을 먼저 보냄 (S9).

**자주 보는 파라미터** (이름 · 기본값 · 뜻)
| 묶음 | 파라미터 |
|---|---|
| 주행 | `enable` false · `target_dist` 1.0 (카메라 → RC카, 범퍼 기준 약 0.85 m) · `max_lin` 0.3 (run_real 은 0.1) · `max_ang` 1.0 |
| 제어 | `lambda_lin` 0.6 · `lambda_ang` 1.2 · `dist_dead` 0.1 · `bearing_dead_deg` 5 / `bearing_start_deg` 8 |
| 검출 | `conf` 0.4 (새로 잡을 때) · `track_conf` 0.1 (추적기에 넘길 때) · `ground_only` true (벽 TV 같은 바닥 위 아닌 것 버림) |
| 놓침 | `reid_frames` 15 · `reid_accept` 0.6 · `search_max_s` 10 · `search_ang` 0.8 |
| 안전 | `min_dist` 0.45 · `stop_base` 0.12 · `stop_gain` 0.6 · `rear_stop` 0.35 · `max_rev` 0.1 · `cam_timeout` 1.0 |
| 카메라 | `fx_override` (run_real 909) · `image_transport` compressed · `min_fresh` 3 (재획득 직후 튀는 거리 무시) |

**S1~S18 이 뭔가**: 원본 코드(v3)를 시뮬·실물에서 고친 번호. 예) S13 = Wi-Fi 에서 원본 영상 대신 압축 영상, S15 = 놓친 뒤 한 바퀴 훑기, S18 = fx 909. 전체 목록: `docs/HANDOFF_261003.md`, `docs/FOLLOW_NODE_EXPLAINED.md` 6 절.

### 2-4. 기록 — `real/record_real.sh` (+ `real_recorder.py`, `make_summary.py`)
```bash
bash real/record_real.sh C2-front-1 "yolo12n 정면 1회차"     # Ctrl+C 로 끝
WEBCAM=1 bash real/record_real.sh C4-1 "..."                  # 웹캠 PC 미리보기도 bag 에
```
| 생기는 파일 (`~/tb4_real_logs/<시도ID>/`) | 내용 |
|---|---|
| `bag/` | 압축 영상·depth·라이다·odom·TF·명령·배터리·범퍼·노드 상태 (나중에 재생) |
| `video.mp4` | 디버그 화면 + 상태·속도·라이다 그림 |
| `telemetry.csv` / `events.csv` | 초당 10 줄 기록 / 상태 바뀜·경고·범퍼 |
| `summary.json` | **놓침 횟수·시간, 상태별 시간(TRACKING 비율), 후진 거리, 최소 전방 여유** → 실험일지에 옮김 |
| `meta.json`, `node_params.yaml` | 언제·어떤 설정으로 했는지 |

### 2-5. 도크
```bash
ros2 action send_goal /robot1/undock irobot_create_msgs/action/Undock "{}"   # 도크 위에서는 카메라·라이다 꺼짐
ros2 action send_goal /robot1/dock   irobot_create_msgs/action/Dock "{}"     # 도크에서 15 cm 이상 떨어져 도크를 볼 때 성공
```

## 3. 웹캠 PC 코드 — `real/webcam_rec.py` (파일 하나, ROS 없어도 됨)
```bash
python3 webcam_rec.py --trial C4-1                 # 녹화 (Ctrl+C 로 끝). 시도 ID 는 AMR PC 와 똑같이
python3 webcam_rec.py --trial test --seconds 10 --show   # 화면 확인
python3 webcam_rec.py --trial C4-1 --publish       # + ROS 로 작은 미리보기 2 Hz (웹캠 PC 에 ROS 있을 때만)
```
| 생기는 파일 (`~/webcam_logs/<시도ID>/`) | 내용 |
|---|---|
| `webcam_000.mp4`, `001` … | 10 분마다 나눈 영상 (중간에 꺼져도 앞부분 남음) |
| `frames.csv` | 프레임마다 **이 PC 시각**과 보인 ArUco 번호 (표시판 ID 3 을 찾을 때 씀) |
| `meta.json` | 해상도·실제 fps·PC 이름 |
- 원본 영상은 네트워크로 보내지 않는다 → 로봇 Wi-Fi 에 영향 없음 (10/3 원본 영상 때문에 RGB 가 15~20 초 밀린 일 때문).

## 4. 분석 코드 (시험 뒤, `scripts/`)
| 파일 | 한 줄 설명 | 입력 → 출력 |
|---|---|---|
| `webcam_geom.py` | **웹캠 카메라 모델** — 숫자 8 개(위치 x·y·z, 방향 3 개, 초점거리 f, 렌즈 휨 k1)로 "바닥 위 점 ↔ 화면 픽셀"을 오가는 계산 모음 | (다른 파일이 가져다 씀) |
| `webcam_marker_calib.py` | **마커 방식 보정** — 바닥 마커 4 장의 화면 위치와 줄자 좌표로 8 개 숫자를 찾음 | 10 초 영상 + `floor_markers.csv` → `calib/webcam_calib_marker.yaml` + 확인 그림 |
| `webcam_calib.py` | **마커 없는 방식 보정** — 로봇을 30 곳에 세우고 지도 위 위치(AMCL)와 화면 속 로봇 상자로 같은 8 개 숫자를 찾음 | 로봇 위치 기록 → `calib/webcam_calib.yaml` |
| `webcam_track_offline.py` | **정답 만들기** — 웹캠 YOLO 로 RC카 상자 → 치수로 바닥 위치, 로봇은 윗판 마커 ID 0 → 위치·정면 | 녹화 폴더 → `webcam_track.csv`(프레임별 위치·간격·"로봇 시야 안인가"), `webcam_track_summary.json` |
| `webcam_car_localizer.py` | 위와 같은 RC카 위치를 실시간 ROS 토픽으로 (2 Hz 미리보기 받을 때) | `/webcam/car_pose` |
| `webcam_clock_offset.py` | **두 PC 시계 차이** — "찍은 시각 − 받은 시각" 중 가장 큰 값 = 지연이 가장 짧았던 메시지 → 시계 차이 (Olson 2010) | AMR PC bag → 시계 차이 A (웹캠 시각 − A = AMR 시각) |
| `bag_model_compare.py` (hv01 패키지) | 같은 bag 을 여러 YOLO 모델로 다시 돌려 비교 | bag + 모델들 → 비교표·다른 결과 사진 |

**왜 웹캠 영상을 가운데만 잘라 640 으로 줄이나**: 웹캠 YOLO 를 학습시킨 사진이 Roboflow 에서 그렇게("Fill (with center crop)") 만들어졌기 때문. 같은 모양으로 넣어야 학습 때와 같은 결과가 나온다.

## 5. 자주 나올 질문
| 질문 | 답 |
|---|---|
| 왜 처음엔 `enable false`? | 인식·거리·방향이 맞는지 화면으로 먼저 보고, 맞을 때만 바퀴를 돌리려고 |
| 왜 704×704? | 로봇 카메라 설정이 1280×720 영상의 가운데를 정사각으로 잘라 냄 (깊이 맞춤에 16 의 배수, YOLO 에 32 의 배수가 필요한 것과 맞는 크기로 추정). 그래서 화각이 약 42° 로 좁음 |
| 왜 fx 909 로 덮어씀? | 로봇이 보내는 564.65 는 자르기 전 기준이라 틀림. 10/3 실측이 909 |
| 왜 압축 영상? | 원본(프레임당 1.4 MB)을 Wi-Fi 로 받으니 15~20 초 밀려 노드가 0 프레임 처리 (T2) |
| 왜 깃발을 뗌? | 가즈보에서 깃발이 YOLO 검출을 떨어뜨림 → 정답이 필요한 정지 시험(C0·C1)에만 (재측정 중, 계획 문서 참고) |
| 마커로 따라가면 안 되나? | 시뮬에서 마커 추종은 유지율 41/65/26 %, YOLO 91/87/71 % — 차가 돌면 마커가 안 보임. 마커는 **재는 데**만 |
| 웹캠이 꺼지면? | 추종은 그대로. 그 시도에 바깥 정답만 없음 |
| SLAM 지도 필요? | 오늘은 아님. 나중을 위해 벽 따라 한 바퀴 bag 만 남김 (계획 문서 SLAM 절) |

## 6. 파일 지도
| 폴더 | 파일 | 어디서 |
|---|---|---|
| `real/` | `preflight.sh`, `run_real.sh`, `run_plan1.sh`, `run_plan2.sh`, `record_real.sh`, `real_recorder.py`, `make_summary.py`, `dock_test.py` | AMR PC |
| `real/` | `webcam_rec.py` | 웹캠 PC |
| `scripts/` | `follow_object_node_v3_safe.py` (1안), `follow_object_node_v3_motion.py` (2안) (원본 v3 의 설명 주석판: `follow_object_node_v3_annotated.py`), `check_iface.py`, `lidar_check.py`, `detect_test.py` | AMR PC |
| `scripts/` | `webcam_*.py` | 분석 (아무 PC) |
| `calib/` | `floor_markers_example.csv` (복사해서 `floor_markers.csv` 로) | 웹캠 보정 |
| `weights/` | `yolo11n/12n_amr_best.pt` (로봇 카메라용), `webcam_yolo26s_best.pt` (웹캠용) | — |
| `docs/` | 이 문서, 계획과 근거, 시험 순서, 노드 원리, 논문 정리 | — |
