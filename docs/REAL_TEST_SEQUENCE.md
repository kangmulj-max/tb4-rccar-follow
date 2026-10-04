# 실물 시험 순서 — AMR PC + 웹캠 PC, ArUco 시험 포함 (2026-10-04)

> 지금까지 웹캠·마커 도구는 **시뮬·합성 영상·실제 웹캠 사진**으로만 확인했다. 이 문서는 실물에서 처음부터 끝까지 하는 순서다.
> 로봇 시간 7 시간을 아끼려고 **로봇 없이 할 수 있는 것은 전날·시작 전에** 끝낸다. 웹캠 쪽이 안 되면 웹캠을 빼고도 C0~C5 는 그대로 진행할 수 있다.
> 시험 내용의 근거: `2_깃발없이_하는_시험안`(C0~C5), `docs/WEBCAM_2PC_GUIDE.md`(웹캠), `docs/WEBCAM_AMR_COMM_PAPERS.md`(시계·통신).

## 한눈에
| 단계 | 언제 | 로봇 | 내용 | 시간 |
|---|---|---|---|---|
| A | 전날 | ✗ | 인쇄·준비물·두 PC 설치 확인 | 1 시간 |
| B | 당일, 로봇 켜기 전 | ✗ | 바닥 테이프·마커, 웹캠 보정(W1), 시계 맞추기 | 30~40 분 |
| C | 0:00–0:30 | ○ | 언도킹, preflight, 웹캠 로봇 위치 확인(W2), 두 PC 연동 확인(W3) | 30 분 |
| D | 0:30–0:55 | ○ | C0 깃발 영향 → C1 fx·depth (깃발) | 25 분 |
| E | 0:55–4:40 | ○ | C3 장애물 → C2 모델 비교 → C4 이동 추종(웹캠 구역) → C5 다시 찾기 | 3 시간 45 분 |
| F | 4:40–6:00 | ○ | 최종 모델 반복, target_dist 0.7 m | 80 분 |
| G | 6:00–7:00 | — | 예비·충전 | 60 분 |
| H | 시험 뒤 | ✗ | 분석 | — |

## A. 전날 (로봇 없이)
1. **마커 인쇄** — 인쇄 시트 v5, 배율 100 %, 각 장의 10 cm 자 확인
   | 쪽 | 마커 | 용도 |
   |---|---|---|
   | 2 | ID 0 (10/8/6 cm 중 하나) | 로봇 윗판 — 윗변 = 로봇 정면 |
   | 3~6 | ID 10~13 (10/8 cm 중 하나) | 바닥 기준 (웹캠 보정) |
   | 8 | ID 1 10 cm | RC카 깃발 — **C0·C1 에만** |
   | 11 | ID 2·3 8 cm | 장애물 상자 / 표시판(손잡이 판지) |
   인쇄 후 **검은 테두리 한 변을 자로 재서 기록** (계산에 이 값을 씀).
2. **깃발**: 두꺼운 판지에 ID 1 을 붙이고 클립·양면테이프로 RC카 지붕에 떼었다 붙였다 할 수 있게. 붙일 위치를 지붕에 표시.
3. **RC카 치수 실측** (길이·폭·높이, m) → 분석 때 `--car-dims` 에 넣음.
4. **AMR PC**: 팀 저장소 `feature/real-trial-261003` 의 `tb4_follow_sim/` 최신 받기, `weights/yolo11n·12n_amr_best.pt` 확인, 로봇 네트워크 설정 source 해서 `ros2 topic list` 가 보이는지.
5. **웹캠 PC**: 같은 폴더(최소 `real/webcam_rec.py`)를 복사. `python3 -c "import cv2; print(cv2.__version__)"` 가 4.7 이상인지 (ArUco 내장). 웹캠 동작:
   ```bash
   python3 real/webcam_rec.py --trial test --seconds 10 --show      # 화면이 뜨고 ~/webcam_logs/test/ 에 mp4·frames.csv
   ```
   웹캠 PC 에 ROS 2 가 있고 로봇과 같은 네트워크 설정이 가능하면 `--publish` 도 한 번 확인 (없으면 녹화만 — 그래도 분석 가능).
   설치가 필요하면 **설치 전에 담당자에게 확인**. 로봇 Raspberry Pi 에는 아무것도 설치·변경하지 않는다.
6. **분석용 PC**(어느 쪽이든): ultralytics·scipy 가 있고 `weights/webcam_yolo26s_best.pt` 가 있는지.

## B. 당일, 로봇 켜기 전 (로봇 시간 아님)
1. **바닥 테이프** (`2_깃발없이_하는_시험안` 2 절): 출발점 3 곳(정면·좌 20°·우 20°, 2 m), RC카 정지 위치, 장애물 위치, 1.5 m 정사각 추종 코스.
   - 추종 코스는 **웹캠 화면 안**에 오게 깐다 (C4 에 정답이 생김).
2. **웹캠 고정** — 이후 움직이면 보정 다시. 해상도는 기본 1280×720 그대로.
3. **W1 웹캠 보정 (마커 방식)**
   1. ID 10~13 을 코스 둘레 바닥에 붙임. 웹캠 바로 아래 바닥점에서 1.0 m 이내 8 cm, 1.25 m 이내 10 cm. **윗변은 모두 같은 방향**.
   2. 줄자로 중심 좌표를 재서 `calib/floor_markers.csv` 작성 (원점 = ID 10, x 축 = ID 10 → 11, y 축 = 그 왼쪽, size = 잰 한 변, yaw = 윗변 방향).
   3. 웹캠 PC: `python3 real/webcam_rec.py --trial calib --seconds 10` (아무도 마커를 가리지 않게)
   4. `python3 scripts/webcam_marker_calib.py ~/webcam_logs/calib/webcam_000.mp4 calib/floor_markers.csv`
   - **통과**: 4 장 모두 찾음, `reproj_rms_px` ≤ 3, `loo_err_cm_max` ≤ 10, `_check.jpg` 의 0.5 m 격자가 바닥 테이프와 맞음.
   - **실패하면**: csv 의 좌표·size·yaw 다시 확인 → 마커를 웹캠 쪽으로 옮김 → 그래도 안 되면 **웹캠 단계(W2·W3·C4 정답)를 빼고 진행**.
4. **시계 맞추기**: 두 PC 모두 NTP (Ubuntu `timedatectl` → synchronized yes / Windows 관리자 `w32tm /resync`).

## C. 0:00–0:30 로봇 켜기·점검
1. 로봇이 도크 위면 카메라·라이다가 꺼져 있음 → 언도킹 후 30 초:
   `ros2 action send_goal /robot1/undock irobot_create_msgs/action/Undock "{}"`
2. **AMR PC**: `bash real/preflight.sh`
   - 0 단계: `OK 도크에서 내려와 있음`, `camera_info: 704x704, fx=564.65`, 압축 RGB·depth 약 10 Hz
   - 4 단계 라이다: 로봇 정면 0.5 m 에 상자 → 정면 값 ≈ 0.5
   - 5 단계 YOLO: RC카 1 m 앞 → `detect_y11/12.jpg` 에 박스
   - `!!` 가 나오면 시험 전에 해결 (특히 704x704 가 아니면 로봇 카메라 설정이 바뀐 것 — 담당자에게 알림)
3. **W2 웹캠 로봇 위치 확인 (5 분)**: 윗판에 ID 0 을 붙인 로봇을 바닥 테이프의 줄자 좌표 3 곳(코스 안쪽·가장자리)에 세우고 웹캠 녹화 `--trial w2 --seconds 20`.
   분석: `python3 scripts/webcam_track_offline.py ~/webcam_logs/w2 --calib calib/webcam_calib_marker.yaml --stride 5`
   → `webcam_track.csv` 의 `rob_x, rob_y` 가 줄자 값과 **10 cm 이내**면 OK. 윗판 높이가 0.351 m 와 다르면 `--robot-marker-z` 로 실측값.
4. **W3 두 PC 연동 확인 (5 분)**:
   - 웹캠 PC: `webcam_rec.py --trial w3 --publish` (ROS 가 있을 때) / AMR PC: `WEBCAM=1 bash real/record_real.sh w3 "연동 확인"`
   - 표시판 ID 3 을 웹캠과 로봇 카메라에 **동시에 1 초**, 30 초 뒤 한 번 더 → 두 쪽 Ctrl+C
   - AMR PC: `python3 scripts/webcam_clock_offset.py ~/tb4_real_logs/w3/bag` → 시계 차이 A 가 나오면 OK (NTP 를 맞췄다면 수십 ms 안쪽 기대)
   - `--publish` 를 못 쓰면 이 단계는 표시판 사건만으로 (시험 뒤 확인).

## D. 0:30–0:55 깃발 시험 (ArUco)
### C0 깃발 영향 확인 (10 분) — 가장 먼저
- RC카를 1.0 m·1.8 m 앞에 뒷모습. 각 위치에서 **깃발 있음 10 초 → 없음 10 초**: `record_real.sh C0-1.0m "깃발 있음/없음"`
- 디버그 화면 car 점수 비교. 가즈보: 1 m 0.79 → 0.57, 1.8 m 0.81 → 0 (dummy 오인).
- **판정**: 깃발이 점수를 눈에 띄게 낮추면 → 이 순서 그대로 (깃발은 C1 까지만). 차이가 거의 없으면 → C2·C4 에도 깃발을 달아 정답 거리를 얻어도 됨.

### C1 fx·depth 확인 (15 분, 깃발 붙임)
- RC카(깃발)를 줄자로 0.5 / 1.0 / 1.5 / 2.0 m × 정면·좌 15°·우 15° 에 놓고 각 5 초 기록 (`record_real.sh C1 ...`).
- 같이 기록할 것: 각 위치의 줄자 값(범퍼 앞 ↔ 깃발 판).
- 현장 확인: 디버그 화면 `d=` 가 줄자 값과 대략 맞는지. 정밀 계산(마커 크기로 fx)은 시험 뒤 (H 단계).
- **끝나면 깃발을 뗀다.**

## E. 0:55–4:40 깃발 없이
| 순서 | 시험 | 설정 | 웹캠 | 기록·현장 판정 |
|---|---|---|---|---|
| 1 | **C3 장애물 정지거리** (20 분) | RC카 2 m, 사이 0.45 m 에 상자(로봇 쪽 면 ID 2). 높은 상자·10 cm 낮은 상자 × `max_lin` 0.1/0.2 × 2 회 | 선택 | 멈춘 뒤 줄자. 낮은 상자에서 `depthobs` 로 멈추는지 |
| 2 | **C2 모델 비교** (90 분) | RC카 정지 위치, 로봇 출발점 3 곳 × 3 회 × 상위 모델 3 개 (`run_real.sh 11/12`, 점심 bag 비교 결과로 선택) | 선택 | 멈춘 뒤 줄자 (`target_dist` 1.0 m = 카메라 기준, 범퍼 기준 약 0.85 m), 출발~정지 시간 |
| 3 | **C4 이동 추종** (90 분) | 1.5 m 정사각 코스 3 바퀴, 0.1~0.15 m/s, 상위 2 모델 × 조명 2 종 | **필수 녹화** (코스가 웹캠 화면 안) | 시도마다 시작할 때 표시판 ID 3 을 두 카메라에 1 초. `summary.json` 의 놓침·TRACKING |
| 4 | **C5 다시 찾기** (25 분) | RC카를 화면 밖으로 3~5 초 뺐다 정면 1 m 에 다시. 내놓는 순간 표시판 ID 3 을 로봇 카메라 가장자리에 1 초 | 선택 | 재획득 시간 (시험 뒤 계산) |

매 시도 공통:
- **AMR PC**: `run_real.sh <모델>` (enable false 로 시작) → `record_real.sh <ID> "메모"` (웹캠 PC 가 `--publish` 중이면 `WEBCAM=1`)
- **웹캠 PC**: `webcam_rec.py --trial <같은 ID>`
- 주행: `ros2 param set /follow_object_node enable true` → 끝나면 `enable false` → 두 PC Ctrl+C
- 실험일지에 시도 ID, 모델, 조명, 깃발 유무, 줄자 값.

## F. 4:40–6:00 최종
- 가장 좋은 모델로 C4 반복, `target_dist` 0.7 m 주행 (handoff 할 일). 배터리 확인.

## G. 예비 (6:00–7:00)
- 실패한 시도 재시험, 충전. 도크 복귀: `ros2 action send_goal /robot1/dock irobot_create_msgs/action/Dock "{}"`

## H. 시험 뒤 분석
| 무엇 | 명령 | 결과 |
|---|---|---|
| 시도 요약 | `record_real.sh` 가 자동 (`summary.json`) | 놓침 횟수·시간, TRACKING 비율, 후진, 최소 전방 여유 |
| 모델 비교 (같은 장면) | `bag_model_compare.py` (점심 비교용 hv01 패키지에 있음 — 이 저장소에는 없음) | 같은 bag 을 모델별로 재생해 비교 |
| 웹캠 정답 (C4) | `python3 scripts/webcam_track_offline.py ~/webcam_logs/<ID> --calib calib/webcam_calib_marker.yaml --car-dims L W H` | RC카·로봇 위치, 간격, 로봇 시야 안 비율 |
| 두 시계 차이 | `python3 scripts/webcam_clock_offset.py ~/tb4_real_logs/<ID>/bag` + 표시판 ID 3 프레임 비교 | 웹캠 t_wall − A = bag 시계 |
| C1 fx·depth, C3 상자 거리 | **아직 도구 없음** — bag 영상에서 ID 1·2 마커 크기로 fx·거리를 계산하는 스크립트를 시험 뒤 작성 (시험 중에는 기록만 하면 됨) | fx 실측, 노드 거리 오차 |

## 막히면
| 증상 | 조치 |
|---|---|
| preflight `!! 도크 위` / 토픽 `no data` | 언도킹 후 30 초 |
| `camera_info` 가 704x704 아님 | 로봇 카메라 설정이 바뀜 → 시험 중단하고 담당자 확인 (로봇은 직접 수정하지 않음) |
| 웹캠 보정 `loo_err_cm_max` > 10 | csv 좌표·yaw 확인, 마커를 웹캠 가까이 → 안 되면 웹캠 단계 생략 |
| 웹캠 PC 에서 `--publish` 안 됨 | 녹화만. 시계는 표시판 사건으로 |
| C0 에서 깃발 영향 없음 | 깃발을 C2·C4 에도 써서 정답 거리 확보 (B안) |
