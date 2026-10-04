# 웹캠 연동 — AMR PC + 웹캠 PC 두 대 구성 (2026-10-04)

> 추종은 지금처럼 **AMR PC 혼자** 한다 (웹캠 없이도 동작). 웹캠은 **평가용 정답**(RC카·로봇의 실제 위치, 간격, 로봇 시야 안이었는지)을 만든다.
> 웹캠 영상은 **웹캠 PC 에 녹화**하는 것이 기본 — 로봇 Wi-Fi 에 부하를 주지 않는다 (10/3: 원본 영상 구독으로 RGB 가 15~20 s 밀린 사례).

## 1. 어느 PC 에서 무엇을
| | AMR PC (로봇 제어 노트북) | 웹캠 PC |
|---|---|---|
| 필요 | ROS 2 Jazzy, 로봇 네트워크 설정 (지금 그대로) | Python 3 + OpenCV (ROS 없어도 됨). 분석까지 하려면 ultralytics, scipy |
| 시험 중 | `real/preflight.sh` → `real/run_real.sh 12` → `real/record_real.sh <ID>` | `python3 real/webcam_rec.py --trial <같은 ID>` |
| 선택 | `WEBCAM=1 record_real.sh` (웹캠 미리보기를 bag 에), `webcam_car_localizer.py` (실시간 위치, 2 Hz) | `--publish` (웹캠 PC 에 ROS 2 + 같은 네트워크 설정이 있을 때만) |
| 시험 뒤 | bag, summary.json | `scripts/webcam_track_offline.py <녹화 폴더>` → 위치·간격·시야 판정 |

- **시도 ID 를 두 PC 에서 똑같이** 쓴다 (예: `261004-1430`). 웹캠 녹화 폴더: `~/webcam_logs/<ID>/`.
- `--publish` 를 쓰면 640 px JPEG 2 Hz (약 0.1 MB/s). 웹캠 PC 도 AMR PC 와 같은 `ROS_DOMAIN_ID`·디스커버리 서버 설정을 source 해야 보인다.

## 2. 두 PC 시계 맞추기 (필수)
웹캠 기록은 웹캠 PC 시계, bag 은 AMR PC 시계로 찍힌다.
1. 시험 전 두 PC 모두 인터넷 시간 동기화:
   - Ubuntu: `timedatectl` 에서 `System clock synchronized: yes` 확인 (아니면 `sudo timedatectl set-ntp true`)
   - Windows: `w32tm /resync` (관리자), `w32tm /query /status`
2. 시도마다 **시작할 때 표시판 마커(ID 3)를 웹캠과 로봇 카메라에 함께 1 초** 보여 준다.
   → 웹캠 `frames.csv` 의 `aruco_ids` 와 로봇 bag 에서 ID 3 이 처음 보인 시각의 차이 = 두 시계 차이 (확인용).
3. `WEBCAM=1` 로 기록했다면 시험 뒤 `python3 scripts/webcam_clock_offset.py ~/tb4_real_logs/<ID>/bag` 로 시계 차이를 bag 만으로 추정 (Olson 2010). 근거·한계: `docs/WEBCAM_AMR_COMM_PAPERS.md`

## 3. 시도 1회 절차
| 순서 | AMR PC | 웹캠 PC |
|---|---|---|
| 1 | preflight (도크에서 내려와 있는지 포함) | 웹캠 화면 확인 (`--show` 로 잠깐) |
| 2 | `run_real.sh 12` (enable false) | `webcam_rec.py --trial <ID>` 시작 |
| 3 | `record_real.sh <ID> "메모"` | |
| 4 | 표시판 ID 3 을 두 카메라에 1 초 | ← 같이 |
| 5 | enable true → 주행 → enable false | |
| 6 | Ctrl+C | Ctrl+C |

## 4. 웹캠 보정 — 두 방식
보정은 웹캠을 세운 뒤 **한 번** (웹캠이 움직이면 다시). 두 방식 모두 같은 카메라 모델·같은 yaml 형식이라 분석 도구는 공통.

### 4-1. 마커 없는 방식 (기준 — 기존 `webcam_calib.py`)
로봇을 움직이는 기준물로 쓰고, 로봇 위치는 AMCL(맵) 에서 받는다.
1. AMR PC: 지도 + AMCL(위치 추정) 실행, 로봇 초기 위치 지정
2. 웹캠 PC: `webcam_rec.py --trial calib --publish`
3. AMR PC: `python3 scripts/webcam_calib.py collect --ros-args -p image_topic:=/webcam/image_raw/compressed`
   - 처음 5 초는 로봇·RC카가 화면 밖 (배경 학습), 그다음 로봇을 화면 안 **30 곳 이상**(가장자리 포함)에 세움
4. `python3 scripts/webcam_calib.py solve` → `calib/webcam_calib.yaml` (좌표계 = **맵**)
- 로봇 위치는 웹캠이 아니라 AMCL 이 주므로 분석 때 bag 의 `/robot1/amcl_pose` 를 시각으로 맞춰 붙인다.

### 4-2. 마커 방식 (`webcam_marker_calib.py`)
바닥 마커 4 장 + 줄자. ROS·로봇·지도 불필요.
1. 인쇄 시트 v5 3~6 쪽(ID 10~13) 을 시험 구역 바닥에 붙임. 크기: 웹캠 바로 아래 바닥점에서 1.0 m 이내 8 cm, 1.25 m 이내 10 cm. **윗변은 모두 같은 방향**.
2. 줄자로 좌표를 재서 `calib/floor_markers.csv` 작성 (예: `calib/floor_markers_example.csv`)
   - 원점 = ID 10 중심, x 축 = ID 10 → ID 11, y 축 = 위에서 볼 때 x 축의 왼쪽
   - `size` = 인쇄 후 잰 검은 테두리 한 변 (m), `yaw_deg` = 마커 윗변이 가리키는 방향 (x 축이면 0, y 축이면 90)
3. 웹캠 PC: `webcam_rec.py --trial calib --seconds 10` (사람·로봇이 마커를 가리지 않게)
4. `python3 scripts/webcam_marker_calib.py ~/webcam_logs/calib/webcam_000.mp4 calib/floor_markers.csv`
   → `calib/webcam_calib_marker.yaml` (좌표계 = **바닥**) + 검증 그림 `_check.jpg` (0.5 m 격자가 바닥과 맞는지 눈으로 확인)
   - 출력의 `loo_err_cm_max` (한 장씩 빼고 맞힌 오차) 가 10 cm 를 넘으면 csv 의 좌표·크기·방향을 다시 확인
- 로봇 위치는 **윗판 마커(ID 0, 시트 2 쪽)** 로 웹캠이 직접 잰다 (윗변 = 로봇 정면). 웹캠 YOLO 모델에는 로봇 클래스가 없음 (`car`, `dummy` 만).

### 4-3. 비교
| | 마커 없는 방식 | 마커 방식 |
|---|---|---|
| 로봇 시간 | 20~30 분 (30 곳 세우기) + 지도·AMCL 필요 | **0** (사람이 10~15 분) |
| 좌표계 | SLAM 맵 (내비게이션과 같은 좌표) | 바닥 (줄자) |
| 로봇 위치 | AMCL (bag 에서 시각 맞춤) | 윗판 마커 ID 0 (웹캠이 직접, 같은 시계) |
| 정확도 (시뮬·합성 시험) | 보정점 30 개: RC카 위치 중앙값 2.5 cm (AMCL σ 5 cm 가정) | 줄자 오차 1 cm 일 때 마커 위치 최대 3.9 cm (한 장씩 빼고) |
| 실물 검증 | 아직 | 아직 (합성 영상으로 카메라 값 그대로 복원 확인) |
| 웹캠이 밀렸을 때 | 30 곳 다시 | 10 초 녹화 후 다시 계산 |
| 이런 때 | 맵 좌표가 필요할 때 (내비게이션 연동) | **이번 실물 시험** (시간 절약, 로봇·RC카를 같은 시계·같은 좌표로) |

## 5. 시험 뒤 분석
```bash
python3 scripts/webcam_track_offline.py ~/webcam_logs/<ID> --calib calib/webcam_calib_marker.yaml \
    --car-dims 0.28 0.14 0.14      # RC카 실측 (길이 폭 높이, m) 으로 바꿀 것
```
- `webcam_track.csv`: 프레임별 RC카 위치·방향, 로봇 위치·정면(ID 0), 간격(범퍼 ↔ 차 뒤 근사), 로봇 기준 방위, 로봇 카메라 화각(42°) 안인지
- `webcam_track_summary.json`: RC카·로봇이 보인 비율, 간격 중앙값·최소·최대, 로봇 시야 안 비율
- 확인: 실제 웹캠 사진 42 장에서 RC카 위치가 기존 참고값과 중앙값 3.6 cm 차이, 합성 윗판 마커는 0.1 cm·0.14° 로 복원

## 6. 주의
- 웹캠 해상도는 보정 때와 시험 때 **같게** (기본 1280×720). 모든 도구는 가운데 정사각을 잘라 640 으로 줄인 화면(학습 데이터와 같은 전처리)에서 계산한다.
- 웹캠 위치·각도가 바뀌면 보정 다시.
- 웹캠 시야 밖 구간은 정답이 없다 — 지표는 시야 안 구간만.
