# 웹캠 PC ↔ AMR PC 데이터 연동 — 근거 논문과 우리 설계 검토 (2026-10-04)

> **결론.** 우리 구성(웹캠 PC 는 녹화가 기본, ROS 로는 작은 압축 미리보기만 2 Hz best-effort)은 아래 논문들의 권고와 맞는다.
> 논문 기준으로 빠져 있던 것은 **두 PC 시계 차이를 재는 방법**이었고, Olson(2010) 방식으로 `scripts/webcam_clock_offset.py` 를 추가했다.
> 남는 한계: 고정 최소 지연(수십 ms)은 이 방법으로 없앨 수 없어 **표시판(ID 3) 이벤트로 교차 확인**한다.

## 0. 우리 구성 요약 (`docs/WEBCAM_2PC_GUIDE.md`)
| 데이터 | 경로 | 크기·주기 | QoS |
|---|---|---|---|
| 로봇 RGB·depth | 로봇 → AMR PC (Wi-Fi) | 압축 RGB ≈ 86 KB + depth ≈ 210 KB, 10 Hz (10/3 실측) | 노드는 best-effort 구독 |
| 웹캠 원본 | 웹캠 PC 디스크 | 1280×720, 15 fps (mp4) | 네트워크 안 씀 |
| 웹캠 미리보기 (선택) | 웹캠 PC → AMR PC (ROS 2) | 640 폭 JPEG, 2 Hz. 실제 웹캠 사진으로 잰 크기 약 16 KB (실물 720p 는 이보다 클 수 있음, 그래도 64 KB 아래) | best-effort, depth 1 |
| 시각 | 웹캠: `frames.csv` t_wall + `header.stamp` (웹캠 PC 시계) / bag: 받은 시각 (AMR PC 시계) | | |

## 1. 네트워크·ROS 2 통신

### [P1] Macenski et al., "Robot Operating System 2: Design, architecture, and uses in the wild", *Science Robotics* 7, 2022
- **내용**: ROS 2 설계와 성능 측정. 프로세스 간 통신 95 % 지연은 1 MB 까지 1 ms 미만, 8 MB 에서 7.85 ms. 54 Mb/s 로 제한한 "느린 무선" 흉내 망에서 패킷 손실 0~10 % 는 전달이 유지되고 20 % 는 크게 떨어짐.
- **기준**: 기본 DDS 설정은 **1 MB 보다 큰 메시지에 효과적이지 않음** (UDP 버퍼, 조각화, 신뢰성 재전송). **센서 데이터처럼 새 값이 옛 값을 쓸모없게 만드는 경우 best-effort** + keep-last.
- **우리 설계**: ✔ 미리보기 16 KB 수준, best-effort·depth 1 → 권고와 일치. ✔ 원본 웹캠은 네트워크로 보내지 않음.

### [P2] Kronauer et al., "Latency Analysis of ROS2 Multi-Node Systems", arXiv:2101.02074, 2021
- **실험**: i7 데스크톱·Raspberry Pi 4, FastRTPS·CycloneDDS·Connext, 128 B~500 KB, 1~100 Hz.
- **결과**: 64 KB(UDP 조각화 경계)를 넘으면 지연이 크게 늘고, RELIABLE 은 약 15 % 지연 추가. ROS 2 계층이 작은 메시지에서 최대 50 % 지연 추가. 지연은 DDS 구현에 크게 좌우.
- **기준**: **64 KB 넘는 메시지 피하기**, 미들웨어 선택이 중요.
- **우리 설계**: ✔ 웹캠 미리보기는 64 KB 아래. △ 로봇 depth(210 KB)는 경계 위 — 10/3 에 압축으로 바꿔 10 Hz 를 맞춘 것(S13)이 이 결과와 같은 방향. 로봇·DDS 설정은 바꾸지 않는다는 원칙이라 그대로 둠.

### [P3] Lee, Kim, Chae, Park, "Optimizing ROS 2 Communication for Wireless Robotic Systems", arXiv:2508.11366, 2025 (DGIST)
- **실험**: 노트북 2 대, 802.11ac 5 GHz, ROS 2 Humble + Fast DDS 2.6.9, 32~512 KB 를 30 Hz, 손실 0 / 1 / 20 % 와 20 초 끊김.
- **문제 3 가지 (수치)**:
  1. IP 조각화: 기본 RTPS 최대 64 KB → 메시지 하나가 약 44 개 IP 조각. 메시지가 온전히 올 확률 = p^(조각 수) → 1 % 손실에서 33 KB 97 % → 330 KB 73 % 전달.
  2. 재전송 간격: 기본 3 s (0.33 Hz) → 재전송 때 순간 160 Mb/s 몰림.
  3. 버퍼 몰림: 10 초 끊긴 뒤 쌓인 400 개가 92 MB 로 한꺼번에 → 복구 후 5 Hz.
- **제안**: RTPS 최대 크기 1472 B, 재전송 주기 = 발행 주기 × 2, 버퍼 크기를 링크 속도에 맞춤 → 512 KB 1.16 → 28.9 Hz, 1 % 손실에서 256 KB 실패 → 30 Hz.
- **우리 설계**: ✔ 미리보기 16 KB ≈ 11 개 IP 조각 → 1 % 손실에서 메시지 90 %, 5 % 에서 57 % 도착 (계산값). best-effort 라 놓친 프레임은 버리고 다음 것을 씀 — 재전송·버퍼 몰림(문제 2·3)은 RELIABLE 의 문제라 해당 없음. ✔ **분석용 정답은 네트워크가 아니라 웹캠 PC 디스크 녹화**를 쓰므로 미리보기가 빠져도 결과에 영향 없음. ✗ 논문의 XML 튜닝은 로봇 쪽 DDS 도 바꿔야 효과가 커서 이번 범위 밖.

## 2. 두 컴퓨터 시계 맞추기

### [P4] Mills, "Internet time synchronization: the Network Time Protocol", *IEEE Trans. Communications* 39(10), 1991
- **내용**: 왕복 지연으로 시계 차이를 재는 NTP. 인터넷 대부분에서 **수 ms** 정확도.
- **우리 설계**: ✔ 안내서 2 절에서 두 PC 모두 NTP 동기화 확인 (Ubuntu `timedatectl`, Windows `w32tm`).
- **주의 [P4']**: Microsoft 문서(Windows Time service support boundary) — Windows 10 이상은 조건이 맞을 때 1 s / 50 ms / 1 ms 등급. 기본 설정의 Windows 는 **ms 단위를 보장하지 않음** → 웹캠 PC 가 Windows 면 아래 P5 로 확인 필수.

### [P5] Olson, "A Passive Solution to the Sensor Synchronization Problem", IROS 2010 ★ 적용
- **문제 설정**: 상대가 동기화에 협조하지 않아도 (NTP 불가), 상대 시각 p_i 와 받은 시각 q_i 만으로 시계 차이 A 를 추정.
- **원리**: p_i − q_i = A − e_i, 지연 e_i ≥ 0 → **A ≥ max(p_i − q_i)**. 가끔 오는 지연 짧은 메시지가 A 를 알려 줌. 시계가 어긋나는 속도를 |ΔA| ≤ α·Δt 로 두고 앞뒤로 한 번씩 훑는 O(N) 알고리즘 (Alg. 1).
- **보장**: 받은 시각을 그대로 쓰는 것보다 **절대 나빠지지 않음** (Claim 2). 남는 오차 = 가장 짧았던 지연.
- **논문 결과**: 지연 0~0.5 s 균일, 1 초 간격 관측에서 동기화 안 할 때 평균 오차 0.25 s → 시계가 좋을수록(α 작을수록) 크게 줄어듦. 관측이 잦을수록 좋음.
- **우리 적용**: `scripts/webcam_clock_offset.py` — bag 의 웹캠 미리보기에서 p = `header.stamp`(웹캠 PC 시계, 촬영 시각), q = bag 기록 시각(AMR PC 시계). PC 시계는 수십 ppm → α = 50 ppm, 2 Hz 관측.
- **합성 시험** (웹캠 시계 +350 ms, 20 ppm 어긋남, 지연 20~400 ms, 10 분): 추정 오차 평균 21 ms (= 최소 지연 20 ms), 받은 시각 그대로 쓰면 평균 97 ms. 10 분 동안의 시계 어긋남(12 ms)도 따라감.
- **한계**: 최소 지연(무선 수십 ms)만큼은 남음 — 논문도 "최소 지연을 알면 빼라"고 함 → P6 로 보완.

### [P6] Zaman & Illingworth, "Interval-based time synchronization of sensor data in a mobile robot", ISSNIP 2004 (Olson 2010 의 관련 연구 [5])
- **원리**: 두 데이터 흐름에서 모두 보이는 **사건**(예: 정지에서 출발하는 순간)을 맞춰 시각 대응. 사건이 늘수록 불확실성 구간을 교집합으로 줄임.
- **우리 적용**: 시도 시작(과 끝)에 **표시판 마커 ID 3 를 두 카메라에 동시에** 보여 줌. 웹캠 `frames.csv` 의 `aruco_ids` 와 로봇 bag 영상에서 ID 3 이 처음 보인 시각 → 두 시계 차이를 사건으로 직접 확인. P5 의 최소 지연 오차를 이것으로 점검. 정확도는 프레임 간격(웹캠 15 fps ≈ 67 ms, 로봇 10 Hz = 100 ms)에 묶임 → 여러 번 보여 주면 줄어듦.

### [P7] Furgale, Rehder, Siegwart, "Unified Temporal and Spatial Calibration for Multi-Sensor Systems", IROS 2013 (Kalibr)
- **내용**: 시간 차이를 공간 보정과 함께 하나의 추정 문제로 풀어 측정 주기보다 작은 오차까지 추정.
- **우리 판단**: 같은 물체를 두 카메라가 연속으로 보는 구간(웹캠의 RC카 위치 ↔ 로봇 depth 의 RC카 거리)으로 시간 차이까지 최적화할 수 있으나, 7 시간 안에서는 P5 + P6 으로 충분. **필요 정확도 계산**: 추종 속도 0.1~0.2 m/s 에서 50 ms 시각 오차는 0.5~1 cm 위치 오차 — 웹캠 보정 오차(수 cm)보다 작음.

## 3. 외부 카메라 정답의 좌표 (참고 — 기존 보고서에서 이미 사용)
- Zhang, "A flexible new technique for camera calibration", *IEEE TPAMI* 22(11), 2000 — 바닥 평면 호모그래피 분해로 카메라 초기값 (`webcam_geom.init_from_homography`, 마커 방식·마커 없는 방식 공통).
- Garrido-Jurado et al., "Automatic generation and detection of highly reliable fiducial markers under occlusion", *Pattern Recognition* 47(6), 2014 — ArUco. 바닥 마커(ID 10~13) 모서리, 로봇 윗판(ID 0), 표시판(ID 3).
- Mousavian et al., "3D Bounding Box Estimation Using Deep Learning and Geometry", CVPR 2017 — 2D 박스 + 알려진 치수로 바닥 위치 (`localize_car`).

## 4. 논문 기준 점검표
| 기준 (출처) | 우리 | 상태 |
|---|---|---|
| 64 KB 넘는 메시지 피하기 [P2][P3] | 웹캠 미리보기 ≈ 16 KB | ✔ |
| 센서 영상은 best-effort, keep-last [P1] | best-effort, depth 1 | ✔ |
| 무선에서 큰 영상은 손실·몰림 위험 [P3] | 원본은 웹캠 PC 디스크, 네트워크는 2 Hz 미리보기만 | ✔ |
| 두 PC 시계 동기화 [P4] | NTP 확인 절차 (안내서 2 절) | ✔ (Windows 정확도 주의) |
| 협조 없는 시계 차이 추정 [P5] | `webcam_clock_offset.py` | ✔ 추가 |
| 사건 기반 교차 확인 [P6] | 표시판 ID 3 | ✔ |
| 시간·공간 공동 보정 [P7] | 안 함 (필요 정확도 대비 불필요) | — |
| DDS 파라미터 튜닝 [P3] | 안 함 (로봇 쪽 설정 변경 금지) | — |

## 5. 쓰는 법 (시험 뒤)
```bash
# AMR PC 의 bag 에 웹캠 미리보기가 있을 때 (record_real.sh 를 WEBCAM=1 로 실행)
python3 scripts/webcam_clock_offset.py ~/tb4_real_logs/<시도ID>/bag
#   → 시계 차이 A. 웹캠 webcam_track.csv 의 t_wall − A = bag 과 같은 시계
```
미리보기를 보내지 않았으면(녹화만) A 는 표시판 ID 3 사건으로만 구한다.

## 출처
- [P1] https://arxiv.org/abs/2211.07752
- [P2] https://arxiv.org/abs/2101.02074
- [P3] https://arxiv.org/abs/2508.11366
- [P4] https://www.eecis.udel.edu/~mills/database/papers/trans/trans.txt
- [P4'] https://learn.microsoft.com/windows-server/networking/windows-time-service/support-boundary
- [P5] https://april.eecs.umich.edu/pdfs/olson2010.pdf
- [P7] https://github.com/ethz-asl/kalibr/wiki
