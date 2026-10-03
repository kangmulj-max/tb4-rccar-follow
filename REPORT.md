> ⚠ **이 문서는 1차 보고(2026-10-03 오전)이며 갱신 예정입니다.** 카메라↔RC카 거리 수치는 카메라 위치 가정 오류로 약 0.18 m 작게 계산되어 있고(M1), 상태 시간·YOLO 감지율 비교에도 정정 사항이 있습니다(M2, M5). 정정 목록은 `docs/CODE_REVIEW_GUIDE.md` 3-4, 최신 YOLO 비교는 `docs/YOLO_COMPARISON.md` 를 보세요.

# follow_object_node_v3 — 가즈보 TurtleBot4 추종 실험 보고 (2026-10-03)

모든 수치는 이 세션에서 실제로 돌린 결과입니다. 원자료는 `logs/` 에 있습니다 (`*_summary.json`, `*.csv`, `*_node.log`, `batch_results.md`).

## 1. 세팅

| 구성 | 내용 |
|---|---|
| OS / ROS | WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-turtlebot4-simulator` 2.0.2 설치) |
| YOLO | `~/yolo_venv` (torch 2.6 cu124, ultralytics 8.4.171, numpy 1.26 — cv_bridge 호환 위해 numpy<2), RTX 3060 사용 |
| 로봇 | TB4 standard, 네임스페이스 `robot1` (스냅샷과 동일) |
| 카메라 | turtlebot4_description 오버레이로 OAK-D 를 **704×704, HFOV 1.1152 rad(fx≈564.5), 10 Hz** 로 변경 (스냅샷 caminfo 기준) |
| 어댑터 | `tb4_real_iface_adapter.py`: 시뮬 토픽 → 실기 토픽/형식 (`oakd/rgb/image_raw` bgr8, `oakd/stereo/image_raw` 16UC1 mm, camera_info 2종, `cmd_vel_unstamped`→`cmd_vel` TwistStamped) |
| 라이다 | `sim_rplidar.py`: 실기 RPLIDAR 사양(640, ±π, 0.164–12 m, +90° 장착)으로 월드 충돌 박스를 레이캐스트. 스캔 평면 높이 0.19 m |
| 월드 | `follow_arena.sdf`: 8×6 m 회색 바닥 + 나무 펜스, **검은 RC카**(G바겐형: 박스 차체·캐빈·바퀴4·스페어타이어·은색 그릴·헤드램프), dummy 박스(0.1 m), 기둥(0.6 m), 폴(5 cm) |
| 노드 실행 | 원본 무수정. `-p target:=car -p lidar_offset_deg:=-90 -r /robot1/cmd_vel:=/robot1/cmd_vel_unstamped -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static` |

환경 구축 중 해결한 문제
- WSL 기본 OpenGL 이 llvmpipe(CPU) → 카메라 ~1 Hz. `GALLIUM_DRIVER=d3d12` 로 RTX 렌더링 → 10 Hz, RTF 1.0.
- gz `gpu_lidar` 가 WSL d3d12 + ogre1 에서 640 개 전부 `range_min(0.164 m)` 반환, ogre2 로 바꾸면 카메라 ~2 Hz → 기하 레이캐스트 라이다로 대체.
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 에서 프로세스 ~60개일 때 일부 노드끼리 디스커버리 실패(어댑터 카메라 0 Hz) → 도메인 ID 격리로 변경.
- Windows 쪽 편집 시 `+x` 상실로 런치 노드가 안 뜸 → `python3 script` 로 실행.

## 2. 버그 (v3 코드)

| # | 내용 | 영향 | 근거 |
|---|---|---|---|
| B1 | `cmd_vel` 을 `Twist` 로 퍼블리시 — 실기·시뮬 모두 `/robot1/cmd_vel` 은 `TwistStamped` | 그대로 쓰면 **로봇이 안 움직임** (타입 불일치로 매칭 안 됨) | 스냅샷 topics.txt, create3 bridge |
| B2 | `target` 기본값 `rc_car` — 모델 클래스는 `['car','dummy']` | 아무것도 추종 안 함 | data.yaml |
| B3 | `lidar_offset_deg` 기본 0 — RPLIDAR 는 base 대비 +90° 장착 | 정면 감시가 실제로 **좌측**을 봄 | lidar_check: 정면 0.7 m 기둥이 offset 0 → 2.97 m(좌측 벽), −90 → 0.60 m. pole 시나리오 offset 0: 17 s 동안 반복 충돌(범퍼 624건) |
| B4 | depth 무효 시 `dist_hist` 의 옛 값 중앙값을 계속 사용 | **거리를 모르는데 전진/후진을 무한 지속**, `min_dist` 도 같은 값을 써서 무력화 | depthloss: depth 상실 후 lin 0.15 m/s 고정. depthstop: 0.78 m 고정값으로 −0.13 m/s 후진을 끝없이 지속 |
| B5 | 라이다 정지 판정이 `send()` 의 가감속 제한(0.4 m/s²)을 그대로 거침 + `lidar_stop` 0.35 m 를 라이다 중심에서 잼(범퍼까지 여유 ≈0.14 m) + 판정이 카메라 콜백(~7–8 Hz)에서만 | 보정 offset 에서도 0.22 m/s 접근 시 **한 번 충돌 후 정지** | pole_y11_offset-90 |
| B6 | 정면 섹터 ±20° 고정 — 로봇 반폭 0.171 m 를 0.47 m 안쪽에서는 못 덮음 | 진로를 옆에서 일부 침범한 장애물은 가까워질수록 섹터 밖으로 빠짐 | pillar(4.5 cm 침범): offset 0 / −90 모두 충돌 |
| B7 | 후진 시 후방 감시 없음 (`lidar_blocks` 는 전진만 막음) | 타깃이 다가오거나 depth 이상 시 뒤로 계속 감 | approach: −0.25 m/s 후진으로 뒤 기둥 충돌 |
| B8 | 종료 시 rclpy 컨텍스트가 먼저 내려가 `stop()` 퍼블리시 실패 (`RCLError: publisher's context is invalid`) | 종료 시 정지 명령이 **안 나감** — 베이스 cmd_vel 타임아웃에만 의존 | shutdown 시나리오 노드 로그 |
| B9 | REID 상태 `send(prev_lin*0.5, prev_ang)` 는 라이다·min_dist 검사 없이 전진 | 짧은 구간 무검사 주행 | 코드 |
| B10 | 카메라 워치독 없음 — 영상이 끊기면 명령 갱신이 멈출 뿐 정지 명령을 내지 않음 | 베이스 타임아웃에 의존 | camdrop: 3.3 s 동안 속도 0 (베이스가 멈춤) |

## 3. 디버그 (시나리오 결과)

| 시나리오 | 결과 요약 |
|---|---|
| square/random/stopgo/lost × 2모델 | 추종 자체는 동작. 거리오차 평균 0.12–0.23 m, 시야 유지 72–100 %, 제어 6.7–8.5 Hz. ByteTrack ID 전환으로 TRACKING→REID 가 실행당(45–60 s) 3–31 회 (REID 대부분 성공) |
| random_y11 | 차체 최근접 0.167 m (가장 가까운 접근) |
| depthstop | 0.3 m/s 접근 시 목표 1.0 m 를 지나 0.59 m 까지 진입(지연 오버슈트) 후, depth 상실로 무한 후진 |
| pole offset 0 vs −90 | 0: 반복 충돌 / −90: 1회 충돌 후 정지 |
| pillar, lowbox | 둘 다 충돌 (섹터 폭 / 라이다 평면 아래) |
| approach | 후진 중 뒤 기둥 충돌 + RC카와 접촉 |
| camdrop | 카메라 3 s 정지 동안 로봇 정지 (베이스 타임아웃 효과) |
| shutdown | 노드 종료 후 0.3 s 내 정지 (베이스 타임아웃 효과, 노드 stop() 은 실패) |

측정 한계: 모니터의 `MOVE_WITHOUT_CAMERA` 는 yolo12n 실행 때만 다수 발생했지만 어댑터는 계속 10 Hz 였음 → CPU 부하로 인한 모니터 수신 지연(측정 오차)으로 판단하고 결론에서 제외. 첫 배치의 모니터 상태 시간은 첫 상태 전환을 놓쳐 IDLE 이 부풀려짐 → 상태 통계는 노드 로그로 재계산(`scripts/node_log_stats.py`).

## 4. 실물 데이터와 비교 / 실물에서 체크할 점

| 항목 | 실기 스냅샷 | 시뮬 | 실기에서 확인할 것 |
|---|---|---|---|
| RGB 왜곡 | rational_polynomial, k1≈7.17 (강함), image_raw 미보정 | 왜곡 없음 | 화면 가장자리 방위각 오차 → 타깃이 화면 가장자리일 때 회전 정확도 |
| Depth | OAK-D 스테레오, RGB 정렬, 16UC1 | 완벽 depth | **검은 무광 RC카에서 depth 유효 비율** (B4 위험 직결), 0.35 m 이내 무효 |
| 주기·대역폭 | rgb 10 Hz 14.9 MB/s + depth 10 Hz → Wi-Fi 로 ~30 MB/s | 로컬 | 노트북 실행 시 Wi-Fi 지연·끊김 (스냅샷 수집 중 로봇이 네트워크에서 끊기고 oakd 토픽 1분 30초 소실 기록 있음) |
| cmd_vel | `cmd_vel` TwistStamped + `cmd_vel_unstamped` 존재 | 어댑터로 재현 | `cmd_vel_unstamped` 리맵 후 실제로 움직이는지 |
| 정지 안전장치 | Create3 cmd_vel 타임아웃·후진 제한·최대속도 | gz diffdrive 타임아웃(~0.5 s) | 노드 종료/영상 끊김 시 실제 정지까지 시간·거리 |
| 라이다 | `/robot1/scan`, rplidar_link +90° | 동일 | `scripts/lidar_check.py` 를 실기에서 실행 → 정면에 물체 두고 offset −90 이 맞는지 |
| 디스커버리 | Fast DDS 디스커버리 서버, DOMAIN 1 | DOMAIN 30 | 노트북 env 설정 |
| 데이터셋 | 81 장 train, 같은 방에서 촬영 | 시뮬 RC카 | valid 20 원본 중 19, test 12 중 10 이 train 과 같은 원본 → mAP 0.995 는 일반화 지표 아님. 다른 장소·조명·거리에서 별도 검증 필요 |

실기용 도구: `check_iface.py`, `lidar_check.py`, `detect_test.py` 는 토픽 이름이 실기와 같아 그대로 사용 가능(env 만 실기용으로). 주행 전 bag 녹화(스냅샷 step 9 미수집) 후 노드를 `enable=false` 로 돌려 debug 이미지로 검증 권장.

## 5. 실물에서 발생할 수 있는 문제 (사고 시나리오)

1. offset 0 그대로 실행 → 정면 장애물/사람 다리를 못 보고 충돌 (B3)
2. 검은 차체 depth 실패 → 옛 거리로 RC카 추돌 또는 무한 후진 (B4, B7)
3. 보정해도 0.2 m/s 이상 접근 시 정지 거리 부족 (B5)
4. 측면 걸침 장애물·책상 다리·케이블·낮은 물체(0.19 m 미만: RC카 자체, 박스, 문턱) 미감지 (B6, 라이다 평면)
5. 타깃이 다가오면 후진 → 뒤 사람·벽 (B7)
6. Wi-Fi 지연/끊김 → 오래된 영상 기반 명령, 끊기면 베이스 타임아웃까지 진행 (B10)
7. Ctrl+C 종료가 정지 명령을 보장하지 않음 (B8) → 실기 테스트 시 E-Stop(조이스틱 B 버튼/Create3 버튼) 대기 필수
8. 다른 '차' 오검출 시 IDLE 에서 가장 큰 박스를 새 타깃으로 잡음 — 사람 많은 실습장 주의
9. 도킹 스테이션 위 시작 시 바로 앞으로 출발(시뮬은 도크를 치우고 시작)

권장 수정(원본 미수정, 제안): depth 무효 N 프레임이면 lin=0 + dist_hist 비우기 / 안전정지는 가감속 제한 우회 / lidar_stop 을 범퍼 기준 v²/2a + v·지연 + 여유(0.3 m/s 기준 ≈ 0.45 m@라이다)로 / 섹터를 로봇 폭 기준 직사각형 영역으로 / 후진 금지 또는 lin ≥ 0 / 카메라·스캔 워치독에서 정지 퍼블리시 / 종료 시 rclpy shutdown 전에 정지 퍼블리시.

## 6. YOLO 2종 비교

| 항목 | yolo11n | yolo12n |
|---|---|---|
| 파라미터 / GFLOPs | 2.58 M / 6.4 | 2.56 M / 7.3 |
| 추론 (RTX 3060 / CPU, 704→640) | **6.8 ms** / **22.3 ms** | 10.7 ms / 27.6 ms |
| 실물 valid mAP50 / 50-95 | 0.995 / **0.971** | 0.995 / 0.950 |
| 실물 test mAP50 / 50-95 | 0.995 / 0.905 | 0.995 / 0.903 |
| 시뮬 감지율 (거리 4 × 방향 5, conf≥0.4) | 75 % (15/20) | **85 % (17/20)** |
| 약점 (시뮬) | 45°·90° 비스듬한 자세에서 누락 | 원거리 정지 뒷모습 conf 하락 (1.25 m 정지 뒷모습: 11n 0.73 vs 12n 0.33 → 추종 시작 실패) |
| 추종 안정성 (노드 로그, TRACKING→REID 횟수) | square 19 / random 31 / stopgo 3 / lost 27 | square 30 / random 8 / stopgo 9 / lost 11 |

해석: 실물 지표는 데이터 누수 때문에 둘 다 포화되어 우열 판단 근거가 약함. 시뮬에서는 12n 이 다양한 자세에서 더 많이 잡지만 conf 가 경계값(0.4) 근처로 떨어지는 경우가 있고, 11n 은 더 빠르고(1.6배) 잡을 때 conf 가 높음. 노트북 CPU 실행이면 둘 다 10 Hz 카메라를 따라갈 수 있는 속도(22–28 ms). 실기 선택은 다른 장소 데이터로 재검증 후 결정, 우선은 conf 를 0.3 근처로 낮춘 12n 또는 11n 을 같은 시나리오로 비교 권장.
