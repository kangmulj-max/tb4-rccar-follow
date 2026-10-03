# 실기 1차 연동 기록 — 2026-10-03 (robot1)

- 노트북: Ubuntu 24.04, ROS 2 Jazzy, RTX 3060, `~/venvs/rokey_venv` (ultralytics 8.4.170, torch CUDA, numpy 1.26.4)
- 로봇: robot1 (192.168.107.101, Wi-Fi `turtle07`), ROS_DOMAIN_ID=1, Fast DDS 디스커버리 서버
- 노드: `scripts/follow_object_node_v3_safe.py` + yolo12n, `max_lin 0.1`
- 시도: `261003-1500` (기록 `~/tb4_real_logs/261003-1500/`, 주행 14:45:51 ~ 14:57:08)
- 로봇(라즈베리파이)에는 아무것도 설치·수정하지 않음. 노트북 venv 설치: `lap 0.5.13` 하나

## 1. 시도 결과 요약 (`summary.json` + telemetry 분석)

| 항목 | 값 |
|---|---|
| 기록 시간 | 694.7 s |
| 상태별 시간 | TRACKING 394.9 s / REID 66.9 s / SEARCH 99.3 s / IDLE 133.5 s |
| 놓침 (→SEARCH) | 39 회, 놓친 시간 약 233 s |
| TRACKING→REID | 103 회, 이 중 54 회는 0.5 s 안에 다시 TRACKING (id 깜빡임) |
| 범퍼 충돌 | 0 |
| 후진 | 총 0.61 m, 최대 −0.1 m/s, 후진 중 최소 후방 여유 0.55 m |
| 전방 여유 < 0.15 m | 757 샘플 (~76 s), 최소 −0.05 m (SEARCH 제자리 회전 중) |
| 카메라/라이다 끊김 | 0 / 0 |
| 영상 지연 (img_age) | 중앙값 0.08 s, p95 0.20 s, 최대 0.41 s (S13 적용 후) |
| 최대 속도 | 명령 0.10 m/s, odom 최대 0.136 m/s |
| 배터리 | 90 % → 79 % (11.5 분) |

정상 동작 확인: 1.1 m → 1.0 m 접근 후 정지·유지, 놓치면 REID 회전 → 재획득, SEARCH → IDLE 포기, enable=false / Ctrl+C 시 정지 명령 전송.

## 2. 시행착오 목록 — 가즈보 적용 포함

| # | 증상 | 원인 | 이번 조치 | 가즈보에 적용할 것 |
|---|---|---|---|---|
| T1 | 첫 프레임에서 노드 종료 가능 | ByteTrack 의존 `lap` 미설치 + `run_real.sh` 가 `YOLO_AUTOINSTALL=False` | `rokey_venv` 에 `lap 0.5.13` 설치 | 시뮬 PC 도 `python -c "import lap"` 확인. `run_*.sh` 시작 시 import 점검 추가 권장 |
| T2 | **노드가 떠도 영상 처리 0 프레임**, `follow/state` 안 나옴 | Wi-Fi 로 raw (RGB 1.4 MB + depth 1 MB/프레임) 구독 → 로봇 oakd 가 밀려 RGB 가 15~20 s 늦게 도착(계속 증가), depth 는 ~2 s → RGB·depth 시각 차가 동기화 허용 0.1 s 를 넘어 콜백이 한 번도 안 불림. raw 구독을 끊자 compressed 는 10 Hz, 지연 0.05/0.13 s | **S13**: 노드가 `.../compressed`(jpeg) + `.../compressedDepth`(png) 구독·디코드, 파라미터 `image_transport` (기본 compressed). 결과 10.2 Hz, 디버그 영상 지연 0.27 s | 시뮬은 대역폭 한계가 없어 재현 안 됨. `run_follow.sh` 는 `image_transport:=raw` 로 고정함. 실기 경로를 시뮬에서 검증하려면 `image_transport republish` 로 compressed 토픽을 만들고 compressed 모드로 돌릴 것. 추가로 RGB 에만 수 초 지연을 넣는 시나리오(카메라 lag)를 만들어 노드가 경고하는지 확인 |
| T3 | T2 상황에서 노드가 아무 경고도 안 냄 | S8 카메라 타임아웃은 TRACKING/REID 에서만, 동기화 실패는 감시 안 함 | (미수정) | "camera_info 는 오는데 동기화 프레임이 N 초째 0" 경고 추가 → 시뮬에서 depth 지연 주입으로 검증 |
| T4 | 재획득 순간 거리 7.03 m 1 회 → 최대 전진 0.1 m/s 명령 | REID→TRACKING 직후 depth 패치가 배경을 읽음. `dist_hist` 를 비운 직후라 중앙값 필터가 못 거름 | (미수정, 다음 프레임 0.94 m 로 복귀) | 재획득 직후 거리 2~3 프레임 확인 후 사용, 또는 이전 값 대비 급변 거부. 시뮬에서 박스 가장자리 depth 를 배경으로 바꾸는 시나리오로 검증. **max_lin 올리기 전 필수** |
| T5 | id 깜빡임 다수 (103 회 중 54 회 < 0.5 s) / 놓침 39 회 | 실기 RC카(검정, 저채도) + 실제 조명·모션블러로 검출 점수 변동, ByteTrack 새 id 부여 | (미수정) | 시뮬 결과의 놓침 횟수·TRACKING 비율과 같은 시나리오(square/random 등)로 비교. 시뮬 조명·텍스처를 실기 영상(`video.mp4`)에 맞춰 조정. bag 오프라인 재생으로 yolo11n/12n·track_conf·reid_accept 비교 |
| T6 | 전방 여유 < 0.15 m 가 ~76 s, 최소 −0.05 m | 아레나 벽 근처 / SEARCH 제자리 회전 중. 회전은 안전 게이트 대상이 아님 (S3 은 전진만 막음) | 충돌 0 | 벽 0.1 m 앞 제자리 회전 시나리오 추가. 몸체 안쪽(−0.05) 점이 무엇인지 bag 의 scan 으로 확인 (자기 부품·케이블 반사 가능) |
| T7 | odom 0.136 m/s > 명령 0.10 | 출발·정지 오버슈트 또는 odom 잡음 | 없음 | 시뮬 diff-drive 의 가감속이 실기와 다를 수 있음 → 시뮬 stop_base/stop_gain 결과를 실기 정지거리로 다시 보정 |
| T8 | undock 7 cm 후진 후 멈춤, 재시도도 goal accepted 후 무반응 | 미확인 (바퀴 enabled, 배터리 94 %, 후방 2.3 m 비어 있음) | 사람이 로봇을 손으로 옮김 | 시뮬은 도크에서 시작하지 않음 → 해당 없음. 실기 절차에 "undock 실패 시 손으로 옮기기" 포함 |
| T9 | 스크립트에서 `ros2 topic list` 결과 0 개 | `/etc/turtlebot4_discovery/setup.bash` 가 터미널이 아닐 때 `ROS_SUPER_CLIENT=False` → CLI 가 그래프를 못 봄 | 자동화 스크립트에서 `export ROS_SUPER_CLIENT=True` | 시뮬은 디스커버리 서버를 안 써서 무관. 단 `preflight.sh`/`record_real.sh` 를 cron·원격으로 돌릴 땐 export 필요 |
| T10 | 로봇이 ping 안 됨 | 노트북 Wi-Fi 가 `turtle07` → `turtle09` 로 자동 전환 | `turtle07` 재연결 | 무관 (운영). 실기 전 `nmcli -t -f active,ssid dev wifi` 확인 |
| T11 | `preflight.sh` RGB 5.3 Hz, YOLO 1 프레임 테스트 | `detect_test.py`·`check_iface.py` 도 raw 를 구독 (T2 와 같은 부하, 짧게) | 없음 | 점검 도구도 compressed 로 바꾸는 것이 좋음 |

## 3. 수동 실행 절차 (실기)

준비: 노트북 Wi-Fi `turtle07`, 로봇 전원 ON (라이트링 켜짐), `ping 192.168.107.101` 응답.
터미널을 새로 열면 `~/.bashrc` 가 ROS·디스커버리 설정을 자동으로 불러온다 (`echo $ROS_DOMAIN_ID` → 1).
시뮬용 `env.sh` 는 실기 터미널에서 쓰지 말 것.

```bash
# 0) 연결 확인 (oakd 토픽은 부팅 후 1~2 분 늦게 올라올 수 있음)
ping -c2 192.168.107.101
ros2 topic list | grep -E "oakd/rgb/image_raw/compressed|scan$|cmd_vel_unstamped"

# 1) [터미널 A] 점검 — 로봇 정지 상태
bash ~/tb4_follow_sim/real/preflight.sh

# 2) [터미널 A] 노드 — enable=false 로 시작 (계산·디버그 영상만, 바퀴 안 움직임)
bash ~/tb4_follow_sim/real/run_real.sh 12            # yolo12n, max_lin 0.1, compressed
#    로그에 'image_transport=compressed', 'camera_info ok', 'lidar TF ok' 확인

# 3) [터미널 B] 노드가 영상을 처리하는지 (약 10 Hz 여야 함)
ros2 topic hz /robot1/follow/state
ros2 topic echo /robot1/follow/state                # state/d/front/rear/block
ros2 run rqt_image_view rqt_image_view /robot1/follow/image   # 디버그 화면

# 4) 도크에서 빼기 (실패하면 손으로 옮김). 로봇 앞 1.5 m 이상 비우고 RC카를 ~1 m 앞에
robot-undock        # = ros2 action send_goal /robot1/undock irobot_create_msgs/action/Undock "{}"

# 5) [터미널 C] 기록 시작
bash ~/tb4_follow_sim/real/record_real.sh <시도ID> "메모"

# 6) 디버그 화면이 [TRACKING] 인 것 확인 → E-Stop 대기 → 주행
ros2 param set /follow_object_node enable true

# 7) 종료 (순서대로)
ros2 param set /follow_object_node enable false     # 정지 명령 3 회
#    터미널 C 에서 Ctrl+C → summary.json 출력
#    터미널 A 에서 Ctrl+C → 'shutdown: stop' 로그 후 종료
robot-dock          # 필요 시 도킹
```

비상 시: E-Stop, 또는 아무 터미널에서 `ros2 param set /follow_object_node enable false`.
노드 Ctrl+C 도 정지 명령을 보내고 끝난다 (S9).

자주 쓰는 파라미터 (`run_real.sh 12 -p 이름:=값` 또는 실행 중 `ros2 param set`):
`max_lin` (기본 0.1 — T4 해결 전 올리지 말 것), `target_dist` 1.0, `conf` 0.4, `track_conf` 0.1, `reid_accept` 0.6,
`image_transport` compressed|raw, `cam_timeout` 1.0.

## 4. 근거리 추종 연구 (같은 날 오후)

### 4.1 줄자 실측 (로봇 정지, RC카 뒷면이 로봇을 향함) — `~/tb4_real_logs/sweep_261003-1545_tape047`, `..._1546_tape022`
| 범퍼 앞 | 카메라 기준 | 검출 | depth | 오차 | 비고 |
|---|---|---|---|---|---|
| 0.47 m | 0.70 m | car 0.99 | 0.745 m (σ 1 mm) | +4.4 cm | 차 전체 보임 (y2=607) |
| 0.22 m | 0.45 m | car 0.18 / **dummy 0.62** | 0.683 m | **+23 cm** | 차 아래가 화면 밖 → 지붕만 보이고 depth 가 지붕을 읽음 |

- 잘림 시작 ≈ 카메라 0.50 m (y2 ∝ 1/d 로 0.70 m 의 y2=607 에서 외삽).
- 결론: target_dist 0.7 m (30 cm 가까이) 는 센서상 가능. 단 신뢰 구간 끝(0.5 m)까지 여유 0.2 m.
- 같은 날 정지 스윕 2 회(`sweep_*_INVALID_*`)는 로봇 방향/RC카 배치가 단계와 안 맞아 무효 — 원격 주행 중 frames.csv 만 참고.
- 박스 바닥(그림자 포함)으로 거리 추정은 depth 와 10~24 % 차이 → 줄자 대체 불가. (카메라 기울기 추정은 근거 부족으로 철회)

### 4.2 S14 노드 수정 (근거리·오검출 보호)
| 파라미터 | 기본 | 내용 |
|---|---|---|
| `ground_only` | true | 박스 아래 끝이 지평선(cy) 위면 버림 → 벽 TV 를 car 로 본 7 m 튐 제거 |
| `cut_bottom_px` | 4 | 박스가 화면 하단에 닿으면 거리 불신 → 전·후진 0, block 에 `cut` |
| `min_fresh` | 3 | 유효 거리 3 개 이상일 때만 중앙값 사용 |

bag 재생 A/B (261003-1500, 20~70 s, 격리 도메인): 7 m 튐 3 → 0, TRACKING 프레임 162 → 182, 재식별 6 → 11.

### 4.3 시도 261003-T085 (target_dist 0.85, S13+S14, max_lin 0.1, 1083 s)
- 차 정지 3 분: 0.864 m 유지 (σ 3 mm), TRACKING 100 %.
- depth_profile: 잘림 없는 최소 0.629 m, 0.6~0.7 m 에서 conf 0.90 / 잘림 프레임 317 개 (depth 중앙값 0.67 = 지붕) — S14 가 거리로 쓰지 않음.
- 상태 처리율이 ~4.4 Hz 로 낮아짐 (첫 시도 10 Hz) — 원인 미확인 (기록 부하 의심).
- `record_real.sh` 종료 시 부모·자식 모두에 SIGINT → 이중 인터럽트로 summary.json 미생성 (T12). **터미널 Ctrl+C 또는 부모에게만 INT.**

### 4.4 뒤로 간 RC카를 못 찾음 (T13)
| 시도 | →SEARCH | 찾음 | 포기 | 포기한 탐색의 회전량 |
|---|---|---|---|---|
| 1500 | 39 | 27 | 12 | 최대 174°, 2~48° 도 다수 |
| T085 | 54 | 39 | 15 | 168~174° 가 10 회 |
- 원인 1: `search_ang 0.5 rad/s × search_max_s 6 s = 172°` → **뒤(180°)까지 돌기 전에 포기**.
- 원인 2: LOP 방향(±8°)을 보면 회전을 멈추고 대기 → 차가 이미 움직였으면 그대로 시간 초과.
- 카메라 FOV ~64° 정면만, 라이다는 360° 이지만 RC카(~0.18 m)가 라이다 평면(0.19 m)보다 낮아 15~65 % 만 보임.
- Nav2 는 경로계획·장애물 회피(코스트맵) 스택이라 인식 능력을 더하지 않음 → 단독으로는 해결 안 됨.
- 가즈보: 차가 로봇 옆을 지나 뒤로 가는 시나리오(pass-behind) 추가, 탐색 성공률·소요 시간 지표.

### 4.5 S15 탐색 수정 (T13 대응)
- 파라미터: `search_ang` 0.5 → **0.8 rad/s**, `search_max_s` 6 → **10 s**, 새 `search_wait_s` 0.7 s.
- 로직: `turn`(LOP 쪽 회전) → `wait`(LOP 방향 0.7 s 확인) → `sweep`(차가 사라진 쪽으로 계속 회전). 원본은 LOP 를 보면 시간 초과까지 정지 대기.
- 오프라인 로직 시험 (격리 도메인, 퍼블리시 차단, LOP=100°): turn 2.4 s(→99°) → wait 0.7 s → sweep, 10 s 동안 총 **411°** 회전 후 IDLE. 원본 설정 최대 172°.
- 실기 검증 필요: 차가 로봇 옆을 지나 뒤로 가는 동작에서 SEARCH→TRACKING 비율, 재획득까지 시간.
- 「주의」 0.8 rad/s 제자리 회전은 안전 게이트(전진만 판정) 대상이 아님 — TB4 는 원형이라 회전 충돌 위험은 낮지만, 몸체 안쪽 라이다 점(T6)이 있으면 확인.
