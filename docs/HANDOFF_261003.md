# 인수인계 — 2026-10-03 실기 작업 (다른 PC 에서 이어서 하기)

이 문서 하나로 오늘 한 일, 현재 상태, 새 PC 준비, 다음 할 일을 알 수 있게 정리했다.
상세 근거: `docs/REAL_TRIAL_261003.md` (시행착오·실측), `docs/THEORY_PAPERS.md` (이론·논문).

## 0. 원칙 (사용자 지시)
- **로봇 robot1 의 라즈베리파이에는 아무것도 설치·수정하지 않는다** (읽기·토픽 구독·명령 전송만).
- 노트북 venv 에 무엇을 설치하면 **바로 패키지·버전·위치·정확한 용도를 보고**한다.
  - 지금까지: `lap 0.5.13` → `~/venvs/rokey_venv` (ultralytics ByteTrack 의 선형 할당, 없으면 run_real.sh 의 `YOLO_AUTOINSTALL=False` 때문에 첫 프레임에서 노드 종료).

## 1. 번들 구성
| 경로 | 내용 |
|---|---|
| `tb4_follow_sim/` | 프로젝트 (git 이력 포함, 브랜치 `real-trial-261003`), `weights/` (yolo11n/12n), `data/amr_dataset.zip` |
| `tb4-rccar-follow.bundle` | git bundle (모든 브랜치) — `git clone tb4-rccar-follow.bundle` 로도 복원 가능 |
| `tb4_real_logs/` | 오늘 실기 기록 (bag 제외): summary/telemetry/events/depth_profile/video, 줄자 실측, 도킹 시험, 노드 로그 |
| `tb4_real_bags_261003.zip` (별도) | 시도별 bag (mcap, 압축 RGB·compressedDepth·scan·odom·imu·tf·cmd, A 는 follow/state 포함) — 가즈보·오프라인 재생용 |

## 2. 새 PC 준비 (Ubuntu 24.04)
```bash
# 1) 번들의 프로젝트를 홈으로
cp -a tb4_follow_sim ~/tb4_follow_sim            # 이미 있으면 이름 확인 후
cd ~/tb4_follow_sim && git status && git log --oneline -3   # 브랜치 real-trial-261003

# 2) ROS 2 Jazzy + TB4 도구 + YOLO venv (sudo, 인터넷, 15~30 분) — 이미 있으면 건너뜀
bash install_new_pc.sh            # ~/yolo_venv 에 torch 2.6 / ultralytics 8.4.171 / numpy<2 / lap
#   이 PC(hv-01) 는 install_new_pc 대신 기존 ~/venvs/rokey_venv (ultralytics 8.4.170, torch CUDA, numpy 1.26.4) + lap 사용

# 3) 로봇 네트워크: Wi-Fi turtle07, ROS_DOMAIN_ID=1, 디스커버리 서버 192.168.107.101:11811
#    (hv-01 은 /etc/turtlebot4_discovery/setup.bash 를 ~/.bashrc 에서 source)
#    스크립트·비대화형 셸에서 ros2 CLI 를 쓸 땐 export ROS_SUPER_CLIENT=True

# 4) 오프라인 bag 재생은 반드시 격리 도메인으로 (실로봇과 섞이지 않게)
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT; export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```
- GitHub 푸시: 원격 `kangmulj-max/tb4-rccar-follow` (비공개). hv-01 에는 HTTPS 인증이 없고 SSH 키는 다른 계정(`shbong06-sketch`)이라 **푸시 안 했음**. 새 PC 에서 본인 계정으로 `git push -u origin real-trial-261003`.

## 3. 오늘 바뀐 것 (커밋 a43c362, 9a01d72 — 브랜치 real-trial-261003)
| 코드 | 내용 | 근거 |
|---|---|---|
| S13 | 압축 영상(jpeg + compressedDepth) 구독·디코드 (`image_transport`, 시뮬은 raw) | raw 구독 시 RGB 15~20 s 지연 → 동기화 0 회 |
| S14 | `ground_only` 지평선 필터, `cut_bottom_px` 잘림 거리 불신, `min_fresh` 3 | 벽 TV 오검출 7 m, 0.45 m 에서 지붕 depth +23 cm |
| S15 | 탐색 0.8 rad/s × 10 s, turn→wait→sweep | 원본 172° 에서 포기 |
| S16 | 거리 데드밴드 연속, `ff_gain` (기본 0) | 멈춤-출발 0.23 회/s, 후진 20 % |
| S17 | 원형 몸체 통로(여유 0.02), 방위 히스테리시스 5°/8° | 로봇 옆 벽을 '앞'으로 판정, ±15° 진동 |
| S18 | `fx_override` (run_real.sh 기본 909) | camera_info fx 564.65 ≠ 실제 ~909 (FOV ~42°) |
| 도구 | `real/dock_test.py`, `scripts/bag_depth_profile.py`, `scripts/static_sweep.py` | |

## 4. 아직 실기 검증 안 된 것 (다음에 할 일 순서)
1. **시도 B** (S16~S18, target 0.85, max_lin 0.1): 직선 멀어짐/멈춤 ×3 → S자 → 옆 지나 뒤로 ×2.
   - 판정: bag 에서 차 정지·로봇 회전 구간의 `방위 = k·yaw` 기울기가 **−1.06 근처면 fx 909 맞음** (`docs/REAL_TRIAL_261003.md` 5.3 방법).
   - 회전 부호 반전 횟수(A: 36 회/52 s)가 크게 줄어야 함.
   - 로봇은 벽에서 30 cm 이상, RC카는 정면 ~1 m 에서 시작 (FOV 가 ±21° 뿐).
2. target_dist 0.7 m 주행 (줄자 실측상 센서는 가능, 차 아래 잘림은 카메라 0.5 m 부터).
3. SLAM 지도 → Nav2 로 '벽에 가려진 차' 의 LOP 까지 이동 (아래 5 절).
4. 가즈보 반영: 카메라 FOV 42°, Create3 최소 회전 0.26 rad/s·명령 지연 0.2 s·인지 지연 ~0.4 s, 코너/가림 시나리오.

## 5. SLAM (노트북에서 실행, 로봇 설치 없음) — 배터리 50 % 이상에서
```bash
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot1        # 터미널 1
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/robot1    # 터미널 2 (RViz)
robot-undock; robot-keyboard                                               # 터미널 3: 0.2 m/s 이하로 벽 따라 한 바퀴, 출발점으로 복귀
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/arena -t /robot1/map
# 이후: ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 map:=$HOME/maps/arena.yaml
#       ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1
```
- `turtlebot4_navigation` 은 hv-01 의 `~/turtlebot4_ws` 에 빌드돼 있음 (새 PC 는 `ros-jazzy-turtlebot4-navigation` apt 또는 같은 ws).
- RC카는 라이다(0.19 m)보다 낮아 지도에 거의 안 찍힘.

## 6. 운영 메모 (오늘 겪은 것)
- 노트북 Wi-Fi 가 turtle09 로 자동 전환된 적 있음 → 실기 전 `nmcli -t -f active,ssid dev wifi`.
- 로봇 부팅 후 oakd 토픽은 1~2 분 늦게 올라옴.
- `record_real.sh` 종료는 터미널 Ctrl+C 또는 **부모 프로세스에만** SIGINT (자식까지 보내면 summary.json 미생성).
- `pkill -f <패턴>` 은 그 패턴을 담은 자기 셸까지 죽일 수 있음 → `pgrep -f "^python3 /경로/..."` 처럼 앵커.
- 원격 도킹: 도크에서 ~15 cm 이상 떨어져 도크를 향할 때 성공 (dock_test 3/3). 도크에 너무 붙여 두면 ABORTED.
- 배터리: 오늘 90 % → 19 % (오후 내내 실험). 실험 전 충전.
