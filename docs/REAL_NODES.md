# 실기 구성 노드 해설 — robot1 + 노트북

근거: 실기 스냅샷 `tb4_snapshot_20261002_1909` (nodes.txt, topics.txt, param_*.yaml, tf_static.yaml).
**[확인]** = 스냅샷에 있는 사실 / **[추정]** = 이름·구조로 판단, 실기에서 `real/preflight.sh` 로 확인 필요.

## 0. 전체 그림 — 명령 한 번이 바퀴까지 가는 길
```
 OAK-D ──(USB)── oakd 드라이버 ─┐                                           ┌─ follow_object_node (v3_safe)
 RPLIDAR ─(USB)─ rplidar 드라이버 ┼─ /robot1/... 토픽 ─── Wi-Fi (Fast DDS, 디스커버리 서버) ───┤   = 인식·판단
 URDF ── robot_state_publisher ─┘                                           ├─ real_recorder, ros2 bag (기록)
                                                                            └─ rqt_image_view (화면)
                                 /robot1/cmd_vel_unstamped (Twist) ◀────────────── 속도 명령
 Create3 motion_control ◀─ /robot1/cmd_vel (TwistStamped) ◀─ create3_repub [추정: Twist→TwistStamped]
   └ 바퀴, 범퍼·절벽 반사동작, cmd_vel 타임아웃, odom
```
한 주기 지연 = 카메라 노출·전송(Wi-Fi) + YOLO 추론(노트북) + 명령 전송(Wi-Fi) + Create3 제어 주기.

---

## 1. 로봇(RPi4) 쪽 노드 — TurtleBot4 bringup 이 자동 실행 [확인: nodes.txt]

### 1-1. `/robot1/oakd` (+ `/robot1/oakd_container`) — OAK-D Pro 카메라 드라이버 (depthai-ros)
| 항목 | 값 |
|---|---|
| 파이프라인 | `i_pipeline_type: RGBD` — RGB + 스테레오 depth [확인] |
| 우리가 쓰는 출력 | `oakd/rgb/image_raw` 704×704 bgr8 10 Hz, `oakd/stereo/image_raw` 16UC1 mm (RGB 에 정렬, K 동일), `oakd/rgb/camera_info` [확인: image_stats, caminfo] |
| 그 외 출력 | 각 영상의 `/compressed`, `/compressedDepth`, `/theora`, `/zstd` (image_transport 플러그인), `rgb/preview/*`, `imu/data` [확인: topics.txt] |
| IR 도트 프로젝터 | `i_laser_dot_brightness: 100` (켜짐) — 무늬 없는 표면에 점을 쏴 스테레오 depth 를 돕는다. **검은 RC카 depth 에 유리** [확인: 값] / 효과 크기는 실측 필요 |
| 주의 | 스냅샷 수집 중 oakd 토픽이 약 1분 30초 사라졌다 복구 (드라이버 재시작 추정) → v3_safe 는 1 s 후 정지(S8) |
| 대역폭 | 원본 RGB 14.9 MB/s (+ depth) — 노트북이 원본을 구독하면 Wi-Fi 부하 큼 |

### 1-2. `/robot1/rplidar_composition` — RPLIDAR 드라이버
| 항목 | 값 |
|---|---|
| 출력 | `/robot1/scan` (LaserScan, frame `rplidar_link`) [확인] |
| 설정 | `serial_port: /dev/RPLIDAR`, 115200 baud, `angle_compensate: true` [확인] |
| 장착 | base_link 기준 x = −0.04 m, 높이 0.193 m, **yaw +90°** [확인: tf_static] → 원본 v3 의 정면 오프셋 0° 가 틀린 이유 |
| 한계 | 0.193 m 높이 평면만 봄 → RC카(0.14 m)·낮은 상자는 안 보임 |

### 1-3. `/robot1/robot_state_publisher`, `/robot1/joint_state_publisher` — 좌표계(TF)
- URDF(`robot_description`)로 고정 좌표 45 개를 `/robot1/tf_static` 에, 바퀴 관절을 `/robot1/tf` 에 발행 [확인].
- v3_safe 가 쓰는 변환: `base_link → rplidar_link`(라이다 장착), `base_link → oakd_rgb_camera_optical_frame`(카메라: x −0.06, 높이 0.244), `odom → base_link`(마지막 관측 위치) [확인: tf_static, frames.gv].
- 토픽이 네임스페이스 안(`/robot1/tf`)이라 노드 실행 시 `-r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static` 필요 (run_real.sh 가 처리).

### 1-4. `/robot1/turtlebot4_node` — 버튼·메뉴·화면·LED, 도킹
| 설정 [확인: param_turtlebot4_node.yaml] | 의미 |
|---|---|
| 컨트롤러 **B = EStop** | **실험 중 비상정지 버튼.** Create3 를 E-Stop 상태로 만들어 모든 주행 명령 차단 |
| Home = Dock / Undock (3 s) | 도킹·해제 |
| L2 / R2 = 벽 따라가기 | 실험 중 누르지 말 것 |
| Create3 전원 버튼 = EStop / Power (3 s) | 로봇 본체 버튼으로도 E-Stop |
| `power_saver: true` | **도크에 올라가 있으면 카메라·라이다를 끔** [추정: TurtleBot4 절전 기능 — 시뮬 로그의 `oakd/stop_camera`, `stop_motor` 서비스 호출과 일치] → 실험 전 Undock 필수, preflight 에서 `no data` 면 먼저 확인 |

### 1-5. `/robot1/turtlebot4_base_node` — RPi HAT 의 GPIO
LED(전원·모터·통신·Wi-Fi·배터리·사용자), 사용자 버튼 4 개 [확인]. 추종과는 무관.

### 1-6. `/robot1/create3_repub` — Create3 ↔ RPi 토픽 중계 [추정]
- 스냅샷에 `cmd_vel`(TwistStamped)과 `cmd_vel_unstamped`(Twist)가 함께 있고 이 노드가 있다 → `cmd_vel_unstamped` 를 받아 `cmd_vel` 로 넘기는 역할로 추정.
- **v3_safe 는 `cmd_vel_unstamped` 로 명령을 보내므로 이 노드가 실제로 구독하고 있어야 바퀴가 움직인다** → `real/preflight.sh` 3) 에서 구독자 확인.

### 1-7. `/robot1/joy_linux_node` + `/robot1/teleop_twist_joy_node` — 블루투스 컨트롤러
| 설정 [확인: param_teleop_twist_joy_node.yaml] | 의미 |
|---|---|
| `enable_button: 4` (L1), `require_enable_button: true` | L1 을 누르고 있을 때만 주행 명령 |
| `scale_linear.x 0.2`, turbo(버튼 5) 0.5 m/s, 회전 0.5 / turbo 1.9 rad/s | 수동 주행 속도 |
| `publish_stamped_twist: true` → `/robot1/cmd_vel` (TwistStamped) | **v3_safe 와 같은 바퀴 명령 경로에 동시에 들어감** |

주의: L1 을 누른 채 스틱을 놓으면 조이스틱이 0 을 계속 보내 노드 명령과 번갈아 섞인다 → 멈추긴 하지만 확실한 정지는 아님. **비상시에는 B(EStop) 또는 `enable false`.**

### 1-8. Create3 본체 (iRobot Create 3 내부 ROS) [확인: topics.txt 의 odom, battery_state, dock_status, wheel_status, imu, hazard 관련]
- `motion_control`: 속도 명령 실행, **cmd_vel 이 일정 시간 끊기면 정지**, 범퍼·절벽·바퀴 들림 반사동작, 후진 제한, 최대 속도 ~0.306 m/s.
- 발행: `odom`, `imu`, `battery_state`, `dock_status`, `wheel_status`, `hazard_detection`(범퍼 등 — 기록 도구가 사용).
- 노드 목록에 안 보이는 이유: Create3 는 별도 컴퓨터(본체)에서 돈다 [추정: 디스커버리 서버 구성상 목록에 안 잡힘].
- **원본 v3 가 Ctrl+C 시 정지 명령을 못 보내도(B8) 로봇이 멈췄던 것은 이 타임아웃 덕분** (시뮬에서는 diffdrive 타임아웃으로 재현). 실기 타임아웃 값은 미확인.

### 1-9. 그 밖에 스냅샷에 보인 것
- `/launch_ros_1740`: bringup 런치 프로세스.
- `/teleop_twist_keyboard`, `/rqt_gui_cpp_node_*`: 스냅샷 때 노트북에서 띄워 둔 키보드 조종·rqt [확인]. `teleop_twist_keyboard` 는 기본으로 `/cmd_vel`(네임스페이스 없음, Twist)에 보내므로 리맵하지 않으면 로봇에 안 간다.

---

## 2. 노트북 쪽 노드 — 우리가 실행

| 노드 | 실행 | 구독 | 발행 | 역할 |
|---|---|---|---|---|
| `follow_object_node` | `real/run_real.sh 12` | rgb·stereo image_raw, camera_info, scan, tf | `cmd_vel_unstamped`, `follow/image`, `follow/state` | 인식·판단·제어 (`docs/REAL_CODE_STRUCTURE.md`) |
| `real_recorder` | `real/record_real.sh` | follow/image, scan, odom, cmd_vel(2), hazard, battery, /rosout | (파일) | 영상·텔레메트리 기록 |
| `rosbag2_recorder` | `record_real.sh` 가 같이 실행 | 압축 영상, depth, scan, odom, tf, cmd … | (mcap 파일) | 재생용 원자료 |
| `rqt_image_view` | `ros2 run rqt_image_view rqt_image_view /robot1/follow/image` | follow/image | — | 디버그 화면 |
| 점검 스크립트 | `real/preflight.sh` | 센서 토픽 (best-effort) | — | 출발 전 확인 |

---

## 3. 네트워크 — 노트북이 로봇 토픽을 보려면 [확인: 스냅샷 README]
- 로봇: `ROS_DOMAIN_ID=1`, **Fast DDS 디스커버리 서버** 방식, IP 192.168.107.101 (스냅샷 당시).
- 노트북도 같은 도메인과 디스커버리 서버 설정이 필요 (TurtleBot4 사용자 매뉴얼의 사용자 PC Discovery Server 설정 절차). CLI(`ros2 topic list`)가 전체를 보려면 super client 설정도 필요.
- **WSL2 에서 실기를 쓸 때**: 기본 NAT 네트워크는 로봇과 DDS 통신이 잘 안 된다. Windows 의 `%USERPROFILE%\.wslconfig` 에
  ```
  [wsl2]
  networkingMode=mirrored
  ```
  를 넣고 `wsl --shutdown` 후 다시 시작 (Windows 방화벽에서 UDP 허용 필요할 수 있음). 가능하면 일반 Ubuntu 노트북 권장.

## 4. 실험 전 노드 관점 체크
1. 로봇 Undock (절전으로 센서 꺼짐 방지) → `preflight.sh` 1) 에 `no data` 없음
2. `cmd_vel_unstamped` 구독자 존재 (create3_repub) → preflight 3)
3. 컨트롤러 연결·B(EStop) 동작 확인 — 저속에서 한 번 눌러 보기
4. 다른 사람이 `teleop` 을 띄워 두지 않았는지 (`ros2 node list`)
5. 노트북 1 대만 `follow_object_node` 실행 (두 대가 동시에 명령하면 섞임)
