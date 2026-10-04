# 실기 실험 기록 키트 (robot1, 노트북 실행)

시뮬(tb4_follow_sim)과 같은 형식으로 실기 시도를 남기기 위한 도구. 노트북으로 이 폴더만 복사하면 됨
(필요: ROS 2 Jazzy, python3-opencv, numpy, ffmpeg, irobot_create_msgs).

## 시도 1회 절차
1. 실기 ROS 환경을 source 한 터미널 3개 (ROS_DOMAIN_ID=1, 디스커버리 서버 설정 그대로)
2. 터미널 A: 추종 노드 실행 (`enable:=false` 로 시작, 첫 시도는 `-p max_lin:=0.1`)
3. 터미널 B: `bash record_real.sh 261003-1430 "yolo12n square 첫 시도"`
   → `~/tb4_real_logs/261003-1430/` 에 bag, 디버그 영상, 텔레메트리, 노드 파라미터 저장 시작
4. 휴대폰으로 외부 촬영 시작, 시도 ID 를 소리 내 말하기 (영상 동기화용)
5. 'TB4 추종 실험일지' 의 D. 출발 전 안전 확인 → `ros2 param set /follow_object_node enable true`
6. RC카 조종 → 끝나면 `enable false` → 터미널 B 에서 Ctrl+C
7. 출력된 summary 의 `for_trial_log`(놓침 횟수·놓친 시간·후진·경고)를 실험일지에 옮겨 적고 저장

## 폴더 내용 (시도 ID 마다)
| 파일 | 내용 |
|---|---|
| `video.mp4` | 왼쪽 디버그 화면(`/robot1/follow/image`, 없으면 압축 RGB) + 오른쪽 패널(상태, cmd/odom, 전·후방 통로 여유, 라이다 탑뷰, 최근 이벤트) |
| `telemetry.csv` | 10 Hz: state, cmd_lin/ang, odom_v/w, front/rear_clear, img_age, scan_age, bump, battery |
| `events.csv` | 상태 전환, 노드 경고(camera timeout 등), 범퍼, 카메라/스캔 끊김 |
| `summary.json` | 상태별 시간, 놓침 횟수, 범퍼, 후진 거리, 최소 전방 여유, 기록지용 값 |
| `bag/` | 압축 RGB, compressedDepth, camera_info, scan, odom, imu, tf, cmd_vel(2종), battery, hazard, rosout (mcap) |
| `meta.json`, `node_params.yaml`, `topics.txt` | 재현용 환경·노드 파라미터·토픽 목록 |

옵션: `NO_BAG=1` (bag 생략), `BAG_RAW=1` (원본 영상까지 bag — Wi-Fi 부하 큼), `NS=/robot2`.

웹캠(별도 PC): `webcam_rec.py` 를 웹캠 PC 에서 같은 시도 ID 로 실행. 보정·분석·두 PC 절차는 `docs/WEBCAM_2PC_GUIDE.md`. `WEBCAM=1` 이면 웹캠 PC 가 `--publish` 로 보내는 미리보기도 bag 에 기록.

## 시뮬과 비교할 때
- 시나리오 이름(line/square/random/stopgo/lost/pillar/lowbox/pole/approach)과 노드 상태 지표(놓침 횟수, TRACKING 비율)는 시뮬과 같은 정의.
- 실기는 RC카 위치 ground truth 가 없으므로 "최대 이격·최소 거리"는 외부 영상/눈대중, depth 거리는 디버그 화면의 `d=` 값.
- bag 을 오프라인 재생하면 같은 장면으로 yolo11n/12n 을 다시 비교할 수 있음 (`ros2 bag play bag/` + `enable:=false`).
