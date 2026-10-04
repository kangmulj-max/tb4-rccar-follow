# 로봇 bringup·카메라 설정 메모 (2026-10-04)

출처: [turtlebot4_robot/turtlebot4_bringup (jazzy)](https://github.com/turtlebot/turtlebot4_robot/tree/jazzy/turtlebot4_bringup), 수업 자료 `kimandreas/to_students` Day2 (60~70 쪽).

## 1. 도크 위에서는 카메라·라이다가 꺼짐
- `config/turtlebot4.yaml` 의 `power_saver: true` → 도크에 올라가 있으면 OAK-D·라이다 정지.
- 시험 전 언도킹, 그 뒤 약 30 초 기다림 (`standard.launch.py` 가 OAK-D 를 30 초 늦게 켬):
  ```bash
  ros2 action send_goal /robot1/undock irobot_create_msgs/action/Undock "{}"
  ros2 action send_goal /robot1/dock   irobot_create_msgs/action/Dock "{}"     # 충전 복귀
  ```
- `real/preflight.sh` 0 단계가 도크 상태를 확인하고 언도킹 명령을 보여 줌.

## 2. 카메라 설정은 기본값이 아님
| | 저장소 기본 `oakd_pro.yaml` | 로봇1 (10/3 실측) |
|---|---|---|
| 파이프라인 | RGB 만 (depth 없음) | RGB + depth (RGB 에 정렬) |
| 발행 영상 | 250×250 미리보기만 (`i_publish_topic: false`) | `rgb/image_raw` 704×704, 10 Hz (camera_info fx 564.65, 실측 fx ≈ 909) |

- 로봇1은 수업(Day2 67 쪽) 방식으로 `/opt/ros/jazzy/share/turtlebot4_bringup/config/oakd_pro.yaml` 을 교체한 상태로 보임 (교체본 `oakd_pro_new.yaml` 은 Notion 에만 있음).
- 기본값으로 되돌아가면(원본 복구, 로봇에서 `apt upgrade`) **depth 가 없어 추종 노드가 동작하지 않음**. 로봇 Pi 는 수정하지 않는다.
- `camera_info` 는 fx = 564.65 를 보내지만 704×704 영상의 실제 fx 는 약 909 (10/3 실측) → 노드는 `fx_override`(`run_real.sh` 의 `FX`, 기본 909)로 덮어씀 (S18).
- `real/preflight.sh` 0 단계가 `camera_info` 가 704×704·fx 564.65 (10/3 과 같은 설정)인지 확인하고, 다르면 경고.

## 3. 수업 예제와 다른 점
- 수업 예제(`day2/2_4_g`, `2_1_d`)는 `rgb/preview/image_raw`(작은 미리보기)를 구독 → 우리 노드(`rgb/image_raw/compressed`, 704×704)와 해상도가 달라 결과를 직접 비교하지 말 것.
- `2_1_e_capture_comp_image.py` 는 compressed 구독 → 우리 S13 과 같은 방식.
- `2_4_g/h` 는 `.onnx`/`.engine` 모델도 읽음 → 노트북 추론이 느릴 때의 선택지 (지금은 불필요).
