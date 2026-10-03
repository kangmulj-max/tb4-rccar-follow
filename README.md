# tb4_follow_sim — TurtleBot4 RC카 추종 (v3 / v3_safe) 시뮬·실기 실험 키트

TurtleBot4(robot1)가 움직이는 검은 RC카를 YOLO 로 찾아 1 m 거리로 따라가는 노드와, 이를 가즈보에서 실기와 같은 인터페이스로
검증·녹화·비교하는 환경. 실기 기록 도구와 공유 실험일지까지 포함.

## 먼저 읽을 문서
| 문서 | 내용 |
|---|---|
| `REPORT.md` | 최종 결과 요약 (비교표, 결론, 실기 주의점) |
| `docs/CODE_REVIEW_GUIDE.md` | 기초 개념부터 구조, 결함(B)·환경 오류(E)·측정 정정(M) 목록, 리뷰 체크리스트 |
| `docs/FOLLOW_NODE_EXPLAINED.md` | 추종 원리 단계별 설명, 문헌 대조, 참고문헌 |
| `docs/YOLO_COMPARISON.md` | yolo11n vs yolo12n 공식 자료 ↔ 실측 대조, 다른 모델 추가 절차 |
| `real/README.md` | 실기 시도 기록 절차 |

## 노드
| 파일 | 설명 |
|---|---|
| `scripts/follow_object_node_v3.py` | 원본 (수정 금지) |
| `scripts/follow_object_node_v3_annotated.py` | 원본 + 리뷰 주석. `python3 scripts/check_annotated_identical.py` 로 코드 동일 검증 |
| `scripts/follow_object_node_v3_safe.py` | 안전 수정본 (S1~S11), 한 줄씩 주석 |

## 설치 (WSL Ubuntu 24.04 / ROS 2 Jazzy)
```bash
sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-desktop ros-jazzy-teleop-twist-keyboard mesa-utils python3-venv ffmpeg
python3 -m venv --system-site-packages ~/yolo_venv && . ~/yolo_venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
pip install ultralytics "numpy<2" lap          # numpy 2.x 는 ROS cv_bridge 와 충돌
bash setup.sh                                   # 카메라 오버레이(704x704) + 가중치/노드 복사 (weights/ 는 git 미포함, 따로 공유)
```

## 실행
```bash
bash run_sim.sh                                 # 터미널 1: 가즈보 (gui:=false 헤드리스, world_file:=follow_arena_bright 밝은 조명)
NODE=v3safe bash run_follow.sh 12               # 터미널 2: 노드 (NODE 생략 = 원본 v3, 11 = yolo11n)
source env.sh; ros2 param set /follow_object_node enable true      # 터미널 3: 주행 시작
python3 scripts/rc_car_driver.py --ros-args -p pattern:=square     # RC카 (line square circle figure8 stopgo zigzag random approach dash)
ros2 run rqt_image_view rqt_image_view /robot1/follow/image        # 디버그 영상
bash stop_all.sh                                # 전부 종료
```

## 자동 실험·비교
```bash
NODE=v3safe bash experiment.sh <scenario> <11|12> [-p 파라미터:=값]   # 1회 실험 + 녹화 (logs/videos/)
bash run_batch2.sh                              # 전체 매트릭스 (시나리오 12 × 노드 2 × 모델 2)
python3 scripts/summarize2.py                   # logs/compare.md 비교표
bash scripts/make_compare_videos.sh             # logs/videos/compare/ 비교 영상
python scripts/model_bench.py weights/*.pt [--sweep --light dim|bright] [--onnx]   # 모델 벤치
```
시나리오: square random stopgo lost (추종) / pillar lowbox pole (장애물) / approach (타깃 접근·후진) / camdrop depthstop depthloss (센서 이상) / shutdown (종료 시 정지)

## 시뮬 ↔ 실기 인터페이스
| 항목 | 실기 robot1 (스냅샷 2026-10-02) | 이 시뮬 |
|---|---|---|
| RGB | `oakd/rgb/image_raw` 704×704 bgr8 10 Hz, fx 564.65 | 동일 형식 ~9.5 Hz, fx 564.45 (어댑터) |
| Depth | `oakd/stereo/image_raw` 16UC1 mm | 동일 (어댑터 변환) |
| cmd | `cmd_vel` TwistStamped, `cmd_vel_unstamped` Twist | 동일 (어댑터가 unstamped→stamped) |
| 라이다 | `scan` 640, rplidar_link +90° 장착 | 기하 레이캐스트 (gz 라이다 WSL 고장 대체) |
| 렌즈 왜곡 | 강함, 미보정 | 없음 |
| 통신 | Wi-Fi, 디스커버리 서버, DOMAIN 1 | 로컬, DOMAIN 30 |

## 환경 주의 (WSL)
`env.sh` 가 처리: GPU 렌더링(`GALLIUM_DRIVER=d3d12`), 도메인 격리(LOCALHOST 디스커버리 금지), `YOLO_AUTOINSTALL=False`.
셸 스크립트는 항상 `bash x.sh` 로 (Windows 쪽 편집 시 실행권한이 빠짐). 상세는 CODE_REVIEW_GUIDE 3-3.
