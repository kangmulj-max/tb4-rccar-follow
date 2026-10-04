# turtlebot_gazbo_v3_safe_test — TurtleBot4 RC카 추종: Gazebo 시뮬 검증

TurtleBot4(robot1)가 YOLO 로 움직이는 RC카를 찾아 약 1 m 거리로 따라가는 노드(`follow_object_node_v3`)를
**실물에 올리기 전에 Gazebo 에서 위험 상황을 재현해 검증**한 작업 묶음. 실기 robot1 과 같은 토픽·센서 사양으로 맞춘 시뮬 환경,
안전 수정본 `v3_safe`, YOLO 모델 3 종(yolov8n / yolo11n / yolo12n) 비교, 지금까지의 실험 데이터 전체.

- 이름의 `v3_safe` = 이 폴더의 기준 노드 버전 (원본 v3 + 안전 수정 S1~S12 + 정리 R1)
- **모든 수치는 시뮬레이션 결과**. 실물 주행 데이터는 아직 없음 (`real/` 의 점검·기록 도구로 수집 예정)

## 먼저 볼 것
| 문서 | 내용 |
|---|---|
| [`RESULTS.md`](RESULTS.md) | **실험 데이터 한눈에 보기** — 회차별 결론, 추종·안전 표, 모델 비교, 리팩터 검증 |
| [`results/all_runs.csv`](results/all_runs.csv) | 지금까지 모든 실행(회차·노드·모델·시나리오별) 한 표 — 엑셀로 열림 |
| [`REPORT.md`](REPORT.md) | 3 차 회차(2026-10-03) 최종 보고 |
| [`docs/YOLO_COMPARISON.md`](docs/YOLO_COMPARISON.md) | YOLO 8n·11n·12n 공식 자료 ↔ 실측 대조 (8n 은 10절) |
| [`docs/CODE_REVIEW_GUIDE.md`](docs/CODE_REVIEW_GUIDE.md) | 기초 개념, 결함 B1~B15, 수정 S1~S12·R1, 정정 목록, 리뷰 체크리스트 |
| [`docs/FOLLOW_NODE_EXPLAINED.md`](docs/FOLLOW_NODE_EXPLAINED.md) | 추종 원리와 참고문헌 |
| [`docs/REAL_CODE_STRUCTURE.md`](docs/REAL_CODE_STRUCTURE.md), [`docs/REAL_NODES.md`](docs/REAL_NODES.md) | 실기 실행 구조·구성 노드 |

## 코드
| 파일 | 설명 |
|---|---|
| `scripts/follow_object_node_v3.py` | 원본 v3 (수정 금지, 비교 기준) |
| `scripts/follow_object_node_v3_annotated.py` | 원본 + 리뷰 주석 (코드 동일) |
| `scripts/follow_object_node_v3_safe_annotated.py` | **안전 수정본 주석본 — 리뷰·수정은 이 파일로** |
| `scripts/follow_object_node_v3_safe.py` | 위 파일에서 주석만 뺀 **실행본** (`make_lean.py` 로 자동 생성) |
| `real/` | 실기 출발 전 점검(`preflight.sh`), 실행(`run_real.sh`), 기록(`record_real.sh`) |
| `launch/`, `worlds/`, `run_sim.sh`, `experiment.sh`, `run_batch2.sh` | 시뮬 환경과 시나리오 자동 실험 |
| `weights/` | 이번에 시험한 YOLO 가중치 3 종 (`yolo{8,11,12}n_amr_best.pt`, 같은 데이터·같은 학습 조건) |

코드 동일 확인:
```bash
python3 scripts/check_annotated_identical.py                     # 원본 v3 ↔ 주석본
python3 scripts/check_annotated_identical.py --ignore-docstrings \
  scripts/follow_object_node_v3_safe.py scripts/follow_object_node_v3_safe_annotated.py   # 실행본 ↔ 주석본
```

## 실행 (Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic)
```bash
bash setup.sh                                   # 최초 1 회 (경로는 ~/tb4_follow_sim 기준)
bash run_sim.sh                                 # 터미널 1: 시뮬 (gui:=false 헤드리스)
NODE=v3safe bash run_follow.sh 12               # 터미널 2: 노드 (12 = yolo12n, 8/11 도 가능)
source env.sh; ros2 param set /follow_object_node enable true
NODE=v3safe bash experiment.sh pillar 12        # 시나리오 1 회 자동 실험 + 녹화
VERS="8 11 12" bash run_batch2.sh               # 전체 매트릭스 (끊기면 RESUME=1)
MODELS="8 11 12" python3 scripts/summarize2.py  # 비교표 / python3 scripts/collect_runs.py → results/all_runs.csv
```
데이터셋(`amr_dataset.zip`)과 실행 로그(`logs/`)는 용량 때문에 올리지 않음 — 표·영상·요약은 `results/` 에 있음.
설치·새 PC 세팅·실험일지 등 키트 설명은 [`README_KIT.md`](README_KIT.md).
