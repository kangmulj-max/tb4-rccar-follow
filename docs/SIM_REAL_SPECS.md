# 시뮬을 실측값에 맞춤 (2026-10-05)

> 가즈보 시뮬의 치수·카메라·지연·depth 를 지금까지 잰 실측값으로 바꿨다. **기본이 실측값**이고, 예전처럼 이상적인 조건이 필요하면 `ideal:=true`.

## 바꾼 값
| 항목 | 이전 시뮬 | 실측 → 지금 시뮬 | 출처 | 파일 |
|---|---|---|---|---|
| RC카 크기 (길이 × 폭 × 높이) | 0.28 × 0.15 × 0.14 m (바퀴 포함 외곽) | **0.25 × 0.08 × 0.085 m** | 10/4 줄자 (`measurements/field_env.sh`) | `worlds/follow_arena*.sdf` (`scripts/apply_real_specs.py` 로 축별 비율 조정) |
| 로봇 카메라 수평 화각 | 64° (camera_info fx 564.65 기준) | **42.4°** (fx 909) | 10/3 회전 시험 회귀 (`REAL_TRIAL_261003.md` 5.3) | `setup.sh` → overlay `oakd.urdf.xacro` |
| 영상 지연 (인지) | 0 | **0.4 s** | 10/3 | `scripts/tb4_real_iface_adapter.py` |
| 명령 → 바퀴 지연 | 0 | **0.2 s** | 10/3 (0.18~0.20 s) | 〃 |
| Create3 제자리 회전 최소 | 없음 | **0.26 rad/s** | 10/3 | 〃 |
| depth 거리 오차 | 없음 (정확) | **+0.0595·(d² − 1) m**, 칸마다 잡음 σ 0.008·d | 10/2 1차 실험: 0.9→0.861, 1.0→1.000, 2.1→2.248, 3.76→4.542 m | 〃 |
| TB4 높이 (웹캠 계산) | 0.351 m | **0.355 m** (반지름 0.170) | 10/4 줄자 | `scripts/webcam_geom.py` |
| RC카 크기 (웹캠 계산) | 0.28 × 0.14 × 0.14 | **0.25 × 0.08 × 0.085** | 10/4 줄자 | `scripts/webcam_geom.py` |
| 실험 점수 계산의 로봇 반경·화각 | 0.171 m · 64° | **0.170 m · 42.4°** | 10/4 줄자, 10/3 | `scripts/experiment_monitor.py` |
| 시험장 | 8 × 6 m 펜스 | **실물 SLAM 지도 벽** (`follow_arena_slam1004`) | 10/4 SLAM | `docs/SLAM_WORLD.md` |
| 카메라 위치 | base 중심 뒤 0.0596 m (URDF) | 그대로 — 줄자로 범퍼 뒤 0.231 m = 중심 뒤 0.061 m 와 1.5 mm 차이 | 10/3 줄자 | 변경 없음 |

- RC카 폭 8 cm 는 실측 메모의 "밑면 세로 8 cm" 를 차 전체 외곽 폭으로 본 것 (바퀴 포함 여부는 확인 필요).
- depth 오차 식은 4 개 거리의 최소제곱 근사. 0.9 m 와 1.0 m 의 부호가 달라 근거리 오차는 거칠게 맞춘 것이다.
- 추종 노드(`follow_object_node_v3_safe.py`, 1안)는 바꾸지 않았다.

## 같이 고친 버그 (모두 "다른 월드를 띄워도 기본 월드를 씀")
| 파일 | 문제 | 고침 |
|---|---|---|
| `launch/follow_sim.launch.py` | 시뮬 라이다가 항상 `follow_arena.sdf` 로 레이캐스트 | 고른 `world_file` 사용 |
| `scripts/experiment_monitor.py` | 실험 점수(전방 여유·충돌 판정)가 항상 기본 월드 기준 | `experiment.sh` 의 `WORLD` 로 받음 |
| `stop_all.sh` | `tb4_follow_sim` 경로만 정리 → 다른 폴더 사본·강제 종료한 launch 의 자식이 고아로 남음 (10/4 밤 157 개, WSL 부하 232) | 자기 폴더 기준 + 같은 ROS_DOMAIN_ID 고아 정리 |

## 확인 (10/5 00:00, 고아 정리 후 깨끗한 환경)
| 확인 | 결과 |
|---|---|
| camera_info | fx **907** → 화각 42.4° (실측 909) |
| depth (차 뒷면 2.1 m) | 시뮬 **2.285 m** ↔ 실물 2.248 m |
| depth (1.0 m) | 시뮬 0.929 m — 측정 창에 바닥 칸이 섞인 값. 식 자체는 1.0 m 에서 오차 0 (실물 1.000) |
| 라이다 ↔ 실물 지도 | 10 cm 이내 99.5 % |
| 추종 (1안, yolo12n, SLAM 월드, square 60 s) | 차가 화면 안 95.6 %, TRACKING 41 s / REID 13 s, **RC카와 충돌 3 회** (끝 무렵), 최소 전방 여유 0.157 m |

**충돌 3 회에 대해**: 실측 지연(0.6 s)과 작아진 차(높이 8.5 cm — 라이다 평면 0.19 m 아래라 라이다로는 안 보임)에서 생겼다. 이 시뮬 실행은 노드 기본 최대 속도 **0.3 m/s** 였고, 실물 `run_real.sh` 는 **0.1 m/s** 로 시작한다. 실물에서도 속도를 올릴 때는 이 조건을 먼저 시뮬로 확인할 것 (1회 실행이라 원인 분리는 아직).

## 주의 — 이전 시뮬 결과와 비교할 때
- 이 변경 전 결과(64° 화각, 지연 0, 큰 차)와 숫자를 직접 비교하지 말 것. 비교하려면 `ideal:=true` 로 다시 돌린다.
- 1안·2안 시뮬 비교(`PLAN1_PLAN2.md`)는 고아 프로세스가 쌓여 CPU 부하가 높던 때 돌린 것이라 제어 주기가 낮았다 (3.2 → 1.4 Hz). 그 비교는 참고만.

## 쓰는 법
```bash
bash setup.sh                                   # overlay 다시 만들기 (화각 42.4° 반영) — 처음 한 번
bash run_sim.sh gui:=false world_file:=follow_arena_slam1004 x:=-0.125 y:=-0.003 yaw:=3.1224   # 실측 조건 (기본)
bash run_sim.sh gui:=false ideal:=true          # 예전 이상적 조건
WORLD=follow_arena_slam1004 RX=-3.5 RY=-1.55 CX=-2.3 CY=-1.55 NODE=v3safe bash experiment.sh square 12 -p max_lin:=0.1   # 실물과 같은 속도
python3 scripts/apply_real_specs.py worlds/<다른 월드>.sdf      # 다른 월드의 RC카도 실측 크기로
```
