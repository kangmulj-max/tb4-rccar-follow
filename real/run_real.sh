#!/bin/bash
# 실기 robot1 에서 v3_safe 노드 실행 (노트북). 시뮬용 run_follow.sh / env.sh 를 쓰지 말 것.
#   bash real/run_real.sh 12                      yolo12n, 기본 저속 max_lin 0.1
#   bash real/run_real.sh 11 -p max_lin:=0.2      추가 파라미터
#   NS=/robot2 bash real/run_real.sh 12           다른 로봇
# 사전: 로봇과 같은 ROS 환경을 source (ROS_DOMAIN_ID=1, 디스커버리 서버 등). 먼저 bash real/preflight.sh 권장.
# 시작은 enable=false (계산·디버그 영상만). 주행: ros2 param set /follow_object_node enable true
HERE="$(cd "$(dirname "$0")/.." && pwd)"
V=${1:-12}; shift || true
NS=${NS:-/robot1}
MODEL=$HERE/weights/yolo${V}n_amr_best.pt
[ -f "$MODEL" ] || { echo "가중치 없음: $MODEL"; exit 1; }
[ -n "${ROS_DISTRO:-}" ] || { echo "ROS 환경이 없습니다: source /opt/ros/jazzy/setup.bash (+ 로봇 네트워크 설정)"; exit 1; }
if [ "${ROS_DOMAIN_ID:-0}" = 30 ]; then echo "ROS_DOMAIN_ID=30 은 시뮬용입니다 — 실기 설정을 source 하세요"; exit 1; fi
echo "[실기] ns=$NS model=$(basename "$MODEL") ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} DISCOVERY=${ROS_DISCOVERY_SERVER:-없음}"
unset GALLIUM_DRIVER MESA_D3D12_DEFAULT_ADAPTER_NAME          # 시뮬 전용 설정이 남아 있으면 제거
export YOLO_AUTOINSTALL=False                                 # numpy 2.x 자동 설치 방지 (cv_bridge 보호)
[ -f ~/yolo_venv/bin/activate ] && source ~/yolo_venv/bin/activate
mkdir -p ~/tb4_real_logs; cd ~/tb4_real_logs
exec python3 "$HERE/scripts/follow_object_node_v3_safe.py" --ros-args \
  -p model:="$MODEL" -p ns:="$NS" -p max_lin:=0.1 \
  -r /tf:="$NS/tf" -r /tf_static:="$NS/tf_static" \
  "$@"
