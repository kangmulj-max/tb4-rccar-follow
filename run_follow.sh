#!/bin/bash
# [터미널 2] 추종 노드 실행
#   bash run_follow.sh 11            yolo11n_amr_best.pt, v3 원본 (수정 없음, 파라미터/리맵만)
#   bash run_follow.sh 12            yolo12n_amr_best.pt
#   bash run_follow.sh 11 -p target_dist:=0.8 -p max_lin:=0.2   (추가 파라미터)
#   NODE=v3safe bash run_follow.sh 11   → 안전 수정본 follow_object_node_v3_safe.py
source "$(dirname "$0")/env.sh"
source ~/yolo_venv/bin/activate
V=${1:-11}; shift || true
MODEL=$TB4SIM/weights/yolo${V}n_amr_best.pt
[ -f "$MODEL" ] || { echo "no model: $MODEL"; exit 1; }
echo "model: $MODEL  node: ${NODE:-v3}"
cd "$TB4SIM/logs" 2>/dev/null || { mkdir -p "$TB4SIM/logs"; cd "$TB4SIM/logs"; }
if [ "${NODE:-v3}" = v3safe ]; then
  exec python "$TB4SIM/scripts/follow_object_node_v3_safe.py" --ros-args \
    -p model:="$MODEL" -p ns:=/robot1 -p use_sim_time:=true \
    -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static "$@"
fi
exec python "$TB4SIM/scripts/follow_object_node_v3.py" --ros-args \
  -p model:="$MODEL" -p ns:=/robot1 -p target:=car \
  -p lidar_offset_deg:=-90.0 \
  -p use_sim_time:=true \
  -r /robot1/cmd_vel:=/robot1/cmd_vel_unstamped \
  -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static \
  "$@"
