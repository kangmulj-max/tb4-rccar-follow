#!/bin/bash
# 2안 — 모션 개선: follow_object_node_v3_motion.py (1안 노드 + M1 지연 보상 + M2 가장자리에서 회전 먼저 [+ M3 피드포워드])
#   bash real/run_plan2.sh [11|12] [추가 -p ...]       2a: M1+M2
#   FF=0.5 bash real/run_plan2.sh 12                    2b: + M3 (차 속도 피드포워드 0.5)
#   실행 중 끄기/켜기 (같은 시도 안에서 비교): ros2 param set /follow_object_node lat_comp false / slow_bearing_deg 0.0
HERE="$(cd "$(dirname "$0")" && pwd)"
V=${1:-12}; shift || true
export NODE_FILE=follow_object_node_v3_motion.py; exec bash "$HERE/run_real.sh" "$V" -p target_dist:=0.85 -p ff_gain:=${FF:-0.0} "$@"
