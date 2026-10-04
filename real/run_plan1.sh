#!/bin/bash
# 1안 — 품 가장 적게: 10/3 실기에서 돌린 노드(follow_object_node_v3_safe.py) 그대로, 코드 변경 없음.
#   설정은 10/3 가장 오래 안정적으로 돈 시도 T085 와 같게: target_dist 0.85, max_lin 0.1, 압축 영상, fx 909 (S13~S18 기본값)
#   bash real/run_plan1.sh [11|12] [추가 -p ...]       (기본 12)
#   주행: ros2 param set /follow_object_node enable true   /  정지: ... enable false
HERE="$(cd "$(dirname "$0")" && pwd)"
V=${1:-12}; shift || true
exec bash "$HERE/run_real.sh" "$V" -p target_dist:=0.85 "$@"
