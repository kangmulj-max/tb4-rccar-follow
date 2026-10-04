#!/bin/bash
# 비교 매트릭스: 시나리오 × (v3, v3safe) × (yolo11n, yolo12n), 전부 녹화. run_sim.sh 가 떠 있어야 함.
cd "$(dirname "$0")"
SCS=${SCS:-"square random stopgo lost pillar lowbox pole approach camdrop depthstop depthloss shutdown"}
for S in $SCS; do
  for N in v3 v3safe; do
    for V in ${VERS:-11 12}; do
      # RESUME=1: 이미 끝난 조합(.done 표시 — 중간에 죽은 실행도 요약 json 은 남으므로 json 으로 판단하지 않음)은 건너뜀 — 중간에 끊긴 배치를 이어서 돌릴 때
      [ "${RESUME:-0}" = 1 ] && [ -f logs/${N}_${S}_y${V}.done ] && { echo "######## skip $N $S $V (done)"; continue; }
      echo "######## $N $S $V  $(date +%T)"
      NODE=$N bash experiment.sh $S $V 2>&1 | grep -aE "=====|events:|\[shutdown\]|\[depth|node not ready"
      touch logs/${N}_${S}_y${V}.done
    done
  done
done
# 원본 그대로(lidar_offset_deg=0) 의 정면 폴 — 비교 영상용
NODE=v3 TAG=v3_pole_y11_offset0 bash experiment.sh pole 11 -p lidar_offset_deg:=0.0 2>&1 | grep -aE "=====|events:"
echo "######## done $(date +%T)"
