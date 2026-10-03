#!/bin/bash
# 비교 매트릭스: 시나리오 × (v3, v3safe) × (yolo11n, yolo12n), 전부 녹화. run_sim.sh 가 떠 있어야 함.
cd "$(dirname "$0")"
SCS=${SCS:-"square random stopgo lost pillar lowbox pole approach camdrop depthstop depthloss shutdown"}
for S in $SCS; do
  for N in v3 v3safe; do
    for V in 11 12; do
      echo "######## $N $S $V  $(date +%T)"
      NODE=$N bash experiment.sh $S $V 2>&1 | grep -aE "=====|events:|\[shutdown\]|\[depth|node not ready"
    done
  done
done
# 원본 그대로(lidar_offset_deg=0) 의 정면 폴 — 비교 영상용
NODE=v3 TAG=v3_pole_y11_offset0 bash experiment.sh pole 11 -p lidar_offset_deg:=0.0 2>&1 | grep -aE "=====|events:"
echo "######## done $(date +%T)"
