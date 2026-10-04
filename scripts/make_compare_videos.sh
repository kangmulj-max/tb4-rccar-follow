#!/bin/bash
# 비교 영상: (1) 같은 노드에서 yolo11n(위) vs yolo12n(아래)  (2) 같은 모델에서 v3(위) vs v3_safe(아래)
# 결과: logs/videos/compare/*.mp4  (각 1120x480 → 세로로 쌓아 1120x960)
cd ~/tb4_follow_sim/logs/videos; mkdir -p compare
stack() {  # $1 위 $2 아래 $3 출력
  [ -f "$1" ] && [ -f "$2" ] || { echo "skip $3"; return; }
  ffmpeg -y -loglevel error -i "$1" -i "$2" -filter_complex \
    "[0:v]scale=1120:-2,setsar=1[a];[1:v]scale=1120:-2,setsar=1[b];[a][b]vstack=inputs=2:shortest=1" \
    -c:v libx264 -pix_fmt yuv420p -crf 27 "compare/$3.mp4" && echo "ok compare/$3.mp4"
}
for S in square random stopgo lost; do
  for N in v3 v3safe; do stack ${N}_${S}_y11.mp4 ${N}_${S}_y12.mp4 yolo11_vs_12_${N}_${S}; done
done
for S in pillar lowbox pole approach depthstop depthloss camdrop shutdown; do
  stack v3_${S}_y11.mp4 v3safe_${S}_y11.mp4 v3_vs_safe_${S}_y11
done
stack v3_pole_y11_offset0.mp4 v3safe_pole_y11.mp4 v3default_offset0_vs_safe_pole_y11

# (3) 모델 3종 (yolo8n 위 / yolo11n 가운데 / yolo12n 아래) — 같은 노드·시나리오
stack3() {  # $1 $2 $3 → $4
  [ -f "$1" ] && [ -f "$2" ] && [ -f "$3" ] || { echo "skip $4"; return; }
  ffmpeg -y -loglevel error -i "$1" -i "$2" -i "$3" -filter_complex     "[0:v]scale=1120:-2,setsar=1[a];[1:v]scale=1120:-2,setsar=1[b];[2:v]scale=1120:-2,setsar=1[c];[a][b][c]vstack=inputs=3:shortest=1"     -c:v libx264 -pix_fmt yuv420p -crf 27 "compare/$4.mp4" && echo "ok compare/$4.mp4"
}
for S in square random stopgo lost camdrop; do
  for N in v3 v3safe; do stack3 ${N}_${S}_y8.mp4 ${N}_${S}_y11.mp4 ${N}_${S}_y12.mp4 yolo8_11_12_${N}_${S}; done
done
