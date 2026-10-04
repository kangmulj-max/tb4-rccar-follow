#!/bin/bash
# Layout Delta 시뮬 테스트 세션: 회차 00~03 (실물 SLAM 월드 + 실측 사양), 회차마다 독립 실행
#   bash ld_session.sh [rounds...]   (기본 0 1 2 3)
set -u
REPO=$(cd "$(dirname "$0")/.." && pwd)   # tb4_follow_sim 저장소 루트
R=~/tb4_rt; OUT=${OUT:-$HOME/ld_session_out}; mkdir -p $OUT
MAP=$R/worlds/maps/arena_1004_run1.yaml
rsync -a --exclude .git --exclude overlay --exclude weights $REPO/ $R/
sed -i 's|TB4SIM=$HOME/tb4_follow_sim|TB4SIM=$HOME/tb4_rt|' $R/env.sh
rm -rf ~/ld_ws/src/layout_delta && cp -r $REPO/layout_delta ~/ld_ws/src/ && touch ~/ld_ws/src/layout_delta/README.md
cd $R; source env.sh >/dev/null 2>&1
(cd ~/ld_ws && colcon build --packages-select layout_delta >/dev/null 2>&1) && source ~/ld_ws/install/setup.bash || { echo "build failed"; exit 1; }
echo "== $(date +%T) session start, load $(cut -d' ' -f1 /proc/loadavg)"
for k in ${@:-0 1 2 3}; do
  RD=$(printf "round%02d" $k); W=$(printf "ld_round%02d" $k); D=$OUT/$RD; rm -rf $D; mkdir -p $D
  bash $REPO/scripts/kill_domain30.sh >/dev/null 2>&1
  echo "== $(date +%T) $RD: sim ($W)"
  setsid bash run_sim.sh gui:=false world_file:=$W x:=-0.9 y:=-0.25 yaw:=3.1416 > $D/sim.log 2>&1 &
  ok=0; for i in $(seq 1 50); do timeout 4 ros2 topic echo --once /robot1/scan --field header.frame_id >/dev/null 2>&1 && { ok=1; break; }; sleep 2; done
  [ $ok = 1 ] || { echo "   sim not ready — skip round"; continue; }
  sleep 5
  up() {   # $1 = lifecycle node, $2 = launch cmd, $3 = log ; 최대 2 회 시도, 각 75 초
    for try in 1 2; do
      setsid bash -c "$2" >> $3 2>&1 &
      for i in $(seq 1 40); do timeout 3 ros2 lifecycle get $1 2>/dev/null | grep -q "^active" && { echo "   $1 active (try $try)"; return 0; }; sleep 3; done
      echo "   $1 not active — relaunch"; pkill -f "$4"; sleep 3
    done; return 1; }
  up /robot1/amcl "ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 map:=$MAP use_sim_time:=true" $D/localization.log "localization.launch.py" || { echo "   localization failed — skip round"; continue; }
  for j in 1 2 3; do   # map 프레임이 있어야 Nav2 가 활성화됨 → 초기 위치를 먼저 (patrol 의 관문이 다시 확인)
    timeout 5 ros2 topic pub --once /robot1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped       "{header: {frame_id: map}, pose: {pose: {position: {x: -0.9, y: -0.25}, orientation: {z: 1.0, w: 0.0}}, covariance: [0.06,0,0,0,0,0, 0,0.06,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}" >/dev/null 2>&1; sleep 2
  done
  up /robot1/bt_navigator "ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1 use_sim_time:=true" $D/nav2.log "nav2.launch.py" || { echo "   nav2 failed — skip round"; continue; }
  setsid python3 $REPO/scripts/loc_eval.py $D 1200 > $D/loc_eval.log 2>&1 &
  echo "   $(date +%T) patrol"
  setsid ros2 launch layout_delta round.launch.py namespace:=/robot1 ref_map:=$MAP use_sim_time:=true \
      waypoints:=$REPO/layout_delta/config/waypoints_arena1004.yaml out_dir:=$D initial_pose:='[-0.9, -0.25, 180.0]' > $D/round.log 2>&1 &
  for i in $(seq 1 180); do [ -f $D/mission.json ] && break; sleep 5; done      # 최대 15 분
  [ -f $D/mission.json ] && python3 -c "import json; m=json.load(open('$D/mission.json')); print('   mission', m['reached'], '/', m['total'], 'wp,', m['round_s'], 's |', m.get('save','')[:120])" || echo "   mission timeout"
  pkill -INT -f "loc_eval.py $D" ; sleep 3; cat $D/loc_eval.json 2>/dev/null | tr -d '\n '; echo
  bash $REPO/scripts/kill_domain30.sh | tail -1
done
echo "== $(date +%T) rounds done"
# 변화 탐지: 회차 n vs n−1
for k in 1 2 3; do
  A=$OUT/$(printf "round%02d" $((k-1)))/map.yaml; B=$OUT/$(printf "round%02d" $k)/map.yaml
  [ -f $A ] && [ -f $B ] || { echo "detect $k: missing map"; continue; }
  for T in lidar all; do
    TF=$R/worlds/rounds_truth/truth_round$(printf %02d $k)$([ $T = lidar ] && echo _lidar).json
    echo "== detect round$(printf %02d $k) vs prev ($T truth)"
    ros2 run layout_delta detect --ref $A --cur $B --out $OUT/report$(printf %02d $k)_$T --truth $TF --roi-waypoints $REPO/layout_delta/config/waypoints_arena1004.yaml 2>&1 | tail -12
  done
done
echo "== $(date +%T) DONE"
