#!/bin/bash
# 가상 공장 월별 순회 세션: 공장 F 의 M0~M3 을 같은 경로로 순회 → 회차 지도 → 직전 달 대비 변화 판정
#   bash fac_session.sh f1 [f2 f3]
set -u
P=${LD_WORK:?set LD_WORK to the folder that holds realtrial}   # 작업 폴더 (realtrial 의 상위)
R=~/tb4_rt; OUT=$P/fac_session_out; mkdir -p $OUT
rsync -a --exclude .git --exclude overlay --exclude weights $P/realtrial/ $R/
sed -i 's|TB4SIM=$HOME/tb4_follow_sim|TB4SIM=$HOME/tb4_rt|' $R/env.sh
rm -rf ~/ld_ws/src/layout_delta && cp -r $P/realtrial/layout_delta ~/ld_ws/src/
cd $R; source env.sh >/dev/null 2>&1
(cd ~/ld_ws && colcon build --packages-select layout_delta >/dev/null 2>&1) && source ~/ld_ws/install/setup.bash || { echo "build failed"; exit 1; }
up() {   # $1 lifecycle node, $2 launch cmd, $3 log, $4 pkill pattern
  for try in 1 2; do
    setsid bash -c "$2" >> $3 2>&1 &
    for i in $(seq 1 40); do timeout 3 ros2 lifecycle get $1 2>/dev/null | grep -q "^active" && { echo "   $1 active (try $try)"; return 0; }; sleep 3; done
    echo "   $1 not active — relaunch"; pkill -f "$4"; sleep 3
  done; return 1; }
for F in ${@:-f1 f2 f3}; do
  FD=$R/worlds/factory/$F; MAP=$FD/map_m0.yaml; WP=$FD/waypoints.yaml
  read SX SY SYAW <<< $(python3 -c "import json;s=json.load(open('$FD/spec.json'));print(*[float(v) for v in s['start']])")
  QZ=$(python3 -c "import math;print(math.sin(math.radians($SYAW)/2))"); QW=$(python3 -c "import math;print(math.cos(math.radians($SYAW)/2))")
  HAS_FL=$(python3 -c "import json;print(1 if json.load(open('$FD/spec.json'))['forklift'] else 0)")
  echo "== $(date +%T) $F start ($SX, $SY, $SYAW°), load $(cut -d' ' -f1 /proc/loadavg)"
  for k in ${MONTHS:-0 1 2 3}; do
    D=$OUT/$F/m$k; rm -rf $D; mkdir -p $D
    bash $P/kill_domain30.sh >/dev/null 2>&1
    echo "== $(date +%T) $F M$k"
    setsid bash run_sim.sh gui:=false world_file:=fac_${F}_m$k x:=$SX y:=$SY yaw:=$(python3 -c "import math;print(math.radians($SYAW))") > $D/sim.log 2>&1 &
    ok=0; for i in $(seq 1 60); do timeout 4 ros2 topic echo --once /robot1/scan --field header.frame_id >/dev/null 2>&1 && { ok=1; break; }; sleep 2; done
    [ $ok = 1 ] || { echo "   sim not ready — skip"; continue; }
    sleep 5
    [ $HAS_FL = 1 ] && setsid python3 $P/realtrial/scripts/fac_forklift.py $FD/spec.json > $D/forklift.log 2>&1 &
    up /robot1/amcl "ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 map:=$MAP use_sim_time:=true" $D/localization.log "localization.launch.py" || { echo "   localization failed — skip"; continue; }
    for j in 1 2 3; do
      timeout 5 ros2 topic pub --once /robot1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: map}, pose: {pose: {position: {x: $SX, y: $SY}, orientation: {z: $QZ, w: $QW}}, covariance: [0.06,0,0,0,0,0, 0,0.06,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}" >/dev/null 2>&1; sleep 2
    done
    up /robot1/bt_navigator "ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1 use_sim_time:=true" $D/nav2.log "nav2.launch.py" || { echo "   nav2 failed — skip"; continue; }
    setsid python3 $P/realtrial/scripts/loc_eval.py $D 3600 > $D/loc_eval.log 2>&1 &
    setsid python3 $P/realtrial/scripts/fac_traj.py $D/traj.csv > $D/traj.log 2>&1 &
    echo "   $(date +%T) patrol"
    setsid ros2 launch layout_delta round.launch.py namespace:=/robot1 ref_map:=$MAP use_sim_time:=true \
        waypoints:=$WP out_dir:=$D initial_pose:="[$SX, $SY, $SYAW]" undock_backup_m:=0.35 > $D/round.log 2>&1 &
    for i in $(seq 1 360); do [ -f $D/mission.json ] && break; sleep 5; done      # 최대 30 분
    [ -f $D/mission.json ] && python3 -c "import json; m=json.load(open('$D/mission.json')); print('   mission', m['reached'], '/', m['total'], 'wp,', m['round_s'], 's')" || echo "   mission timeout"
    pkill -INT -f "loc_eval.py $D"; pkill -INT -f "fac_traj.py $D"; pkill -f fac_forklift.py; sleep 3
    bash $P/kill_domain30.sh | tail -1
  done
  for k in ${DMONTHS:-1 2 3}; do
    A=$OUT/$F/m$((k-1))/map.yaml; B=$OUT/$F/m$k/map.yaml
    [ -f $A ] && [ -f $B ] || { echo "detect $F m$k: missing map"; continue; }
    ros2 run layout_delta detect --ref $A --cur $B --out $OUT/$F/report_m$k --truth $FD/truth_m$k.json --roi-waypoints $WP \
        --title "$F M$((k-1))→M$k 변경 리포트" 2>&1 | grep -E '"(tp|fp|fn|precision|recall)"|changes' | tr -d '\n '; echo
  done
done
echo "== $(date +%T) DONE"
