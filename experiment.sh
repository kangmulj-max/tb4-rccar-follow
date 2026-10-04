#!/bin/bash
# [터미널 2] 시나리오 주행실험 1회 자동 실행 (run_sim.sh 가 떠 있어야 함)
#   bash experiment.sh <scenario> <11|12> [추가 노드 파라미터 ...]
#   예) bash experiment.sh square 11
#       bash experiment.sh pillar 12 -p lidar_offset_deg:=0.0      ← v3 기본값 재현
# scenario:
#   square   : RC카 정사각형 0.15 m/s, 60 s           (기본 추종 성능)
#   random   : RC카 무작위 0.20 m/s, 60 s             (기본 추종 성능)
#   stopgo   : RC카 0.25 m/s 주행/급정지 반복, 40 s   (급정지 → 근접·추돌)
#   approach : RC카가 로봇 쪽으로 후진, 로봇 뒤 0.5 m 기둥, 30 s (후방 미감지 후진)
#   pillar   : 추종 경로 옆을 기둥이 4.5 cm 침범, 35 s  (라이다 정면 섹터)
#   lowbox   : 추종 경로에 높이 0.1 m 박스, 35 s       (라이다 평면 아래 물체)
#   camdrop  : 추종 중 20 s 지점 카메라 3 s 정지, 45 s (센서 끊김)
#   pole     : RC카 1.5 m 앞 정지 + 0.45 m 앞 5 cm 폴 → 라이다 정지 동작 A/B (offset 0 vs -90)
#   depthstop: RC카 2.6 m 앞 정지, 접근 중 depth 무효 → 이전 거리로 계속 전진하는지
#   lost     : RC카 급선회로 시야 이탈 반복, 45 s      (REID/SEARCH 동작)
echo "[exp] $*"
cd "$(dirname "$0")"; source ./env.sh
SC=${1:?scenario}; V=${2:-11}; shift 2 || true; EXTRA=("$@")
NODE=${NODE:-v3}; export NODE
TAG=${TAG:-${NODE}_${SC}_y${V}$( [ ${#EXTRA[@]} -gt 0 ] && echo _custom )}
mkdir -p logs
SP() { gz service -s /world/follow_arena/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
       --req "name: \"$1\", position: {x: $2, y: $3, z: ${5:-0}}, orientation: {z: $(python3 -c "import math;print(math.sin(${4:-0}/2))"), w: $(python3 -c "import math;print(math.cos(${4:-0}/2))")}" >/dev/null; }
stop_robot() { for i in 1 2 3; do ros2 topic pub -1 /robot1/cmd_vel_unstamped geometry_msgs/msg/Twist "{}" >/dev/null 2>&1; done; }
car_stop() { ros2 topic pub -1 /rc_car/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true; }

# ---- 초기 배치 (로봇 x=-2.5 → +x 방향, RC카 1.2 m 앞, 도크/기둥/박스는 치움)
car_stop
SP robot1/standard_dock -3.6 -2.6 1.57; SP pillar 6 5; SP pole 6 -5; SP dummy_box 3.0 2.4 0.3
SP robot1/turtlebot4 ${RX:--2.5} ${RY:-0} 0 0.01; SP rc_car ${CX:--1.3} ${CY:-0} 0   # RX RY CX CY 로 바꿀 수 있음 (SLAM 월드 등)
PATTERN=square; SPEED=0.15; DUR=60
case $SC in
  square)   ;;
  random)   PATTERN=random; SPEED=0.20 ;;
  stopgo)   PATTERN=stopgo; SPEED=0.25; DUR=40 ;;
  approach) PATTERN=approach; SPEED=0.15; DUR=30; SP pillar -3.15 0 ;;      # 로봇 후면에서 0.5 m
  pillar)   PATTERN=line; SPEED=0.12; DUR=35; SP pillar -0.4 0.27 ;;         # 로봇 진로(±0.171) 4.5 cm 침범
  lowbox)   PATTERN=line; SPEED=0.12; DUR=35; SP dummy_box -0.4 0.2 0 ;;     # 높이 0.1 m, 진로 침범
  camdrop)  ;;
  lost)     PATTERN=dash; SPEED=0.15; DUR=45 ;;
  pole)     PATTERN=none; DUR=20; SP rc_car -1.0 0 0; SP pole -2.05 0.10 ;;  # RC카 1.5 m 앞 정지, 0.45 m 앞 폴(진로·라이다 섹터 안, 시선 밖)
  depthstop) PATTERN=none; DUR=20; SP rc_car 0.1 0 0 ;;                      # RC카 2.6 m 앞 정지, 접근 중 depth 상실
  depthloss) PATTERN=stopgo; SPEED=0.2; DUR=30 ;;                           # 10 s 지점부터 depth 전부 무효
  shutdown) DUR=25 ;;                                                        # 15 s 지점 주행 중 노드 SIGINT
  *) echo "unknown scenario $SC"; exit 1 ;;
esac
sleep 1.5; echo "[exp] placed (pattern=$PATTERN speed=$SPEED dur=$DUR)"

# ---- 모니터 먼저 (노드의 첫 상태 전환까지 기록)
python3 scripts/experiment_monitor.py --ros-args -p tag:="$TAG" -p duration:="$((DUR + 8)).0" -p world:="$TB4SIM/worlds/${WORLD:-follow_arena}.sdf" > logs/${TAG}_monitor.log 2>&1 &
MON=$!; REC=
[ "${RECORD:-1}" = 1 ] && { python3 scripts/recorder.py --ros-args -p tag:="$TAG" -p duration:="$((DUR + 8)).0"     -p label:="$NODE | yolo${V}n | $SC ${EXTRA[*]}" > logs/${TAG}_rec.log 2>&1 & REC=$!; }
sleep 2
# ---- 노드 실행 (enable=true)
NODELOG=logs/${TAG}_node.log
bash run_follow.sh "$V" -p enable:=true "${EXTRA[@]}" > "$NODELOG" 2>&1 &
for i in $(seq 1 60); do grep -q "camera_info ok" "$NODELOG" 2>/dev/null && break; sleep 0.5; done
grep -q "camera_info ok" "$NODELOG" || { echo "node not ready"; tail -20 "$NODELOG"; exit 1; }
sleep 2

# ---- RC카 구동
python3 scripts/rc_car_driver.py --ros-args -p pattern:=$PATTERN -p speed:=$SPEED -p start_delay:=2.0 > logs/${TAG}_driver.log 2>&1 &
DRV=$!
if [ "$SC" = camdrop ]; then
  sleep 20; AD=$(pgrep -f tb4_real_iface_adapter.py | head -1)
  echo "[camdrop] camera frozen 3 s"; kill -STOP "$AD"; sleep 3; kill -CONT "$AD"
fi
if [ "$SC" = depthstop ]; then
  sleep 1.5; echo "[depthstop] depth -> all invalid (while approaching)"; ros2 param set /tb4_real_iface_adapter depth_valid false
fi
if [ "$SC" = depthloss ]; then
  sleep 10; echo "[depthloss] depth -> all invalid"; ros2 param set /tb4_real_iface_adapter depth_valid false
fi
if [ "$SC" = shutdown ]; then
  sleep 15; NP=$(pgrep -f "python.*scripts/follow_object_node_v3" | tail -1)
  echo "[shutdown] SIGINT to node pid=$NP while driving"; kill -INT "$NP"; sleep 3
  pgrep -f "python.*scripts/follow_object_node_v3" >/dev/null && echo "[shutdown] node still alive after SIGINT" || echo "[shutdown] node exited"
  tail -5 "$NODELOG" | cut -c1-160
fi
wait $MON || true

# ---- 정리: enable=false → 노드 종료 → RC카/로봇 정지
ros2 param set /follow_object_node enable false >/dev/null 2>&1 || true
kill -TERM $DRV 2>/dev/null || true; pkill -TERM -f "scripts/follow_object_node_v3" || true
car_stop; stop_robot
[[ "$SC" == depth* ]] && ros2 param set /tb4_real_iface_adapter depth_valid true >/dev/null 2>&1
[ -n "$REC" ] && { wait $REC; ffmpeg -y -loglevel error -i logs/videos/${TAG}_raw.mp4 -c:v libx264 -pix_fmt yuv420p -crf 26 logs/videos/${TAG}.mp4 && rm -f logs/videos/${TAG}_raw.mp4; }
sed -n '/=====/,$p' logs/${TAG}_monitor.log
grep -aE "WARN" logs/${TAG}_monitor.log | sed 's/.*\]: //' | head -15
