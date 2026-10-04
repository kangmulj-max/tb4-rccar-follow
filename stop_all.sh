#!/bin/bash
# 시뮬/노드 전부 종료 (이 폴더에서 띄운 프로세스와 그 자식 + 같은 ROS_DOMAIN_ID 로 남은 고아 프로세스)
#  10/4: launch 를 강제 종료하면 자식 노드(joint_state_publisher, gz_toolbox 등)가 고아로 남아 쌓임 → WSL 부하 232, 시뮬 시작 불가.
#        폴더 이름이 다른 사본(~/tb4_rt 등)도 정리되도록 경로를 이 파일 위치에서 계산하고, 도메인 기준으로 한 번 더 정리.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh" >/dev/null 2>&1
DOM=${ROS_DOMAIN_ID:-30}
desc() { for c in $(pgrep -P "$1"); do desc "$c"; echo "$c"; done; }
pkill -INT -f "$HERE/scripts/" ; sleep 1
for p in $(pgrep -f "$HERE/launch/follow_sim.launch.py"); do
  kids=$(desc "$p"); kill -INT "$p" 2>/dev/null; sleep 3
  kill -9 $kids "$p" 2>/dev/null
done
pkill -9 -f "gz sim follow_arena" ; pkill -9 -f "$HERE/scripts/"
# 같은 도메인으로 남은 ROS·gz 프로세스 (다른 도메인을 쓰는 다른 작업은 건드리지 않음)
orph=""
for p in $(pgrep -u "$(id -u)" -f "/opt/ros|ros2|gz sim|$HERE"); do
  [ "$p" = "$$" ] && continue
  tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$DOM" && orph="$orph $p"
done
[ -n "$orph" ] && kill -9 $orph 2>/dev/null
sleep 1; ros2 daemon stop >/dev/null 2>&1
# 강제 종료로 남은 FastDDS 공유메모리 좀비 정리 (남아 있으면 재시작 후 이미지 토픽 수신 불가)
fastdds shm clean >/dev/null 2>&1
echo "remaining (domain $DOM): $(for p in $(pgrep -u "$(id -u)" -f '/opt/ros|ros2|gz sim'); do tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$DOM" && echo $p; done | wc -l)"
