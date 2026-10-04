#!/bin/bash
# ROS_DOMAIN_ID=30 (이 시뮬) 프로세스만 정리. 다른 도메인은 건드리지 않음.
n30=0; n31=0; other=0; kill30=""
for p in $(pgrep -u $(id -u) -f "/opt/ros|ros2|gz sim|tb4_rt|tb4_follow_sim|python3"); do
  [ "$p" = "$$" ] && continue
  d=$(tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep '^ROS_DOMAIN_ID=' | cut -d= -f2)
  case "$d" in 30) n30=$((n30+1)); kill30="$kill30 $p";; 31) n31=$((n31+1));; *) other=$((other+1));; esac
done
echo "domain30(mine)=$n30 domain31(other)=$n31 other=$other"
[ -n "$kill30" ] && kill -9 $kill30 2>/dev/null; sleep 3
echo "after: domain30 left $(for p in $(pgrep -u $(id -u) -f '/opt/ros|ros2|gz sim'); do tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -q '^ROS_DOMAIN_ID=30' && echo $p; done | wc -l)"; uptime
