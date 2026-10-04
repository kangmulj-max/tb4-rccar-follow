#!/bin/bash
# 시뮬/노드 전부 종료 (이 폴더에서 띄운 프로세스와 그 자식만)
desc() { for c in $(pgrep -P "$1"); do desc "$c"; echo "$c"; done; }
pkill -INT -f "tb4_follow_sim/scripts/" ; sleep 1
for p in $(pgrep -f "tb4_follow_sim/launch/follow_sim.launch.py"); do
  kids=$(desc "$p"); kill -INT "$p" 2>/dev/null; sleep 3
  kill -9 $kids "$p" 2>/dev/null
done
pkill -9 -f "gz sim follow_arena" ; pkill -9 -f "tb4_follow_sim/scripts/"
sleep 1; ros2 daemon stop >/dev/null 2>&1
# 강제 종료로 남은 FastDDS 공유메모리 좀비 정리 (남아 있으면 재시작 후 이미지 토픽 수신 불가)
fastdds shm clean >/dev/null 2>&1
echo "remaining: $(pgrep -f 'follow_sim.launch|gz sim follow_arena|tb4_follow_sim/scripts' | wc -l)"
