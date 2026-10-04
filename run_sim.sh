#!/bin/bash
# [터미널 1] 가즈보 + TurtleBot4(robot1) + 검은 RC카 + 실기 인터페이스 어댑터
#   ./run_sim.sh            (GUI)
#   ./run_sim.sh gui:=false (헤드리스)
source "$(dirname "$0")/env.sh"
exec ros2 launch "$TB4SIM/launch/follow_sim.launch.py" "$@"
