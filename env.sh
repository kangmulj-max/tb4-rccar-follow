# source ~/tb4_follow_sim/env.sh  — 모든 터미널에서 먼저 실행
set +u
source /opt/ros/jazzy/setup.bash
export TB4SIM=$HOME/tb4_follow_sim
# 스냅샷 카메라 사양 오버레이(turtlebot4_description) 를 /opt/ros 보다 먼저 찾도록
export AMENT_PREFIX_PATH=$TB4SIM/overlay:$AMENT_PREFIX_PATH
# 실기 로봇(ROS_DOMAIN_ID=1)·Isaac 세팅(102)과 섞이지 않게 별도 도메인.
# 주의: ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST 는 쓰지 말 것 — 시뮬 프로세스가 ~60개라
#       로컬 유니캐스트 피어 범위를 넘은 노드끼리 서로 못 찾음(어댑터가 카메라 0 Hz 로 확인됨)
export ROS_DOMAIN_ID=30
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_BUILTIN_TRANSPORTS
# WSL 기본 OpenGL 은 llvmpipe(CPU) → 카메라 ~1 Hz. d3d12 로 RTX GPU 렌더링 강제
# (WSL 일 때만: /dev/dxg 존재. 일반 Ubuntu 에서는 설정하면 렌더링이 깨질 수 있음)
if [ -e /dev/dxg ]; then
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
fi
# ultralytics 의 패키지 자동 설치 끔: ONNX 내보내기 때 onnx 계열을 설치하며 numpy 를 2.x 로 올려
# cv_bridge(numpy 1.x 빌드)가 깨지는 것을 이 세션에서 확인함. 필요한 패키지는 직접 설치할 것
export YOLO_AUTOINSTALL=False
