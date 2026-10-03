#!/bin/bash
# 새 PC 한 번에 설치 (Ubuntu 24.04 — WSL2 또는 일반 설치 모두)
#   bash install_new_pc.sh
# 하는 일: ROS 2 Jazzy(+TurtleBot4 시뮬레이터) apt 설치 → ~/yolo_venv (이 프로젝트와 같은 버전 고정) →
#          이 폴더를 ~/tb4_follow_sim 으로 복사(이미 그 위치면 생략) → setup.sh (카메라 오버레이)
# sudo 비밀번호를 물을 수 있음. 인터넷 필요 (약 3~5 GB 다운로드).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
. /etc/os-release
[ "$VERSION_CODENAME" = noble ] || { echo "Ubuntu 24.04(noble) 가 필요합니다 (지금: $PRETTY_NAME)"; exit 1; }
[ -e /dev/dxg ] && echo "[info] WSL2 감지 — GPU 렌더링은 env.sh 가 d3d12 로 설정" || echo "[info] 일반 Ubuntu"

# ---- 1) ROS 2 Jazzy apt 저장소 (공식 ros-apt-source 패키지 방식) ----
if [ ! -d /opt/ros/jazzy ]; then
  sudo apt-get update && sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-jazzy-desktop ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-desktop \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt-image-view ros-jazzy-irobot-create-msgs \
  mesa-utils python3-venv python3-pip ffmpeg git
echo "[ok] ROS 2 Jazzy + TurtleBot4 simulator"

# ---- 2) YOLO 파이썬 환경 (ROS 패키지를 같이 쓰도록 --system-site-packages) ----
if [ ! -x ~/yolo_venv/bin/python ]; then python3 -m venv --system-site-packages ~/yolo_venv; fi
. ~/yolo_venv/bin/activate
pip install -q --upgrade pip
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  pip install -q torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124
else
  echo "[info] NVIDIA GPU 없음 → CPU 용 torch"
  pip install -q torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cpu
fi
pip install -q ultralytics==8.4.171 "numpy<2" lap      # 가중치를 학습한 버전. numpy 2.x 는 ROS cv_bridge 와 충돌
python -c "import torch, ultralytics, numpy; print('[ok] torch', torch.__version__, 'cuda', torch.cuda.is_available(), 'ultralytics', ultralytics.__version__, 'numpy', numpy.__version__)"
deactivate

# ---- 3) 프로젝트 위치 ----
if [ "$HERE" != "$HOME/tb4_follow_sim" ]; then
  if [ -e ~/tb4_follow_sim ]; then echo "[warn] ~/tb4_follow_sim 이 이미 있어 복사 생략 (지금 폴더: $HERE)"
  else cp -a "$HERE" ~/tb4_follow_sim && echo "[ok] 복사 → ~/tb4_follow_sim"; fi
fi
cd ~/tb4_follow_sim
chmod +x *.sh scripts/*.py scripts/*.sh real/*.sh real/*.py 2>/dev/null || true
bash setup.sh
echo
echo "완료. 다음:  bash ~/tb4_follow_sim/run_sim.sh      (새 터미널)  NODE=v3safe bash ~/tb4_follow_sim/run_follow.sh 12"
