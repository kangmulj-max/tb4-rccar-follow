#!/bin/bash
# 새 PC 한 번에 설치 (Ubuntu 24.04 — WSL2 또는 일반 설치 모두)
#   bash install_new_pc.sh          실기 전용 (기본): ROS 2 Jazzy + TurtleBot4 메시지·도구 + YOLO 환경
#   bash install_new_pc.sh sim      + 가즈보 TurtleBot4 시뮬레이터와 시뮬 카메라 설정(setup.sh)
# 하는 일: apt 설치 → ~/yolo_venv (가중치를 학습한 버전으로 고정) → 이 폴더를 ~/tb4_follow_sim 으로 복사
# sudo 비밀번호를 물을 수 있음. 인터넷 필요 (실기 ~2 GB, 시뮬 포함 ~4 GB).
set -e
MODE=${1:-real}
[ "$MODE" = real ] || [ "$MODE" = sim ] || { echo "사용법: bash install_new_pc.sh [real|sim]"; exit 1; }
echo "[info] 설치 모드: $MODE"
HERE="$(cd "$(dirname "$0")" && pwd)"
. /etc/os-release
[ "$VERSION_CODENAME" = noble ] || { echo "Ubuntu 24.04(noble) 가 필요합니다 (지금: $PRETTY_NAME)"; exit 1; }
[ -e /dev/dxg ] && echo "[info] WSL2 감지" || echo "[info] 일반 Ubuntu"

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
PKGS="ros-jazzy-desktop ros-jazzy-turtlebot4-desktop ros-jazzy-irobot-create-msgs ros-jazzy-rqt-image-view"
PKGS="$PKGS ros-jazzy-teleop-twist-keyboard ros-jazzy-rosbag2-storage-mcap python3-venv python3-pip ffmpeg git"
[ "$MODE" = sim ] && PKGS="$PKGS ros-jazzy-turtlebot4-simulator mesa-utils"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y $PKGS
echo "[ok] ROS 2 Jazzy ($MODE)"

# ---- 2) YOLO 파이썬 환경 (ROS 패키지를 같이 쓰도록 --system-site-packages) ----
if [ ! -x ~/yolo_venv/bin/python ]; then python3 -m venv --system-site-packages ~/yolo_venv; fi
. ~/yolo_venv/bin/activate
pip install -q --upgrade pip
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  pip install -q torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124
else
  echo "[info] NVIDIA GPU 없음 → CPU 용 torch (추론 ~25-35 ms, 10 Hz 카메라에 충분)"
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
if ls weights/*.pt >/dev/null 2>&1; then echo "[ok] 가중치: $(ls weights | tr '\n' ' ')"
else echo "[warn] weights/ 에 가중치(.pt)가 없습니다 — 압축본의 weights/ 를 복사하세요"; fi
echo
if [ "$MODE" = sim ]; then
  bash setup.sh
  echo "완료(시뮬). 다음:  bash ~/tb4_follow_sim/run_sim.sh   (새 터미널)  NODE=v3safe bash ~/tb4_follow_sim/run_follow.sh 12"
else
  echo "완료(실기). 다음 (로봇 ROS 환경 = ROS_DOMAIN_ID·디스커버리 서버 설정을 source 한 뒤):"
  echo "  bash ~/tb4_follow_sim/real/preflight.sh     # 점검"
  echo "  bash ~/tb4_follow_sim/real/run_real.sh 12   # 노드 (enable=false 로 시작)"
  echo "  상세: ~/tb4_follow_sim/docs/REAL_NODES.md, docs/REAL_CODE_STRUCTURE.md"
  if [ -e /dev/dxg ]; then
    echo "  [WSL] 로봇과 통신하려면 Windows %USERPROFILE%\\.wslconfig 에 [wsl2] networkingMode=mirrored 를 넣고 wsl --shutdown"
  fi
fi
