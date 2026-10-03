#!/bin/bash
# 1회 실행: 스냅샷(tb4_snapshot_20261002_1909) 카메라 사양으로 turtlebot4_description 오버레이 생성 + 가중치/노드 복사
set -e
cd "$(dirname "$0")"
ROOT=$PWD
# 가중치·원본 노드를 찾을 곳: 1) 이 폴더(weights/, scripts/)에 이미 있으면 그대로 2) TB4_DL 3) WSL 의 Windows 다운로드 폴더
DL=${TB4_DL:-$(ls -d /mnt/c/Users/*/Downloads 2>/dev/null | head -1)}

# --- 1) turtlebot4_description 오버레이 (OAK-D 시뮬 카메라를 실기와 맞춤) ---
#  실기: 704x704, fx=fy=564.65 → HFOV = 2*atan(352/564.65) = 1.1152 rad, 10 Hz
SRC=/opt/ros/jazzy/share/turtlebot4_description
OV=$ROOT/overlay
rm -rf "$OV"; mkdir -p "$OV/share/ament_index/resource_index/packages"
cp -r "$SRC" "$OV/share/"
touch "$OV/share/ament_index/resource_index/packages/turtlebot4_description"
X=$OV/share/turtlebot4_description/urdf/sensors/oakd.urdf.xacro
sed -i -e 's#<horizontal_fov>1.25</horizontal_fov>#<horizontal_fov>1.1152</horizontal_fov>#' \
       -e 's#<width>320</width>#<width>704</width>#' \
       -e 's#<height>240</height>#<height>704</height>#' \
       -e '/rgbd_camera/,/<\/sensor>/ s#<update_rate>30</update_rate>#<update_rate>10</update_rate>#' "$X"
grep -q '<width>704</width>' "$X" && grep -q '1.1152' "$X" && grep -q '<update_rate>10</update_rate>' "$X" \
  && echo "[ok] OAK-D overlay: 704x704, HFOV 1.1152 rad, 10 Hz" || { echo "[fail] oakd patch"; exit 1; }

# --- 1b) gz RPLIDAR 센서 제거 → scripts/sim_rplidar.py(기하 레이캐스트)가 /robot1/scan 담당 ---
#  WSL(d3d12): ogre1 gpu_lidar 는 전 방향 range_min(0.164 m)만 반환, ogre2 는 카메라가 ~2 Hz 로 저하
L=$OV/share/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro
sed -i 's#<xacro:ray_sensor sensor_name="${name}" gazebo="${gazebo}"#<xacro:ray_sensor sensor_name="${name}" gazebo="none"#' "$L"
grep -q 'gazebo="none"' "$L" && echo "[ok] gz rplidar sensor disabled (sim_rplidar.py 사용)" || { echo "[fail] lidar patch"; exit 1; }

# --- 2) YOLO 가중치 / v3 노드 복사 ---
mkdir -p weights scripts
for f in yolo11n_amr_best.pt yolo12n_amr_best.pt; do
  [ -f "weights/$f" ] || cp -v "$DL/$f" weights/ 2>/dev/null || echo "[warn] weights/$f 없음 — 직접 넣을 것"
done
[ -f scripts/follow_object_node_v3.py ] || cp -v "$DL/follow_object_node_v3.py" scripts/
echo "[ok] weights: $(ls weights | tr '\n' ' ')"

# --- 3) YOLO venv 확인 ---
~/yolo_venv/bin/python -c "import ultralytics, torch; print('[ok] ultralytics', ultralytics.__version__, 'cuda', torch.cuda.is_available())"
