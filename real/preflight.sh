#!/bin/bash
# 실기 출발 전 점검 (로봇은 정지 상태). 결과: ~/tb4_real_logs/preflight_<시각>.txt
#   bash real/preflight.sh
# 확인: 도크 상태·카메라 해상도/fx·압축 영상 주기 / 토픽 주기·형식 / QoS(best-effort 여부) / cmd_vel_unstamped 를 받는 노드 / 라이다 정면 방향 / 현재 영상 YOLO
HERE="$(cd "$(dirname "$0")/.." && pwd)"
NS=${NS:-/robot1}
[ -n "${ROS_DISTRO:-}" ] || { echo "ROS 환경 없음"; exit 1; }
mkdir -p ~/tb4_real_logs; OUT=~/tb4_real_logs/preflight_$(date +%y%m%d-%H%M).txt
{
echo "== $(date -Iseconds)  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  DISCOVERY=${ROS_DISCOVERY_SERVER:-없음}"
echo; echo "== 0) 도크·카메라 설정 (power_saver: 도크 위에서는 카메라·라이다가 꺼짐)"
DOCKED=$(timeout 8 ros2 topic echo --once --field is_docked "$NS/dock_status" 2>/dev/null | head -1)
case "$DOCKED" in
  [Tt]rue)  echo "!! 도크 위에 있음 → 카메라·라이다 토픽 없음. 언도킹 후 30 초 기다렸다 다시 실행:"
         echo "   ros2 action send_goal $NS/undock irobot_create_msgs/action/Undock \"{}\"" ;;
  [Ff]alse) echo "OK 도크에서 내려와 있음" ;;
  *)     echo "?? dock_status 를 못 받음 (네트워크·ROS_DOMAIN_ID 확인)" ;;
esac
CI=$(timeout 8 ros2 topic echo --once "$NS/oakd/rgb/camera_info" 2>/dev/null)
if ! echo "$CI" | grep -q "^width:"; then echo "!! rgb/camera_info 없음 (도크 위이거나 OAK-D 가 아직 안 켜짐 — bringup 후 약 30 초 걸림)"
else
  W=$(echo "$CI" | awk '/^width:/{print $2}'); H=$(echo "$CI" | awk '/^height:/{print $2}')
  FXC=$(echo "$CI" | awk '/^k:/{getline; print $2}')
  echo "camera_info: ${W}x${H}, fx=${FXC}  (기대 704x704, fx 564.65 — 이 값은 실제와 안 맞아 노드는 실측 fx ${FX:-909} 로 덮어씀, S18)"
  [ "$W" = 704 ] && [ "$H" = 704 ] || echo "!! 해상도가 704x704 가 아님 → 로봇 oakd_pro.yaml 이 기본값으로 바뀌었을 수 있음 (depth 없으면 추종 불가). 시험 전에 확인"
  python3 -c "import sys; sys.exit(0 if abs(float(sys.argv[1]) - 564.65) < 1 else 1)" "$FXC" 2>/dev/null \
    || echo "!! camera_info fx 가 10/3 값(564.65)과 다름 → 로봇 카메라 설정이 바뀌었을 수 있음. 실측 fx(${FX:-909})도 달라졌을 수 있으니 C1 시험으로 다시 확인"
fi
for t in oakd/rgb/image_raw/compressed oakd/stereo/image_raw/compressedDepth; do
  R=$(timeout 7 ros2 topic hz "$NS/$t" 2>/dev/null | awk '/average rate/{r=$3} END{print r}')
  echo "$t: ${R:-no data} Hz (기대 ≈10)"
done
echo; echo "== 1) 토픽 주기·형식 (no data 가 있으면 네트워크 또는 QoS 문제)"
python3 "$HERE/scripts/check_iface.py" 2>&1 | grep -v "_internal\|rc_car"
echo; echo "== 2) QoS — 발행자 Reliability (v3_safe 는 best-effort 로 구독하므로 어느 쪽이든 연결됨)"
for t in oakd/rgb/image_raw oakd/stereo/image_raw scan; do
  echo "-- $NS/$t"; timeout 10 ros2 topic info -v "$NS/$t" 2>/dev/null | grep -E "Node name|Reliability|Endpoint type" | head -6
done
echo; echo "== 3) cmd_vel_unstamped 구독자 (create3_repub 등이 있어야 명령이 바퀴까지 감)"
timeout 10 ros2 topic info -v "$NS/cmd_vel_unstamped" 2>/dev/null | grep -B1 -A1 "SUBSCRIPTION" | grep -E "Node name|Endpoint" || echo "구독자 없음 — 명령이 로봇에 전달되지 않음"
echo; echo "== 4) 라이다 방향 (로봇 정면 0.5 m 에 상자를 두고 실행: '정면' 값이 ~0.5 여야 함)"
python3 "$HERE/scripts/lidar_check.py" 2>&1
echo; echo "== 5) 현재 카메라 1 프레임 YOLO (RC카를 1 m 앞에 두고)"
( [ -f ~/yolo_venv/bin/activate ] && source ~/yolo_venv/bin/activate; YOLO_AUTOINSTALL=False python3 "$HERE/scripts/detect_test.py" 2>&1 | grep -v Warn )
echo "   결과 이미지: $HERE/logs/detect_y11.jpg, detect_y12.jpg"
} 2>&1 | tee "$OUT"
echo; echo "저장: $OUT"
