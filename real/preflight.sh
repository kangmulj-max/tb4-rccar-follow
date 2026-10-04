#!/bin/bash
# 실기 출발 전 점검 (로봇은 정지 상태). 결과: ~/tb4_real_logs/preflight_<시각>.txt
#   bash real/preflight.sh
# 확인: 토픽 주기·형식 / QoS(best-effort 여부) / cmd_vel_unstamped 를 받는 노드 / 라이다 정면 방향 / 현재 영상 YOLO
HERE="$(cd "$(dirname "$0")/.." && pwd)"
NS=${NS:-/robot1}
[ -n "${ROS_DISTRO:-}" ] || { echo "ROS 환경 없음"; exit 1; }
mkdir -p ~/tb4_real_logs; OUT=~/tb4_real_logs/preflight_$(date +%y%m%d-%H%M).txt
{
echo "== $(date -Iseconds)  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  DISCOVERY=${ROS_DISCOVERY_SERVER:-없음}"
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
