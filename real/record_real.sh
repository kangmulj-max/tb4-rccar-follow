#!/bin/bash
# 실기 1회 시도 기록: bag + 디버그 영상 + 텔레메트리 + 메타를 시도 ID 폴더 하나에.
#   bash record_real.sh [시도ID] [메모]          (시도ID 생략 시 YYMMDD-HHMM)
#   Ctrl+C 로 종료 → video.mp4 변환, summary.json 출력. 이 ID 를 'TB4 추종 실험일지' 에 그대로 입력.
# 실기 ROS 환경(ROS_DOMAIN_ID=1, 디스커버리 서버 등)을 먼저 source 한 터미널에서 실행. 시뮬 env.sh 는 쓰지 말 것.
# 환경변수: NS(기본 /robot1)  NO_BAG=1(bag 생략)  BAG_RAW=1(원본 영상까지 bag, Wi-Fi 부하 큼)
#           WEBCAM=1 (웹캠 PC 의 /webcam/image_raw/compressed 도 bag 에 — 원본 웹캠 영상은 웹캠 PC 에 따로 저장됨)
set -u
set -m   # job control: 백그라운드 프로세스가 SIGINT 를 무시하지 않도록 (bag 정상 종료)
DIR0="$(cd "$(dirname "$0")" && pwd)"
TRIAL=${1:-$(date +%y%m%d-%H%M)}; MEMO=${2:-}
NS=${NS:-/robot1}; OUT=~/tb4_real_logs/$TRIAL
mkdir -p "$OUT"; [ -e "$OUT/summary.json" ] && { echo "이미 있는 시도 ID: $OUT"; exit 1; }

# 메타 (재현용): 시각, ROS 환경, 실행 중 추종 노드의 파라미터
{
  echo "{\"trial\": \"$TRIAL\", \"start\": \"$(date -Iseconds)\", \"host\": \"$(hostname)\","
  echo " \"ROS_DOMAIN_ID\": \"${ROS_DOMAIN_ID:-}\", \"RMW\": \"${RMW_IMPLEMENTATION:-}\", \"ROS_DISCOVERY_SERVER\": \"${ROS_DISCOVERY_SERVER:-}\","
  echo " \"memo\": \"${MEMO//\"/\\\"}\"}"
} > "$OUT/meta.json"
timeout 8 ros2 param dump /follow_object_node > "$OUT/node_params.yaml" 2>/dev/null \
  || echo "# follow_object_node 가 아직 없음 — 노드를 먼저 띄우면 파라미터가 기록됨" > "$OUT/node_params.yaml"
timeout 5 ros2 topic list -t > "$OUT/topics.txt" 2>/dev/null

TOPICS=( $NS/oakd/rgb/image_raw/compressed $NS/oakd/stereo/image_raw/compressedDepth
         $NS/oakd/rgb/camera_info $NS/oakd/stereo/camera_info $NS/scan $NS/odom $NS/imu
         $NS/tf $NS/tf_static $NS/cmd_vel $NS/cmd_vel_unstamped $NS/battery_state
         $NS/hazard_detection $NS/wheel_status $NS/dock_status $NS/follow/state /rosout )   # follow/state: 노드 d·vcar·block (S16)
[ "${BAG_RAW:-0}" = 1 ] && TOPICS+=( $NS/oakd/rgb/image_raw $NS/oakd/stereo/image_raw )
[ "${WEBCAM:-0}" = 1 ] && TOPICS+=( ${WEBCAM_TOPIC:-/webcam/image_raw/compressed} )   # 웹캠 PC 가 webcam_rec.py --publish 로 보내는 저속 미리보기

PIDS=()
if [ "${NO_BAG:-0}" != 1 ]; then
  ros2 bag record -o "$OUT/bag" "${TOPICS[@]}" > "$OUT/bag.log" 2>&1 & PIDS+=($!)
fi
python3 "$DIR0/real_recorder.py" --ros-args -p trial:="$TRIAL" -p out:="$OUT" -p ns:="$NS" > "$OUT/recorder.log" 2>&1 & REC=$!
echo "● 기록 중: $TRIAL  →  $OUT"
echo "  외부 촬영(휴대폰)을 켜고 시도 ID '$TRIAL' 을 말한 뒤 주행 시작. 끝나면 Ctrl+C."

stop() {
  echo; echo "■ 종료 중..."
  kill -INT $REC "${PIDS[@]}" 2>/dev/null
  for i in $(seq 1 20); do kill -0 $REC "${PIDS[@]}" 2>/dev/null || break; sleep 0.5; done
  kill -TERM $REC "${PIDS[@]}" 2>/dev/null; wait 2>/dev/null
  if [ -f "$OUT/video_raw.mp4" ]; then
    ffmpeg -y -loglevel error -i "$OUT/video_raw.mp4" -c:v libx264 -pix_fmt yuv420p -crf 24 "$OUT/video.mp4" \
      && rm -f "$OUT/video_raw.mp4" || echo "ffmpeg 변환 실패 — video_raw.mp4 를 그대로 둠"
  fi
  echo "===== $TRIAL"; cat "$OUT/summary.json" 2>/dev/null
  echo; echo "실험일지 입력: 시도 ID=$TRIAL, 기록 폴더=$OUT  (놓침 횟수·놓친 시간은 summary.json 의 for_trial_log 참고)"
  exit 0
}
trap stop INT TERM
wait $REC
stop
