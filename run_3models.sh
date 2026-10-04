#!/bin/bash
# 모델 3종 재실험 (yolo8n 추가): 조명 2종 감지 스윕(8n) → 매트릭스(v3/v3safe × 8/11/12, 녹화, 같은 회차) → 시뮬 끄고 지연·mAP
# 이전 회차(11/12 만) 결과는 logs/run3_matrix/ 로 보관
cd "$(dirname "$0")"; source ./env.sh
# 메모리 감시 (30 s 마다 사용량 상위 5 프로세스) — 2026-10-04 1차 시도가 메모리 압박으로 느려진 뒤 중단돼 추가
( while true; do echo "=== $(date +%T) $(free -m | awk '/Mem/{print "used",$3,"avail",$7}') load $(cut -d' ' -f1 /proc/loadavg)"; ps -eo rss,comm --sort=-rss | head -6; sleep 30; done ) > logs/mem3.log 2>&1 &
MON=$!; trap "kill $MON 2>/dev/null" EXIT
log() { echo "######## $* $(date +%T)"; }
sim() {  # $1 = world_file
  bash stop_all.sh >/dev/null 2>&1; sleep 2
  (nohup bash run_sim.sh gui:=false world_file:=$1 > logs/sim_$1.log 2>&1 &)
  for i in $(seq 1 60); do python3 scripts/check_iface.py 2>/dev/null | grep -q "image_raw: [0-9]" && break; sleep 3; done
  python3 scripts/check_iface.py | head -1
}
mkdir -p logs/run3_matrix/videos
# (이전 회차 보관은 1차 시도 때 완료 — 다시 옮기면 이번 회차 결과가 섞이므로 하지 않음)

[ -f logs/models/yolo8n_amr_best.json ] && grep -q dim logs/models/yolo8n_amr_best.json && SKIP_SWEEP=1
[ -z "$SKIP_SWEEP" ] && { log "sweep bright"; sim follow_arena_bright
( source ~/yolo_venv/bin/activate; python scripts/model_bench.py weights/yolo8n_amr_best.pt --sweep --light bright > logs/model_bench8_bright.log 2>&1 )
log "sweep dim";    sim follow_arena
( source ~/yolo_venv/bin/activate; python scripts/model_bench.py weights/yolo8n_amr_best.pt --sweep --light dim > logs/model_bench8_dim.log 2>&1 ); }

log "matrix"; RESUME=1 VERS="8 11 12" bash run_batch2.sh >> logs/batch4.log 2>&1
MODELS="8 11 12" python3 scripts/summarize2.py > /dev/null
bash scripts/make_compare_videos.sh > logs/compare_videos.log 2>&1

log "latency+mAP (sim off)"; bash stop_all.sh >/dev/null 2>&1; sleep 3
( source ~/yolo_venv/bin/activate; MODELS="8 11 12" python scripts/yolo_compare.py > logs/yolo_compare_3models.log 2>&1 )
log "done"
