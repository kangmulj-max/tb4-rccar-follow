#!/bin/bash
# 최종 재실험 일괄: 조명 2종 감지 스윕 → 매트릭스(v3/v3safe × 11/12, 녹화) → 시뮬 끄고 지연시간
cd "$(dirname "$0")"; source ./env.sh
log() { echo "######## $* $(date +%T)"; }
sim() {  # $1 = world_file
  bash stop_all.sh >/dev/null 2>&1; sleep 2
  (nohup bash run_sim.sh gui:=false world_file:=$1 > logs/sim_$1.log 2>&1 &)
  for i in $(seq 1 60); do python3 scripts/check_iface.py 2>/dev/null | grep -q "image_raw: [0-9]" && break; sleep 3; done
  python3 scripts/check_iface.py | head -1
}
mkdir -p logs/run2_matrix
mv logs/v3_* logs/v3safe_* logs/compare.md logs/batch2.log logs/run2_matrix/ 2>/dev/null
mkdir -p logs/run2_matrix/videos; mv logs/videos/*.mp4 logs/run2_matrix/videos/ 2>/dev/null
rm -f logs/models/*.json

log "sweep bright"; sim follow_arena_bright
( source ~/yolo_venv/bin/activate; python scripts/model_bench.py weights/yolo11n_amr_best.pt weights/yolo12n_amr_best.pt --sweep --light bright > logs/model_bench_bright.log 2>&1 )
log "sweep dim";    sim follow_arena
( source ~/yolo_venv/bin/activate; python scripts/model_bench.py weights/yolo11n_amr_best.pt weights/yolo12n_amr_best.pt --sweep --light dim > logs/model_bench_dim.log 2>&1 )

log "matrix"; bash run_batch2.sh > logs/batch3.log 2>&1
python3 scripts/summarize2.py > /dev/null
bash scripts/make_compare_videos.sh > logs/compare_videos.log 2>&1

log "latency (sim off)"; bash stop_all.sh >/dev/null 2>&1; sleep 3
( source ~/yolo_venv/bin/activate; python scripts/yolo_compare.py > logs/yolo_compare_clean.log 2>&1 )
log "done"
