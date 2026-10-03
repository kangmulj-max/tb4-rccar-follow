#!/bin/bash
# S12 검증: 카메라 발행 QoS 가 reliable / best_effort 일 때 v3_safe 와 원본 v3 가 영상을 받는지
#   bash scripts/test_qos.sh   → logs/qos_*  (약 10 분)
cd "$(dirname "$0")/.."; source ./env.sh
run_sim() { bash stop_all.sh >/dev/null 2>&1; sleep 2
  (nohup bash run_sim.sh gui:=false pub_qos:=$1 > logs/sim_qos_$1.log 2>&1 &)
  for i in $(seq 1 60); do python3 scripts/check_iface.py 2>/dev/null | grep -q "image_raw: [0-9]" && break; sleep 3; done; }
summ() { python3 -c "
import json,sys; s=json.load(open('logs/$1_summary.json'))
print('$1', 'max_cmd_lin', s['max_cmd_lin'], 'state_time', s['state_time_s'], 'events', s['events'])"; }

echo "## A) 발행 reliable"; run_sim reliable
bash real/preflight.sh > logs/qos_preflight_sim.txt 2>&1; grep -E "\[ok\]|\[--\]|정면|car@" logs/qos_preflight_sim.txt
RECORD=0 NODE=v3safe TAG=qos_rel_v3safe bash experiment.sh square 12 >/dev/null 2>&1; summ qos_rel_v3safe

echo "## B) 발행 best_effort (실기 드라이버가 이렇게 낼 경우)"; run_sim best_effort
RECORD=0 NODE=v3safe TAG=qos_be_v3safe bash experiment.sh square 12 >/dev/null 2>&1; summ qos_be_v3safe
RECORD=0 NODE=v3     TAG=qos_be_v3     bash experiment.sh square 12 > logs/qos_be_v3_exp.log 2>&1; grep -a "node not ready" logs/qos_be_v3_exp.log; summ qos_be_v3 2>/dev/null || echo "qos_be_v3: summary 없음"
grep -a "camera_info ok\|IDLE -> TRACKING" logs/qos_be_v3_node.log | head -3
bash stop_all.sh >/dev/null 2>&1
echo "## done"
