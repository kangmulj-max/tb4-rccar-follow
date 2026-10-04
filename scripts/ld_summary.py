#!/usr/bin/env python3
"""Layout Delta 시뮬 세션 요약 → markdown 표 (보고서 5-3 절)."""
import json, os, sys, glob
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'layout_delta'))
from layout_delta.diff import DiffParams, diff, evaluate
from layout_delta.grid import MapGrid
from layout_delta.detect import roi_from_waypoints
WP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'layout_delta', 'config', 'waypoints_arena1004.yaml')

OUT = sys.argv[1]; TRUTH = sys.argv[2]
L = ['| 회차 | 웨이포인트 도달 | 회차 시간 [s] | 지도에 쓴 스캔 (사용/전체, 위치 불량 제외) | 위치 추정 오차 중앙값 / 95% [m] | 방향 오차 중앙값 / 95% [°] |', '|---|---|---|---|---|---|']
for k in range(4):
    d = f'{OUT}/round{k:02d}'
    m = json.load(open(f'{d}/mission.json')) if os.path.exists(f'{d}/mission.json') else {}
    s = json.load(open(f'{d}/map_stats.json')) if os.path.exists(f'{d}/map_stats.json') else {}
    e = json.load(open(f'{d}/loc_eval.json')) if os.path.exists(f'{d}/loc_eval.json') else {}
    L.append(f"| {k:02d} | {m.get('reached', '–')}/{m.get('total', '–')} | {m.get('round_s', '–')} | {s.get('scans_used', '–')}/{s.get('scans_in', '–')} ({s.get('skipped_bad_pose', 0)}) | "
             f"{e.get('err_m_median', '–')} / {e.get('err_m_p95', '–')} | {e.get('err_deg_median', '–')} / {e.get('err_deg_p95', '–')} |")
L += ['', '| 비교 | 정답(보이는 상자) | 검출 | TP | FP | FN | 유형 오류 | 재현율 | 정밀도 | 낮은 상자(0.12 m) 정답 → 검출 |', '|---|---|---|---|---|---|---|---|---|---|']
tot = dict(tp=0, fp=0, fn=0, typ=0, n=0, low=0, lowhit=0)
for k in (1, 2, 3):
    a, b = f'{OUT}/round{k-1:02d}/map.yaml', f'{OUT}/round{k:02d}/map.yaml'
    if not (os.path.exists(a) and os.path.exists(b)):
        L.append(f'| {k-1:02d}→{k:02d} | 지도 없음 |||||||||'); continue
    ga = MapGrid.load(a)
    res = diff(ga, MapGrid.load(b), roi=roi_from_waypoints(ga, WP, 3.0))
    tv = json.load(open(f'{TRUTH}/truth_round{k:02d}_lidar.json')); tall = json.load(open(f'{TRUTH}/truth_round{k:02d}.json'))
    ev = evaluate(res.changes, tv)
    low = [t for t in tall if t['low']]
    lowhit = sum(1 for t in low if evaluate(res.changes, [t])['tp'])
    L.append(f"| {k-1:02d}→{k:02d} | {len(tv)} | {len(res.changes)} | {ev['tp']} | {ev['fp']} | {ev['fn']} | {ev['type_errors']} | {ev['recall']:.0%} | {ev['precision']:.0%} | {len(low)} → {lowhit} |")
    for kk, v in (('tp', ev['tp']), ('fp', ev['fp']), ('fn', ev['fn']), ('typ', ev['type_errors']), ('n', len(tv)), ('low', len(low)), ('lowhit', lowhit)):
        tot[kk] += v
if tot['n']:
    L.append(f"| 합계 | {tot['n']} | {tot['tp'] + tot['fp']} | {tot['tp']} | {tot['fp']} | {tot['fn']} | {tot['typ']} | {tot['tp'] / tot['n']:.0%} | {tot['tp'] / max(1, tot['tp'] + tot['fp']):.0%} | {tot['low']} → {tot['lowhit']} |")
print('\n'.join(L))
json.dump(tot, open(f'{OUT}/summary_totals.json', 'w'), indent=1)
