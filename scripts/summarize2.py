#!/usr/bin/env python3
"""batch2 결과 → 비교표 (logs/compare.md).  v3 vs v3safe, yolo11n vs yolo12n"""
import glob, json, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from follow_metrics import metrics
L = os.path.expanduser('~/tb4_follow_sim/logs')
SCS = ['square', 'random', 'stopgo', 'lost', 'pillar', 'lowbox', 'pole', 'approach', 'camdrop', 'depthstop', 'depthloss', 'shutdown']
pat = re.compile(r'\[(?:INFO|WARN)\] \[(\d+\.\d+)\] \[follow_object_node\]: (.*)')


def node_stats(tag):
    f = f'{L}/{tag}_node.log'
    if not os.path.exists(f): return {}
    ev = [(float(m.group(1)), m.group(2)) for m in map(pat.match, open(f, errors='ignore')) if m]
    if not ev: return {}
    st, t0, dur = 'IDLE', ev[0][0], {}
    end = next((t for t, m in ev if 'enable=false' in m or 'shutdown' in m), ev[-1][0])
    for t, m in ev:
        if t > end: break
        if '->' in m and m.split('->')[0].strip() in ('IDLE', 'TRACKING', 'REID', 'SEARCH'):
            dur[st] = dur.get(st, 0) + t - t0; st, t0 = m.split('->')[1].strip(), t
    dur[st] = dur.get(st, 0) + end - t0
    msgs = [m for t, m in ev if t <= end]
    return {'dur': dur, 'reid': sum('TRACKING -> REID' in m for m in msgs), 'search': sum('-> SEARCH' in m for m in msgs),
            'total': end - ev[0][0]}


def load(tag):
    f = f'{L}/{tag}_summary.json'
    if not os.path.exists(f): return None
    s = json.load(open(f)); s['node'] = node_stats(tag); return s


def coll(s):
    e = s['events']; n = e.get('COLLISION_OBS', 0) + e.get('COLLISION_CAR', 0)
    return ('충돌' if n or s['bumper_hits'] else '없음')


out = ['# 비교표 (v3 vs v3_safe × yolo11n vs yolo12n)', '',
       '## 핵심: 움직이는 RC카 추종 — 추종유지율[%] / 놓침 횟수 / 놓친 시간[s] / 최대 이격[m]', '']
cols = [('v3', '11'), ('v3', '12'), ('v3safe', '11'), ('v3safe', '12')]
out.append('| 시나리오 | ' + ' | '.join(f'{n} y{v}' for n, v in cols) + ' |')
out.append('|---|' + '---|' * len(cols))
fagg = {c: [] for c in cols}
for sc in SCS:
    row = []
    for c in cols:
        f = f'{L}/{c[0]}_{sc}_y{c[1]}.csv'
        m = metrics(f) if os.path.exists(f) else None
        if not m or m['follow_pct'] is None: row.append('-'); continue
        row.append(f"{m['follow_pct']} / {m['lost_events']} / {m['lost_s']} / {m['far_max']}")
        if sc in ('square', 'random', 'stopgo', 'lost', 'camdrop', 'depthloss'): fagg[c].append(m)
    out.append(f'| {sc} | ' + ' | '.join(row) + ' |')
out += ['', '| 이동 타깃 시나리오 평균 | 추종유지율 | 놓침 합 | 놓친 시간 합[s] |', '|---|---|---|---|']
for c in cols:
    ms = fagg[c]
    if ms: out.append(f"| {c[0]} y{c[1]} | {sum(m['follow_pct'] for m in ms)/len(ms):.1f}% | {sum(m['lost_events'] for m in ms)} | {sum(m['lost_s'] for m in ms):.1f} |")
out += ['', '## 안전',
       '안전: 충돌 여부 / 장애물 여유 min[m] / 차체(RC카) 여유 min[m] / 후진거리[m]', '']
cols = [('v3', '11'), ('v3', '12'), ('v3safe', '11'), ('v3safe', '12')]
out.append('| 시나리오 | ' + ' | '.join(f'{n} y{v}' for n, v in cols) + ' |')
out.append('|---|' + '---|' * len(cols))
for sc in SCS:
    row = []
    for n, v in cols:
        s = load(f'{n}_{sc}_y{v}')
        row.append('-' if s is None else f"{coll(s)} / {s['clear_obs_min']:.2f} / {s['gap_car_min']:.2f} / {s['reverse_distance_m']:.2f}")
    out.append(f'| {sc} | ' + ' | '.join(row) + ' |')

out += ['', '추종: 거리오차 평균[m](TRACKING 중, 카메라↔RC카) / TRACKING 비율[%] / TRACKING→REID 횟수 / 제어 Hz', '']
out.append('| 시나리오 | ' + ' | '.join(f'{n} y{v}' for n, v in cols) + ' |')
out.append('|---|' + '---|' * len(cols))
agg = {c: {'trk': 0, 'tot': 0, 'reid': 0, 'err': [], 'hz': []} for c in cols}
for sc in SCS:
    row = []
    for c in cols:
        s = load(f'{c[0]}_{sc}_y{c[1]}')
        if s is None: row.append('-'); continue
        ns = s['node']; trk = ns.get('dur', {}).get('TRACKING', 0); tot = ns.get('total', 1) or 1
        err = s['abs_err_mean_tracking']
        row.append(f"{err if err is not None else 'n/a'} / {100 * trk / tot:.0f} / {ns.get('reid', 0)} / {s['control_rate_hz']}")
        if sc in ('square', 'random', 'stopgo', 'lost', 'camdrop'):
            a = agg[c]; a['trk'] += trk; a['tot'] += tot; a['reid'] += ns.get('reid', 0); a['hz'].append(s['control_rate_hz'])
            if err is not None: a['err'].append(err)
    out.append(f'| {sc} | ' + ' | '.join(row) + ' |')

out += ['', '추종 시나리오(square/random/stopgo/lost/camdrop) 합계', '',
        '| | TRACKING 비율 | TRACKING→REID 합 | 거리오차 평균 | 제어 Hz 평균 |', '|---|---|---|---|---|']
for c in cols:
    a = agg[c]
    if not a['tot']: continue
    out.append(f"| {c[0]} y{c[1]} | {100 * a['trk'] / a['tot']:.0f}% | {a['reid']} | "
               f"{sum(a['err']) / max(len(a['err']), 1):.3f} | {sum(a['hz']) / max(len(a['hz']), 1):.1f} |")
open(f'{L}/compare.md', 'w').write('\n'.join(out) + '\n'); print('\n'.join(out))
