#!/usr/bin/env python3
"""가상 공장 세션 평가 → markdown 표 + summary.json
  python3 fac_eval.py <session_out> <worlds/factory>
주행: 웨이포인트 도달, 회차 시간, 주행 거리, 복구 동작 수, 위치 오차
회피: 로봇 몸체(반지름 0.17 m)와 물체(높이 0.35 m 아래로 내려오는 것) 사이 최소 여유, 접촉(여유 < 0) 횟수 — 라이다에 보이는/안 보이는 물체 구분,
      지게차(이동 장애물)와 최소 여유
판정: 정답 대장 대비 맞음·오탐·놓침, 범주별(보임 / 라이다 아래 / 구조물 근접 / 다리만 / 형상 변화), 오탐 중 지게차 차선 비율
사업 지표: 시간당 순회 면적, 5,000 ㎡ 환산 시간"""
import csv, glob, json, math, os, re, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'layout_delta'))
from layout_delta.diff import Change, evaluate

OUT, FAC = sys.argv[1], sys.argv[2]
ROBOT_R, ROBOT_H, LZ = 0.17, 0.35, 0.193

def rect_dist(px, py, b):
    c, s = math.cos(math.radians(b['yaw'])), math.sin(math.radians(b['yaw']))
    u, v = c * (px - b['x']) + s * (py - b['y']), -s * (px - b['x']) + c * (py - b['y'])
    du, dv = np.maximum(np.abs(u) - b['sx'] / 2, 0), np.maximum(np.abs(v) - b['sy'] / 2, 0)
    return np.hypot(du, dv)

def events(mask):
    return int(np.sum(mask[1:] & ~mask[:-1]) + (1 if len(mask) and mask[0] else 0))

def nav(F, k, spec):
    D = f'{OUT}/{F}/m{k}'
    r = dict(F=F, k=k)
    if not os.path.exists(f'{D}/mission.json'): r['missing'] = True; return r
    m = json.load(open(f'{D}/mission.json'))
    r.update(reached=m['reached'], total=m['total'], round_s=m.get('round_s', 0),
             failed=[w['id'] for w in m['waypoints'] if w['result'] != 'reached'], gate=m.get('localization', {}).get('ok'))
    e = json.load(open(f'{D}/loc_eval.json')) if os.path.exists(f'{D}/loc_eval.json') else {}
    r.update(loc_med=e.get('err_m_median'), loc_p95=e.get('err_m_p95'))
    s = json.load(open(f'{D}/map_stats.json')) if os.path.exists(f'{D}/map_stats.json') else {}
    r.update(scans=s.get('scans_used'), badpose=s.get('skipped_bad_pose'), coverage=s.get('coverage_of_ref_known'))
    log = open(f'{D}/nav2.log', errors='ignore').read() if os.path.exists(f'{D}/nav2.log') else ''
    beh = re.findall(r'behavior_server.*?Running (\w+)', log)
    r['recoveries'] = {b: beh.count(b) for b in sorted(set(beh))}
    if os.path.exists(f'{D}/traj.csv'):
        T = np.genfromtxt(f'{D}/traj.csv', delimiter=',', names=True)
        if T.size > 2:
            x, y = T['x'], T['y']
            r['path_m'] = round(float(np.sum(np.hypot(np.diff(x), np.diff(y)))), 1)
            g = json.load(open(f'{FAC}/{F}/geom_m{k}.json'))
            body = [b for b in g['structure'] + g['objects'] if b['z0'] < ROBOT_H]
            vis = np.full(len(x), np.inf); inv = np.full(len(x), np.inf); who = {}
            for b in body:
                d = rect_dist(x, y, b) - ROBOT_R
                seen = b['z0'] <= LZ <= b['z1'] or b.get('obj') == '_structure'
                if seen: vis = np.minimum(vis, d)
                else: inv = np.minimum(inv, d)
                if d.min() < 0: who[b.get('obj')] = 'visible' if seen else 'below_lidar'
            r.update(min_clear_visible=round(float(vis.min()), 3), contact_visible=events(vis < 0),
                     min_clear_low=round(float(inv.min()), 3) if np.isfinite(inv.min()) else None, contact_low=events(inv < 0),
                     contact_objects=who)
            if spec.get('forklift') and np.isfinite(T['fx']).any():
                fs = spec['forklift']['size']
                fb = [dict(x=fx, y=fy, yaw=0, sx=fs[0], sy=fs[1]) for fx, fy in zip(T['fx'], T['fy'])]
                fd = np.array([rect_dist(xx, yy, b) for xx, yy, b in zip(x, y, fb)]) - ROBOT_R
                fd = fd[np.isfinite(fd)]
                r.update(forklift_min_clear=round(float(fd.min()), 3), forklift_close_s=round(float(np.sum(fd < 0.3) * 0.2), 1),
                         forklift_overlap=events(fd < 0))
    return r

def det(F, k, spec):
    rp = f'{OUT}/{F}/report_m{k}/changes.json'
    if not os.path.exists(rp): return None
    ch = [Change(**{kk: v for kk, v in c.items()}) for c in json.load(open(rp))['changes']]
    for c in ch:
        if c.from_xy: c.from_xy = tuple(c.from_xy)
    truth = json.load(open(f'{FAC}/{F}/truth_m{k}.json', encoding='utf-8'))
    # 정답 매칭: 물체 중심이 아니라 물체 외곽선에서 0.5 m 이내 (2D 라이다는 큰 설비의 앞면만 봄 — CNC 2.2 × 1.6 m 는 앞면이 중심에서 0.8 m)
    g_now = json.load(open(f'{FAC}/{F}/geom_m{k}.json'))['objects']; g_prev = json.load(open(f'{FAC}/{F}/geom_m{k - 1}.json'))['objects']
    def fdist(px, py, oid, geom):
        bs = [b for b in geom if b['obj'] == oid]
        return min(float(rect_dist(px, py, b)) for b in bs) if bs else 1e9
    def match(t, c):
        geom = g_prev if t['kind'] == 'removed' else g_now
        pts = [(c.x, c.y)] + ([tuple(c.from_xy)] if c.from_xy and t['kind'] in ('removed', 'moved') else [])
        if t['kind'] == 'moved':   # 이동: 새 위치 또는 원위치(이전 달 외곽선) 어느 쪽이든 잡으면 검출로 봄
            return min(fdist(c.x, c.y, t['obj'], g_now), *(fdist(px, py, t['obj'], g_prev) for px, py in pts)) <= 0.5
        return min(fdist(px, py, t['obj'], geom) for px, py in pts) <= 0.5
    used, tp, typ, hits = set(), 0, 0, {}
    for t in truth:
        for i, c in enumerate(ch):
            if i not in used and match(t, c):
                used.add(i); tp += 1; typ += c.kind != t['kind'] and not (t['kind'] == 'changed' and c.kind == 'added'); hits[t['obj']] = 1; break
    # 이미 맞춘 물체의 다른 조각(한 물체가 여러 덩어리로 갈라짐)은 오탐과 따로 셈 — 리포트에는 중복 항목으로 보임
    dup = sum(1 for i, c in enumerate(ch) if i not in used and any(match(t, c) for t in truth if t['obj'] in hits))
    fp = len(ch) - len(used) - dup
    ev = dict(tp=tp, fp=fp, dup=dup, fn=len(truth) - tp, type_errors=int(typ), precision=round(tp / max(1, tp + fp), 3), precision_raw=round(tp / max(1, len(ch)), 3), recall=round(tp / max(1, len(truth)), 3))
    cats = {}
    def cat(t):
        if not t['visible']: return 'below_lidar'
        if t['kind'] == 'changed': return 'shape_change'
        if t['legs_only']: return 'legs_only'
        if t['near_structure']: return 'near_structure'
        return 'visible_clear'
    for t in truth:
        c = cat(t); hit = hits.get(t['obj'], 0)
        a = cats.setdefault(c, [0, 0]); a[0] += 1; a[1] += hit
    # 오탐 중 지게차 차선 위
    fp_dyn = 0
    if spec.get('forklift'):
        (x0, y0), (x1, y1) = spec['forklift']['path']
        fp_dyn = sum(1 for i, c in enumerate(ch) if i not in used and min(x0, x1) - 1.5 <= c.x <= max(x0, x1) + 1.5 and abs(c.y - y0) <= 1.0)
    return dict(F=F, k=k, n_truth=len(truth), n_det=len(ch), **ev, cats=cats, fp_forklift_lane=fp_dyn)

rows_n, rows_d = [], []
for F in sorted(os.listdir(FAC)):
    if not os.path.isdir(f'{FAC}/{F}'): continue
    spec = json.load(open(f'{FAC}/{F}/spec.json', encoding='utf-8'))
    for k in range(4): rows_n.append(dict(nav(F, k, spec), title=spec['title'], free_m2=spec['free_m2_lidar'], floor_m2=spec['floor_m2']))
    for k in (1, 2, 3):
        d = det(F, k, spec)
        if d: rows_d.append(d)

L = ['### 주행·회피', '', '| 공장 | 월 | 도달 | 실패 지점 | 회차 [분] | 주행 [m] | 복구 동작 | 위치 오차 중앙/95% [m] | 보이는 물체 최소 여유 [m] / 접촉 | 라이다 아래 물체 최소 여유 [m] / 접촉 | 지게차 최소 여유 [m] / 0.3 m 이내 [s] |',
     '|---|---|---|---|---|---|---|---|---|---|---|']
for r in rows_n:
    if r.get('missing'): L.append(f"| {r['title']} | M{r['k']} | 실행 안 됨 |||||||||"); continue
    rec = sum(r['recoveries'].values()); recs = ', '.join(f'{k} {v}' for k, v in r['recoveries'].items()) or '0'
    L.append(f"| {r['title']} | M{r['k']} | {r['reached']}/{r['total']} | {' '.join(r['failed']) or '–'} | {r['round_s'] / 60:.1f} | {r.get('path_m', '–')} | {recs} | "
             f"{r['loc_med']} / {r['loc_p95']} | {r.get('min_clear_visible', '–')} / {r.get('contact_visible', '–')} | "
             f"{r.get('min_clear_low', '–')} / {r.get('contact_low', '–')} | {r.get('forklift_min_clear', '–')} / {r.get('forklift_close_s', '–')} |")
L += ['', '### 변화 판정', '', '| 공장 | 비교 | 정답 | 검출 항목 | 맞음 | 같은 물체 조각 | 오탐 (지게차 차선) | 놓침 | 종류 오판 | 재현율 | 정밀도 (조각 제외) | 리포트 항목 중 맞음 |', '|---|---|---|---|---|---|---|---|---|---|---|---|']
tot = dict(tp=0, fp=0, fn=0, n=0, typ=0, fpd=0, dup=0, det=0); catt = {}
for d in rows_d:
    t = next(r['title'] for r in rows_n if r['F'] == d['F'])
    L.append(f"| {t} | M{d['k'] - 1}→M{d['k']} | {d['n_truth']} | {d['n_det']} | {d['tp']} | {d['dup']} | {d['fp']} ({d['fp_forklift_lane']}) | {d['fn']} | {d['type_errors']} | {d['recall']:.0%} | {d['precision']:.0%} | {d['precision_raw']:.0%} |")
    tot['tp'] += d['tp']; tot['fp'] += d['fp']; tot['fn'] += d['fn']; tot['n'] += d['n_truth']; tot['typ'] += d['type_errors']; tot['fpd'] += d['fp_forklift_lane']; tot['dup'] += d['dup']; tot['det'] += d['n_det']
    for c, (n, h) in d['cats'].items():
        a = catt.setdefault(c, [0, 0]); a[0] += n; a[1] += h
if tot['n']:
    L.append(f"| **합계** | | {tot['n']} | {tot['det']} | {tot['tp']} | {tot['dup']} | {tot['fp']} ({tot['fpd']}) | {tot['fn']} | {tot['typ']} | "
             f"{tot['tp'] / tot['n']:.0%} | {tot['tp'] / max(1, tot['tp'] + tot['fp']):.0%} | {tot['tp'] / max(1, tot['det']):.0%} |")
NAMES = dict(visible_clear='라이다에 보이고 주변이 빈 물체', near_structure='벽·기둥·다른 물체에 0.3 m 이내로 붙은 물체', legs_only='작업대 (라이다엔 다리만)',
             shape_change='같은 자리 형상 변화 (빈 랙 ↔ 적재)', below_lidar='라이다 평면(0.193 m) 아래 물체')
L += ['', '### 범주별 재현율', '', '| 범주 | 정답 | 맞음 | 재현율 |', '|---|---|---|---|']
for c in ['visible_clear', 'near_structure', 'legs_only', 'shape_change', 'below_lidar']:
    if c in catt: n, h = catt[c]; L.append(f'| {NAMES[c]} | {n} | {h} | {h / n:.0%} |')
L += ['', '### 사업 지표 (시뮬 — 상한선)', '', '| 공장 | 바닥 면적 [㎡] | 라이다 기준 빈 공간 [㎡] | 회차 시간 중앙 [분] | 시간당 순회 바닥 면적 [㎡/h] | 5,000 ㎡ 환산 [h] |', '|---|---|---|---|---|---|']
for F in sorted({r['F'] for r in rows_n}):
    rs = [r for r in rows_n if r['F'] == F and not r.get('missing') and r['reached'] >= 0.8 * r['total']]
    if not rs: continue
    t = float(np.median([r['round_s'] for r in rs])) / 3600
    a = rs[0]['floor_m2']
    L.append(f"| {rs[0]['title']} | {a:.0f} | {rs[0]['free_m2']:.0f} | {t * 60:.1f} | {a / t:.0f} | {5000 / (a / t):.1f} |")
print('\n'.join(L))
json.dump(dict(nav=rows_n, det=rows_d, totals=tot, categories=catt), open(f'{OUT}/summary.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1, default=str)
