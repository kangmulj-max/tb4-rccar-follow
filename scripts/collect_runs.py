#!/usr/bin/env python3
"""모든 시뮬 실험 회차를 표 하나로 모음 → results/all_runs.csv
  python3 scripts/collect_runs.py [출력 csv]
회차(폴더)·태그에서 노드·모델·시나리오를 읽고, 요약 json(안전) + 주행 csv(추종 지표, follow_metrics)를 한 행으로.
측정 정정(M1~M9, docs/CODE_REVIEW_GUIDE.md 3-4)이 있는 회차는 note 열에 표시."""
import csv, datetime, glob, json, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from follow_metrics import metrics

L = os.path.expanduser('~/tb4_follow_sim/logs')
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/tb4_follow_sim/results/all_runs.csv')
RUNS = [  # (회차 이름, 폴더, 태그 패턴, 메모)
    ('run1_v3', f'{L}/run1_v3', None, '1차: 원본 v3 만. 카메라 위치 가정 오류(M1)로 카메라↔RC카 거리 약 0.18 m 작게 기록, 상태시간 일부 누락(M2)'),
    ('run2_matrix', f'{L}/run2_matrix', None, '2차 매트릭스 (v3/v3_safe × 11n/12n). v3_safe 추종률은 재집계값(M3)'),
    ('run3_matrix', f'{L}/run3_matrix', None, '3차 매트릭스 (2026-10-03, 최종 보고서 기준)'),
    ('run4_3models', L, r'^v3(safe)?_', '4차 (2026-10-04, 8n/11n/12n 같은 회차). 34 회에서 중단 — 미실행 조합은 없음'),
    ('qos_test', L, r'^qos_', 'S12 QoS 시험 (best-effort / reliable 발행)'),
    ('refactor_check', L, r'^refac_', '리팩터(R1) 실행본 검증 (2026-10-04)'),
]
pat = re.compile(r'^(?:refac_)?(?:(v3safe|v3)_)?([a-z]+)_y(\d+)(_.*)?$')
rows = []
for run, d, tag_re, note in RUNS:
    for js in sorted(glob.glob(f'{d}/*_summary.json')):
        tag = os.path.basename(js)[:-len('_summary.json')]
        if tag_re and not re.match(tag_re, tag):
            continue
        if run == 'run4_3models' and not os.path.exists(f'{d}/{tag}.done'):
            continue                                    # 중간에 끊긴 실행 제외
        s = json.load(open(js)); m = pat.match(tag.replace('qos_be_', '').replace('qos_rel_', ''))
        node, sc, ver = (m.group(1) or 'v3', m.group(2), m.group(3)) if m else ('', tag, '')
        if run == 'qos_test':
            node, sc, ver = tag.split('_')[-1], tag, '11'
        fm = metrics(f'{d}/{tag}.csv') if os.path.exists(f'{d}/{tag}.csv') else None
        date = ''
        try:
            first = next(csv.DictReader(open(f'{d}/{tag}.csv')))
            date = datetime.datetime.fromtimestamp(float(first['wall'])).strftime('%Y-%m-%d %H:%M')
        except Exception:                               # 1차 회차 csv 에는 wall 열이 없음 → 파일 수정 시각
            date = datetime.datetime.fromtimestamp(os.path.getmtime(js)).strftime('%Y-%m-%d %H:%M')
        e = s.get('events', {})
        coll = e.get('COLLISION_OBS', 0) + e.get('COLLISION_CAR', 0) + (1 if s.get('bumper_hits') else 0)
        rows.append({
            'run': run, 'date': date, 'tag': tag, 'node': node, 'model': f'yolo{ver}n' if ver else '', 'scenario': sc,
            'follow_pct': fm and fm['follow_pct'], 'lost_events': fm and fm['lost_events'], 'lost_s': fm and fm['lost_s'],
            'collision': 'Y' if coll else 'N', 'clear_obs_min_m': s.get('clear_obs_min'), 'gap_car_min_m': s.get('gap_car_min'),
            'reverse_m': s.get('reverse_distance_m'), 'abs_err_tracking_m': s.get('abs_err_mean_tracking'),
            'tracking_s': round((s.get('state_time_s') or {}).get('TRACKING', 0), 1), 'duration_s': s.get('duration_s'),
            'control_hz': s.get('control_rate_hz'), 'max_cmd_lin': s.get('max_cmd_lin'),
            'events': ';'.join(f'{k}={v}' for k, v in sorted(e.items())), 'note': note})
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, 'w', newline='', encoding='utf-8-sig') as f:   # utf-8-sig: 엑셀에서 한글 깨짐 방지
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
cnt = {}
for r in rows: cnt[r['run']] = cnt.get(r['run'], 0) + 1
print(OUT, len(rows), 'rows', cnt)
