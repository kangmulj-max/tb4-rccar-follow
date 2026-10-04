#!/usr/bin/env python3
"""기록 도구(real_recorder.py)가 중간에 끊겨 summary.json 이 없을 때, telemetry.csv + events.csv 로 다시 만든다.
  python3 real/make_summary.py ~/tb4_real_logs/261003-T085            → summary.json 작성
  python3 real/make_summary.py ~/tb4_real_logs/261003-A --check       → 기존 summary.json 과 비교만 (검증용)
계산은 real_recorder.close() 와 같다. 차이: 후진 거리는 odom 콜백 간격 대신 telemetry 10 Hz 로 적분 (근사)."""
import csv, json, math, os, sys

d = os.path.expanduser(sys.argv[1]); check = '--check' in sys.argv
tel = list(csv.DictReader(open(f'{d}/telemetry.csv')))
ev = [(float(r['t']), r['type'], r['detail']) for r in csv.DictReader(open(f'{d}/events.csv'))]

state_t, rev, min_front = {}, 0.0, float('inf')
for r in tel:                                            # 10 Hz 한 행 = 0.1 s (real_recorder.tick 과 같음)
    state_t[r['state']] = state_t.get(r['state'], 0.0) + 0.1
    min_front = min(min_front, float(r['front_clear']))
    v = float(r['odom_v'])
    if v < -0.02: rev += -v * 0.1
started, lost_n = False, 0                              # 상태 전환은 노드 로그 기반 events 의 STATE 로 (10 Hz 표본은 짧은 전환을 놓침)
for t, kind, det in ev:
    if kind != 'STATE' or '->' not in det: continue
    old, new = (x.strip() for x in det.split('->'))
    if new == 'TRACKING': started = True
    if started and new in ('SEARCH', 'IDLE') and old not in ('SEARCH', 'IDLE'): lost_n += 1


def warn_labels(kind, det):
    if kind == 'CAMLOST': return ['camera drop']
    if kind == 'SCANLOST': return ['scan drop']
    if kind == 'BUMP': return ['bumper']
    if kind == 'NODE':
        for key, lab in (('camera timeout', 'camera timeout'), ('lidar TF', 'lidar TF fallback'), ('scan', 'scan warning')):
            if key in det: return [lab]
    return []


lost_s = sum(v for k, v in state_t.items() if k in ('SEARCH', 'IDLE')) if started else 0.0
s = {'trial': os.path.basename(os.path.normpath(d)), 'duration_s': round(float(tel[-1]['t']), 1), 'frames': len(tel),
     'state_time_s': {k: round(v, 1) for k, v in state_t.items()}, 'lost_events': lost_n,
     'bumps': sum(1 for e in ev if e[1] == 'BUMP'), 'reverse_m': round(rev, 2),
     'min_front_clear_m': None if math.isinf(min_front) else round(min_front, 2),
     'camera_drops': sum(1 for e in ev if e[1] == 'CAMLOST'), 'scan_drops': sum(1 for e in ev if e[1] == 'SCANLOST'),
     'for_trial_log': {'lostN': lost_n, 'lostS_approx': round(lost_s, 1), 'reverse': 'yes' if rev > 0.05 else 'no',
                       'warnings': sorted({lab for e in ev for lab in warn_labels(e[1], e[2])})},
     'note': 'telemetry.csv + events.csv 로 다시 만든 요약 (real/make_summary.py). 후진 거리는 10 Hz 적분 근사'}
if check:
    old = json.load(open(f'{d}/summary.json'))
    for k in ('duration_s', 'frames', 'state_time_s', 'lost_events', 'bumps', 'reverse_m', 'min_front_clear_m', 'camera_drops', 'scan_drops', 'for_trial_log'):
        print(f"{'OK  ' if old.get(k) == s[k] else 'DIFF'} {k}: 원본 {old.get(k)} / 재계산 {s[k]}")
else:
    out = f'{d}/summary.json'
    if os.path.exists(out): sys.exit(f'{out} 이 이미 있음 — 덮어쓰지 않음 (--check 로 비교)')
    json.dump(s, open(out, 'w'), ensure_ascii=False, indent=1); print(json.dumps(s, ensure_ascii=False, indent=1))
