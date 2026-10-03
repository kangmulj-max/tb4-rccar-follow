#!/usr/bin/env python3
"""
'움직이는 RC카를 따라가는가' 지표 (logs/<tag>.csv 기반).
  python3 follow_metrics.py [tag ...]      (인자 없으면 logs/v3*_*.csv 전부)
 - follow_pct  : RC카가 움직이는(>0.03 m/s) 시간 중, 상태 TRACKING 이고 카메라↔RC카 거리가 target±band 인 비율
 - lost_events : 첫 TRACKING 이후 SEARCH/IDLE 로 떨어진 횟수 (상태 열 기준)
 - lost_s      : RC카가 움직이는 동안 TRACKING/REID 가 아니었던 시간 합
 - speed_ratio : RC카 이동 중 로봇 평균 전진속도 / RC카 평균 속도
 - far_max     : 최대 카메라↔RC카 거리 (멀어지면 = 못 따라감)
"""
import csv, glob, math, os, sys
L = os.path.expanduser('~/tb4_follow_sim/logs'); TARGET, BAND = 1.0, 0.6


def metrics(path):
    r = list(csv.DictReader(open(path)))
    if len(r) < 10: return None
    last = 'IDLE'                       # 구버전 모니터가 'camera timeout -> stop' 을 상태로 기록한 행 복원
    for x in r:
        if x['state'] in ('IDLE', 'TRACKING', 'REID', 'SEARCH'): last = x['state']
        else: x['state'] = last
    mv, ok, lost_s, rs, cs = 0, 0, 0.0, [], []
    started = False; lost_ev = 0; prev_state = 'IDLE'
    for a, b in zip(r, r[1:]):
        dt = float(b['t']) - float(a['t'])
        if dt <= 0: continue
        cv = math.hypot(float(b['carx']) - float(a['carx']), float(b['cary']) - float(a['cary'])) / dt
        st = b['state']
        if st == 'TRACKING': started = True
        if started and st in ('SEARCH', 'IDLE') and prev_state not in ('SEARCH', 'IDLE'): lost_ev += 1
        prev_state = st
        if cv > 0.03 and started:
            mv += dt; cs.append(cv); rs.append(max(0.0, float(b['gt_speed'])))
            if st == 'TRACKING' and abs(float(b['cam_car']) - TARGET) < BAND: ok += dt
            if st not in ('TRACKING', 'REID'): lost_s += dt
    if mv == 0: return {'follow_pct': None, 'lost_events': lost_ev, 'lost_s': 0, 'speed_ratio': None,
                        'far_max': max(float(x['cam_car']) for x in r), 'moving_s': 0}
    return {'follow_pct': round(100 * ok / mv, 1), 'lost_events': lost_ev, 'lost_s': round(lost_s, 1),
            'speed_ratio': round(sum(rs) / max(sum(cs), 1e-6), 2), 'far_max': round(max(float(x['cam_car']) for x in r), 2),
            'moving_s': round(mv, 1)}


if __name__ == '__main__':
    tags = sys.argv[1:] or sorted(os.path.basename(f)[:-4] for f in glob.glob(f'{L}/v3*_*.csv'))
    print(f'{"tag":34s} {"follow%":>8} {"lost#":>6} {"lost_s":>7} {"spd_ratio":>9} {"far_max":>8} {"moving_s":>8}')
    for t in tags:
        m = metrics(f'{L}/{t}.csv')
        if m: print(f"{t:34s} {str(m['follow_pct']):>8} {m['lost_events']:>6} {m['lost_s']:>7} {str(m['speed_ratio']):>9} {m['far_max']:>8} {m['moving_s']:>8}")
