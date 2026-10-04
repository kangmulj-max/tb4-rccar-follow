#!/usr/bin/env python3
"""Layout Delta 시뮬 시험: 정답 대장(ledger)대로 회차별 월드를 만든다 (실물 SLAM 월드 + 상자 배치).
  python3 make_round_worlds.py [--base worlds/follow_arena_slam1004.sdf] [--out-dir worlds] [--truth-dir <dir>]
- 보이는 상자 0.45 × 0.45 × 0.5 m (라이다 평면 0.19 m 를 지남), 낮은 상자 0.45 × 0.45 × 0.12 m (라이다 아래 — 2D 라이다의 사각 확인용)
- RC카는 회차 시험에서 치움 (높이 8.5 cm 라 라이다에 안 보이지만 로봇과 부딪힘)
- 회차 n 의 정답 = 회차 n−1 대비 변경 (added / removed / moved, moved 는 새 위치)
"""
import argparse, json, os, re

# 10/4 실물 지도에서 자동 선택: 벽 0.5 m+, 웨이포인트 0.6 m+, 순회 경로 0.45 m+, 서로 0.75 m+ (낮은 상자 l1·l2 는 순회 경로에서 0.42 m+ — 코스트맵에 안 보이므로)
SLOTS = {'s1': (-0.85, -0.75), 's2': (-2.35, -1.6), 's3': (-0.9, -1.5), 's4': (-1.25, 0.4), 's5': (-3.3, 0.45),
         'l1': (-1.45, -0.6), 'l2': (-0.25, -0.9)}
# 회차별 배치: 물체 → 슬롯 (없으면 그 회차에 없음). L 은 낮은 상자.
ROUNDS = [
    {'A': 's1', 'B': 's2', 'C': 's3', 'L': 'l1'},
    {'A': 's4', 'C': 's3', 'D': 's5', 'L': 'l2'},                 # A 이동, B 제거, D 추가, L 이동(낮음)
    {'A': 's4', 'C': 's2', 'B': 's1'},                            # C 이동, D 제거, B 추가, L 제거(낮음)
    {'C': 's2', 'B': 's3', 'E': 's5', 'L': 'l1'},                 # A 제거, B 이동, E 추가, L 추가(낮음)
]
LOW = {'L'}


def box(name, x, y, sx, sy, h):
    g = f'<geometry><box><size>{sx} {sy} {h}</size></box></geometry>'
    return (f'    <model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 0</pose><link name="l">'
            f'<collision name="c"><pose>0 0 {h / 2} 0 0 0</pose>{g}</collision>'
            f'<visual name="v"><pose>0 0 {h / 2} 0 0 0</pose>{g}<material><ambient>0.3 0.35 0.6 1</ambient><diffuse>0.3 0.35 0.6 1</diffuse></material></visual>'
            f'</link></model>\n')


def truth(prev, cur):
    out = []
    for o in sorted(set(prev) | set(cur)):
        low = o in LOW
        if o in prev and o not in cur:
            out.append(dict(obj=o, kind='removed', x=SLOTS[prev[o]][0], y=SLOTS[prev[o]][1], low=low))
        elif o in cur and o not in prev:
            out.append(dict(obj=o, kind='added', x=SLOTS[cur[o]][0], y=SLOTS[cur[o]][1], low=low))
        elif prev[o] != cur[o]:
            out.append(dict(obj=o, kind='moved', x=SLOTS[cur[o]][0], y=SLOTS[cur[o]][1],
                            from_x=SLOTS[prev[o]][0], from_y=SLOTS[prev[o]][1], low=low))
    return out


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--base', default=os.path.join(here, '..', 'worlds', 'follow_arena_slam1004.sdf'))
    ap.add_argument('--out-dir', default=os.path.join(here, '..', 'worlds'))
    ap.add_argument('--truth-dir', default=os.path.join(here, '..', 'worlds', 'rounds_truth'))
    a = ap.parse_args()
    base = open(a.base, encoding='utf-8').read()
    base = re.sub(r'(<model name="rc_car">\s*<pose>)[^<]*(</pose>)', r'\g<1>20 20 0 0 0 0\g<2>', base, count=1)
    os.makedirs(a.truth_dir, exist_ok=True)
    for i, R in enumerate(ROUNDS):
        objs = ''.join(box(f'obj_{o}', *SLOTS[s], 0.45, 0.45, 0.12 if o in LOW else 0.5) for o, s in R.items())
        w = base.replace('  </world>', f'    <!-- Layout Delta 회차 {i:02d} 배치: {R} -->\n' + objs + '  </world>', 1)
        path = os.path.join(a.out_dir, f'ld_round{i:02d}.sdf'); open(path, 'w', encoding='utf-8').write(w)
        if i:
            t = truth(ROUNDS[i - 1], R)
            json.dump(t, open(os.path.join(a.truth_dir, f'truth_round{i:02d}.json'), 'w'), indent=1)
            json.dump([x for x in t if not x['low']], open(os.path.join(a.truth_dir, f'truth_round{i:02d}_lidar.json'), 'w'), indent=1)
        print(path, R)
    json.dump(dict(slots=SLOTS, rounds=ROUNDS, low=sorted(LOW)), open(os.path.join(a.truth_dir, 'ledger.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
