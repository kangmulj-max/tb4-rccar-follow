#!/usr/bin/env python3
"""월드 파일의 RC카 모델을 실측 치수로 줄인다 (10/4 실측: 길이 0.25 × 폭 0.08 × 높이 0.085 m, measurements/field_env.sh).
  python3 apply_real_specs.py worlds/follow_arena.sdf worlds/follow_arena_bright.sdf ...
- 모델 전체 외곽(바퀴·스페어타이어 포함, 원본 약 0.28 × 0.156 × 0.139 m)이 실측 외곽에 맞도록 축별로 비율 조정.
  (실측 '밑면 가로 25 · 세로 8 · 높이 8.5 cm' 를 외곽으로 봄 — 웹캠 3D 상자 맞춤도 같은 외곽을 씀)
- 위치(pose) x·y·z, 상자 크기, 원기둥(축 방향 길이 / 단면 반지름)을 각각 맞는 축의 비율로.
- 이미 적용된 파일은 건너뜀 (표시 주석으로 판단).
"""
import re, sys
REAL = (0.25, 0.08, 0.085)
MARK = '<!-- rc_car: 실측 치수 적용 (apply_real_specs.py) -->'


def scale_block(blk):
    # 현재 외곽 계산
    xs, ys, zs = [], [], []
    for m in re.finditer(r'<pose>([^<]*)</pose>\s*(?:<geometry>|<mass>|<inertia>)?\s*<geometry>(.*?)</geometry>|<(?:visual|collision) name="[^"]*">\s*<pose>([^<]*)</pose>\s*<geometry>(.*?)</geometry>', blk, re.S):
        pose = (m.group(1) or m.group(3)).split(); geo = m.group(2) or m.group(4)
        x, y, z = map(float, pose[:3]); roll, pitch = float(pose[3]), float(pose[4])
        b = re.search(r'<box><size>([^<]*)</size>', geo); c = re.search(r'<radius>([^<]*)</radius><length>([^<]*)</length>', geo)
        if b: hx, hy, hz = [float(v) / 2 for v in b.group(1).split()]
        elif c:
            r, L = float(c.group(1)), float(c.group(2)) / 2
            hx, hy, hz = (r, L, r) if abs(roll) > 1 else (L, r, r) if abs(pitch) > 1 else (r, r, L)
        else: continue
        xs += [x - hx, x + hx]; ys += [y - hy, y + hy]; zs += [z - hz, z + hz]
    ext = (max(xs) - min(xs), max(ys) - min(ys), max(zs))
    s = [REAL[i] / ext[i] for i in range(3)]

    def pose_fix(m):
        v = m.group(1).split(); x, y, z = float(v[0]) * s[0], float(v[1]) * s[1], float(v[2]) * s[2]
        return f'<pose>{x:.4f} {y:.4f} {z:.4f} {" ".join(v[3:])}</pose>'
    out = []
    # 요소 단위로: pose 와 geometry 를 같이 보고 원기둥 축을 정함
    for part in re.split(r'(<(?:visual|collision|inertial) name="[^"]*">.*?</(?:visual|collision)>|<inertial>.*?</inertial>)', blk, flags=re.S):
        if part.startswith(('<visual', '<collision', '<inertial')):
            pm = re.search(r'<pose>([^<]*)</pose>', part); pose = pm.group(1).split() if pm else ['0'] * 6
            roll, pitch = float(pose[3]), float(pose[4])
            part = re.sub(r'<pose>([^<]*)</pose>', pose_fix, part, count=1)
            part = re.sub(r'<box><size>([^<]*)</size>', lambda m: '<box><size>' + ' '.join(f'{float(v) * s[i]:.4f}' for i, v in enumerate(m.group(1).split())) + '</size>', part)
            if '<cylinder>' in part:
                ax = 1 if abs(roll) > 1 else 0 if abs(pitch) > 1 else 2          # 원기둥 축이 가리키는 모델 축
                rs = min(s[i] for i in range(3) if i != ax)
                part = re.sub(r'<radius>([^<]*)</radius><length>([^<]*)</length>',
                              lambda m: f'<radius>{float(m.group(1)) * rs:.4f}</radius><length>{float(m.group(2)) * s[ax]:.4f}</length>', part)
        out.append(part)
    return ''.join(out), ext, s


for path in sys.argv[1:]:
    w = open(path, encoding='utf-8').read()
    if MARK in w: print(f'{path}: 이미 적용됨'); continue
    m = re.search(r'(<model name="rc_car">\s*<pose>[^<]*</pose>)(.*?)(<plugin)', w, re.S)
    if not m: print(f'{path}: rc_car 없음'); continue
    blk, ext, s = scale_block(m.group(2))
    w = w[:m.start()] + f'    {MARK}\n    ' + m.group(1).lstrip() + blk + m.group(3) + w[m.end():]
    open(path, 'w', encoding='utf-8').write(w)
    print(f'{path}: 외곽 {ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f} m → {REAL} (비율 {s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f})')
