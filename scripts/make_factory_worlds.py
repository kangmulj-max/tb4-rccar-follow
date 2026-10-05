#!/usr/bin/env python3
"""가상 공장 3종 × 월별 레이아웃(M0~M3) 생성 — Layout Delta 순회·변화 판정 시험용.
  python3 make_factory_worlds.py [--out-dir worlds] [--base worlds/follow_arena_slam1004.sdf]
출력 (공장 F = f1 조립 / f2 물류창고 / f3 가공):
  worlds/fac_<F>_m<k>.sdf                가즈보 월드 (world name follow_arena — 런치 호환)
  worlds/factory/<F>/map_m0.yaml/pgm      M0 기준 지도 (라이다 평면 0.193 m 단면을 래스터화 = 회차 0 의 CAD 기준도)
  worlds/factory/<F>/waypoints.yaml       순회 웨이포인트 (M0 여유 공간에서 자동 선택, 월이 바뀌어도 고정)
  worlds/factory/<F>/truth_m<k>.json      M(k-1) → M(k) 정답 대장 (가시성·벽 근접 꼬리표 포함)
  worlds/factory/<F>/geom_m<k>.json       충돌 박스 목록 (주행 여유거리·충돌 분석용)
  worlds/factory/<F>/spec.json            면적·시작 위치·이동 장애물
치수 근거: 한국 표준 파렛트 T11 1.1×1.1 m (높이 0.15 m), 터틀봇 4 라이다 평면 0.193 m (10/4 실측 TF).
"""
import argparse, json, math, os, re
from collections import deque
import numpy as np

LIDAR_Z, RES, ROBOT_R, ROBOT_H = 0.193, 0.05, 0.17, 0.35

# ---------- 물체 형상: (dx, dy, sx, sy, z0, z1) 물체 좌표계 ----------
def solid(sx, sy, h): return [(0, 0, sx, sy, 0, h)]
def table(sx, sy, h=0.88):        # 작업대: 상판 + 다리 4 개 (라이다는 다리만 봄)
    lx, ly = sx / 2 - 0.06, sy / 2 - 0.06
    return [(0, 0, sx, sy, h - 0.05, h)] + [(ix * lx, iy * ly, 0.06, 0.06, 0, h - 0.05) for ix in (-1, 1) for iy in (-1, 1)]
def rack_empty(sx, sy, h=2.0):    # 빈 랙: 기둥 4 개 + 바닥 빔(0.10~0.16 m, 라이다 아래)
    lx, ly = sx / 2 - 0.05, sy / 2 - 0.05
    return ([(ix * lx, iy * ly, 0.08, 0.08, 0, h) for ix in (-1, 1) for iy in (-1, 1)] +
            [(0, iy * ly, sx, 0.05, 0.10, 0.16) for iy in (-1, 1)])
def rack_loaded(sx, sy, h=2.0): return rack_empty(sx, sy, h) + [(0, 0, sx - 0.2, sy - 0.1, 0.16, 1.3)]  # 1단 적재
def fence(length, h=1.8): return [(0, 0, length, 0.04, 0, h)]
def pallet(): return [(0, 0, 1.1, 1.1, 0, 0.15)]                     # T11 빈 파렛트 — 라이다 아래
def pallet_load(h=1.2): return [(0, 0, 1.1, 1.1, 0, h)]               # 적재 파렛트

TYPES = {  # 이름: (형상 함수 결과, 색)
    'conveyor': lambda L: (solid(L, 0.8, 0.9), '0.35 0.45 0.35'),
    'bench': lambda: (table(1.5, 0.75), '0.6 0.5 0.3'),
    'cart': lambda: (solid(0.8, 0.5, 1.0), '0.3 0.4 0.7'),
    'machine': lambda sx=1.6, sy=1.2, h=1.6: (solid(sx, sy, h), '0.5 0.5 0.55'),
    'cnc': lambda: (solid(2.2, 1.6, 2.0), '0.45 0.5 0.6'),
    'cabinet': lambda: (solid(1.0, 0.5, 1.8), '0.55 0.3 0.3'),
    'bin': lambda: (solid(0.7, 0.7, 0.8), '0.35 0.35 0.25'),
    'rack_e': lambda: (rack_empty(2.7, 1.1), '0.7 0.4 0.1'),
    'rack_l': lambda: (rack_loaded(2.7, 1.1), '0.7 0.4 0.1'),
    'pallet': lambda: (pallet(), '0.6 0.45 0.25'),
    'pload': lambda h=1.2: (pallet_load(h), '0.55 0.5 0.4'),
    'fence': lambda L: (fence(L), '0.8 0.75 0.1'),
    'cable': lambda: (solid(1.5, 0.3, 0.05), '0.1 0.1 0.1'),
}

# ---------- 공장 정의 ----------
# structure: 바뀌지 않는 벽·기둥·칸막이 [(x, y, sx, sy, h)]. months: [{id: (type, args, x, y, yaw_deg)}]
def walls(W, H, t=0.2, h=2.5):
    return [(0, H / 2 + t / 2, W + 2 * t, t, h), (0, -H / 2 - t / 2, W + 2 * t, t, h),
            (W / 2 + t / 2, 0, t, H, h), (-W / 2 - t / 2, 0, t, H, h)]
def columns(xs, ys, s=0.4): return [(x, y, s, s, 3.0) for x in xs for y in ys]

def months(base, *edits):
    """base: M0 dict. edits: 월별 변경 {id: spec 또는 None(제거)} — 누적 적용."""
    out = [dict(base)]
    for e in edits:
        m = dict(out[-1])
        for k, v in e.items():
            if v is None: m.pop(k, None)
            else: m[k] = v
        out.append(m)
    return out

F1_BASE = {   # 조립 공장 20 × 14 m: 컨베이어 2 라인, 작업대, 부품 카트, 벽면 랙, 파렛트 적치
    'L1': ('conveyor', [9.0], -2.5, 4.2, 0), 'L2': ('conveyor', [8.0], -3.0, -4.2, 0),
    'WS1': ('bench', [], -5.0, 2.9, 0), 'WS2': ('bench', [], -1.5, 2.9, 0), 'WS3': ('bench', [], -4.5, -2.9, 0),
    'WS4': ('bench', [], -1.0, -2.9, 0),
    'C1': ('cart', [], 3.8, 2.6, 0), 'C2': ('cart', [], 3.8, -2.6, 0),
    'R1': ('rack_l', [], 8.2, 4.5, 90), 'R2': ('rack_l', [], 8.2, 1.2, 90), 'R3': ('rack_e', [], 8.2, -2.1, 90),
    'P1': ('pload', [1.0], 6.0, -5.4, 0), 'P2': ('pallet', [], 4.4, -5.4, 0),
    'M1': ('machine', [], -8.2, 0.0, 90),
}
F1 = dict(name='f1', title='조립 공장', W=20.0, H=14.0, start=(0.0, 0.0, 0.0), intrusions={'C3'},
          structure=walls(20, 14) + columns([-6.0, 0.0, 6.0], [-1.6, 1.6]),
          months=months(F1_BASE,
              {'L2b': ('conveyor', [2.5], 2.4, -4.2, 0), 'WS5': ('bench', [], 2.3, -2.9, 0),     # M1: 2 라인 연장 + 작업대 증설
               'C1': ('cart', [], 4.6, 0.9, 0), 'P3': ('pallet', [], 6.0, 5.6, 0)},              #     카트 이동, 빈 파렛트(낮음) 추가
              {'WS2': None, 'C3': ('cart', [], 2.2, -1.0, 0), 'R3': ('rack_l', [], 8.2, -2.1, 90),  # M2: 작업대 철거, 통로에 카트, 빈 랙 적재
               'P2': None, 'WS1': ('bench', [], -6.8, 2.9, 0)},                                      #     파렛트(낮음) 제거, 작업대 이동
              {'L1': ('conveyor', [6.0], -4.0, 4.2, 0), 'MX': ('machine', [1.8, 1.4, 1.7], 3.0, 4.6, 0),  # M3: 1 라인 단축(이동으로 보임), 신규 설비
               'C1': None, 'P4': ('pallet', [], -7.6, -5.6, 0), 'C3': None}))                    #     카트 철수, 빈 파렛트 추가

F2_BASE = {   # 물류 창고 20 × 14 m: 랙 3 열(베이 2.7 m), 동쪽 입출고 적치 구역
    **{f'A{i}': ('rack_l' if i % 2 else 'rack_e', [], -8.0 + 2.8 * i, 4.6, 0) for i in range(5)},
    **{f'B{i}': ('rack_l', [], -8.0 + 2.8 * i, 0.0, 0) for i in range(5)},
    **{f'C{i}': ('rack_l' if i != 2 else 'rack_e', [], -8.0 + 2.8 * i, -4.6, 0) for i in range(5)},
    'S1': ('pload', [1.2], 7.0, 4.5, 0), 'S2': ('pload', [1.5], 8.4, 4.5, 0), 'S3': ('pload', [1.0], 7.0, -1.5, 0),
    'S4': ('pallet', [], 8.4, -4.5, 0),
}
F2 = dict(name='f2', title='물류 창고', W=20.0, H=14.0, start=(5.6, 0.0, 180.0), intrusions={'T1', 'T2'},
          structure=walls(20, 14) + columns([-1.0], [-6.2, 6.2]),
          forklift=dict(size=(2.3, 1.2, 2.0), path=[(-7.0, 3.2), (3.0, 3.2)], speed=0.6, pause=6.0),  # 통로 A-B 랙 쪽 차선 왕복 (로봇과 통로 공유)
          months=months(F2_BASE,
              {'A0': ('rack_l', [], -8.0, 4.6, 0), 'B2': ('rack_e', [], -2.4, 0.0, 0),            # M1: 베이 적재/출고
               'S1': None, 'S5': ('pload', [1.4], 7.0, 1.5, 0), 'S4': ('pload', [1.0], 8.4, -4.5, 0)},
              {'T1': ('pload', [1.2], -3.4, -2.6, 0), 'T2': ('pload', [1.2], -2.2, -2.6, 0),       # M2: 통로 C 임시 적치(통로 좁아짐)
               'S2': ('pload', [1.5], 8.4, 1.5, 0), 'C2': ('rack_l', [], -2.4, -4.6, 0)},
              {'T1': None, 'T2': None, 'A5': ('rack_l', [], 6.0, 4.6, 0),                        # M3: 임시 적치 해소, 랙 베이 증설
               'S3': None, 'S6': ('pallet', [], 7.0, -3.0, 0), 'B4': ('rack_e', [], 3.2, 0.0, 0)}))

F3_BASE = {   # 가공 공장 18 × 12 m: CNC 2 열, 안전 펜스, 벽면 공구함, 칩 수거함, 바닥 케이블 보호대
    'N1': ('cnc', [], -6.0, 3.4, 0), 'N2': ('cnc', [], -2.6, 3.4, 0), 'N3': ('cnc', [], 0.8, 3.4, 0),
    'N4': ('cnc', [], -6.0, -3.4, 0), 'N5': ('cnc', [], -2.6, -3.4, 0),
    'FN1': ('fence', [3.0], 3.4, 2.0, 90), 'FN2': ('fence', [3.0], 3.4, -2.0, 90),
    'K1': ('cabinet', [], -8.65, 0.8, 90), 'K2': ('cabinet', [], 8.65, 3.0, 90), 'K3': ('cabinet', [], 5.0, 5.7, 0),
    'BN1': ('bin', [], -4.3, 1.4, 0), 'BN2': ('bin', [], -0.9, -1.4, 0),
    'CB1': ('cable', [], 6.5, 0.0, 90), 'PL1': ('pallet', [], 6.6, -4.6, 0),
}
F3 = dict(name='f3', title='가공 공장', W=18.0, H=12.0, start=(-0.8, 0.0, 0.0),
          structure=walls(18, 12) + [(6.8, -5.0, 0.15, 2.0, 2.5), (8.0, -4.05, 2.2, 0.15, 2.5)],  # 사무실 칸막이
          months=months(F3_BASE,
              {'N6': ('cnc', [], 0.8, -3.4, 0), 'BN3': ('bin', [], 2.3, -1.6, 0),                  # M1: CNC 신규 설치, 수거함 추가
               'PL1': None},
              {'N2': ('cnc', [], -2.0, 3.6, 0), 'FN1': ('fence', [3.0], 4.6, 2.0, 90),              # M2: CNC 재배치(0.6 m), 펜스 이설
               'BN1': ('bin', [], -4.3, -1.4, 0), 'CB2': ('cable', [], -0.8, 1.6, 0)},
              {'K2': ('cabinet', [], 8.65, -0.8, 90), 'BN2': None, 'PL2': ('pallet', [], 6.4, 4.2, 0),  # M3: 공구함 이동(벽 붙음), 파렛트(낮음)
               'N4': None}))

FACTORIES = [F1, F2, F3]

# ---------- 기하 ----------
def parts(spec):
    typ, args, x, y, yaw = spec
    shape, color = TYPES[typ](*args)
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    out = []
    for dx, dy, sx, sy, z0, z1 in shape:
        out.append(dict(x=x + c * dx - s * dy, y=y + s * dx + c * dy, yaw=yaw, sx=sx, sy=sy, z0=z0, z1=z1))
    return out, color

def rect_mask(shape_hw, origin, b, grow=0.0):
    """회전 박스 b 를 격자에 칠함 (중심 샘플)"""
    H, W = shape_hw
    ys, xs = np.mgrid[0:H, 0:W]
    X = origin[0] + (xs + 0.5) * RES - b['x']; Y = origin[1] + (ys + 0.5) * RES - b['y']
    c, s = math.cos(math.radians(b['yaw'])), math.sin(math.radians(b['yaw']))
    u, v = c * X + s * Y, -s * X + c * Y
    g = grow + RES / 2      # 칸이 박스에 조금이라도 걸리면 칠함 — 얇은 펜스(0.04 m)가 칸 중심 사이로 빠지지 않게
    return (np.abs(u) <= b['sx'] / 2 + g) & (np.abs(v) <= b['sy'] / 2 + g)

def boxes_of(F, m):
    st = [dict(x=x, y=y, yaw=0, sx=sx, sy=sy, z0=0, z1=h, obj='_structure') for x, y, sx, sy, h in F['structure']]
    ob = []
    for oid, spec in m.items():
        ps, _ = parts(spec)
        for p in ps: p['obj'] = oid; ob.append(p)
    return st, ob

def grid_frame(F):
    W, H = F['W'] + 2.0, F['H'] + 2.0
    origin = (-W / 2, -H / 2)
    return (int(round(H / RES)), int(round(W / RES))), origin

def raster(F, m, zsel):
    shp, origin = grid_frame(F)
    st, ob = boxes_of(F, m)
    occ = np.zeros(shp, bool)
    for b in st + ob:
        if zsel(b): occ |= rect_mask(shp, origin, b)
    return occ, origin

def edt(occ):
    import cv2
    return cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * RES

# ---------- SDF ----------
def sdf_model(name, boxes, color, static=True):
    body = ''
    for i, b in enumerate(boxes):
        h = b['z1'] - b['z0']
        g = f'<geometry><box><size>{b["sx"]:.3f} {b["sy"]:.3f} {h:.3f}</size></box></geometry>'
        pose = f'<pose>{b["x"]:.3f} {b["y"]:.3f} {b["z0"] + h / 2:.3f} 0 0 {math.radians(b["yaw"]):.4f}</pose>'
        body += (f'<collision name="c{i}">{pose}{g}</collision>'
                 f'<visual name="v{i}">{pose}{g}<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>')
    return (f'    <model name="{name}"><static>{"true" if static else "false"}</static><pose>0 0 0 0 0 0</pose>'
            f'<link name="l">{body}</link></model>\n')

def make_sdf(base, F, m):
    s = re.sub(r'\s*<model name="slam_walls">.*?</model>\n', '\n', base, count=1, flags=re.S)
    for i, n in enumerate(['rc_car', 'dummy_box', 'pillar', 'pole', 'webcam_cam', 'overhead_cam']):   # 기존 시험장 물체(웹캠 삼각대 포함)는 멀리 치움
        s = re.sub(rf'(<model name="{n}">.*?<pose>)[^<]*(</pose>)', rf'\g<1>{60 + 3 * i} 60 0 0 0 0\g<2>', s, count=1, flags=re.S)
    s = s.replace('<size>20 20</size>', '<size>80 80</size>')
    st, _ = boxes_of(F, {})
    add = sdf_model('structure', st, '0.75 0.75 0.72')
    for oid, spec in sorted(m.items()):
        ps, color = parts(spec)
        add += sdf_model(f'obj_{oid}', ps, color)
    if F.get('forklift'):   # 이동 장애물: 물리 충돌체는 멀리 두고, 라이다는 /forklift/pose 로 위치를 받음 (fac_forklift.py)
        sx, sy, h = F['forklift']['size']
        add += sdf_model('forklift', [dict(x=0, y=0, yaw=0, sx=sx, sy=sy, z0=0, z1=h)], '0.9 0.6 0.1', static=False).replace(
            '<pose>0 0 0 0 0 0</pose>', '<pose>70 70 0 0 0 0</pose>', 1)
    return s.replace('  </world>', f'    <!-- {F["title"]} -->\n' + add + '  </world>', 1)

# ---------- 지도·웨이포인트·정답 ----------
def save_map(path, occ, origin, known):
    H, W = occ.shape
    img = np.full((H, W), 205, np.uint8); img[known] = 254; img[occ] = 0
    img = np.flipud(img)
    with open(path + '.pgm', 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode()); f.write(img.tobytes())
    open(path + '.yaml', 'w').write(f'image: {os.path.basename(path)}.pgm\nresolution: {RES}\norigin: [{origin[0]}, {origin[1]}, 0]\n'
                                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')

def bfs(passable, start):
    H, W = passable.shape; d = np.full((H, W), -1, int); q = deque([start]); d[start] = 0
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W and passable[rr, cc] and d[rr, cc] < 0:
                d[rr, cc] = d[r, c] + 1; q.append((rr, cc))
    return d

def cell(origin, x, y): return int((y - origin[1]) / RES), int((x - origin[0]) / RES)

def waypoints(F, m0, spacing=3.5, clear=0.65):
    # 통로 = 실제 공장의 바닥 구획선(운반·보행 통로)에 해당. 시험 월드에서는 4 개월 내내 비어 있는 칸으로 정의하되,
    # 의도적 통로 침범 물체(intrusions: 통로 카트, 임시 적치)는 통로 정의에서 빼서 그 영향을 시험한다.
    body, origin = raster(F, m0, lambda b: b['z0'] < ROBOT_H)      # 로봇 몸체가 닿는 모든 물체 (낮은 물체 포함)
    for m in F['months'][1:]:
        keep = {k: v for k, v in m.items() if k not in F.get('intrusions', set())}
        body |= raster(F, keep, lambda b: b['z0'] < ROBOT_H)[0]
    dist = edt(body)
    sx, sy, syaw = F['start']
    reach = bfs(dist >= 0.32, cell(origin, sx, sy))
    cands = []
    for y in np.arange(-F['H'] / 2 + 1.0, F['H'] / 2 - 0.9, spacing):
        for x in np.arange(-F['W'] / 2 + 1.0, F['W'] / 2 - 0.9, spacing):
            # 격자점 주변 0.8 m 안에서 가장 여유 있는 칸
            r0, c0 = cell(origin, x, y); k = int(0.8 / RES)
            sub = dist[max(0, r0 - k):r0 + k, max(0, c0 - k):c0 + k]
            if sub.size == 0: continue
            rr, cc = np.unravel_index(np.argmax(sub), sub.shape); rr += max(0, r0 - k); cc += max(0, c0 - k)
            if dist[rr, cc] >= clear and reach[rr, cc] >= 0:
                cands.append((origin[0] + (cc + 0.5) * RES, origin[1] + (rr + 0.5) * RES))
    cands = [p for i, p in enumerate(cands) if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 1.2 for q in cands[:i])]
    # 측지 거리 기준 최근접 순회
    tour, cur = [], (sx, sy)
    left = list(cands)
    while left:
        d = bfs(dist >= 0.32, cell(origin, *cur))
        nxt = min(left, key=lambda p: d[cell(origin, *p)] if d[cell(origin, *p)] >= 0 else 1e9)
        left.remove(nxt); tour.append(nxt); cur = nxt
    wps, px, py = [], sx, sy
    for i, (x, y) in enumerate(tour):
        wps.append(dict(id=f'W{i + 1:02d}', x=round(float(x), 2), y=round(float(y), 2),
                        yaw_deg=round(math.degrees(math.atan2(y - py, x - px)))))
        px, py = x, y
    return wps

def visible(spec):
    ps, _ = parts(spec)
    return any(p['z0'] <= LIDAR_Z <= p['z1'] for p in ps)

def near_structure(F, spec, others, margin=0.3):
    """물체 둘레 margin 안에 벽·기둥·다른 물체가 있는가 (diff 의 벽 어긋남 규칙에 걸릴 수 있음)"""
    shp, origin = grid_frame(F)
    me = np.zeros(shp, bool)
    for p in parts(spec)[0]:
        if p['z0'] <= LIDAR_Z <= p['z1']: me |= rect_mask(shp, origin, p)
    if not me.any(): return False
    ring = np.zeros(shp, bool)
    for p in parts(spec)[0]: ring |= rect_mask(shp, origin, p, grow=margin)
    st, ob = boxes_of(F, others)
    for b in st + ob:
        if b['z0'] <= LIDAR_Z <= b['z1'] and (rect_mask(shp, origin, b) & ring & ~me).any(): return True
    return False

def truth(F, prev, cur):
    out = []
    for o in sorted(set(prev) | set(cur)):
        a, b = prev.get(o), cur.get(o)
        if a == b: continue
        spec = b or a
        others = {k: v for k, v in (cur if b else prev).items() if k != o}
        tag = dict(obj=o, type=spec[0], visible=visible(spec), legs_only=spec[0] == 'bench',
                   near_structure=near_structure(F, spec, others))
        if a and not b: out.append(dict(kind='removed', x=a[2], y=a[3], **tag))
        elif b and not a: out.append(dict(kind='added', x=b[2], y=b[3], **tag))
        elif (a[2], a[3]) != (b[2], b[3]): out.append(dict(kind='moved', x=b[2], y=b[3], from_x=a[2], from_y=a[3], **tag))
        else:   # 같은 자리에서 형상만 바뀜 (예: 빈 랙 → 적재)
            out.append(dict(kind='changed', x=b[2], y=b[3], prev_type=a[0], **tag))
    return out

def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--out-dir', default=os.path.join(here, '..', 'worlds'))
    ap.add_argument('--base', default=os.path.join(here, '..', 'worlds', 'follow_arena_slam1004.sdf'))
    a = ap.parse_args()
    base = open(a.base, encoding='utf-8').read()
    for F in FACTORIES:
        d = os.path.join(a.out_dir, 'factory', F['name']); os.makedirs(d, exist_ok=True)
        for k, m in enumerate(F['months']):
            open(os.path.join(a.out_dir, f'fac_{F["name"]}_m{k}.sdf'), 'w', encoding='utf-8').write(make_sdf(base, F, m))
            st, ob = boxes_of(F, m)
            json.dump(dict(structure=st, objects=ob), open(os.path.join(d, f'geom_m{k}.json'), 'w'), indent=0)
            if k: json.dump(truth(F, F['months'][k - 1], m), open(os.path.join(d, f'truth_m{k}.json'), 'w', encoding='utf-8'),
                            ensure_ascii=False, indent=1)
        occ, origin = raster(F, F['months'][0], lambda b: b['z0'] <= LIDAR_Z <= b['z1'])
        inside = np.zeros_like(occ)
        r0, c0 = cell(origin, -F['W'] / 2, -F['H'] / 2); r1, c1 = cell(origin, F['W'] / 2, F['H'] / 2)
        inside[r0:r1, c0:c1] = True
        save_map(os.path.join(d, 'map_m0'), occ, origin, inside | occ)
        wps = waypoints(F, F['months'][0])
        with open(os.path.join(d, 'waypoints.yaml'), 'w', encoding='utf-8') as f:
            f.write(f'# {F["title"]} — 통로(4 개월 내내 빈 칸, 통로 침범 물체 제외) 위 자동 선택, 벽·물체 0.65 m+, 간격 3.5 m, 측지 최근접 순회. 월이 바뀌어도 고정\nwaypoints:\n')
            for w in wps: f.write(f'  - {{id: {w["id"]}, x: {w["x"]}, y: {w["y"]}, yaw_deg: {w["yaw_deg"]}}}\n')
        # 월별 웨이포인트 여유거리 (막힘 예측)
        block = {}
        for k, m in enumerate(F['months']):
            body, _ = raster(F, m, lambda b: b['z0'] < ROBOT_H)
            dist = edt(body)
            block[f'm{k}'] = [w['id'] for w in wps if dist[cell(origin, w['x'], w['y'])] < 0.32]
        free_area = float(((~occ) & inside).sum() * RES * RES)
        # 도크에서 출발: 로봇은 도크를 바라본 채 시작하므로, 첫 웨이포인트 반대쪽을 보게 두고 후진으로 도크를 벗어난다
        sx, sy, _ = F['start']
        syaw = round((math.degrees(math.atan2(wps[0]['y'] - sy, wps[0]['x'] - sx)) + 180 + 180) % 360 - 180)
        spec = dict(title=F['title'], W=F['W'], H=F['H'], floor_m2=F['W'] * F['H'], free_m2_lidar=round(free_area, 1),
                    start=(sx, sy, syaw), forklift=F.get('forklift'), n_waypoints=len(wps), blocked_waypoints=block,
                    objects_per_month=[len(m) for m in F['months']])
        json.dump(spec, open(os.path.join(d, 'spec.json'), 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        nt = [json.load(open(os.path.join(d, f'truth_m{k}.json'), encoding='utf-8')) for k in (1, 2, 3)]
        print(f'{F["name"]} {F["title"]}: {len(wps)} wps, changes/month {[len(t) for t in nt]}, '
              f'invisible {[sum(not c["visible"] for c in t) for t in nt]}, near {[sum(c["near_structure"] for c in t) for t in nt]}, blocked {block}')

if __name__ == '__main__':
    main()
