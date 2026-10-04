#!/usr/bin/env python3
"""실물 SLAM 지도(pgm + yaml) → 가즈보 월드. 지도의 '막힌 칸'을 벽 상자로 바꿔 넣는다.
  python3 make_slam_world.py <map.yaml> [--name follow_arena_slam1004] [--wall-h 0.5] [--min-blob 2]
결과: worlds/<name>.sdf, worlds/maps/<지도>.{pgm,yaml} (사본), worlds/<name>_check.png (지도 위에 벽 상자 겹친 확인 그림)
- 좌표는 **지도 좌표 그대로** (월드 원점 = SLAM map 원점) → 같은 yaml 로 시뮬에서 AMCL/Nav2 를 쓸 수 있다.
- 기본 월드(follow_arena.sdf)의 바닥·조명·RC카·방해물·탑뷰 카메라는 그대로, 8 x 6 m 펜스만 지도 벽으로 바꾼다.
  월드 이름(follow_arena)도 그대로라 experiment.sh 등의 set_pose 가 동작한다.
- 지도 칸 0.05 m 를 행 단위로 이어 붙이고 같은 폭의 행을 위아래로 합쳐 상자 수를 줄인다.
- 미탐색(회색) 칸은 비어 있는 것으로 둔다 (지도 바깥 경계는 실제 벽이 아님) — 지도에 벽이 없는 곳으로는 로봇이 나갈 수 있음.
"""
import argparse, math, os, re, shutil, sys
import numpy as np, cv2

ap = argparse.ArgumentParser()
ap.add_argument('map_yaml'); ap.add_argument('--name', default='follow_arena_slam1004')
ap.add_argument('--base', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds', 'follow_arena.sdf'))
ap.add_argument('--wall-h', type=float, default=0.5); ap.add_argument('--min-blob', type=int, default=2, help='이보다 작은 막힌 덩어리(칸 수)는 잡음으로 버림')
ap.add_argument('--start', type=float, nargs=3, default=None, metavar=('X', 'Y', 'YAW_DEG'), help='로봇 시작 자세 (기본: 지도 원점 근처 SLAM 시작점은 직접 지정)')
a = ap.parse_args()

# ---- 지도 읽기 (map_server 형식)
Y = {}
for line in open(a.map_yaml, encoding='utf-8'):
    if ':' in line: k, v = line.split(':', 1); Y[k.strip()] = v.strip()
res = float(Y['resolution']); ox, oy = [float(t) for t in re.findall(r'-?[\d.]+', Y['origin'])[:2]]
pgm = os.path.join(os.path.dirname(a.map_yaml), Y['image'])
img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE); H, W = img.shape
neg = int(Y.get('negate', 0)); occ_t = float(Y.get('occupied_thresh', 0.65))
p_occ = (img.astype(float) / 255.0) if neg else (255.0 - img) / 255.0       # map_server 규칙: 점유 확률
occ = (p_occ > occ_t).astype(np.uint8)
n, lab, st, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
keep = np.zeros_like(occ)
for i in range(1, n):
    if st[i, cv2.CC_STAT_AREA] >= a.min_blob: keep[lab == i] = 1
print(f'지도 {W}x{H} 칸, 해상도 {res} m, 원점 ({ox}, {oy}). 막힌 칸 {occ.sum()} → 잡음 제거 후 {keep.sum()}')

# ---- 칸 → 상자 (행 단위 연속 구간 → 같은 구간끼리 세로로 합침)
boxes = []; open_runs = {}
for r in range(H):
    runs = []; c = 0
    while c < W:
        if keep[r, c]:
            c0 = c
            while c < W and keep[r, c]: c += 1
            runs.append((c0, c))
        else: c += 1
    nxt = {}
    for run in runs:
        nxt[run] = (open_runs.pop(run)[0], r) if run in open_runs else (r, r)
    for run, (r0, r1) in open_runs.items(): boxes.append((run[0], run[1], r0, r1))
    open_runs = {k: (v[0], r) for k, v in nxt.items()}
for run, (r0, r1) in open_runs.items(): boxes.append((run[0], run[1], r0, r1))
def to_world(c0, c1, r0, r1):          # 픽셀 (열 c, 행 r; 행 0 = 지도 위쪽) → 월드 중심·크기
    cx = ox + (c0 + c1) / 2 * res; cy = oy + (H - (r0 + r1 + 1) / 2) * res
    return cx, cy, (c1 - c0) * res, (r1 - r0 + 1) * res
print(f'벽 상자 {len(boxes)} 개')

# ---- 월드 만들기: 기본 월드에서 fence 모델만 교체
base = open(a.base, encoding='utf-8').read()
m = re.search(r'\n\s*<!-- 나무 펜스.*?-->\s*\n\s*<model name="fence">.*?</model>\n', base, re.S) or re.search(r'\n\s*<model name="fence">.*?</model>\n', base, re.S)
if not m: sys.exit('기본 월드에서 fence 모델을 못 찾음')
mat = '<material><ambient>0.66 0.52 0.36 1</ambient><diffuse>0.78 0.62 0.43 1</diffuse></material>'
parts = []
for i, b in enumerate(boxes):
    cx, cy, sx, sy = to_world(*b); z = a.wall_h / 2
    g = f'<geometry><box><size>{sx:.3f} {sy:.3f} {a.wall_h}</size></box></geometry>'
    parts.append(f'        <collision name="c{i}"><pose>{cx:.3f} {cy:.3f} {z} 0 0 0</pose>{g}</collision>\n'
                 f'        <visual name="v{i}"><pose>{cx:.3f} {cy:.3f} {z} 0 0 0</pose>{g}{mat}</visual>\n')
walls = (f'\n    <!-- 실물 SLAM 지도 {os.path.basename(pgm)} 의 막힌 칸 → 벽 (높이 {a.wall_h} m). 좌표 = 지도 좌표 -->\n'
         f'    <model name="slam_walls">\n      <static>true</static>\n      <link name="link">\n' + ''.join(parts) + '      </link>\n    </model>\n')
world = base[:m.start()] + walls + base[m.end():]
# 시작 자세: 로봇은 launch 의 x, y, yaw 로. RC카는 로봇 정면 1.2 m, 탑뷰 카메라는 벽 상자들 가운데 위로
if a.start:
    sx0, sy0, syaw = a.start[0], a.start[1], math.radians(a.start[2])
    cx, cy = sx0 + 1.2 * math.cos(syaw), sy0 + 1.2 * math.sin(syaw)
    world = re.sub(r'(<model name="rc_car">\s*<pose>)[^<]*(</pose>)', lambda mm: f'{mm.group(1)}{cx:.3f} {cy:.3f} 0 0 0 {syaw:.4f}{mm.group(2)}', world, count=1)
ys, xs = np.nonzero(keep)
mx, my = ox + xs.mean() * res, oy + (H - ys.mean()) * res
world = re.sub(r'(<model name="overhead_cam">\s*<static>true</static>\s*<pose>)[^<]*(</pose>)', lambda mm: f'{mm.group(1)}{mx:.2f} {my:.2f} 9.0 0 1.5708 0{mm.group(2)}', world, count=1)
world = world.replace('<world name="follow_arena">', f'<!-- {a.name}: make_slam_world.py 로 생성 ({os.path.basename(a.map_yaml)}) -->\n  <world name="follow_arena">', 1)
wd = os.path.dirname(os.path.abspath(a.base)); out = os.path.join(wd, a.name + '.sdf')
open(out, 'w', encoding='utf-8').write(world)
md = os.path.join(wd, 'maps'); os.makedirs(md, exist_ok=True)
for f in (pgm, a.map_yaml):
    if os.path.abspath(os.path.dirname(f)) != os.path.abspath(md): shutil.copy(f, md)
print(f'월드: {out}\n지도 사본: {md}/{os.path.basename(pgm)}')

# ---- 확인 그림: 지도(회색) 위에 상자(빨강 테두리), 시작 자세
S = 6; vis = cv2.cvtColor(cv2.resize(img, (W * S, H * S), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
for b in boxes:
    cv2.rectangle(vis, (b[0] * S, b[2] * S), (b[1] * S - 1, (b[3] + 1) * S - 1), (0, 0, 255), 1)
def px(x, y): return int((x - ox) / res * S), int((H - (y - oy) / res) * S)
if a.start:
    p0 = px(sx0, sy0); cv2.circle(vis, p0, 10, (255, 120, 0), 2); cv2.line(vis, p0, px(sx0 + 0.4 * math.cos(syaw), sy0 + 0.4 * math.sin(syaw)), (255, 120, 0), 2)
    cv2.circle(vis, px(cx, cy), 6, (0, 160, 0), -1)
cv2.circle(vis, px(0, 0), 5, (0, 0, 0), -1)
cv2.imwrite(os.path.join(wd, a.name + '_check.png'), vis)
