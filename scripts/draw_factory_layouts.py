#!/usr/bin/env python3
"""가상 공장 월별 배치도 (평면) — 구조물 회색, 물체(라이다 보임) 파랑, 라이다 아래 물체 주황 점선, 변경 물체 빨강 테두리, 순회 경로·웨이포인트.
  python3 draw_factory_layouts.py [worlds/factory]  → <F>/layout_m<k>.png, layouts_all.png"""
import json, math, os, sys
import cv2, numpy as np, yaml

D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds', 'factory')
PX, LZ = 40, 0.193                       # 1 m = 40 px

def draw(F, k):
    spec = json.load(open(f'{D}/{F}/spec.json', encoding='utf-8'))
    g = json.load(open(f'{D}/{F}/geom_m{k}.json'))
    tr = json.load(open(f'{D}/{F}/truth_m{k}.json', encoding='utf-8')) if k else []
    changed = {t['obj'] for t in tr}
    W, H = spec['W'] + 2, spec['H'] + 2
    img = np.full((int(H * PX) + 40, int(W * PX), 3), 255, np.uint8)
    to = lambda x, y: (int((x + W / 2) * PX), int((H / 2 - y) * PX))
    def poly(b):
        c, s = math.cos(math.radians(b['yaw'])), math.sin(math.radians(b['yaw']))
        pts = [(b['x'] + c * u - s * v, b['y'] + s * u + c * v) for u, v in
               ((-b['sx'] / 2, -b['sy'] / 2), (b['sx'] / 2, -b['sy'] / 2), (b['sx'] / 2, b['sy'] / 2), (-b['sx'] / 2, b['sy'] / 2))]
        return np.array([to(*p) for p in pts], np.int32)
    for b in g['structure']: cv2.fillPoly(img, [poly(b)], (150, 150, 150))
    for b in g['objects']:
        vis = b['z0'] <= LZ <= b['z1']
        col = (200, 120, 60) if vis else (60, 160, 240)
        if vis: cv2.fillPoly(img, [poly(b)], col)
        else: cv2.polylines(img, [poly(b)], True, col, 2)
    for t in tr:   # 변경 표시
        p = to(t['x'], t['y'])
        cv2.circle(img, p, 22, (0, 0, 220), 2)
        cv2.putText(img, {'added': '+', 'removed': '-', 'moved': '>', 'changed': '*'}[t['kind']] + t['obj'], (p[0] + 18, p[1] - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 200), 1)
        if t['kind'] == 'moved': cv2.arrowedLine(img, to(t['from_x'], t['from_y']), p, (0, 0, 220), 1, tipLength=0.08)
    wps = yaml.safe_load(open(f'{D}/{F}/waypoints.yaml', encoding='utf-8'))['waypoints']
    pts = [to(*spec['start'][:2])] + [to(w['x'], w['y']) for w in wps]
    for a, b in zip(pts, pts[1:]): cv2.line(img, a, b, (80, 180, 80), 1)
    for w in wps:
        p = to(w['x'], w['y']); cv2.circle(img, p, 4, (40, 140, 40), -1)
        cv2.putText(img, w['id'][1:], (p[0] + 4, p[1] + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (40, 120, 40), 1)
    cv2.drawMarker(img, to(*spec['start'][:2]), (0, 0, 0), cv2.MARKER_TRIANGLE_UP, 14, 2)
    if spec.get('forklift'):
        (x0, y0), (x1, y1) = spec['forklift']['path']; cv2.line(img, to(x0, y0), to(x1, y1), (0, 140, 255), 3)
    cv2.putText(img, f'{F} M{k}  ({len(tr)} changes)  1 grid = 1 m', (8, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
    for x in range(int(W) + 1): cv2.line(img, (x * PX, 0), (x * PX, int(H * PX)), (235, 235, 235), 1)
    return img

rows = []
for F in sorted(d for d in os.listdir(D) if os.path.isdir(f'{D}/{d}')):
    ims = [draw(F, k) for k in range(4)]
    for k, im in enumerate(ims): cv2.imwrite(f'{D}/{F}/layout_m{k}.png', im)
    h = max(i.shape[0] for i in ims); ims = [cv2.copyMakeBorder(i, 0, h - i.shape[0], 0, 6, cv2.BORDER_CONSTANT, value=(255, 255, 255)) for i in ims]
    rows.append(np.hstack(ims))
w = max(r.shape[1] for r in rows)
cv2.imwrite(f'{D}/layouts_all.png', np.vstack([cv2.copyMakeBorder(r, 0, 6, 0, w - r.shape[1], cv2.BORDER_CONSTANT, value=(255, 255, 255)) for r in rows]))
print('ok')
