"""Change report: PNG overlay + Markdown table + JSON (customer-facing artefacts)."""
from __future__ import annotations

import datetime as dt
import os

import numpy as np

from .grid import FREE, OCC, MapGrid

COL = {'added': (0, 170, 0), 'removed': (0, 0, 220), 'moved': (220, 120, 0)}   # BGR
KO = {'added': '추가', 'removed': '제거', 'moved': '이동'}


def render(ref: MapGrid, cur: MapGrid, changes, path_png: str, scale: int = 6) -> None:
    import cv2
    H, W = ref.shape
    img = np.full((H, W, 3), 200, np.uint8)
    img[cur.data == FREE] = (255, 255, 255)
    img[ref.data == OCC] = (90, 90, 90)
    img[cur.data == OCC] = (40, 40, 40)
    img = cv2.resize(np.flipud(img), (W * scale, H * scale), interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        return int((x - ref.origin[0]) / ref.resolution * scale), int((H - (y - ref.origin[1]) / ref.resolution) * scale)
    for c in changes:
        p = px(c.x, c.y); r = max(8, int(np.sqrt(c.area_m2) / ref.resolution * scale * 0.7))
        cv2.circle(img, p, r, COL[c.kind], 2)
        if c.kind == 'moved' and c.from_xy:
            q = px(*c.from_xy); cv2.circle(img, q, r, (150, 150, 255), 1); cv2.arrowedLine(img, q, p, COL['moved'], 2, tipLength=0.15)
        cv2.putText(img, str(c.id), (p[0] + r, p[1] - r), cv2.FONT_HERSHEY_SIMPLEX, 0.6, COL[c.kind], 2)
    cv2.imwrite(path_png, img)


def write(ref, cur, result, out_dir: str, title: str = 'Layout Delta report', meta: dict | None = None) -> str:
    os.makedirs(out_dir, exist_ok=True)
    render(ref, cur, result.changes, os.path.join(out_dir, 'changes.png'))
    with open(os.path.join(out_dir, 'changes.json'), 'w', encoding='utf-8') as f:
        f.write(result.to_json())
    n = {k: sum(c.kind == k for c in result.changes) for k in ('added', 'removed', 'moved')}
    lines = [f'# {title}', '', f'생성: {dt.datetime.now():%Y-%m-%d %H:%M}', '']
    for k, v in (meta or {}).items():
        lines.append(f'- {k}: {v}')
    lines += ['', f'변경 {len(result.changes)} 건 — 추가 {n["added"]}, 제거 {n["removed"]}, 이동 {n["moved"]} '
                  f'(두 회차 모두 관측한 면적 비율 {result.observed_both_ratio:.0%})', '', '![changes](changes.png)', '',
              '| # | 유형 | x [m] | y [m] | 면적 [m²] | 이전 위치 |', '|---|---|---|---|---|---|']
    for c in result.changes:
        lines.append(f'| {c.id} | {KO[c.kind]} | {c.x:.2f} | {c.y:.2f} | {c.area_m2:.2f} | '
                     f'{"" if not c.from_xy else f"({c.from_xy[0]:.2f}, {c.from_xy[1]:.2f})"} |')
    path = os.path.join(out_dir, 'report.md')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    return path
