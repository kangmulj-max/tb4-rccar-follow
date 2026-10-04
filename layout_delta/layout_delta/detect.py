"""CLI: compare two rounds and write the change report.
  ros2 run layout_delta detect --ref round00/map.yaml --cur round01/map.yaml --out report01 \
      [--roi-waypoints waypoints.yaml --roi-radius 3.0] [--truth ledger.json]
"""
import argparse
import json
import os
import sys

import numpy as np
import yaml

from .diff import DiffParams, diff, evaluate
from .grid import MapGrid
from .report import write


def roi_from_waypoints(grid, wp_file, radius):
    """Cells within `radius` of any patrol waypoint — far cells are seen at long range / grazing angles only."""
    wps = yaml.safe_load(open(wp_file, encoding='utf-8'))['waypoints']
    rows, cols = np.indices(grid.shape)
    x = grid.origin[0] + (cols + 0.5) * grid.resolution
    y = grid.origin[1] + (rows + 0.5) * grid.resolution
    m = np.zeros(grid.shape, bool)
    for w in wps:
        m |= (x - w['x']) ** 2 + (y - w['y']) ** 2 <= radius ** 2
    return m


def main(argv=None):
    ap = argparse.ArgumentParser(description='Layout Delta: round-to-round change report')
    ap.add_argument('--ref', required=True); ap.add_argument('--cur', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--truth', help='ground-truth ledger json: [{kind,x,y}] (evaluation only)')
    ap.add_argument('--roi-waypoints', help='waypoints yaml: compare only cells near the patrol route')
    ap.add_argument('--roi-radius', type=float, default=3.0)
    ap.add_argument('--min-area', type=float, default=DiffParams.min_area_m2)
    ap.add_argument('--wall-margin', type=float, default=DiffParams.wall_margin_m)
    ap.add_argument('--shift-margin', type=float, default=DiffParams.shift_margin_m)
    ap.add_argument('--min-extent', type=float, default=DiffParams.min_extent_m)
    ap.add_argument('--title', default='Layout Delta 변경 리포트')
    a = ap.parse_args(argv)
    ref, cur = MapGrid.load(a.ref), MapGrid.load(a.cur)
    roi = roi_from_waypoints(ref, a.roi_waypoints, a.roi_radius) if a.roi_waypoints else None
    res = diff(ref, cur, DiffParams(min_area_m2=a.min_area, wall_margin_m=a.wall_margin, shift_margin_m=a.shift_margin, min_extent_m=a.min_extent), roi=roi)
    meta = {'기준 회차': a.ref, '비교 회차': a.cur}
    os.makedirs(a.out, exist_ok=True)
    if a.truth:
        ev = evaluate(res.changes, json.load(open(a.truth, encoding='utf-8')))
        meta['정답 대비'] = f"재현율 {ev['recall']:.0%}, 정밀도 {ev['precision']:.0%} (TP {ev['tp']}, FP {ev['fp']}, FN {ev['fn']}, 유형 오류 {ev['type_errors']})"
        json.dump(ev, open(f'{a.out}/eval.json', 'w'), indent=1)
        print(json.dumps(ev, indent=1))
    path = write(ref, cur, res, a.out, a.title, meta)
    print(f'{len(res.changes)} changes → {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
