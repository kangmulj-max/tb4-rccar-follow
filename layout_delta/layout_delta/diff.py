"""Round-to-round change detection on occupancy grids in the same frame.

A change is reported only where BOTH rounds actually observed the cell
(occupied in one, free in the other). Unknown cells never produce changes,
so partial coverage does not create false 'removed' objects.

Output: list of Change(kind in {added, removed, moved}, centroid, size, ...).
'moved' = a removed blob and an added blob of similar area within move_radius.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field

import numpy as np

from .grid import FREE, OCC, MapGrid


@dataclass
class Change:
    kind: str                    # added | removed | moved
    x: float                     # map frame [m] (moved: new position)
    y: float
    area_m2: float
    from_xy: tuple | None = None  # moved: old position
    cells: int = 0
    score: float = 1.0
    id: int = 0


@dataclass
class DiffParams:
    min_area_m2: float = 0.04         # keep a blob if its area reaches this ...
    min_extent_m: float = 0.45        # ... or if its longest side reaches this (a 2D lidar sees one box face as a thin line); 0 = off
    wall_margin_m: float = 0.15       # ignore changes within this distance of persisting structure (alignment jitter)
    move_radius_m: float = 2.5        # pair removed+added within this distance as 'moved'
    shift_margin_m: float = 0.15      # added: ignore if ref is occupied within this distance (and vice versa) — map misalignment
    area_ratio: float = 2.5           # ...and with area ratio below this
    dilate_cells: int = 1             # merge nearby cells into one object


@dataclass
class DiffResult:
    changes: list = field(default_factory=list)
    observed_both_ratio: float = 0.0
    params: dict = field(default_factory=dict)

    def to_json(self) -> str:
        return json.dumps({'changes': [asdict(c) for c in self.changes],
                           'observed_both_ratio': self.observed_both_ratio, 'params': self.params},
                          ensure_ascii=False, indent=1)


def _blobs(mask, res, origin, dilate):
    import cv2
    m = mask.astype(np.uint8)
    if dilate:
        m = cv2.dilate(m, np.ones((2 * dilate + 1, 2 * dilate + 1), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, connectivity=8)
    out = []
    for i in range(1, n):
        cells = int((mask & (lab == i)).sum())
        if cells == 0:
            continue
        rr, cc = np.nonzero(mask & (lab == i))
        out.append(dict(x=origin[0] + (cc.mean() + 0.5) * res, y=origin[1] + (rr.mean() + 0.5) * res,
                        cells=cells, area=cells * res * res,
                        extent=(max(rr.max() - rr.min(), cc.max() - cc.min()) + 1) * res))
    return out


def diff(ref: MapGrid, cur: MapGrid, p: DiffParams | None = None, roi=None) -> DiffResult:
    """roi: optional bool mask (same shape) — only cells inside are compared (e.g. near the patrol route)."""
    import cv2
    p = p or DiffParams()
    if ref.shape != cur.shape or abs(ref.resolution - cur.resolution) > 1e-9:
        raise ValueError('grids must share size and resolution (same reference frame)')
    added = (cur.data == OCC) & (ref.data == FREE)
    removed = (ref.data == OCC) & (cur.data == FREE)
    # walls / static structure of the reference: occupied in ref AND occupied in cur → never a change;
    # cells right next to persisting structure are alignment jitter, not objects
    persist = ((ref.data == OCC) & (cur.data == OCC)).astype(np.uint8)
    k = max(1, int(round(p.wall_margin_m / ref.resolution)))
    near = cv2.dilate(persist, np.ones((2 * k + 1, 2 * k + 1), np.uint8)).astype(bool)
    added &= ~near
    removed &= ~near
    # structure that merely shifted by a cell or two between rounds: an added cell with ref structure right next to
    # it (or a removed cell with cur structure next to it) is misalignment, not an object
    s = max(1, int(round(p.shift_margin_m / ref.resolution)))
    ker = np.ones((2 * s + 1, 2 * s + 1), np.uint8)
    added &= ~cv2.dilate((ref.data == OCC).astype(np.uint8), ker).astype(bool)
    removed &= ~cv2.dilate((cur.data == OCC).astype(np.uint8), ker).astype(bool)
    if roi is not None:
        added &= roi
        removed &= roi
    keep = lambda b: b['area'] >= p.min_area_m2 or (p.min_extent_m > 0 and b['extent'] >= p.min_extent_m)
    a_bl = [b for b in _blobs(added, ref.resolution, ref.origin, p.dilate_cells) if keep(b)]
    r_bl = [b for b in _blobs(removed, ref.resolution, ref.origin, p.dilate_cells) if keep(b)]
    changes, used_a = [], set()
    for rb in sorted(r_bl, key=lambda b: -b['area']):
        best, bd = None, None
        for j, ab in enumerate(a_bl):
            if j in used_a:
                continue
            ratio = max(ab['area'], rb['area']) / max(min(ab['area'], rb['area']), 1e-9)
            d = math.hypot(ab['x'] - rb['x'], ab['y'] - rb['y'])
            if d <= p.move_radius_m and ratio <= p.area_ratio and (bd is None or d < bd):
                best, bd = j, d
        if best is not None:
            used_a.add(best); ab = a_bl[best]
            changes.append(Change('moved', ab['x'], ab['y'], ab['area'], (round(rb['x'], 3), round(rb['y'], 3)), ab['cells']))
        else:
            changes.append(Change('removed', rb['x'], rb['y'], rb['area'], None, rb['cells']))
    for j, ab in enumerate(a_bl):
        if j not in used_a:
            changes.append(Change('added', ab['x'], ab['y'], ab['area'], None, ab['cells']))
    for i, c in enumerate(changes, 1):
        c.id = i; c.x, c.y, c.area_m2 = round(c.x, 3), round(c.y, 3), round(c.area_m2, 3)
    both = ((ref.data != 1) & (cur.data != 1)).sum() / max(1, (ref.data != 1).sum())
    return DiffResult(changes, round(float(both), 3), asdict(p))


def evaluate(changes, truth, match_m=0.5):
    """Precision/recall against a ground-truth ledger.
    truth: list of dict(kind, x, y[, from_x, from_y]). A 'moved' truth may be matched by a 'moved' detection
    (new position within match_m) or by an added+removed pair (counted as a partial: type error)."""
    det_used, tp, typ_err = set(), 0, 0
    for t in truth:
        hit = None
        for i, c in enumerate(changes):
            if i in det_used:
                continue
            pts = [(c.x, c.y)]
            if t['kind'] == 'removed' and c.from_xy:   # a 'moved' detection also asserts removal at its origin
                pts.append(tuple(c.from_xy))
            if any(math.hypot(px - t['x'], py - t['y']) <= match_m for px, py in pts):
                hit = i
                break
        if hit is not None:
            det_used.add(hit); tp += 1
            if changes[hit].kind != t['kind']:
                typ_err += 1
    fp = len(changes) - len(det_used)
    prec = tp / max(1, tp + fp)
    rec = tp / max(1, len(truth))
    return dict(tp=tp, fp=fp, fn=len(truth) - tp, type_errors=typ_err, precision=round(prec, 3), recall=round(rec, 3))
