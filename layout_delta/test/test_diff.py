"""Unit tests for layout_delta change detection (no ROS needed): pytest layout_delta/test"""
import math

import numpy as np

from layout_delta.diff import DiffParams, diff, evaluate
from layout_delta.grid import FREE, OCC, UNKNOWN, MapGrid, ScanAccumulator


def room(h=80, w=100, res=0.05):
    d = np.full((h, w), FREE, np.uint8)
    d[0, :] = d[-1, :] = OCC; d[:, 0] = d[:, -1] = OCC
    return MapGrid(d, res, (0.0, 0.0))


def put(g, x, y, s=0.4):
    r0, c0 = g.world_to_cell(x - s / 2, y - s / 2); r1, c1 = g.world_to_cell(x + s / 2, y + s / 2)
    g.data[r0:r1, c0:c1] = OCC


def test_no_change_no_report():
    a, b = room(), room()
    assert diff(a, b).changes == []


def test_added_removed_moved():
    ref, cur = room(), room()
    put(ref, 1.0, 1.0); put(ref, 3.0, 2.5)            # box1 moves, box2 removed
    put(cur, 2.0, 1.0); put(cur, 4.0, 3.0, 0.5)       # box1 new pos, box3 added (far from box2? 1.1 m → paired)
    res = diff(ref, cur, DiffParams(move_radius_m=1.2))
    kinds = sorted(c.kind for c in res.changes)
    assert kinds.count('moved') >= 1 and len(res.changes) <= 3


def test_unknown_is_not_change():
    ref, cur = room(), room()
    put(ref, 2.0, 2.0)
    cur.data[20:60, 20:60] = UNKNOWN                  # area not observed this round
    assert diff(ref, cur).changes == []


def test_wall_jitter_ignored():
    ref, cur = room(), room()
    cur.data[1, :] = OCC                               # wall seen one cell thicker
    assert diff(ref, cur).changes == []


def test_evaluate_counts():
    ref, cur = room(), room()
    put(cur, 2.0, 2.0)
    res = diff(ref, cur)
    ev = evaluate(res.changes, [dict(kind='added', x=2.0, y=2.0), dict(kind='added', x=4.0, y=3.0)])
    assert ev['tp'] == 1 and ev['fn'] == 1 and ev['fp'] == 0


def test_scan_accumulator_sees_box():
    ref = room()
    acc = ScanAccumulator(ref, max_range=6.0)
    n = 360; ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
    # sensor at (1.0, 2.0); a wall at x = 3.0 (only beams pointing +x within ±30° hit it)
    rng = np.where(np.abs(ang) < math.radians(30), 2.0 / np.cos(ang), np.inf)
    for _ in range(5):
        acc.add_scan(1.0, 2.0, 0.0, rng, -math.pi, 2 * math.pi / n)
    g = acc.grid()
    r, c = g.world_to_cell(3.02, 2.0)
    assert g.data[r, c] == OCC
    r, c = g.world_to_cell(2.0, 2.0)
    assert g.data[r, c] == FREE


def test_shifted_wall_ignored():
    ref, cur = room(), room()
    ref.data[40, 20:80] = OCC                          # interior wall
    cur.data[42, 20:80] = OCC                          # same wall, 2 cells (0.10 m) off after alignment error
    assert diff(ref, cur).changes == []


def test_single_face_line_kept_by_extent():
    ref, cur = room(), room()
    cur.data[40, 30:39] = OCC                          # one box face seen as a 0.45 m line (9 cells, 0.0225 m2)
    assert len(diff(ref, cur).changes) == 1
    assert diff(ref, cur, DiffParams(min_extent_m=0.0)).changes == []


def test_roi_excludes_far_cells():
    ref, cur = room(), room()
    put(cur, 4.0, 3.0)
    roi = np.zeros(ref.shape, bool); roi[:, :40] = True   # x < 2 m only
    assert diff(ref, cur, roi=roi).changes == []


def test_moved_detection_counts_for_removed_truth():
    from layout_delta.diff import Change
    det = [Change('moved', 3.0, 3.0, 0.2, (1.0, 1.0))]
    ev = evaluate(det, [{'kind': 'removed', 'x': 1.0, 'y': 1.0}])
    assert ev['tp'] == 1 and ev['type_errors'] == 1
