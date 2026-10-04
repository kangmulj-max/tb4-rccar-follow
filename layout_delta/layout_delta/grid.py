"""Occupancy grid utilities (no ROS dependency).

- MapGrid: map_server style grid (pgm + yaml), world <-> cell conversion.
- ScanAccumulator: log-odds grid built from 2D scans with known sensor poses
  (pose from AMCL/localization on a fixed reference map), so every round lives
  in the same coordinate frame as round 0.
"""
from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass

import numpy as np

FREE, UNKNOWN, OCC = 0, 1, 2


@dataclass
class MapGrid:
    """2D grid in the map frame. data: uint8 {FREE, UNKNOWN, OCC}, row 0 = bottom (y = origin_y)."""
    data: np.ndarray
    resolution: float
    origin: tuple

    @property
    def shape(self):
        return self.data.shape

    def world_to_cell(self, x, y):
        c = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        r = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        return r, c

    def cell_to_world(self, r, c):
        return (self.origin[0] + (np.asarray(c) + 0.5) * self.resolution,
                self.origin[1] + (np.asarray(r) + 0.5) * self.resolution)

    @classmethod
    def load(cls, yaml_path: str) -> "MapGrid":
        """Load a map_server map (trinary or scale). Uses occupied/free thresholds from the yaml."""
        import cv2
        meta = {}
        for line in open(yaml_path, encoding='utf-8'):
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
        img = cv2.imread(os.path.join(os.path.dirname(yaml_path), meta['image']), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(meta['image'])
        neg = int(meta.get('negate', 0))
        p = img.astype(float) / 255.0 if neg else (255.0 - img) / 255.0
        data = np.full(img.shape, UNKNOWN, np.uint8)
        data[p > float(meta.get('occupied_thresh', 0.65))] = OCC
        data[p < float(meta.get('free_thresh', 0.196))] = FREE
        ox, oy = [float(t) for t in re.findall(r'-?[\d.]+(?:e-?\d+)?', meta['origin'])[:2]]
        return cls(np.flipud(data).copy(), float(meta['resolution']), (ox, oy))

    def save(self, yaml_path: str) -> None:
        """Save as map_server trinary map (pgm + yaml)."""
        import cv2
        img = np.full(self.data.shape, 205, np.uint8)
        img[self.data == FREE] = 254
        img[self.data == OCC] = 0
        base = os.path.splitext(yaml_path)[0]
        cv2.imwrite(base + '.pgm', np.flipud(img))
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(f'image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {self.resolution}\n'
                    f'origin: [{self.origin[0]}, {self.origin[1]}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


class ScanAccumulator:
    """Log-odds occupancy from 2D scans. Same size/origin as a reference grid."""

    def __init__(self, like: MapGrid, l_occ=0.85, l_free=-0.4, clamp=5.0, max_range=6.0):
        self.res, self.origin = like.resolution, like.origin
        self.L = np.zeros(like.shape, np.float32)
        self.hits = np.zeros(like.shape, np.int32)       # number of scans that observed each cell
        self.l_occ, self.l_free, self.clamp, self.max_range = l_occ, l_free, clamp, max_range
        self.n_scans = 0

    def add_scan(self, sx, sy, syaw, ranges, angle_min, angle_inc, range_min=0.15):
        """Integrate one scan. (sx, sy, syaw) = sensor pose in map frame."""
        r = np.asarray(ranges, dtype=float)
        a = angle_min + angle_inc * np.arange(len(r)) + syaw
        valid = np.isfinite(r) & (r > range_min)
        hit = valid & (r <= self.max_range)
        rr = np.where(valid, np.minimum(r, self.max_range), self.max_range)   # no-return beams clear up to max_range
        H, W = self.L.shape
        free_mask = np.zeros(self.L.shape, bool)
        step = self.res * 0.5
        for ang, dist in zip(a, rr):
            n = max(int(dist / step), 1)
            t = np.arange(n) * step
            c = np.floor((sx + t * math.cos(ang) - self.origin[0]) / self.res).astype(int)
            rw = np.floor((sy + t * math.sin(ang) - self.origin[1]) / self.res).astype(int)
            ok = (c >= 0) & (c < W) & (rw >= 0) & (rw < H)
            free_mask[rw[ok], c[ok]] = True
        hx = sx + r[hit] * np.cos(a[hit]); hy = sy + r[hit] * np.sin(a[hit])
        hc = np.floor((hx - self.origin[0]) / self.res).astype(int)
        hr = np.floor((hy - self.origin[1]) / self.res).astype(int)
        ok = (hc >= 0) & (hc < W) & (hr >= 0) & (hr < H)
        occ_mask = np.zeros(self.L.shape, bool)
        occ_mask[hr[ok], hc[ok]] = True
        free_mask &= ~occ_mask
        self.L[free_mask] += self.l_free
        self.L[occ_mask] += self.l_occ
        np.clip(self.L, -self.clamp, self.clamp, out=self.L)
        self.hits[free_mask | occ_mask] += 1
        self.n_scans += 1

    def grid(self, occ_th=1.0, free_th=-1.0) -> MapGrid:
        d = np.full(self.L.shape, UNKNOWN, np.uint8)
        d[self.L >= occ_th] = OCC
        d[self.L <= free_th] = FREE
        return MapGrid(d, self.res, self.origin)
