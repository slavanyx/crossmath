"""Persistent interval-dexel stock model for rest-machining.

A `Stock` is a field of rays, each carrying the disjoint intervals of solid
material remaining along it. Operations carve the stock IN PLACE, so the model
is carried across the process -- roughing lowers the field, and finishing then
sees only the REST material roughing left (not the raw block).

Unlike a single-direction height field, the interval dexel removes material from
EITHER end or the middle of a ray, so a top-down rougher and a tilted flank
finisher are both represented correctly (a vertical height map undercounts the
tilted tool). The per-ray removed intervals come from the validated Fortran
ray/capped-cylinder primitive (core.dexel_removed_intervals); this layer keeps
the persistent solid set, the volume bookkeeping, and the stock factories.
"""
from __future__ import annotations

import numpy as np

from . import core


def _subtract(solid, rem):
    """solid \\ rem for two ascending disjoint interval lists of (lo,hi)."""
    out = []
    for lo, hi in solid:
        cur = lo
        for rlo, rhi in rem:
            if rhi <= cur or rlo >= hi:
                continue                      # removed interval clears this segment
            if rlo > cur:                     # solid gap before the removed chunk
                out.append((cur, rlo))        # rlo < hi here (guarded above)
            cur = max(cur, rhi)
            if cur >= hi:
                break
        if cur < hi:
            out.append((cur, hi))
    return out


class Stock:
    """Interval-dexel stock: ray r (origin orig[r], unit dir[r]) holds the solid
    intervals self.solid[r] (ascending, disjoint) along t. cell[r] is the
    cross-sectional area the ray represents, so volume = sum(length*cell)."""

    def __init__(self, orig, dir, height, cell, maxseg: int = 32):
        self.orig = np.ascontiguousarray(orig, float)
        d = np.ascontiguousarray(dir, float)
        self.dir = np.ascontiguousarray(d / np.linalg.norm(d, axis=1, keepdims=True))
        nray = self.orig.shape[0]
        self.cell = (np.full(nray, float(cell)) if np.isscalar(cell)
                     else np.ascontiguousarray(cell, float))
        self.solid = [[(0.0, float(h))] for h in np.asarray(height, float)]
        self.maxseg = maxseg
        self._v0 = self.volume()

    def carve(self, q0, alpha, R, Lf) -> float:
        """Carve the swept capped-cylinder tool (poses q0,alpha (nu,3); flute
        length Lf scalar or (nu,)) out of the stock. Returns the volume removed
        by THIS operation (>= 0)."""
        rlo, rhi, rn = core.dexel_removed_intervals(
            q0, alpha, float(R), Lf, self.orig, self.dir, maxseg=self.maxseg)
        before = self.volume()
        for r in range(self.orig.shape[0]):
            k = int(rn[r])
            if k == 0:
                continue
            rem = [(rlo[r, j], rhi[r, j]) for j in range(k)]
            self.solid[r] = _subtract(self.solid[r], rem)
        return before - self.volume()

    def volume(self) -> float:
        tot = 0.0
        for r, segs in enumerate(self.solid):
            length = 0.0
            for lo, hi in segs:
                length += hi - lo
            tot += length * self.cell[r]
        return float(tot)

    def removed_total(self) -> float:
        return float(self._v0 - self.volume())

    def rest_per_ray(self) -> np.ndarray:
        """Remaining solid length per ray (the rest-material field)."""
        return np.array([sum(hi - lo for lo, hi in s) for s in self.solid], float)


def block(lo, hi, nx: int = 40, ny: int = 40) -> Stock:
    """A rectangular block stock [lo,hi] as a vertical (+Z) interval dexel."""
    lo = np.asarray(lo, float); hi = np.asarray(hi, float)
    xs = np.linspace(lo[0], hi[0], nx); ys = np.linspace(lo[1], hi[1], ny)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    orig = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, lo[2])])
    dirv = np.tile(np.array([0.0, 0.0, 1.0]), (X.size, 1))
    height = np.full(X.size, hi[2] - lo[2])
    dx = (hi[0] - lo[0]) / max(1, nx - 1)
    dy = (hi[1] - lo[1]) / max(1, ny - 1)
    return Stock(orig, dirv, height, dx * dy)


def channel_stock(a, b, a2, b2, nx: int = 40, ny: int = 40) -> Stock:
    """A vertical interval-dexel stock spanning the flow channel between this
    blade's wall (a,b) and the adjacent blade's facing wall (a2,b2). The
    footprint is the XY bounding box of the two walls; material fills it from the
    lowest to the highest Z of the blades (the raw channel slab)."""
    pts = np.vstack([np.asarray(a, float), np.asarray(b, float),
                     np.asarray(a2, float), np.asarray(b2, float)])
    lo = pts.min(axis=0); hi = pts.max(axis=0)
    return block(lo, hi, nx, ny)
