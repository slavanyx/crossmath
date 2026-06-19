"""Dexel-based material-removal verification.

Independent of the analytic models in pipeline.py: it actually subtracts the
swept tool volume from a stock field of rays (core.dexel_carve) and measures
what was removed. Used to (a) verify removed volume / MRR against the analytic
estimate, and (b) measure the real machined-surface error and between-pass
scallops that a per-design-point envelope projection can miss.
"""
from __future__ import annotations

import numpy as np

from . import core


def removed_volume(q0, alpha, R, Lflute, lo, hi, n=80):
    """Volume (mm^3) removed by the swept capped-cylinder tool inside the box
    [lo, hi] (each length-3), via a Z-dexel field of n x n rays (Cavalieri:
    integral of removed length over the x-y grid). Tilted/swept poses are handled
    exactly -- each ray is intersected with every pose and the union is removed."""
    lo = np.asarray(lo, float); hi = np.asarray(hi, float)
    xs = np.linspace(lo[0], hi[0], n)
    ys = np.linspace(lo[1], hi[1], n)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    orig = np.column_stack([gx.ravel(), gy.ravel(),
                            np.full(gx.size, lo[2])])
    dirv = np.tile([0.0, 0.0, 1.0], (orig.shape[0], 1))
    seg0 = np.full(orig.shape[0], hi[2] - lo[2])
    Lf = np.broadcast_to(np.asarray(Lflute, float), (q0.shape[0],)).copy()
    removed, _ = core.dexel_carve(q0, alpha, R, Lf, orig, dirv, seg0)
    cell = (xs[1] - xs[0]) * (ys[1] - ys[0])
    return float(removed.sum() * cell)
