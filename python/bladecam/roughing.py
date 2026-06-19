"""Layered channel roughing toolpath.

Clears the flow channel between a blade's wall and the adjacent blade's wall by
sweeping morph passes across the channel at each axial (height) level. Passes
are convex blends of corresponding wall points, spaced by a radial stepover
(constant engagement by construction). This is a real set of passes (polylines)
with a length-derived cycle time -- a simplified adaptive clearing, not full
trochoidal toolpath generation.
"""
from __future__ import annotations

import numpy as np


def _poly_len(poly: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(poly, axis=0), axis=1)))


def adaptive_rough(a, b, a2, b2, ap: float, stepover: float,
                   feed_mm_min: float):
    """Layered roughing between walls (a,b) and (a2,b2), each (nu,3).

    ap       axial depth of cut (height step), mm
    stepover radial step across the channel, mm
    Returns dict: passes (list of (nu,3) polylines), n_axial, n_radial,
    total_len_mm, removed_volume_mm3, cycle_s, channel_gap_mm.
    """
    a = np.asarray(a, float); b = np.asarray(b, float)
    a2 = np.asarray(a2, float); b2 = np.asarray(b2, float)
    height = float(np.mean(np.linalg.norm(b - a, axis=1)))
    midA = 0.5 * (a + b); midB = 0.5 * (a2 + b2)
    wall_dist = np.linalg.norm(midB - midA, axis=1)
    gap = float(np.mean(wall_dist))
    gap_max = float(np.max(wall_dist))          # widest section
    length = _poly_len(midA)

    n_axial = max(1, int(np.ceil(height / ap)))
    # size radial passes from the WIDEST section so the stepover (engagement)
    # bound holds everywhere, not just on average.
    n_radial = max(1, int(np.ceil(gap_max / stepover)))

    passes = []
    total = 0.0
    vlev = (np.arange(n_axial) + 0.5) / n_axial      # mid-height of each layer
    for v in vlev:
        wA = (1 - v) * a + v * b
        wB = (1 - v) * a2 + v * b2
        for j in range(n_radial + 1):
            s = j / n_radial
            poly = (1 - s) * wA + s * wB
            passes.append(poly)
            total += _poly_len(poly)

    removed_volume = gap * height * length            # channel slab volume
    cycle_s = (total / feed_mm_min) * 60.0 if feed_mm_min > 0 else float("inf")
    return dict(passes=passes, n_axial=n_axial, n_radial=n_radial,
                total_len_mm=total, removed_volume_mm3=removed_volume,
                cycle_s=cycle_s, channel_gap_mm=gap, channel_gap_max_mm=gap_max)
