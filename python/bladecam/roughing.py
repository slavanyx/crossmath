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


def _interp_cols(s, pts, snew):
    """Arc-length resample an (n,3) polyline at parameters snew."""
    return np.column_stack([np.interp(snew, s, pts[:, c]) for c in range(3)])


def trochoidal_channel(a, b, a2, b2, R, ae_target, feed_mm_min, circ_pts=24):
    """Engagement-controlled trochoidal roughing along the channel centreline.

    The tool follows forward-drifting circular loops; the advance per loop is set
    to the target radial bite `ae_target`, so the radial width of cut -- and thus
    the engagement angle acos(1 - ae/R) -- is bounded (unlike a full-width slot
    pass at ~180 deg). Returns the coil path plus engagement/metrics.
    """
    a = np.asarray(a, float); b = np.asarray(b, float)
    a2 = np.asarray(a2, float); b2 = np.asarray(b2, float)
    midA = 0.5 * (a + b); midB = 0.5 * (a2 + b2)
    centre = 0.5 * (midA + midB)
    cross = midB - midA
    halfw = 0.5 * np.linalg.norm(cross, axis=1)
    height = float(np.mean(np.linalg.norm(b - a, axis=1)))

    seg = np.linalg.norm(np.diff(centre, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    L = float(s[-1])
    # ceil(L/ae) intervals => advance per loop = L/intervals <= ae_target
    n_loops = max(2, int(np.ceil(L / ae_target)) + 1)
    sl = np.linspace(0.0, L, n_loops)

    c_i = _interp_cols(s, centre, sl)
    w_i = _interp_cols(s, cross, sl)
    hw_i = np.interp(sl, s, halfw)
    tang = np.gradient(c_i, axis=0)

    pts = []
    phis = np.linspace(0.0, 2.0 * np.pi, circ_pts, endpoint=False)
    for k in range(n_loops):
        t_hat = tang[k] / (np.linalg.norm(tang[k]) + 1e-12)
        w = w_i[k] - (w_i[k] @ t_hat) * t_hat          # orthogonalise cross dir
        w_hat = w / (np.linalg.norm(w) + 1e-12)
        rho = max(0.0, float(hw_i[k]) - R)             # lateral room to loop
        for ph in phis:
            pts.append(c_i[k] + rho * (np.cos(ph) * w_hat + np.sin(ph) * t_hat))
    coil = np.asarray(pts)

    # radial-immersion engagement: cos(theta) = 1 - ae/R is only defined for a
    # stepover ae <= 2R (a full slot is 180 deg). A larger ae is geometrically
    # impossible (stepover wider than the cutter), so flag it rather than letting
    # the clip silently report a valid-looking 180 deg slot.
    engagement_feasible = bool(ae_target <= 2.0 * R)
    engagement_deg = float(np.degrees(np.arccos(np.clip(1.0 - ae_target / R, -1.0, 1.0))))
    path_len = _poly_len(coil)
    removed_volume = float(np.mean(2.0 * halfw) * height * L)
    cycle_s = (path_len / feed_mm_min) * 60.0 if feed_mm_min > 0 else float("inf")
    return dict(points=coil, engagement_deg=engagement_deg, n_loops=n_loops,
                engagement_feasible=engagement_feasible,
                advance_mm=L / (n_loops - 1), path_len_mm=path_len,
                removed_volume_mm3=removed_volume, cycle_s=cycle_s)


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
