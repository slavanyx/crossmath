"""Phase 2 blade-level optimization: per-ruling min-max refinement plus a
global orientation-smoothing pass.

The min-max step minimises the worst-case flank deviation of each ruling
independently (Fortran core). The smoothing step then couples neighbours:
it low-pass-filters the cutter-axis field to reduce orientation jerk -- the
dominant non-cutting-time cost on a 5-axis machine -- and re-evaluates the
resulting deviation so the accuracy/smoothness trade-off is explicit.
"""
from __future__ import annotations

import numpy as np

from . import core


def _ruling_dev(a_i, b_i, q0, alpha, R, nv):
    v = np.linspace(0.0, 1.0, nv)
    pts = (1.0 - v)[:, None] * a_i[None, :] + v[:, None] * b_i[None, :]
    return np.max(np.abs(core.deviation(q0, alpha, R, pts)))


def optimize_blade(a, b, ap, bp, R, nv=41, smooth_window=5, tol_mm=None):
    """Return a dict of per-ruling results for two-point, min-max, and
    tolerance-constrained smoothed strategies, with peak deviation arrays.

    The smoothing budget `tol_mm` defaults to the min-max peak deviation, so
    smoothing is guaranteed never to make the worst ruling worse -- it only
    spends accuracy slack to reduce orientation jerk (rotary-axis effort).
    """
    nu = a.shape[0]
    q0_tp = np.empty((nu, 3)); al_tp = np.empty((nu, 3)); e_tp = np.empty(nu)
    q0_mm = np.empty((nu, 3)); al_mm = np.empty((nu, 3)); e_mm = np.empty(nu)

    for i in range(nu):
        q0, al = core.two_point(a[i], ap[i], b[i], bp[i], R)
        q0_tp[i] = q0; al_tp[i] = al
        e_tp[i] = _ruling_dev(a[i], b[i], q0, al, R, nv)

        q0r, alr, em = core.refine_minmax(a[i], ap[i], b[i], bp[i], R, nv)
        q0_mm[i] = q0r; al_mm[i] = alr; e_mm[i] = em

    # --- tolerance-constrained global smoothing of the min-max axis field ---
    tol = float(e_mm.max()) if tol_mm is None else tol_mm
    al_tgt = _smooth(al_mm, smooth_window)
    al_tgt /= np.linalg.norm(al_tgt, axis=1, keepdims=True)
    q0_tgt = _smooth(q0_mm, smooth_window)

    al_sm = al_mm.copy(); q0_sm = q0_mm.copy(); e_sm = e_mm.copy()
    for i in range(nu):
        t = _max_blend(a[i], b[i], R, nv, tol,
                       q0_mm[i], al_mm[i], q0_tgt[i], al_tgt[i])
        al = al_mm[i] + t * (al_tgt[i] - al_mm[i])
        al /= np.linalg.norm(al)
        q0 = q0_mm[i] + t * (q0_tgt[i] - q0_mm[i])
        al_sm[i] = al; q0_sm[i] = q0
        e_sm[i] = _ruling_dev(a[i], b[i], q0, al, R, nv)

    return {
        "two_point": dict(q0=q0_tp, alpha=al_tp, dev=e_tp),
        "minmax":    dict(q0=q0_mm, alpha=al_mm, dev=e_mm),
        "smoothed":  dict(q0=q0_sm, alpha=al_sm, dev=e_sm),
    }


def orientation_jerk(alpha):
    """L2 norm of the second difference of the axis field -- a proxy for the
    rotary-axis effort / non-cutting time."""
    d2 = np.diff(alpha, n=2, axis=0)
    return float(np.sqrt(np.sum(d2 ** 2)))


def _max_blend(a_i, b_i, R, nv, tol, q0a, ala, q0b, alb):
    """Largest blend factor t in [0,1] from (q0a,ala) toward (q0b,alb) that
    keeps peak deviation <= tol. Bisection (deviation is monotone-ish in t)."""
    def dev(t):
        al = ala + t * (alb - ala)
        n = np.linalg.norm(al)
        al = al / n if n > 0 else ala
        q0 = q0a + t * (q0b - q0a)
        return _ruling_dev(a_i, b_i, q0, al, R, nv)

    if dev(1.0) <= tol:
        return 1.0
    lo, hi = 0.0, 1.0
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        if dev(mid) <= tol:
            lo = mid
        else:
            hi = mid
    return lo


def _smooth(arr, w):
    if w < 3:
        return arr.copy()
    k = np.ones(w) / w
    out = np.empty_like(arr)
    for c in range(arr.shape[1]):
        out[:, c] = np.convolve(arr[:, c], k, mode="same")
    # keep endpoints anchored (convolution 'same' biases the ends)
    out[0] = arr[0]; out[-1] = arr[-1]
    return out
