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


STRATEGIES = ("two_point", "minmax", "smoothed", "global")


def _two_point_field(a, b, ap, bp, R, nv):
    nu = a.shape[0]
    q0 = np.empty((nu, 3)); al = np.empty((nu, 3)); e = np.empty(nu)
    for i in range(nu):
        q0[i], al[i] = core.two_point(a[i], ap[i], b[i], bp[i], R)
        e[i] = _ruling_dev(a[i], b[i], q0[i], al[i], R, nv)
    return dict(q0=q0, alpha=al, dev=e)


def _minmax_field(a, b, ap, bp, R, nv):
    nu = a.shape[0]
    q0 = np.empty((nu, 3)); al = np.empty((nu, 3)); e = np.empty(nu)
    for i in range(nu):
        q0[i], al[i], e[i] = core.refine_minmax(a[i], ap[i], b[i], bp[i], R, nv)
    return dict(q0=q0, alpha=al, dev=e)


def _smoothed_field(a, b, R, nv, mm, smooth_window, tol_mm):
    nu = a.shape[0]
    q0_mm, al_mm = mm["q0"], mm["alpha"]
    tol = float(mm["dev"].max()) if tol_mm is None else tol_mm
    al_tgt = _smooth(al_mm, smooth_window)
    al_tgt /= np.linalg.norm(al_tgt, axis=1, keepdims=True)
    q0_tgt = _smooth(q0_mm, smooth_window)
    q0 = q0_mm.copy(); al = al_mm.copy(); e = mm["dev"].copy()
    for i in range(nu):
        t = _max_blend(a[i], b[i], R, nv, tol,
                       q0_mm[i], al_mm[i], q0_tgt[i], al_tgt[i])
        ai = al_mm[i] + t * (al_tgt[i] - al_mm[i]); ai /= np.linalg.norm(ai)
        q0[i] = q0_mm[i] + t * (q0_tgt[i] - q0_mm[i]); al[i] = ai
        e[i] = _ruling_dev(a[i], b[i], q0[i], ai, R, nv)
    return dict(q0=q0, alpha=al, dev=e)


def optimize_blade(a, b, ap, bp, R, nv=41, smooth_window=5, tol_mm=None,
                   mu=1.0, gamma=0.0, nsweeps=3, strategy="global",
                   swept_w=0.0, swept_window=8, barrel_R=0.0, barrel_pos=0.0):
    """Return {strategy: dict(q0, alpha, dev)} for ONE strategy (default
    'global') or, with strategy='all', every strategy.

    Only the work needed for the requested strategy runs -- 'smoothed' first
    computes the min-max field it refines; the others are independent.
    """
    want = STRATEGIES if strategy == "all" else (strategy,)
    out = {}
    if "two_point" in want:
        out["two_point"] = _two_point_field(a, b, ap, bp, R, nv)
    if "minmax" in want or "smoothed" in want:
        mm = _minmax_field(a, b, ap, bp, R, nv)
        if "minmax" in want:
            out["minmax"] = mm
        if "smoothed" in want:
            out["smoothed"] = _smoothed_field(a, b, R, nv, mm, smooth_window, tol_mm)
    if "global" in want:
        q0, al, e = core.optimize_global(a, b, ap, bp, R, nv=nv,
                                         mu=mu, gamma=gamma, nsweeps=nsweeps,
                                         swept_w=swept_w, window=swept_window,
                                         Rb=barrel_R, lamc=barrel_pos)
        out["global"] = dict(q0=q0, alpha=al, dev=e)
    return out


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
