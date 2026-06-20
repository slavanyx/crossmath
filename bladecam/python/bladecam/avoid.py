"""Collision-AWARE tool-axis positioning (advanced avoidance).

We generate the toolpath ourselves, so collision avoidance belongs INSIDE the
positioning, not as a reactive post-process: the global optimiser fixes the
swept-optimal axes, then this layer tilts the axis at any ruling that collides to
restore clearance -- but ONLY when the tilt pays for itself. The cost it watches
is the SWEPT ENVELOPE error (the real machined deviation), not the per-ruling
contact residual: a small per-ruling tilt can blow the cross-ruling envelope up
by 10x, so every accepted adjustment is gated on the global swept error staying
within a budget. Tilts that cannot clear the obstacle within that budget are
reverted and the ruling is reported as a residual collision -- avoidance cannot
manufacture clearance that does not exist (a too-tight channel or an inflected
flank needs a smaller/barrel tool), and we say so honestly rather than gouging.
"""
from __future__ import annotations

import numpy as np

from . import core
from .machine import tool_branch_capsules


def _rodrigues(v, axis, ang):
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return v
    k = axis / n
    c, s = np.cos(ang), np.sin(ang)
    return v * c + np.cross(k, v) * s + k * (k @ v) * (1.0 - c)


def _swept_overcut(q0, alpha, Lflute, R, surf_flat, gamma):
    sw = core.swept_deviation(q0, alpha, Lflute, R, surf_flat, gamma=gamma,
                              Rb=0.0, lamc=0.0)
    return float(max(0.0, -sw.min()))


def _bulk_clearance(q0, alpha, seg_R, seg_lo, seg_hi, obstacle_tris, blade_pts,
                    hbase, hlen, endwall, nscan):
    """Per-ruling static clearance for the WHOLE path in a few vectorised calls,
    matching exactly the per-ruling checks the pipeline reports: full assembly vs
    neighbour+table mesh; the HOLDER vs the blade being cut (point cloud, as the
    pipeline's holder_clearance); and holder+ vs the hub/shroud endwalls."""
    caps = tool_branch_capsules(q0, alpha, seg_R, seg_lo, seg_hi)
    c = core.mesh_clearance(caps, obstacle_tris, nscan=nscan, signed=False)
    c = np.minimum(c, core.holder_clearance_swept(
        q0, alpha, blade_pts, seg_R[1], hbase, hlen, nscan=nscan))
    if endwall.shape[0]:
        c = np.minimum(c, core.assembly_clearance(
            q0, alpha, seg_R[1:], seg_lo[1:], seg_hi[1:], endwall, nscan=nscan))
    return c


def _pose_clearance(q0i, ai, seg_R, seg_lo, seg_hi, obstacle_tris, blade_pts,
                    hbase, hlen, endwall):
    """Min static clearance of ONE tool pose (a candidate tilt). Obstacle sets as
    in _bulk_clearance; the pose is duplicated so the swept routines accept it."""
    q2 = np.array([q0i, q0i]); a2 = np.array([ai, ai])
    caps = tool_branch_capsules(q2, a2, seg_R, seg_lo, seg_hi)
    c = float(core.mesh_clearance(caps, obstacle_tris, nscan=1, signed=False).min())
    c = min(c, float(core.holder_clearance_swept(
        q2, a2, blade_pts, seg_R[1], hbase, hlen, nscan=1).min()))
    if endwall.shape[0]:
        c = min(c, float(core.assembly_clearance(
            q2, a2, seg_R[1:], seg_lo[1:], seg_hi[1:], endwall, nscan=1).min()))
    return c


def collision_aware_axes(q0, alpha, surf, R, gamma, seg_R, seg_lo, seg_hi,
                         obstacle_tris, blade_pts, hbase, hlen, endwall,
                         tangent, Lflute, *,
                         margin=0.6, trigger=0.25, swept_budget_mm=0.10,
                         max_tilt_deg=20.0, n_grid=5, n_sweeps=2, w_smooth=1.5,
                         nscan=4):
    """Tilt colliding rulings to restore clearance, gated on the swept envelope.

    Returns (alpha_adjusted, report). For each ruling whose pose clearance <
    `margin`, search lead/lean tilts (up to `max_tilt_deg`) and keep the one that
    most increases clearance (with a smoothness bias). After each sweep, if the
    GLOBAL swept-overcut has risen more than `swept_budget_mm` above its
    swept-optimal value, greedily revert the most-tilted rulings until it is back
    within budget -- so avoidance never trades the machined surface for clearance.
    """
    surf_flat = surf.reshape(-1, 3)
    nu = q0.shape[0]
    alpha0 = alpha / np.linalg.norm(alpha, axis=1, keepdims=True)
    alpha = alpha0.copy()
    adjusted = np.zeros(nu, bool)
    tilts = np.linspace(-np.radians(max_tilt_deg), np.radians(max_tilt_deg), n_grid)

    # FAST PRE-CHECK: per-ruling clearance for the whole path in a few bulk calls.
    # If nothing is within `margin`, there is nothing to avoid -- return at once
    # (the collision-free common case pays only the bulk check, not the search).
    clr = _bulk_clearance(q0, alpha, seg_R, seg_lo, seg_hi, obstacle_tris,
                          blade_pts, hbase, hlen, endwall, nscan)
    seg_bad = clr < trigger               # fire only on a real collision (clr<0)
    ruling_bad = np.zeros(nu, bool)
    ruling_bad[:-1] |= seg_bad[:-1]       # ruling i owns segment i
    ruling_bad[1:] |= seg_bad[:-1]        # ...and segment i-1
    ruling_bad[-1] |= seg_bad[-1]         # final static station
    todo = np.where(ruling_bad)[0]
    if todo.size == 0:
        return alpha, dict(adjusted=adjusted, n_adjusted=0,
                           residual_min_clearance=float(clr.min()),
                           swept_after_mm=0.0, swept_before_mm=0.0,
                           infeasible_rulings=[])
    swept0 = _swept_overcut(q0, alpha0, Lflute, R, surf_flat, gamma)

    for _ in range(n_sweeps):
        for i in todo:
            cl = _pose_clearance(q0[i], alpha[i], seg_R, seg_lo, seg_hi,
                                 obstacle_tris, blade_pts, hbase, hlen, endwall)
            if cl >= margin:
                continue
            u = alpha[i]
            t = tangent[i] - (tangent[i] @ u) * u
            if np.linalg.norm(t) < 1e-9:
                t = np.array([1.0, 0.0, 0.0]) - u[0] * u
            t /= np.linalg.norm(t) + 1e-12
            e_lean, e_lead = np.cross(u, t), t
            nbr = alpha[max(0, i - 1)] + alpha[min(nu - 1, i + 1)]
            nbr /= np.linalg.norm(nbr) + 1e-12
            best = (cl - w_smooth * (1.0 - float(u @ nbr)), u)
            for th1 in tilts:
                for th2 in tilts:
                    cand = _rodrigues(_rodrigues(u, e_lean, th1), e_lead, th2)
                    cand /= np.linalg.norm(cand)
                    cc = _pose_clearance(q0[i], cand, seg_R, seg_lo, seg_hi,
                                         obstacle_tris, blade_pts, hbase, hlen, endwall)
                    score = cc - w_smooth * (1.0 - float(cand @ nbr))
                    if score > best[0]:
                        best = (score, cand)
            if not np.allclose(best[1], u):
                alpha[i] = best[1]; adjusted[i] = True
        alpha /= np.linalg.norm(alpha, axis=1, keepdims=True)

        # SWEPT-BUDGET gate: revert the most-tilted rulings until the global
        # machined error is back within budget of the swept-optimal.
        while _swept_overcut(q0, alpha, Lflute, R, surf_flat, gamma) > \
                swept0 + swept_budget_mm and adjusted.any():
            tilt_mag = np.where(adjusted,
                                1.0 - np.einsum("ij,ij->i", alpha, alpha0), -1.0)
            j = int(np.argmax(tilt_mag))
            alpha[j] = alpha0[j]; adjusted[j] = False

    resid = _bulk_clearance(q0, alpha, seg_R, seg_lo, seg_hi, obstacle_tris,
                            blade_pts, hbase, hlen, endwall, nscan)
    swept_final = _swept_overcut(q0, alpha, Lflute, R, surf_flat, gamma)
    infeasible = np.where(resid < 0.0)[0]
    report = dict(adjusted=adjusted, n_adjusted=int(adjusted.sum()),
                  residual_min_clearance=float(resid.min()),
                  swept_after_mm=swept_final, swept_before_mm=swept0,
                  infeasible_rulings=infeasible.tolist())
    return alpha, report
