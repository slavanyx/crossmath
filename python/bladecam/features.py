"""Native-CAD feature recognition for impeller / blisk geometry.

Pure-geometry layer (NumPy only, no CAD kernel) operating on extracted rails
and sampled surface grids, so it is fully testable without OpenCASCADE:

  - classify_blades : split a blisk's blades into MAIN vs SPLITTER by streamwise
                      length (splitters are the shorter, downstream blades).
  - min_curvature_radius / is_fillet_surface : recognise root-fillet / blend
                      faces (small radius of curvature) so they are not mistaken
                      for flank faces.
  - trim_root_fillet : clip the hub rail up the ruling by the fillet tangent
                      offset, so a flank-milling pass starts ABOVE the root
                      fillet instead of gouging it (the fillet is left for a
                      dedicated operation).

The thin OpenCASCADE glue in cadio.py feeds real B-rep faces into these.
"""
from __future__ import annotations

import math

import numpy as np


# --- splitter recognition ----------------------------------------------------
def blade_length(a: np.ndarray, b: np.ndarray) -> float:
    """Streamwise length of a blade = arc length of its mid-rail (LE->TE)."""
    m = 0.5 * (np.asarray(a, float) + np.asarray(b, float))
    return float(np.sum(np.linalg.norm(np.diff(m, axis=0), axis=1)))


def classify_blades(rails, split_gap_frac: float = 0.2):
    """Label each blade 'main' or 'splitter' from a list of (a, b) rail pairs.

    Splitter blades are shorter (they start downstream), so the blade-length
    distribution is bimodal. We sort the lengths and split at the LARGEST gap,
    but only if that gap is significant (>= split_gap_frac of the longest blade);
    otherwise every blade is a 'main'. Returns a list of labels aligned with
    `rails`."""
    L = np.array([blade_length(a, b) for (a, b) in rails], float)
    n = len(L)
    labels = ["main"] * n
    if n < 2:
        return labels
    order = np.argsort(L)
    Ls = L[order]
    gaps = np.diff(Ls)
    gi = int(np.argmax(gaps))
    if gaps[gi] >= split_gap_frac * Ls[-1] and gaps[gi] > 0.0:
        thresh = 0.5 * (Ls[gi] + Ls[gi + 1])
        for k in range(n):
            if L[k] < thresh:
                labels[k] = "splitter"
    return labels


# --- fillet / blend-face recognition -----------------------------------------
def _menger_kappa(p0, p1, p2):
    """Curvature (1/circumradius) of the triangle p0,p1,p2; 0 if collinear."""
    a = np.linalg.norm(p1 - p0)
    b = np.linalg.norm(p2 - p1)
    c = np.linalg.norm(p2 - p0)
    area = 0.5 * np.linalg.norm(np.cross(p1 - p0, p2 - p0))
    denom = a * b * c
    if denom < 1e-15 or area < 1e-15:
        return 0.0
    return 4.0 * area / denom            # = 1 / circumradius


def characteristic_curvature(surf: np.ndarray, pct: float = 90.0) -> float:
    """A robust characteristic curvature (1/mm) of a (nu,nv,3) surface grid:
    the `pct`-th percentile of the discrete curvature of every interior triple
    along BOTH grid directions. Straight rulings contribute ~0, so a ruled flank
    is dominated by its gentle cross-curvature, while a tight blend spikes."""
    surf = np.asarray(surf, float)
    nu, nv, _ = surf.shape
    ks = []
    for i in range(nu):                  # curves along v
        for j in range(1, nv - 1):
            ks.append(_menger_kappa(surf[i, j - 1], surf[i, j], surf[i, j + 1]))
    for j in range(nv):                  # curves along u
        for i in range(1, nu - 1):
            ks.append(_menger_kappa(surf[i - 1, j], surf[i, j], surf[i + 1, j]))
    if not ks:
        return 0.0
    return float(np.percentile(ks, pct))


def min_curvature_radius(surf: np.ndarray, pct: float = 90.0) -> float:
    """Characteristic minimum radius of curvature (mm) of a surface grid; large
    for a gentle flank, small (≈ fillet radius) for a blend. inf if effectively
    flat/straight."""
    k = characteristic_curvature(surf, pct)
    return float("inf") if k <= 1e-12 else 1.0 / k


def is_fillet_surface(surf: np.ndarray, max_radius: float, pct: float = 90.0) -> bool:
    """True if a surface grid is a fillet/blend: its characteristic radius of
    curvature is at or below `max_radius` (a tight blend), so it must NOT be
    treated as a flank face."""
    return min_curvature_radius(surf, pct) <= max_radius


# --- fillet machining (recognised fillet -> ball-nose toolpath) --------------
def _slerp(v0, v1, t):
    """Unit spherical interpolation between unit vectors v0, v1."""
    d = float(np.clip(np.dot(v0, v1), -1.0, 1.0))
    om = math.acos(d)
    if om < 1e-9:
        return v0
    s = math.sin(om)
    return (math.sin((1.0 - t) * om) * v0 + math.sin(t * om) * v1) / s


def fillet_finish(a, b, n_flank, n_hub, fillet_r, r_ball, n_across=5):
    """Ball-nose toolpath for a concave ROOT FILLET of radius `fillet_r` between
    the flank and the hub, running along the blade root edge `a` (nu,3).

    `n_flank`, `n_hub` (nu,3) are the unit surface normals at each station,
    BOTH pointing into the open channel. The fillet centre O_f sits at distance
    fillet_r from each tangent plane; a ball of radius r_ball (<= fillet_r) rolls
    on the fillet arc from the flank-tangent line to the hub-tangent line in
    `n_across` cross-passes. Returns dict(centers, contacts, axis) each
    (n_across, nu, 3): the tool-centre path, the contact points on the fillet,
    and the tool axis (the corner bisector, out of the channel)."""
    a = np.asarray(a, float)
    nf = np.asarray(n_flank, float); nh = np.asarray(n_hub, float)
    nf = nf / np.linalg.norm(nf, axis=1, keepdims=True)
    nh = nh / np.linalg.norm(nh, axis=1, keepdims=True)
    nu = a.shape[0]
    g = np.sum(nf * nh, axis=1)                       # cos(angle between normals)
    Of = a + (fillet_r / (1.0 + g))[:, None] * (nf + nh)
    axis = (nf + nh) / np.linalg.norm(nf + nh, axis=1, keepdims=True)
    centers = np.empty((n_across, nu, 3))
    contacts = np.empty((n_across, nu, 3))
    for k in range(n_across):
        t = k / (n_across - 1) if n_across > 1 else 0.0
        for i in range(nu):
            d = _slerp(-nf[i], -nh[i], t)             # arc direction (unit)
            contacts[k, i] = Of[i] + fillet_r * d
            centers[k, i] = Of[i] + (fillet_r - r_ball) * d
    return dict(centers=centers, contacts=contacts,
                axis=np.broadcast_to(axis, (n_across, nu, 3)).copy(), Of=Of)


# --- root-fillet-aware rail trimming -----------------------------------------
def trim_root_fillet(a: np.ndarray, b: np.ndarray, offset: float):
    """Trim the hub rail UP the ruling by `offset` mm so a flank pass clears the
    root fillet. Each ruling a[i]->b[i] is shortened at the hub end by the
    spanwise tangent offset (for a ~90° flank/hub blend the tangent leaves the
    flank ≈ fillet_radius from the corner, so pass offset≈fillet_radius). The
    shroud rail is unchanged; both rails keep their station count. Returns the
    trimmed (a', b)."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    d = b - a
    L = np.linalg.norm(d, axis=1)
    f = np.clip(np.where(L > 0.0, offset / np.maximum(L, 1e-12), 0.0), 0.0, 0.95)
    a2 = a + f[:, None] * d
    return np.ascontiguousarray(a2), np.ascontiguousarray(b)
