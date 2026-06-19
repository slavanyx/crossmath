"""Point (ball-nose) milling for leading/trailing edges and hub fillets.

Unlike flank milling (line contact), point milling rasters the surface row by
row with a ball-nose tip. The row stepover is set by the allowed scallop height
h between rows: for a ball of radius R on a locally flat surface,
    h = p^2 / (8 R)   =>   p = sqrt(8 R h).
The cutter-contact point is on the surface; the cutter location (ball centre)
is offset by R along the surface normal, and the tool axis is the normal
(3-axis tip contact; a lead/tilt frame can be layered on top).
"""
from __future__ import annotations

import numpy as np


def surface_normals(surf: np.ndarray) -> np.ndarray:
    """Unit normals of a structured (nu,nv,3) surface grid."""
    tu = np.gradient(surf, axis=0)
    tv = np.gradient(surf, axis=1)
    n = np.cross(tu, tv)
    ln = np.linalg.norm(n, axis=2, keepdims=True)
    return np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)


def point_mill(surf: np.ndarray, R_ball: float, scallop_allow: float,
               orient: str = "radial"):
    """Generate ball-nose raster rows over `surf` (nu,nv,3).

    Rows run along u; stepover across v is chosen so the scallop between rows
    stays <= scallop_allow. `orient` fixes the normal/offset side deterministically
    (the cross-product sign alone flips with grid ordering and could put the ball
    INSIDE the part): "radial" points the ball away from the Z (impeller) axis;
    None keeps the raw cross-product sign. Returns the resampled contact grid,
    cutter locations (ball centres), tool axes (normals), row count, achieved
    scallop, and total path length.
    """
    nu, nv, _ = surf.shape
    # physical cross-width (along v) per u-section; size rows from the WIDEST
    # section so the scallop budget holds everywhere (not just on average).
    seg = np.linalg.norm(np.diff(surf, axis=1), axis=2)      # (nu, nv-1)
    sec_w = np.sum(seg, axis=1)                              # (nu,)
    width = float(np.max(sec_w))
    p_max = np.sqrt(8.0 * R_ball * scallop_allow)
    n_rows = max(2, int(np.ceil(width / p_max)) + 1)

    # resample v to n_rows by linear interpolation along the v index
    vidx = np.linspace(0, nv - 1, n_rows)
    lo = np.floor(vidx).astype(int)
    hi = np.minimum(lo + 1, nv - 1)
    frac = (vidx - lo)[None, :, None]
    contact = (1 - frac) * surf[:, lo, :] + frac * surf[:, hi, :]   # (nu, n_rows, 3)

    nrm = surface_normals(surf)
    normals = (1 - frac) * nrm[:, lo, :] + frac * nrm[:, hi, :]
    ln = np.linalg.norm(normals, axis=2, keepdims=True)
    normals = np.divide(normals, ln, out=np.zeros_like(normals), where=ln > 0)

    # deterministic offset side: keep the ball outside the part
    if orient == "radial":
        cen = contact.reshape(-1, 3).mean(0)
        rad = np.array([cen[0], cen[1], 0.0])
        rn = np.linalg.norm(rad)
        if rn > 1e-9:
            rad /= rn
            if float(normals.reshape(-1, 3).mean(0) @ rad) < 0.0:
                normals = -normals

    cl = contact + R_ball * normals
    row_spacing = width / (n_rows - 1)
    scallop = row_spacing ** 2 / (8.0 * R_ball)
    path_len = float(np.sum(np.linalg.norm(np.diff(cl, axis=0), axis=2)))

    return dict(contact=contact, cl=cl, axes=normals, n_rows=n_rows,
                scallop=scallop, width=width, path_len_mm=path_len)


def leading_edge_patch(a: np.ndarray, b: np.ndarray, frac: float = 0.12,
                       nv: int = 16) -> np.ndarray:
    """A small u-band patch at the blade leading edge for edge finishing."""
    nu = a.shape[0]
    k = max(2, int(frac * nu))
    aa, bb = a[:k], b[:k]
    v = np.linspace(0.0, 1.0, nv)[None, :, None]
    return (1 - v) * aa[:, None, :] + v * bb[:, None, :]
