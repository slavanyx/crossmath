"""ctypes bindings to the BladeCAM Fortran numeric core (libbladecam).

Memory layout contract: every (n, 3) array is passed as C-contiguous float64,
which matches the Fortran (3, n) column-major declarations in the C ABI.
"""
from __future__ import annotations

import ctypes
import os
import sys
from ctypes import c_double, c_int, POINTER

import numpy as np

_DBL = POINTER(c_double)


def _find_library() -> str:
    """Locate libbladecam.{so,dylib,dll} from env var or the build/ tree."""
    env = os.environ.get("BLADECAM_LIB")
    if env and os.path.exists(env):
        return env
    names = {
        "linux": "libbladecam.so",
        "darwin": "libbladecam.dylib",
        "win32": "bladecam.dll",
    }
    libname = names.get(sys.platform, "libbladecam.so")
    here = os.path.dirname(os.path.abspath(__file__))
    roots = [
        os.path.join(here, "..", "..", "build", "core"),
        os.path.join(here, "..", "..", "build"),
        os.getcwd(),
    ]
    for r in roots:
        cand = os.path.join(os.path.abspath(r), libname)
        if os.path.exists(cand):
            return cand
    raise FileNotFoundError(
        f"Could not find {libname}. Build the core (see README) or set "
        f"BLADECAM_LIB to the shared library path."
    )


_lib = ctypes.CDLL(_find_library())

_lib.bc_distribution.argtypes = [_DBL, _DBL, c_int, _DBL, _DBL, _DBL]
_lib.bc_two_point.argtypes = [_DBL, _DBL, _DBL, _DBL, c_double, _DBL, _DBL]
_lib.bc_deviation.argtypes = [_DBL, _DBL, c_double, _DBL, c_int, _DBL]
_lib.bc_deviation_cone.argtypes = [_DBL, _DBL, c_double, c_double, _DBL, c_int, _DBL]
_lib.bc_refine_minmax.argtypes = [_DBL, _DBL, _DBL, _DBL, c_double, c_int,
                                  _DBL, _DBL, _DBL]
_lib.bc_optimize_global.argtypes = [_DBL, _DBL, _DBL, _DBL, c_int, c_double,
                                    c_int, c_double, c_double, c_int,
                                    c_double, c_int, _DBL, _DBL, _DBL]
_lib.bc_optimize_double_flank.argtypes = [_DBL, _DBL, _DBL, _DBL, c_int,
                                          c_double, c_int, c_double, c_double,
                                          c_int, _DBL, _DBL, _DBL, _DBL]
_lib.bc_tool_clearance.argtypes = [_DBL, _DBL, c_double, c_double, c_double,
                                   c_double, c_double, c_int, _DBL, c_int, _DBL]
_lib.bc_swept_clearance.argtypes = [_DBL, _DBL, c_double, c_double, c_double,
                                    c_double, c_double, c_int, _DBL, c_int,
                                    c_int, _DBL]
_lib.bc_holder_clearance.argtypes = [_DBL, _DBL, c_double, c_double, c_double,
                                     c_int, _DBL, c_int, _DBL]
_lib.bc_swept_deviation.argtypes = [_DBL, _DBL, _DBL, c_double, c_int,
                                    _DBL, c_int, _DBL]
_lib.bc_swept_surface.argtypes = [_DBL, _DBL, _DBL, c_double, c_int,
                                  _DBL, c_int, _DBL]
_lib.bc_ik_path.argtypes = [c_int, _DBL, _DBL, c_int, _DBL, _DBL]
_lib.bc_topp.argtypes = [_DBL, c_int, c_int, _DBL, _DBL, c_double, c_double,
                         _DBL, _DBL]
_lib.bc_stability_lobes.argtypes = [c_double, c_double, c_double, c_double,
                                    c_int, c_int, c_int, _DBL, _DBL]
_lib.bc_stability_lobes_frf.argtypes = [_DBL, _DBL, _DBL, c_int, c_double,
                                        c_int, c_int, _DBL, _DBL]


def _c(arr: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(arr, dtype=np.float64)


def _ptr(arr: np.ndarray):
    return arr.ctypes.data_as(_DBL)


def distribution(a: np.ndarray, b: np.ndarray):
    """Per-station distribution parameter delta, striction param vstar, and
    striction curve for rails a, b of shape (nu, 3).

    Returns (delta[nu], vstar[nu], strict[nu, 3]).
    """
    a = _c(a); b = _c(b)
    nu = a.shape[0]
    delta = np.empty(nu, dtype=np.float64)
    vstar = np.empty(nu, dtype=np.float64)
    strict = np.empty((nu, 3), dtype=np.float64)
    _lib.bc_distribution(_ptr(a), _ptr(b), c_int(nu),
                         _ptr(delta), _ptr(vstar), _ptr(strict))
    return delta, vstar, strict


def two_point(a_pt, ap, b_pt, bp, R: float):
    """Two-point cutter axis for one ruling. Returns (q0[3], alpha[3])."""
    a_pt = _c(a_pt); ap = _c(ap); b_pt = _c(b_pt); bp = _c(bp)
    q0 = np.empty(3, dtype=np.float64)
    alpha = np.empty(3, dtype=np.float64)
    _lib.bc_two_point(_ptr(a_pt), _ptr(ap), _ptr(b_pt), _ptr(bp),
                      c_double(R), _ptr(q0), _ptr(alpha))
    return q0, alpha


def deviation(q0, alpha, R: float, pts: np.ndarray) -> np.ndarray:
    """Signed deviation g = dist(pt, axis) - R for pts of shape (npts, 3)."""
    q0 = _c(q0); alpha = _c(alpha); pts = _c(pts)
    npts = pts.shape[0]
    g = np.empty(npts, dtype=np.float64)
    _lib.bc_deviation(_ptr(q0), _ptr(alpha), c_double(R),
                      _ptr(pts), c_int(npts), _ptr(g))
    return g


def deviation_cone(q0, alpha, R: float, gamma: float, pts: np.ndarray) -> np.ndarray:
    """Signed deviation for a conical tool (taper half-angle gamma, rad).
    gamma=0 is identical to deviation()."""
    q0 = _c(q0); alpha = _c(alpha); pts = _c(pts)
    npts = pts.shape[0]
    g = np.empty(npts, dtype=np.float64)
    _lib.bc_deviation_cone(_ptr(q0), _ptr(alpha), c_double(R), c_double(gamma),
                           _ptr(pts), c_int(npts), _ptr(g))
    return g


def refine_minmax(a_pt, ap, b_pt, bp, R: float, nv: int = 41):
    """Min-max (Chebyshev) refined cutter axis for one ruling.

    Returns (q0[3], alpha[3], emax) where emax is the worst |g| achieved.
    """
    a_pt = _c(a_pt); ap = _c(ap); b_pt = _c(b_pt); bp = _c(bp)
    q0 = np.empty(3, dtype=np.float64)
    alpha = np.empty(3, dtype=np.float64)
    emax = np.empty(1, dtype=np.float64)
    _lib.bc_refine_minmax(_ptr(a_pt), _ptr(ap), _ptr(b_pt), _ptr(bp),
                          c_double(R), c_int(nv),
                          _ptr(q0), _ptr(alpha), _ptr(emax))
    return q0, alpha, float(emax[0])


def optimize_global(a, b, ap, bp, R: float, nv: int = 41,
                    mu: float = 1.0, gamma: float = 0.0, nsweeps: int = 3,
                    swept_w: float = 0.0, window: int = 8):
    """Global envelope optimization over the whole blade. Rails a,b,ap,bp are
    (nu,3). Returns (q0[nu,3], alpha[nu,3], dev[nu]). mu = smoothness weight
    (0 -> per-ruling min-max); gamma = tool taper (rad); swept_w = weight on the
    swept-overcut penalty (>0 reduces cross-station interference at some cost to
    per-ruling deviation), window = neighbour index half-width for that penalty.
    """
    a = _c(a); b = _c(b); ap = _c(ap); bp = _c(bp)
    nu = a.shape[0]
    q0 = np.empty((nu, 3), dtype=np.float64)
    alpha = np.empty((nu, 3), dtype=np.float64)
    dev = np.empty(nu, dtype=np.float64)
    _lib.bc_optimize_global(_ptr(a), _ptr(b), _ptr(ap), _ptr(bp), c_int(nu),
                            c_double(R), c_int(nv), c_double(mu), c_double(gamma),
                            c_int(nsweeps), c_double(swept_w), c_int(window),
                            _ptr(q0), _ptr(alpha), _ptr(dev))
    return q0, alpha, dev


def stability_lobes(wn_hz: float, zeta: float, k_stiff: float, Kt: float,
                    n_teeth: int = 4, nlobes: int = 6, nptsper: int = 80):
    """Chatter stability-lobe diagram. Returns (rpm[], alim_mm[])."""
    n = nlobes * nptsper
    rpm = np.empty(n, dtype=np.float64)
    alim = np.empty(n, dtype=np.float64)
    _lib.bc_stability_lobes(c_double(wn_hz), c_double(zeta), c_double(k_stiff),
                            c_double(Kt), c_int(n_teeth), c_int(nlobes),
                            c_int(nptsper), _ptr(rpm), _ptr(alim))
    return rpm, alim


def stability_lobes_frf(freq, reG, imG, Kt: float, n_teeth: int = 4,
                        nlobes: int = 6):
    """Stability lobes from a measured tool-tip receptance G=reG+i*imG (mm/N)
    sampled at frequencies `freq` (Hz). Returns (rpm[], alim[]) of length
    nlobes*len(freq); no-chatter samples are NaN."""
    freq = _c(freq); reG = _c(reG); imG = _c(imG)
    nf = freq.shape[0]
    n = nlobes * nf
    rpm = np.empty(n, dtype=np.float64)
    alim = np.empty(n, dtype=np.float64)
    _lib.bc_stability_lobes_frf(_ptr(freq), _ptr(reG), _ptr(imG), c_int(nf),
                                c_double(Kt), c_int(n_teeth), c_int(nlobes),
                                _ptr(rpm), _ptr(alim))
    return rpm, alim


def optimize_double_flank(aL, bL, aR, bR, R: float, nv: int = 41,
                          mu: float = 1.0, gamma: float = 0.0, nsweeps: int = 3):
    """Double-flank channel milling: one cylinder tangent to both walls.
    aL,bL and aR,bR are (nu,3). Returns (q0[nu,3], alpha[nu,3], devL[nu], devR[nu])."""
    aL = _c(aL); bL = _c(bL); aR = _c(aR); bR = _c(bR)
    nu = aL.shape[0]
    q0 = np.empty((nu, 3)); alpha = np.empty((nu, 3))
    devL = np.empty(nu); devR = np.empty(nu)
    _lib.bc_optimize_double_flank(_ptr(aL), _ptr(bL), _ptr(aR), _ptr(bR),
                                  c_int(nu), c_double(R), c_int(nv), c_double(mu),
                                  c_double(gamma), c_int(nsweeps),
                                  _ptr(q0), _ptr(alpha), _ptr(devL), _ptr(devR))
    return q0, alpha, devL, devR


def tool_clearance(q0, alpha, pts, R, flute_len, holder_R, gap, holder_len):
    """Per-station signed clearance of the tool+holder to an obstacle cloud.
    q0,alpha are (nu,3); pts is (npts,3). Returns clr (nu,); <0 = collision."""
    q0 = _c(q0); alpha = _c(alpha); pts = _c(pts)
    nu = q0.shape[0]; npts = pts.shape[0]
    clr = np.empty(nu, dtype=np.float64)
    _lib.bc_tool_clearance(_ptr(q0), _ptr(alpha), c_double(R), c_double(flute_len),
                           c_double(holder_R), c_double(gap), c_double(holder_len),
                           c_int(nu), _ptr(pts), c_int(npts), _ptr(clr))
    return clr


def swept_clearance(q0, alpha, pts, R, flute_len, holder_R, gap, holder_len,
                    nscan=12):
    """Continuous swept-volume clearance: the minimum tool+holder clearance over
    the WHOLE motion from each station to the next (not just sampled poses),
    found by minimising the SDF over the interpolated motion. q0,alpha are
    (nu,3); pts is (npts,3). Returns clr (nu,); clr[i] covers segment [i,i+1],
    clr[-1] is the final static clearance. <0 = collision."""
    q0 = _c(q0); alpha = _c(alpha); pts = _c(pts)
    nu = q0.shape[0]; npts = pts.shape[0]
    clr = np.empty(nu, dtype=np.float64)
    _lib.bc_swept_clearance(_ptr(q0), _ptr(alpha), c_double(R), c_double(flute_len),
                            c_double(holder_R), c_double(gap), c_double(holder_len),
                            c_int(nu), _ptr(pts), c_int(npts), c_int(nscan),
                            _ptr(clr))
    return clr


def holder_clearance(q0, alpha, pts, holder_R, base, holder_len):
    """Per-station clearance of the HOLDER alone (capped cylinder at axial
    [base, base+holder_len], radius holder_R) to an obstacle cloud -- the check
    that applies to the blade being machined (where the flute is tangent by
    design). q0,alpha are (nu,3); pts is (npts,3). Returns clr (nu,); <0 = hit."""
    q0 = _c(q0); alpha = _c(alpha); pts = _c(pts)
    nu = q0.shape[0]; npts = pts.shape[0]
    clr = np.empty(nu, dtype=np.float64)
    _lib.bc_holder_clearance(_ptr(q0), _ptr(alpha), c_double(holder_R),
                             c_double(base), c_double(holder_len), c_int(nu),
                             _ptr(pts), c_int(npts), _ptr(clr))
    return clr


def swept_deviation(q0, alpha, Lflute, R: float, pts: np.ndarray) -> np.ndarray:
    """Swept-envelope deviation: signed distance of each point in pts(npts,3) to
    the closest of all cutter positions (finite flutes), minus R. q0,alpha are
    (nu,3); Lflute is (nu,). g<0 = real overcut (cross-station interference)."""
    q0 = _c(q0); alpha = _c(alpha); Lflute = _c(Lflute); pts = _c(pts)
    nu = q0.shape[0]; npts = pts.shape[0]
    g = np.empty(npts, dtype=np.float64)
    _lib.bc_swept_deviation(_ptr(q0), _ptr(alpha), _ptr(Lflute), c_double(R),
                            c_int(nu), _ptr(pts), c_int(npts), _ptr(g))
    return g


def swept_surface(q0, alpha, Lflute, R: float, pts: np.ndarray) -> np.ndarray:
    """True swept-envelope surface: the machined point for each design point in
    pts(npts,3), projected radially onto the nearest finite-flute cutter over the
    whole path (the boundary of the swept tool volume). q0,alpha are (nu,3);
    Lflute is (nu,). Returns mpts(npts,3) -- the actual machined geometry."""
    q0 = _c(q0); alpha = _c(alpha); Lflute = _c(Lflute); pts = _c(pts)
    nu = q0.shape[0]; npts = pts.shape[0]
    mpts = np.empty((npts, 3), dtype=np.float64)
    _lib.bc_swept_surface(_ptr(q0), _ptr(alpha), _ptr(Lflute), c_double(R),
                          c_int(nu), _ptr(pts), c_int(npts), _ptr(mpts))
    return mpts


def ik_path(Q: np.ndarray, O: np.ndarray, pivot, kind: int = 0) -> np.ndarray:
    """Batch 5-axis inverse kinematics. Q, O are (npts, 3) contact points and
    tool axes; returns machine axes (npts, 5) = [X, Y, Z, A, C] (A, C radians).
    kind: 0 = table-table (workpiece rotates), 1 = head-head (spindle tilts)."""
    Q = _c(Q); O = _c(O); pivot = _c(pivot)
    npts = Q.shape[0]
    m = np.empty((npts, 5), dtype=np.float64)
    _lib.bc_ik_path(c_int(kind), _ptr(Q), _ptr(O), c_int(npts), _ptr(pivot), _ptr(m))
    return m


def topp(q: np.ndarray, vmax, amax, a0: float = 0.0, aN: float = 0.0):
    """Time-optimal parameterization of a joint path q of shape (n, ndof).

    vmax, amax are per-axis limits (length ndof). Returns (aprof[n], ttotal)
    where aprof = sdot^2 along s and ttotal is the traversal time (seconds).
    """
    q = _c(q); vmax = _c(vmax); amax = _c(amax)
    n, ndof = q.shape
    # Fortran expects (ndof, n) column-major == (n, ndof) C-contiguous
    aprof = np.empty(n, dtype=np.float64)
    ttotal = np.empty(1, dtype=np.float64)
    _lib.bc_topp(_ptr(q), c_int(ndof), c_int(n), _ptr(vmax), _ptr(amax),
                 c_double(a0), c_double(aN), _ptr(aprof), _ptr(ttotal))
    return aprof, float(ttotal[0])
