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
_lib.bc_refine_minmax.argtypes = [_DBL, _DBL, _DBL, _DBL, c_double, c_int,
                                  _DBL, _DBL, _DBL]


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
