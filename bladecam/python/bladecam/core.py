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
_INT = POINTER(c_int)


def _find_library() -> str:
    """Locate libbladecam.{so,dylib,dll} from env var or the build/ tree."""
    env = os.environ.get("BLADECAM_LIB")
    if env and os.path.exists(env):
        return env
    # candidate library file names per platform. Windows is listed twice because
    # MSVC emits `bladecam.dll` while MinGW/gfortran emits `libbladecam.dll`.
    names = {
        "linux": ["libbladecam.so"],
        "darwin": ["libbladecam.dylib"],
        "win32": ["bladecam.dll", "libbladecam.dll"],
    }
    libnames = names.get(sys.platform, ["libbladecam.so"])
    here = os.path.dirname(os.path.abspath(__file__))
    roots = [
        os.path.join(here, "..", "..", "build", "core"),
        os.path.join(here, "..", "..", "build"),
        os.path.join(here, "..", "..", "build", "core", "Release"),  # MSVC layout
        os.path.join(here, "..", "..", "build", "Release"),
        os.getcwd(),
    ]
    for r in roots:
        for libname in libnames:
            cand = os.path.join(os.path.abspath(r), libname)
            if os.path.exists(cand):
                return cand
    raise FileNotFoundError(
        f"Could not find {' / '.join(libnames)}. Build the core (see "
        f"INSTALL_WINDOWS.md / README) or set BLADECAM_LIB to the library path."
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
                                    c_double, c_int, c_double, c_double,
                                    _DBL, _DBL, _DBL]
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
_lib.bc_swept_deviation.argtypes = [_DBL, _DBL, _DBL, c_double, c_double,
                                    c_double, c_double, c_int, _DBL, c_int, _DBL]
_lib.bc_swept_surface.argtypes = [_DBL, _DBL, _DBL, c_double, c_double,
                                  c_double, c_double, c_int, _DBL, c_int, _DBL]
_lib.bc_deviation_barrel.argtypes = [_DBL, _DBL, c_double, c_double, c_double,
                                     _DBL, c_int, _DBL]
_lib.bc_dexel_carve.argtypes = [_DBL, _DBL, c_double, _DBL, c_int,
                                _DBL, _DBL, _DBL, c_int, _DBL, _DBL]
_lib.bc_dexel_removed_intervals.argtypes = [_DBL, _DBL, c_double, _DBL, c_int,
                                            _DBL, _DBL, c_int, c_int,
                                            _DBL, _DBL, _INT]
_lib.bc_assembly_clearance.argtypes = [_DBL, _DBL, c_int, _DBL, _DBL, _DBL,
                                       c_int, _DBL, c_int, _DBL, _DBL,
                                       c_int, c_int, _DBL]
_lib.bc_struct_clearance.argtypes = [_DBL, c_int, _DBL, c_int, c_int, c_int, _DBL]
_lib.bc_mesh_clearance.argtypes = [_DBL, c_int, _DBL, c_int, c_int, c_int, c_int, _DBL]
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
                    swept_w: float = 0.0, window: int = 8,
                    Rb: float = 0.0, lamc: float = 0.0):
    """Global envelope optimization over the whole blade. Rails a,b,ap,bp are
    (nu,3). Returns (q0[nu,3], alpha[nu,3], dev[nu]). mu = smoothness weight
    (0 -> per-ruling min-max); gamma = tool taper (rad); swept_w = weight on the
    swept-overcut penalty (>0 reduces cross-station interference at some cost to
    per-ruling deviation), window = neighbour index half-width for that penalty.
    Rb>0 selects a barrel tool (arc radius Rb, widest radius R at axial lamc) so
    the axis is fitted to the circle-segment flank instead of a cylinder/cone.
    """
    a = _c(a); b = _c(b); ap = _c(ap); bp = _c(bp)
    nu = a.shape[0]
    q0 = np.empty((nu, 3), dtype=np.float64)
    alpha = np.empty((nu, 3), dtype=np.float64)
    dev = np.empty(nu, dtype=np.float64)
    _lib.bc_optimize_global(_ptr(a), _ptr(b), _ptr(ap), _ptr(bp), c_int(nu),
                            c_double(R), c_int(nv), c_double(mu), c_double(gamma),
                            c_int(nsweeps), c_double(swept_w), c_int(window),
                            c_double(Rb), c_double(lamc),
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


def deviation_barrel(q0, alpha, R: float, Rb: float, lamc: float,
                     pts: np.ndarray) -> np.ndarray:
    """Per-station signed deviation for a BARREL (circle-segment) tool: arc
    radius Rb, widest radius R at axial position lamc along alpha from q0."""
    q0 = _c(q0); alpha = _c(alpha); pts = _c(pts)
    npts = pts.shape[0]
    g = np.empty(npts, dtype=np.float64)
    _lib.bc_deviation_barrel(_ptr(q0), _ptr(alpha), c_double(R), c_double(Rb),
                             c_double(lamc), _ptr(pts), c_int(npts), _ptr(g))
    return g


def swept_deviation(q0, alpha, Lflute, R: float, pts: np.ndarray,
                    gamma: float = 0.0, Rb: float = 0.0,
                    lamc: float = 0.0) -> np.ndarray:
    """Swept-envelope deviation: signed distance of each point in pts(npts,3) to
    the closest of all cutter positions (finite flutes). q0,alpha are (nu,3);
    Lflute (nu,). Tool family: gamma=taper half-angle (cone), or Rb>0 with widest
    radius R at axial lamc (barrel); Rb<=0,gamma=0 is a cylinder. g<0 = overcut."""
    q0 = _c(q0); alpha = _c(alpha); Lflute = _c(Lflute); pts = _c(pts)
    nu = q0.shape[0]; npts = pts.shape[0]
    g = np.empty(npts, dtype=np.float64)
    _lib.bc_swept_deviation(_ptr(q0), _ptr(alpha), _ptr(Lflute), c_double(R),
                            c_double(gamma), c_double(Rb), c_double(lamc),
                            c_int(nu), _ptr(pts), c_int(npts), _ptr(g))
    return g


def swept_surface(q0, alpha, Lflute, R: float, pts: np.ndarray,
                  gamma: float = 0.0, Rb: float = 0.0,
                  lamc: float = 0.0) -> np.ndarray:
    """True swept-envelope surface: the machined point for each design point in
    pts(npts,3), projected onto the nearest finite-flute cutter over the whole
    path. Tool family as in swept_deviation (cylinder/cone/barrel). Returns
    mpts(npts,3)."""
    q0 = _c(q0); alpha = _c(alpha); Lflute = _c(Lflute); pts = _c(pts)
    nu = q0.shape[0]; npts = pts.shape[0]
    mpts = np.empty((npts, 3), dtype=np.float64)
    _lib.bc_swept_surface(_ptr(q0), _ptr(alpha), _ptr(Lflute), c_double(R),
                          c_double(gamma), c_double(Rb), c_double(lamc),
                          c_int(nu), _ptr(pts), c_int(npts), _ptr(mpts))
    return mpts


def assembly_clearance(q0, alpha, seg_R, seg_lo, seg_hi, pts,
                       plane_pt=None, plane_n=None, nscan=12):
    """Continuous swept clearance of the full tool ASSEMBLY (a stack of coaxial
    capped-cylinder segments: flute + holder + spindle nose) to an obstacle cloud
    over the whole motion, plus an optional fixture HALF-SPACE (forbidden where
    plane_n.(x-plane_pt) < 0). q0,alpha (nu,3); seg_* are (nseg,) arrays of radius
    and axial [lo,hi] from q0 along alpha; pts (npts,3). Returns clr (nu,)."""
    q0 = _c(q0); alpha = _c(alpha); pts = _c(pts)
    seg_R = _c(seg_R); seg_lo = _c(seg_lo); seg_hi = _c(seg_hi)
    nseg = seg_R.shape[0]; nu = q0.shape[0]; npts = pts.shape[0]
    use_plane = 1 if plane_pt is not None else 0
    p0 = _c(plane_pt if plane_pt is not None else np.zeros(3))
    n = _c(plane_n if plane_n is not None else np.array([0.0, 0.0, 1.0]))
    clr = np.empty(nu, dtype=np.float64)
    _lib.bc_assembly_clearance(_ptr(q0), _ptr(alpha), c_int(nseg), _ptr(seg_R),
                               _ptr(seg_lo), _ptr(seg_hi), c_int(nu), _ptr(pts),
                               c_int(npts), _ptr(p0), _ptr(n), c_int(use_plane),
                               c_int(nscan), _ptr(clr))
    return clr


def struct_clearance(acaps, bcaps, nscan=8):
    """Structural machine-model clearance: minimum signed clearance between the
    tool-side capsule set `acaps` and the structure-side capsule set `bcaps`,
    swept per station. Both are (nu, n, 7) arrays whose last axis is
    [p0x,p0y,p0z, p1x,p1y,p1z, radius] -- a capsule (round-capped cylinder).
    Returns clr (nu,); clr[i] covers segment [i,i+1], clr[-1] the final static
    pose. <0 = collision. (A capsule conservatively bounds the flat-capped tool
    cylinder, so the clearance is never optimistic.)"""
    acaps = _c(acaps); bcaps = _c(bcaps)
    nu, na = acaps.shape[0], acaps.shape[1]
    nb = bcaps.shape[1]
    clr = np.empty(nu, dtype=np.float64)
    _lib.bc_struct_clearance(_ptr(acaps), c_int(na), _ptr(bcaps), c_int(nb),
                             c_int(nu), c_int(nscan), _ptr(clr))
    return clr


def dexel_carve(q0, alpha, R, Lf, orig, dir, seg0):
    """Dexel material-removal carve. The swept capped-cylinder tool (poses
    q0,alpha (nu,3); flute lengths Lf (nu,)) is subtracted from a field of rays:
    ray r runs from orig[r] along unit dir[r] with material over t in [0,seg0[r]].
    Returns (removed[nray], first_cut[nray]) -- removed length (union over poses)
    and the smallest removed t (the machined-surface crossing) per ray."""
    q0 = _c(q0); alpha = _c(alpha); Lf = _c(Lf)
    orig = _c(orig); dir = _c(dir); seg0 = _c(seg0)
    nu = q0.shape[0]; nray = orig.shape[0]
    removed = np.empty(nray, dtype=np.float64)
    first_cut = np.empty(nray, dtype=np.float64)
    _lib.bc_dexel_carve(_ptr(q0), _ptr(alpha), c_double(R), _ptr(Lf), c_int(nu),
                        _ptr(orig), _ptr(dir), _ptr(seg0), c_int(nray),
                        _ptr(removed), _ptr(first_cut))
    return removed, first_cut


def mesh_clearance(acaps, tris, nscan=4, signed=True):
    """Swept clearance of the tool-assembly capsule set `acaps` (nu, na, 7) to a
    static triangle mesh `tris` (ntri, 9) where each row is [ax,ay,az, bx,by,bz,
    cx,cy,cz]. Returns clr (nu,); clr[i] covers segment [i,i+1], <0 = collision.
    Use mesh_from_faces() to build `tris` from (verts, faces).

    signed=True (closed solids: fixtures, machine bodies) makes the clearance
    negative inside the volume. signed=False (OPEN sheets: a thin blade flank,
    where the inside test is meaningless) uses the unsigned surface distance --
    a tool of radius r crossing a zero-thickness sheet already reads <r, so the
    unsigned clearance still catches it, with no spurious inside-flip."""
    acaps = _c(acaps); tris = _c(tris)
    nu, na = acaps.shape[0], acaps.shape[1]
    ntri = tris.shape[0]
    clr = np.empty(nu, dtype=np.float64)
    _lib.bc_mesh_clearance(_ptr(acaps), c_int(na), _ptr(tris), c_int(ntri),
                           c_int(nu), c_int(nscan), c_int(1 if signed else 0),
                           _ptr(clr))
    return clr


def tris_from_grid(grid):
    """Triangulate a structured surface grid `grid` (nu, nv, 3) into the (ntri, 9)
    layout mesh_clearance expects -- two triangles per quad cell. Used to turn a
    neighbour blade flank into an EXACT (continuous) collision obstacle, with no
    point-sampling gap a thin tool could thread through."""
    grid = np.ascontiguousarray(grid, dtype=np.float64)
    nu, nv = grid.shape[0], grid.shape[1]
    p00 = grid[:-1, :-1]; p10 = grid[1:, :-1]
    p01 = grid[:-1, 1:];  p11 = grid[1:, 1:]
    t1 = np.concatenate([p00, p10, p11], axis=2)         # (nu-1, nv-1, 9)
    t2 = np.concatenate([p00, p11, p01], axis=2)
    return np.ascontiguousarray(
        np.concatenate([t1.reshape(-1, 9), t2.reshape(-1, 9)], axis=0))


def mesh_from_faces(verts, faces):
    """Flatten a (verts, faces) triangle mesh into the (ntri, 9) layout
    mesh_clearance expects (the three vertices of each triangle, in order)."""
    verts = np.ascontiguousarray(verts, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    tri = verts[faces]                                   # (ntri, 3, 3)
    return np.ascontiguousarray(tri.reshape(tri.shape[0], 9))


def dexel_removed_intervals(q0, alpha, R, Lf, orig, dir, maxseg=32):
    """Merged removed sub-intervals along each ray for the swept capped-cylinder
    tool (poses q0,alpha (nu,3); flute lengths Lf scalar/(nu,)). For an
    interval-dexel STOCK carry-across-operations: ray r runs from orig[r] along
    unit dir[r]; returns (rlo, rhi, rn) where rlo[r,:rn[r]], rhi[r,:rn[r]] are the
    disjoint ascending t-intervals the tool removed (clamped to t>=0). Captures
    removal from either end or the middle of a ray (unlike a height field)."""
    q0 = _c(q0); alpha = _c(alpha); orig = _c(orig); dir = _c(dir)
    nu = q0.shape[0]; nray = orig.shape[0]
    Lf = _c(np.full(nu, Lf) if np.isscalar(Lf) else Lf)
    rlo = np.zeros((nray, maxseg), dtype=np.float64)
    rhi = np.zeros((nray, maxseg), dtype=np.float64)
    rn = np.zeros(nray, dtype=np.int32)
    _lib.bc_dexel_removed_intervals(_ptr(q0), _ptr(alpha), c_double(R), _ptr(Lf),
                                    c_int(nu), _ptr(orig), _ptr(dir), c_int(nray),
                                    c_int(maxseg), _ptr(rlo), _ptr(rhi),
                                    rn.ctypes.data_as(_INT))
    return rlo, rhi, rn


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


# --------------------------------------------------------------------------
# Joint-space (forward-kinematics) swept collision: the machine moves the 5
# JOINTS linearly between stations, so the tool tip+axis it traverses in the
# part frame is the FK of the interpolated joints -- a curve, not the straight
# part-frame chord the bare swept routines interpolate. Densifying each segment
# into FK sub-poses (with a substep count that keeps the chord-vs-arc residual
# under tol -- uniform conservative advancement) and reusing the t-exact
# per-point golden core makes the swept check verify the path the machine RUNS.
# --------------------------------------------------------------------------
def _rotx_np(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rotz_np(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def fk_poses(machine_path, pivot, kind=0):
    """Forward kinematics: machine axes (n,5)=[X,Y,Z,A,C] -> part-frame contact
    point q0 (n,3) and tool axis (n,3). Matches kinematics.f90 forward_kin_ac:
    O = Rz(C)Rx(A) z; Q = Rz(C)Rx(A)(Pm-piv)+piv (kind 0) or Pm (kind 1)."""
    m = _c(machine_path); piv = np.asarray(pivot, float)
    n = m.shape[0]
    q0 = np.empty((n, 3)); ax = np.empty((n, 3))
    z = np.array([0.0, 0.0, 1.0])
    for i in range(n):
        RR = _rotz_np(m[i, 4]) @ _rotx_np(m[i, 3])
        ax[i] = RR @ z
        q0[i] = m[i, 0:3] if kind == 1 else RR @ (m[i, 0:3] - piv) + piv
    return q0, ax


def densify_fk(machine_path, pivot, kind, reach, tol=0.03, cap=400):
    """Per-segment FK densification. Returns the dense machine joints `dense_m`
    (Nd,5), the dense part-frame poses (q0, axis) (Nd,3), and an `owner` array
    mapping each dense pose to its original segment index. The substep count for
    a segment is ceil(dtheta / sqrt(8 tol / rho)), with dtheta = |dA|+|dC| and
    rho bounding the farthest tool point's distance from the rotary axes, so the
    part-frame lerp between consecutive FK sub-poses deviates from the true arc by
    < tol (the conservative-advancement guarantee, #8)."""
    m = _c(machine_path); piv = np.asarray(pivot, float)
    nu = m.shape[0]
    q0s, _ = fk_poses(m, piv, kind)
    dense_m = []; owner = []
    for i in range(nu - 1):
        dth = abs(m[i + 1, 3] - m[i, 3]) + abs(m[i + 1, 4] - m[i, 4])
        rho = max(np.hypot(*q0s[i, :2]), np.hypot(*q0s[i + 1, :2])) + reach
        if dth < 1e-12 or rho < 1e-12:
            nsub = 1
        else:
            nsub = int(np.ceil(dth / np.sqrt(8.0 * tol / rho)))
        nsub = max(1, min(nsub, cap))
        for sidx in range(nsub):
            t = sidx / nsub
            dense_m.append((1.0 - t) * m[i] + t * m[i + 1])
            owner.append(i)
    dense_m.append(m[-1]); owner.append(nu - 2)
    dense_m = np.array(dense_m)
    dq, da = fk_poses(dense_m, piv, kind)
    return dense_m, dq, da, np.array(owner, dtype=int)


def reduce_to_segments(clr_dense, owner, nu):
    return _reduce_to_segments(clr_dense, owner, nu)


def _reduce_to_segments(clr_dense, owner, nu):
    """Collapse a dense per-sub-segment clearance back to nu original entries:
    clr[i] = min over the dense segments owned by original segment i; clr[-1] is
    the dense static endpoint."""
    out = np.full(nu, np.inf)
    for j in range(len(owner) - 1):
        if clr_dense[j] < out[owner[j]]:
            out[owner[j]] = clr_dense[j]
    out[nu - 1] = clr_dense[-1]
    return out


def assembly_clearance_fk(machine_path, pivot, kind, seg_R, seg_lo, seg_hi, pts,
                          plane_pt=None, plane_n=None, nscan=4, tol=0.03):
    """Guaranteed swept clearance of the tool ASSEMBLY along the real joint-space
    (FK) path -- the #1+#8 collision check. Same return contract as
    assembly_clearance (clr (nu,); clr[i] covers segment [i,i+1])."""
    reach = float(np.max(seg_hi) + np.max(seg_R))
    _, dq, da, owner = densify_fk(machine_path, pivot, kind, reach, tol=tol)
    clr_dense = assembly_clearance(dq, da, seg_R, seg_lo, seg_hi, pts,
                                   plane_pt=plane_pt, plane_n=plane_n, nscan=nscan)
    return _reduce_to_segments(clr_dense, owner, len(machine_path))


def holder_clearance_fk(machine_path, pivot, kind, holder_R, base, holder_len,
                        pts, nscan=4, tol=0.03):
    """Guaranteed SWEPT holder-vs-cut-blade clearance along the FK path (#4): the
    holder modelled as a one-segment assembly, densified and swept like the rest
    (replaces the per-station holder_clearance)."""
    reach = float(base + holder_len + holder_R)
    _, dq, da, owner = densify_fk(machine_path, pivot, kind, reach, tol=tol)
    clr_dense = assembly_clearance(dq, da, np.array([holder_R]), np.array([base]),
                                   np.array([base + holder_len]), pts, nscan=nscan)
    return _reduce_to_segments(clr_dense, owner, len(machine_path))


def holder_clearance_swept(q0, alpha, pts, holder_R, base, holder_len, nscan=8):
    """SWEPT holder-vs-obstacle clearance (the #4 fix): the holder as a one-
    segment capped cylinder, swept + golden-refined over each motion segment,
    replacing the per-station holder_clearance which missed a holder swinging
    into the cut blade BETWEEN stations. q0,alpha (nu,3); pts (npts,3)."""
    return assembly_clearance(q0, alpha, np.array([holder_R]), np.array([base]),
                              np.array([base + holder_len]), pts, nscan=nscan)
