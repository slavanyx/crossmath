"""Parametric impeller-blade flank generator (a non-developable ruled surface)
for demos and tests, plus convenience evaluation helpers.
"""
from __future__ import annotations

import numpy as np


def make_blade(nu: int = 60,
               r_hub: float = 30.0, r_shroud: float = 55.0,
               z_span: float = 20.0, z_offset: float = 8.0,
               wrap: float = 0.6, twist: float = 0.7):
    """Return hub rail a(nu,3) and shroud rail b(nu,3) of a twisted blade.

    The shroud rail is angularly advanced by `twist*u` relative to the hub,
    which makes the ruling director rotate along u -> non-developable warp.
    Units are millimetres / radians.
    """
    u = np.linspace(0.0, 1.0, nu)
    th = wrap * u
    a = np.column_stack([r_hub * np.cos(th),
                         r_hub * np.sin(th),
                         z_span * u])
    ths = wrap * u + twist * u
    b = np.column_stack([r_shroud * np.cos(ths),
                         r_shroud * np.sin(ths),
                         z_offset + z_span * u])
    return a, b


def make_complex_blade(nu: int = 80,
                       rh0: float = 30.0, rh1: float = 55.0,
                       rs0: float = 55.0, rs1: float = 80.0,
                       z_span: float = 24.0, z_offset: float = 8.0,
                       wrap: float = 1.0, twist: float = 0.9,
                       backsweep: float = 0.0, warp: float = 0.0,
                       lean: float = 0.0, radial_curve: float = 0.0,
                       u0: float = 0.0, u1: float = 1.0):
    """A richer impeller/blisk flank for hard demos: a non-developable ruled
    surface with independent hub/shroud radius growth and several real
    3-D distortions of a centrifugal/mixed-flow blade.

    wrap          base hub spiral angle (rad) swept along u
    twist         extra angular advance of the shroud (drives non-developability)
    backsweep     trailing-edge lean: extra shroud advance ∝ u² (backswept impeller)
    warp          S-warp: an inflection sin(2πu) added to the spiral (turbine S-curve)
    lean          z-bow of the shroud rail (sin(πu) hump), as a fraction of z_span
    radial_curve  non-linear radius growth (mixed-flow axial→radial bulge)
    u0,u1         machine only a sub-span [u0,u1] of the blade (splitter blades)

    Returns hub rail a(nu,3) and shroud rail b(nu,3). Units mm / rad.
    """
    u = np.linspace(u0, u1, nu)
    un = u                                          # u already in [0,1] sub-span
    bulge = radial_curve * 4.0 * un * (1.0 - un)    # 0 at ends, peak mid-span
    rh = rh0 + (rh1 - rh0) * (un + bulge)
    rs = rs0 + (rs1 - rs0) * (un + bulge)
    th = wrap * un + warp * np.sin(2.0 * np.pi * un)            # hub spiral (+S-warp)
    ths = th + twist * un + backsweep * un ** 2                 # shroud advance
    zh = z_span * un
    zs = z_offset + z_span * un + lean * z_span * np.sin(np.pi * un)
    a = np.column_stack([rh * np.cos(th), rh * np.sin(th), zh])
    b = np.column_stack([rs * np.cos(ths), rs * np.sin(ths), zs])
    return np.ascontiguousarray(a), np.ascontiguousarray(b)


def rail_tangents(a: np.ndarray, b: np.ndarray):
    """Numerical rail tangents a'(u), b'(u) over uniform u (np.gradient)."""
    nu = a.shape[0]
    u = np.linspace(0.0, 1.0, nu)
    # edge_order=2: second-order one-sided differences at the blade tips, so the
    # rail tangents there are as accurate as the interior (first-order ends are
    # ~150x worse and degrade the two-point positioning/seed at the tips).
    ap = np.gradient(a, u, axis=0, edge_order=2)
    bp = np.gradient(b, u, axis=0, edge_order=2)
    return ap, bp


def surface(a: np.ndarray, b: np.ndarray, nv: int = 30) -> np.ndarray:
    """Sample S(u,v) on a (nu, nv, 3) grid for visualization."""
    v = np.linspace(0.0, 1.0, nv)[None, :, None]
    return (1.0 - v) * a[:, None, :] + v * b[:, None, :]
