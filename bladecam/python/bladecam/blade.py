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


def rail_tangents(a: np.ndarray, b: np.ndarray):
    """Numerical rail tangents a'(u), b'(u) over uniform u (np.gradient)."""
    nu = a.shape[0]
    u = np.linspace(0.0, 1.0, nu)
    ap = np.gradient(a, u, axis=0)
    bp = np.gradient(b, u, axis=0)
    return ap, bp


def surface(a: np.ndarray, b: np.ndarray, nv: int = 30) -> np.ndarray:
    """Sample S(u,v) on a (nu, nv, 3) grid for visualization."""
    v = np.linspace(0.0, 1.0, nv)[None, :, None]
    return (1.0 - v) * a[:, None, :] + v * b[:, None, :]
