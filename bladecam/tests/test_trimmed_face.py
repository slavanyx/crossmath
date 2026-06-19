#!/usr/bin/env python3
"""Trimmed-face rail extraction (Tier 1).

A real blade flank is a TRIMMED face: its rail can curve in parameter space, so
reading rails off the UV box is wrong. Edge-based extraction follows the actual
boundary edge and stays correct. This builds a face with a shroud rail that
curves in (u,v) and checks edge-based extraction recovers it, while documenting
the error the UV-box method would have made.
"""
import sys

try:
    import numpy as np
    from bladecam import cadio
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP trimmed-face ({e})")
    sys.exit(0)

try:
    from OCP.gp import gp_Pnt, gp_Pnt2d
    from OCP.TColgp import TColgp_Array1OfPnt, TColgp_Array1OfPnt2d
    from OCP.GeomAPI import GeomAPI_PointsToBSpline
    from OCP.Geom2dAPI import Geom2dAPI_PointsToBSpline
    from OCP.GeomFill import GeomFill_BSplineCurves, GeomFill_CoonsStyle
    from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeFace,
                                    BRepBuilderAPI_MakeEdge,
                                    BRepBuilderAPI_MakeWire)
    from OCP.BRepLib import BRepLib
except ImportError:
    print("SKIP trimmed-face (cadquery-ocp not installed)")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def _bspl(pts):
    a = TColgp_Array1OfPnt(1, len(pts))
    for i, p in enumerate(pts):
        a.SetValue(i + 1, gp_Pnt(*p))
    return GeomAPI_PointsToBSpline(a).Curve()


def _bspl2d(pts):
    a = TColgp_Array1OfPnt2d(1, len(pts))
    for i, p in enumerate(pts):
        a.SetValue(i + 1, gp_Pnt2d(*p))
    return Geom2dAPI_PointsToBSpline(a).Curve()


def _close(p, Q):
    return float(np.min(np.linalg.norm(Q - p, axis=1)))


def main():
    nu = 30
    u = np.linspace(0, 1, nu)
    A = np.column_stack([30*np.cos(0.6*u), 30*np.sin(0.6*u), 20*u])
    B = np.column_stack([55*np.cos(1.3*u), 55*np.sin(1.3*u), 8 + 20*u])
    surf = GeomFill_BSplineCurves(_bspl(A), _bspl(B), GeomFill_CoonsStyle).Surface()

    t = np.linspace(0, 1, 20)
    hub = _bspl2d([(x, 0.0) for x in t])
    te = _bspl2d([(1.0, y) for y in t])
    shr = _bspl2d([(x, 1 - 0.25*np.sin(np.pi*x)) for x in t[::-1]])   # curved!
    le = _bspl2d([(0.0, y) for y in t[::-1]])
    w = BRepBuilderAPI_MakeWire()
    for c in (hub, te, shr, le):
        w.Add(BRepBuilderAPI_MakeEdge(c, surf).Edge())
    face = BRepBuilderAPI_MakeFace(surf, w.Wire()).Face()
    BRepLib.BuildCurves3d_s(face)

    a, b = cadio._rails_from_face(face, nu=40)

    # the true curved shroud in 3D
    sh3 = np.array([(lambda p: (p.X(), p.Y(), p.Z()))(
        surf.Value(x, 1 - 0.25*np.sin(np.pi*x))) for x in np.linspace(0, 1, 400)])
    err = min(max(_close(p, sh3) for p in a), max(_close(p, sh3) for p in b))
    check(err < 0.5, "edge-based recovers the curved (trimmed) shroud",
          f"({err*1000:.0f} um)")

    # document the bug edge-based avoids: UV box reads the shroud at v=1
    v1 = np.array([(lambda p: (p.X(), p.Y(), p.Z()))(surf.Value(x, 1.0))
                   for x in np.linspace(0, 1, 400)])
    uvbox_gap = max(_close(p, sh3) for p in v1)
    check(uvbox_gap > 3.0, "UV-box method would be wrong (edge-based needed)",
          f"(UV-box gap {uvbox_gap:.1f} mm)")

    # extracted rails drive the optimiser
    r = compute(Params(strategy="global", rails=(a, b), nu=40))
    check(np.all(np.isfinite(r["dev"])), "trimmed-blade rails optimise")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nTRIMMED-FACE TESTS PASSED")


if __name__ == "__main__":
    main()
