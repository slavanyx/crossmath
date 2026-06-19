#!/usr/bin/env python3
"""Fuzz the CAD importer / rail extraction with pathological geometry.

These are NOT blade flanks; the contract is graceful degradation -- no
unhandled OpenCASCADE exceptions, finite output (rails fall back to the UV box
on non-4-edge / degenerate faces). Skips without OpenCASCADE.
"""
import sys
import tempfile
import os

try:
    import numpy as np
    from bladecam import cadio
except ImportError as e:
    print(f"SKIP cad-fuzz ({e})")
    sys.exit(0)

try:
    from OCP.BRepPrimAPI import (BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeBox,
                                 BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeCone)
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    from OCP.gp import gp_Pnt
    from OCP.TColgp import TColgp_Array1OfPnt
    from OCP.GeomAPI import GeomAPI_PointsToBSpline
    from OCP.GeomFill import GeomFill_BSplineCurves, GeomFill_CoonsStyle
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
except ImportError:
    print("SKIP cad-fuzz (cadquery-ocp not installed)")
    sys.exit(0)

FAILED = []


def check(c, name):
    print(f"  {'ok  ' if c else 'FAIL'} {name}")
    if not c:
        FAILED.append(name)


def _write(shape):
    d = tempfile.mkdtemp()
    p = os.path.join(d, "x.step")
    w = STEPControl_Writer(); w.Transfer(shape, STEPControl_AsIs); w.Write(p)
    return p


def _bspl(pts):
    a = TColgp_Array1OfPnt(1, len(pts))
    for i, q in enumerate(pts):
        a.SetValue(i + 1, gp_Pnt(*q))
    return GeomAPI_PointsToBSpline(a).Curve()


def main():
    rng = np.random.default_rng(0)
    u = np.linspace(0, 1, 12)
    A2 = np.cumsum(rng.standard_normal((12, 3)), 0) * 5
    B2 = A2 + rng.standard_normal((12, 3)) * 8 + np.array([0, 0, 30.0])
    wild = BRepBuilderAPI_MakeFace(
        GeomFill_BSplineCurves(_bspl(A2), _bspl(B2), GeomFill_CoonsStyle).Surface(),
        1e-6).Face()

    cases = {
        "cylinder": BRepPrimAPI_MakeCylinder(10., 30.).Shape(),
        "box": BRepPrimAPI_MakeBox(20., 30., 40.).Shape(),
        "sphere": BRepPrimAPI_MakeSphere(15.).Shape(),
        "cone": BRepPrimAPI_MakeCone(12., 4., 25.).Shape(),
        "wild_bspline": wild,
    }
    for name, shp in cases.items():
        path = _write(shp)
        # mesh import must not raise
        try:
            v, f = cadio.read_step(path)
            mesh_ok = np.all(np.isfinite(v))
        except Exception as e:
            mesh_ok = False
            print(f"    mesh raised on {name}: {e}")
        # rail extraction must not raise and must be finite
        try:
            a, b = cadio.rails_from_cad(path, nu=30)
            rail_ok = (a.shape == (30, 3) and np.all(np.isfinite(a))
                       and np.all(np.isfinite(b)))
        except Exception as e:
            rail_ok = False
            print(f"    rails raised on {name}: {e}")
        check(mesh_ok and rail_ok, f"{name}: graceful (mesh + finite rails)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nCAD-FUZZ TESTS PASSED")


if __name__ == "__main__":
    main()
