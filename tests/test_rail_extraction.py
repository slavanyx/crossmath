#!/usr/bin/env python3
"""Automatic STEP rail-extraction test (skips without OpenCASCADE).

Builds a ruled surface from KNOWN hub/shroud rails, writes STEP, extracts the
rails back, and verifies they match ground truth and drive the pipeline.
"""
import sys
import os
import tempfile

try:
    import numpy as np
    from bladecam import cadio
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP rail-extraction ({e})")
    sys.exit(0)

try:
    from OCP.gp import gp_Pnt
    from OCP.TColgp import TColgp_Array1OfPnt
    from OCP.GeomAPI import GeomAPI_PointsToBSpline
    from OCP.GeomFill import GeomFill_BSplineCurves, GeomFill_CoonsStyle
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
except ImportError:
    print("SKIP rail-extraction (cadquery-ocp not installed)")
    sys.exit(0)


def _bspline(pts):
    arr = TColgp_Array1OfPnt(1, len(pts))
    for i, p in enumerate(pts):
        arr.SetValue(i + 1, gp_Pnt(*p))
    return GeomAPI_PointsToBSpline(arr).Curve()


def _dist_to_polyline(pt, poly):
    return float(np.min(np.linalg.norm(poly - pt, axis=1)))


def main():
    nu = 30
    u = np.linspace(0, 1, nu)
    A = np.column_stack([30*np.cos(0.6*u), 30*np.sin(0.6*u), 20*u])
    B = np.column_stack([55*np.cos(1.3*u), 55*np.sin(1.3*u), 8 + 20*u])

    with tempfile.TemporaryDirectory() as d:
        step = os.path.join(d, "blade.step")
        surf = GeomFill_BSplineCurves(_bspline(A), _bspline(B),
                                      GeomFill_CoonsStyle).Surface()
        face = BRepBuilderAPI_MakeFace(surf, 1e-6).Face()
        w = STEPControl_Writer(); w.Transfer(face, STEPControl_AsIs); w.Write(step)

        a, b = cadio.rails_from_cad(step, nu=40)

    assert a.shape == (40, 3) and b.shape == (40, 3), "rail shape"
    # corners are exact regardless of reparameterisation
    assert np.allclose(a[0], A[0], atol=1e-3), "hub start"
    assert np.allclose(a[-1], A[-1], atol=1e-3), "hub end"
    assert np.allclose(b[0], B[0], atol=1e-3), "shroud start"
    assert np.allclose(b[-1], B[-1], atol=1e-3), "shroud end"
    # every extracted point lies on the true rail curve
    Ad = np.column_stack([30*np.cos(0.6*np.linspace(0, 1, 800)),
                          30*np.sin(0.6*np.linspace(0, 1, 800)),
                          20*np.linspace(0, 1, 800)])
    err = max(_dist_to_polyline(p, Ad) for p in a)
    assert err < 0.2, f"hub rail off true curve by {err:.3f} mm"

    # extracted rails must drive the pipeline
    r = compute(Params(strategy="global", rails=(a, b), nu=40))
    assert np.all(np.isfinite(r["dev"])), "pipeline on extracted rails"

    print(f"rail extraction OK (max rail error {err*1000:.1f} um, "
          f"global dev {r['dev'].max()*1000:.1f} um)")


if __name__ == "__main__":
    main()
