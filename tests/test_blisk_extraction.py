#!/usr/bin/env python3
"""Multi-face blisk extraction: a shape with several blade faces must yield one
rail pair per blade, each recovering its own geometry."""
import sys
import os
import tempfile

try:
    import numpy as np
    from bladecam import cadio
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP blisk-extraction ({e})")
    sys.exit(0)

try:
    from OCP.gp import gp_Pnt
    from OCP.TColgp import TColgp_Array1OfPnt
    from OCP.GeomAPI import GeomAPI_PointsToBSpline
    from OCP.GeomFill import GeomFill_BSplineCurves, GeomFill_CoonsStyle
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.TopoDS import TopoDS_Compound
    from OCP.BRep import BRep_Builder
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
except ImportError:
    print("SKIP blisk-extraction (cadquery-ocp not installed)")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def _bspline(pts):
    arr = TColgp_Array1OfPnt(1, len(pts))
    for i, p in enumerate(pts):
        arr.SetValue(i + 1, gp_Pnt(*p))
    return GeomAPI_PointsToBSpline(arr).Curve()


def _face(A, B):
    s = GeomFill_BSplineCurves(_bspline(A), _bspline(B), GeomFill_CoonsStyle).Surface()
    return BRepBuilderAPI_MakeFace(s, 1e-6).Face()


def main():
    nu = 28
    u = np.linspace(0, 1, nu)
    # two blades at different angular positions (a 2-blade "blisk")
    blades = []
    for phase in (0.0, 2*np.pi/5):
        A = np.column_stack([30*np.cos(0.6*u + phase), 30*np.sin(0.6*u + phase), 20*u])
        B = np.column_stack([55*np.cos(1.3*u + phase), 55*np.sin(1.3*u + phase), 8 + 20*u])
        blades.append((A, B))

    comp = TopoDS_Compound()
    bld = BRep_Builder(); bld.MakeCompound(comp)
    for A, B in blades:
        bld.Add(comp, _face(A, B))

    with tempfile.TemporaryDirectory() as d:
        step = os.path.join(d, "blisk.step")
        w = STEPControl_Writer(); w.Transfer(comp, STEPControl_AsIs); w.Write(step)
        rails = cadio.rails_list_from_cad(step, nu=30)

    check(len(rails) == 2, "extracted one rail pair per blade", f"(got {len(rails)})")
    # each extracted blade must match ONE of the ground-truth blades at its corners
    for (a, b) in rails:
        matched = any(np.allclose(a[0], A[0], atol=1e-2) or
                      np.allclose(a[0], B[0], atol=1e-2) or
                      np.allclose(b[0], A[0], atol=1e-2) or
                      np.allclose(b[0], B[0], atol=1e-2)
                      for (A, B) in blades)
        check(matched, "extracted blade matches a ground-truth blade corner")
    # rails are consistently oriented: a = hub (inner radius), station 0 = lower Z
    for (a, b) in rails:
        ra = np.mean(np.hypot(a[:, 0], a[:, 1]))
        rb = np.mean(np.hypot(b[:, 0], b[:, 1]))
        check(ra <= rb, "rail a is the hub (inner radius)", f"({ra:.1f}<= {rb:.1f})")
        check(a[0, 2] <= a[-1, 2], "station 0 is the lower-Z (hub) end")

    # direct unit check of the orientation normaliser (no CAD needed): a
    # shroud-first, Z-reversed pair must come back hub-first, low-Z-first
    aa = np.column_stack([55*np.cos(0.6*u), 55*np.sin(0.6*u), 20 - 20*u])  # outer, hi->lo
    bb = np.column_stack([30*np.cos(0.6*u), 30*np.sin(0.6*u), -20*u])      # inner
    oa, ob = cadio._orient_hub_first(aa, bb)
    check(np.mean(np.hypot(oa[:, 0], oa[:, 1])) < np.mean(np.hypot(ob[:, 0], ob[:, 1]))
          and oa[0, 2] <= oa[-1, 2], "orient_hub_first normalises a swapped/reversed pair")

    # each drives the optimiser
    r = compute(Params(strategy="global", rails=rails[0], nu=30))
    check(r["dev"].max() < 0.1, "extracted blisk blade optimises well",
          f"({r['dev'].max()*1000:.1f} um)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nBLISK EXTRACTION TESTS PASSED")


if __name__ == "__main__":
    main()
