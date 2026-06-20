#!/usr/bin/env python3
"""End-to-end acceptance: a multi-blade TRIMMED blisk STEP through the whole
pipeline -- extract every blade, optimise, collision-check, time, post G-code.

This is the closest in-repo stand-in for 'run a real impeller STEP': the part is
CAD (B-rep, trimmed, multi-face), not the parametric generator. Skips without
OpenCASCADE.
"""
import sys
import os
import tempfile

try:
    import numpy as np
    from bladecam import cadio, postproc
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP end-to-end ({e})")
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
    from OCP.TopoDS import TopoDS_Compound
    from OCP.BRep import BRep_Builder
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
except ImportError:
    print("SKIP end-to-end (cadquery-ocp not installed)")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def _bspl(p):
    arr = TColgp_Array1OfPnt(1, len(p))
    for i, q in enumerate(p):
        arr.SetValue(i + 1, gp_Pnt(*q))
    return GeomAPI_PointsToBSpline(arr).Curve()


def _bspl2d(p):
    arr = TColgp_Array1OfPnt2d(1, len(p))
    for i, q in enumerate(p):
        arr.SetValue(i + 1, gp_Pnt2d(*q))
    return Geom2dAPI_PointsToBSpline(arr).Curve()


def _trimmed_blade_face(phase):
    u = np.linspace(0, 1, 30)
    A = np.column_stack([30*np.cos(0.6*u + phase), 30*np.sin(0.6*u + phase), 20*u])
    B = np.column_stack([55*np.cos(1.3*u + phase), 55*np.sin(1.3*u + phase), 8 + 20*u])
    surf = GeomFill_BSplineCurves(_bspl(A), _bspl(B), GeomFill_CoonsStyle).Surface()
    t = np.linspace(0, 1, 20)
    hub = _bspl2d([(x, 0.0) for x in t])
    te = _bspl2d([(1.0, y) for y in t])
    shr = _bspl2d([(x, 1 - 0.2*np.sin(np.pi*x)) for x in t[::-1]])  # curved (trimmed)
    le = _bspl2d([(0.0, y) for y in t[::-1]])
    w = BRepBuilderAPI_MakeWire()
    for c in (hub, te, shr, le):
        w.Add(BRepBuilderAPI_MakeEdge(c, surf).Edge())
    f = BRepBuilderAPI_MakeFace(surf, w.Wire()).Face()
    BRepLib.BuildCurves3d_s(f)
    return f


def main():
    n_blades = 3
    comp = TopoDS_Compound(); bld = BRep_Builder(); bld.MakeCompound(comp)
    for k in range(n_blades):
        bld.Add(comp, _trimmed_blade_face(k * 2 * np.pi / 7))

    with tempfile.TemporaryDirectory() as d:
        step = os.path.join(d, "blisk.step")
        w = STEPControl_Writer(); w.Transfer(comp, STEPControl_AsIs); w.Write(step)
        rails = cadio.rails_list_from_cad(step, nu=40)

    check(len(rails) == n_blades, "extracted every blade from the blisk STEP",
          f"({len(rails)})")

    total_cycle = 0.0
    for i, (a, b) in enumerate(rails):
        r = compute(Params(strategy="global", rails=(a, b), n_blades=7))
        # the default minimises the swept-envelope error (the real machined
        # deviation); assert on that, not the per-ruling residual `dev`.
        ok = (np.all(np.isfinite(r["dev"])) and r["swept_overcut"] < 0.15 and
              np.all(np.isfinite(r["machine_path"])) and r["cycle_time_s"] > 0 and
              np.all(np.isfinite(r["move_times_s"])))
        check(ok, f"blade {i}: optimise+IK+TOPP valid",
              f"(swept {r['swept_overcut']*1000:.1f} um, dev {r['dev'].max()*1000:.1f} um, "
              f"cycle {r['cycle_time_s']:.2f} s)")
        g = postproc.to_gcode(r["machine_path"], r["feed_cap_mm_min"],
                              move_times=r["move_times_s"])
        check("G93" in g and g.strip().endswith("M30"), f"blade {i}: valid G-code")
        total_cycle += r["cycle_time_s"]

    print(f"\n  total finishing cycle for the blisk: {total_cycle:.1f} s")
    if FAILED:
        print(f"FAILED: {FAILED}")
        sys.exit(1)
    print("END-TO-END (CAD blisk -> G-code) PASSED")


if __name__ == "__main__":
    main()
