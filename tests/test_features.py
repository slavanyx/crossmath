#!/usr/bin/env python3
"""Native-CAD feature recognition audit: splitter-blade classification, root-
fillet/blend recognition by curvature, and fillet-aware rail trimming -- both as
pure geometry and end-to-end through a synthetic STEP blisk (main + splitter +
fillet faces)."""
import sys
import os
import tempfile

try:
    import numpy as np
    from bladecam import features, cadio, pipeline
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP features ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def _ruled(A_curve, nu, nv):
    """Sample a ruled surface between two parametric curves into (nu,nv,3)."""
    g = np.empty((nu, nv, 3))
    for i in range(nu):
        a, b = A_curve(i / (nu - 1))
        for j in range(nv):
            t = j / (nv - 1)
            g[i, j] = (1 - t) * a + t * b
    return g


def main():
    # --- splitter recognition (pure geometry) ---
    u = np.linspace(0, 1, 30)
    def blade(phase, u0=0.0):
        uu = u0 + (1 - u0) * u
        a = np.column_stack([30 * np.cos(0.6 * uu + phase),
                             30 * np.sin(0.6 * uu + phase), 20 * uu])
        b = np.column_stack([55 * np.cos(1.3 * uu + phase),
                             55 * np.sin(1.3 * uu + phase), 8 + 20 * uu])
        return a, b
    mains = [blade(0.0), blade(0.6), blade(1.2)]
    splitter = blade(0.3, u0=0.5)            # starts mid-channel -> ~half length
    rails = mains + [splitter]
    labels = features.classify_blades(rails)
    check(labels.count("main") == 3 and labels.count("splitter") == 1,
          "splitter recognised among main blades", f"({labels})")
    check(labels[-1] == "splitter", "the short blade is the splitter")
    # a uniform set has no splitters
    check(set(features.classify_blades(mains)) == {"main"},
          "uniform-length blades are all 'main'")
    check(features.classify_blades([mains[0]]) == ["main"], "single blade is main")
    check(features.blade_length(*blade(0.0, 0.0)) >
          features.blade_length(*blade(0.0, 0.6)), "blade_length tracks span")

    # --- fillet / blend recognition by curvature ---
    # a tight cylindrical blend (radius 3) is a fillet; a gentle flank is not
    def fillet_curve(s):                      # quarter-cylinder radius 3 along x
        ang0, ang1 = 0.0, np.pi / 2
        return (np.array([40 * s, 3 * np.cos(ang0), 3 * np.sin(ang0)]),
                np.array([40 * s, 3 * np.cos(ang1), 3 * np.sin(ang1)]))
    # build a real quarter-cylinder grid (not just two rulings)
    nu_g, nv_g = 12, 12
    fil = np.empty((nu_g, nv_g, 3))
    for i in range(nu_g):
        for j in range(nv_g):
            ang = (np.pi / 2) * j / (nv_g - 1)
            fil[i, j] = (40 * i / (nu_g - 1), 3 * np.cos(ang), 3 * np.sin(ang))
    rad = features.min_curvature_radius(fil)
    check(abs(rad - 3.0) < 0.3, "fillet radius recovered from curvature",
          f"({rad:.2f} vs 3.0)")
    check(features.is_fillet_surface(fil, max_radius=8.0), "tight blend flagged as fillet")
    # gentle flank surface (radius ~ tens of mm) is NOT a fillet
    flank = _ruled(lambda s: blade(0.0)[0][int(s * 29)] if False else
                   (np.array([30 * np.cos(0.6 * s), 30 * np.sin(0.6 * s), 20 * s]),
                    np.array([55 * np.cos(1.3 * s), 55 * np.sin(1.3 * s), 8 + 20 * s])),
                   16, 16)
    check(not features.is_fillet_surface(flank, max_radius=8.0),
          "gentle flank is not a fillet", f"(r={features.min_curvature_radius(flank):.1f})")
    # a flat grid has (near-)infinite radius
    flat = np.zeros((6, 6, 3));
    flat[:, :, 0] = np.linspace(0, 10, 6)[:, None]
    flat[:, :, 1] = np.linspace(0, 10, 6)[None, :]
    check(features.min_curvature_radius(flat) > 1e6, "flat surface has ~infinite radius")
    # straight rulings contribute ~0 curvature (ruled-surface property)
    check(features.characteristic_curvature(flank) <
          features.characteristic_curvature(fil),
          "ruled flank has lower characteristic curvature than a blend")

    # --- root-fillet-aware rail trim ---
    a, b = blade(0.0)
    off = 4.0
    a2, b2 = features.trim_root_fillet(a, b, off)
    L0 = np.linalg.norm(b - a, axis=1)
    L1 = np.linalg.norm(b2 - a2, axis=1)
    check(np.allclose(L0 - L1, off, atol=1e-6) or np.all(L1 < L0),
          "trim shortens each ruling at the hub", f"(ΔL≈{np.mean(L0-L1):.2f})")
    check(np.allclose(b2, b), "trim leaves the shroud rail unchanged")
    # the trimmed hub rail sits between the old hub and the shroud
    frac = np.linalg.norm(a2 - a, axis=1) / np.maximum(L0, 1e-9)
    check(np.all((frac >= 0) & (frac <= 0.95)), "hub rail moves up the ruling, clipped")
    # clipping: a huge offset never inverts the ruling
    a3, _ = features.trim_root_fillet(a, b, 1e6)
    check(np.all(np.linalg.norm(b - a3, axis=1) > 0), "over-trim is clamped, not inverted")

    # --- fillet machining: ball-nose toolpath for the recognised root fillet ---
    # exact 90-degree corner (edge along x, flank +y, hub +z): the geometry is
    # closed-form, so every invariant is checkable to machine precision
    nuf = 6
    ea = np.column_stack([np.linspace(0, 10, nuf), np.zeros(nuf), np.zeros(nuf)])
    eb = ea + np.array([0, 5.0, 0])
    nf = np.tile([0, 1., 0], (nuf, 1)); nh = np.tile([0, 0, 1.], (nuf, 1))
    rf, rb = 4.0, 2.0
    fp = features.fillet_finish(ea, eb, nf, nh, rf, rb, n_across=5)
    C, P = fp["centers"], fp["contacts"]
    check(np.allclose(np.linalg.norm(C - P, axis=2), rb),
          "ball is tangent to the fillet (|center-contact| = r_ball)")
    check(np.allclose(np.linalg.norm(P - fp["Of"][None], axis=2), rf),
          "contacts lie on the fillet arc (|contact-Of| = fillet_r)")
    check(C[..., 1].min() >= rb - 1e-9 and C[..., 2].min() >= rb - 1e-9,
          "tool centre never gouges the flank or hub (>= r_ball from both)",
          f"(min {min(C[...,1].min(), C[...,2].min()):.3f})")
    check(abs(P[0, 0, 1]) < 1e-9 and abs(P[-1, 0, 2]) < 1e-9,
          "fillet pass spans flank-tangent to hub-tangent")
    # a smaller ball reaches DEEPER into the corner (its centre sits closer to
    # the root apex) yet is still gouge-free; the contacts are unchanged
    fine = features.fillet_finish(ea, eb, nf, nh, rf, 1.0, n_across=5)
    deep_fine = np.linalg.norm(fine["centers"][2, 0] - ea[0])
    deep_coarse = np.linalg.norm(C[2, 0] - ea[0])
    check(deep_fine < deep_coarse and fine["centers"][..., 1].min() >= 1.0 - 1e-9
          and np.allclose(fine["contacts"], P),
          "a smaller ball reaches deeper into the fillet, still gouge-free")

    # pipeline op: gouge-free toolpath of the right shape on the real blade
    fm = pipeline.fillet_machining(Params(strategy="global", nu=36,
                                          root_fillet_r=3.0))
    check(fm["centers"].shape == (5, 36, 3) and fm["gouge_free"]
          and np.all(np.isfinite(fm["centers"])),
          "pipeline fillet machining is gouge-free and finite",
          f"(min wall {fm['min_wall_dist_mm']:.2f} vs r_ball {fm['r_ball']:.2f})")

    # --- pipeline integration: trimming shortens the machined rulings ---
    base = compute(Params(strategy="global"))
    trimmed = compute(Params(strategy="global", root_fillet_r=3.0))
    h0 = np.mean(np.linalg.norm(base["b"] - base["a"], axis=1))
    h1 = np.mean(np.linalg.norm(trimmed["b"] - trimmed["a"], axis=1))
    check(h1 < h0 and np.all(np.isfinite(trimmed["dev"])),
          "root-fillet trim shortens rulings & still optimises", f"({h0:.1f}->{h1:.1f})")

    # --- OCC end-to-end: blisk with main + splitter + a fillet face ---
    occ_end_to_end()

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nFEATURE TESTS PASSED")


def occ_end_to_end():
    try:
        from OCP.gp import gp_Pnt, gp_Ax3, gp_Dir
        from OCP.TColgp import TColgp_Array1OfPnt
        from OCP.GeomAPI import GeomAPI_PointsToBSpline
        from OCP.GeomFill import GeomFill_BSplineCurves, GeomFill_CoonsStyle
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
        from OCP.Geom import Geom_CylindricalSurface
        from OCP.TopoDS import TopoDS_Compound
        from OCP.BRep import BRep_Builder
        from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    except ImportError:
        print("  ..  OCC end-to-end skipped (cadquery-ocp not installed)")
        return

    def bspline(pts):
        arr = TColgp_Array1OfPnt(1, len(pts))
        for i, p in enumerate(pts):
            arr.SetValue(i + 1, gp_Pnt(*p))
        return GeomAPI_PointsToBSpline(arr).Curve()

    def flank(A, B):
        s = GeomFill_BSplineCurves(bspline(A), bspline(B), GeomFill_CoonsStyle).Surface()
        return BRepBuilderAPI_MakeFace(s, 1e-6).Face()

    nu = 24
    u = np.linspace(0, 1, nu)
    def curves(phase, u0=0.0):
        uu = u0 + (1 - u0) * u
        A = np.column_stack([30 * np.cos(0.6 * uu + phase),
                             30 * np.sin(0.6 * uu + phase), 20 * uu])
        B = np.column_stack([55 * np.cos(1.3 * uu + phase),
                             55 * np.sin(1.3 * uu + phase), 8 + 20 * uu])
        return A, B

    comp = TopoDS_Compound()
    bld = BRep_Builder(); bld.MakeCompound(comp)
    for ph in (0.0, 2 * np.pi / 5):                  # two MAIN blades
        bld.Add(comp, flank(*curves(ph)))
    bld.Add(comp, flank(*curves(np.pi / 5, u0=0.5)))  # one SPLITTER (short)
    # a fillet/blend face: a tight cylinder patch (radius 2.5), large area
    cyl = Geom_CylindricalSurface(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 2.5)
    fil_face = BRepBuilderAPI_MakeFace(cyl, 0.0, 2 * np.pi, 0.0, 60.0, 1e-6).Face()
    bld.Add(comp, fil_face)

    with tempfile.TemporaryDirectory() as d:
        step = os.path.join(d, "blisk.step")
        w = STEPControl_Writer(); w.Transfer(comp, STEPControl_AsIs); w.Write(step)
        blk = cadio.blades_from_cad(step, nu=26)

    rails, labels = blk["rails"], blk["labels"]
    check(len(rails) == 3, "fillet face excluded; 3 flank blades kept",
          f"(got {len(rails)})")
    check(labels.count("main") == 2 and labels.count("splitter") == 1,
          "blisk classified as 2 main + 1 splitter", f"({labels})")
    # every kept flank optimises
    r = compute(Params(strategy="global", rails=rails[0], nu=26))
    check(np.all(np.isfinite(r["dev"])), "recognised flank drives the optimiser")


if __name__ == "__main__":
    main()
