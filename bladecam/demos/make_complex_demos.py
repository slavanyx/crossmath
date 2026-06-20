#!/usr/bin/env python3
"""Super-complex demo parts: realistic centrifugal / mixed-flow / blisk blades,
each run through the WHOLE pipeline and rendered as a 5-panel workflow montage
(geometry -> positioning -> kinematics -> feed -> verification) so every step is
visualised. Writes for each part:

  cout/<name>_rails.csv   hub/shroud rails (load in the GUI)
  cout/<name>.step        a STEP flank face
  cout/<name>_workflow.png a 5-stage montage of the CAM flow
  cout/SUMMARY.md         the gallery + headline numbers + debug status

Run:  PYTHONPATH=python BLADECAM_LIB=build/core/libbladecam.so \\
          python3 demos/make_complex_demos.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "python"))

from bladecam import blade, cadio, workflow                # noqa: E402
from bladecam.pipeline import (compute, Params, rough_channel,   # noqa: E402
                               rest_machining, fillet_machining,
                               stacked_flank_passes)
from bladecam.process import ProcessParams                 # noqa: E402

OUT = os.path.join(HERE, "cout")
os.makedirs(OUT, exist_ok=True)


# (name, blurb, complex-blade kwargs, ProcessParams kwargs, Params extra)
DEMOS = [
    ("01_mixed_flow",
     "Mixed-flow impeller: the flank sweeps from axial to radial (the radius "
     "bulges mid-span) with real twist — and a cylinder still machines it well.",
     dict(rh0=30, rh1=46, rs0=42, rs1=58, z_span=22, z_offset=6, wrap=0.55,
          twist=0.5, radial_curve=0.5),
     dict(tool_dia=8, holder_dia=14, flute_len=38, holder_len=26),
     dict(n_blades=7, R=4.0, swept_weight=0.6)),
    ("02_backswept",
     "Backswept centrifugal blade: the trailing edge leans back (advance ∝ u²) "
     "with an S-warp — a strongly 3-D flank.",
     dict(rh0=34, rh1=47, rs0=44, rs1=57, z_span=20, z_offset=6, wrap=0.55,
          twist=0.45, backsweep=0.2, warp=0.1),
     dict(tool_dia=8, holder_dia=14, flute_len=38, holder_len=26),
     dict(n_blades=7, R=4.0, swept_weight=0.6)),
    ("03_high_twist",
     "A strongly twisted blisk blade — high non-developability; the swept "
     "penalty tilts the axes to keep the envelope error in check.",
     dict(rh0=34, rh1=48, rs0=46, rs1=60, z_span=20, z_offset=6, wrap=0.45,
          twist=0.65),
     dict(tool_dia=7, holder_dia=11, flute_len=40, holder_len=24),
     dict(n_blades=9, R=3.5, swept_weight=0.6)),
    ("04_s_warp_turbine",
     "An S-warped turbine flank (inflected camber): a cylinder fundamentally "
     "cannot fit it — the render shows WHERE the envelope overcuts (the honest "
     "flank-milling limit; use a barrel/point-mill here).",
     dict(rh0=36, rh1=52, rs0=48, rs1=64, z_span=20, z_offset=6, wrap=0.55,
          twist=0.4, warp=0.45),
     dict(tool_dia=8, holder_dia=14, flute_len=38, holder_len=26),
     dict(n_blades=9, R=4.0, swept_weight=0.6)),
    ("05_tall_leaned",
     "A tall, leaned blade (z-bowed shroud) — machined as stacked flank passes "
     "(each pass a thinner sub-band of the ruling).",
     dict(rh0=34, rh1=50, rs0=44, rs1=58, z_span=30, z_offset=8, wrap=0.5,
          twist=0.45, lean=0.3),
     dict(tool_dia=7, holder_dia=11, flute_len=32, holder_len=22),
     dict(n_blades=7, R=3.5, swept_weight=0.6)),
]


def make_step(path, A, B):
    try:
        from OCP.gp import gp_Pnt
        from OCP.TColgp import TColgp_Array1OfPnt
        from OCP.GeomAPI import GeomAPI_PointsToBSpline
        from OCP.GeomFill import GeomFill_BSplineCurves, GeomFill_CoonsStyle
        from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
        from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    except ImportError:
        return False

    def bspline(pts):
        arr = TColgp_Array1OfPnt(1, len(pts))
        for i, q in enumerate(pts):
            arr.SetValue(i + 1, gp_Pnt(*[float(x) for x in q]))
        return GeomAPI_PointsToBSpline(arr).Curve()
    s = GeomFill_BSplineCurves(bspline(A), bspline(B), GeomFill_CoonsStyle).Surface()
    f = BRepBuilderAPI_MakeFace(s, 1e-6).Face()
    w = STEPControl_Writer(); w.Transfer(f, STEPControl_AsIs); w.Write(path)
    return True


def _draw_scene(pl, scene):
    """Translate a workflow stage scene (renderer-agnostic dicts) into PyVista."""
    import pyvista as pv
    for m in scene["meshes"]:
        t = m["type"]
        if t == "surface":
            pts = np.asarray(m["points"]); nu, nv, _ = pts.shape
            g = pv.StructuredGrid()
            g.points = np.ascontiguousarray(pts.reshape(-1, 3))
            g.dimensions = (nv, nu, 1)
            if m.get("scalar") is not None:
                g["s"] = np.asarray(m["scalar"]).reshape(-1)
                pl.add_mesh(g, scalars="s", cmap=m.get("cmap", "viridis"),
                            opacity=m.get("opacity", 1.0), show_scalar_bar=False)
            else:
                pl.add_mesh(g, color=m.get("color", "lightgray"),
                            opacity=m.get("opacity", 1.0))
        elif t == "polyline":
            pl.add_mesh(pv.lines_from_points(np.ascontiguousarray(m["points"])),
                        color=m.get("color", "black"), line_width=m.get("width", 3))
        elif t == "lines":
            for p0, p1 in m["segments"]:
                pl.add_mesh(pv.Line(p0, p1), color=m.get("color", "black"),
                            line_width=m.get("width", 2))
        elif t == "tube":
            line = pv.lines_from_points(np.ascontiguousarray(m["points"]))
            line["s"] = np.asarray(m["scalar"]).reshape(-1)
            pl.add_mesh(line.tube(radius=m.get("radius", 0.5)), scalars="s",
                        cmap=m.get("cmap", "turbo"), show_scalar_bar=False)


def render_workflow(path, r, title, R):
    import pyvista as pv
    pv.OFF_SCREEN = True
    scenes = workflow.all_scenes(r, R=R)
    pl = pv.Plotter(off_screen=True, shape=(2, 3), window_size=(1700, 1050),
                    border=True)
    for idx, scene in enumerate(scenes):
        pl.subplot(idx // 3, idx % 3)
        _draw_scene(pl, scene)
        head = scene["title"]
        metric = scene["metrics"][0] if scene["metrics"] else ("", "")
        pl.add_text(f"{head}\n{metric[0]}: {metric[1]}", font_size=8,
                    position="upper_left")
        pl.view_isometric()
    # the 6th panel: a title / summary card
    pl.subplot(1, 2)
    coll = "collision-free" if r["collision_free"] else "COLLISION"
    pl.add_text(
        f"{title}\n\n"
        f"machined-surface error: {r['swept_overcut']*1000:.0f} µm\n"
        f"cycle time: {r['cycle_time_s']:.1f} s\n"
        f"{coll} · {'reachable' if r['reachable'] else 'UNREACHABLE'}\n"
        f"cut force: {r['cut_force_peak_N']:.0f} N",
        font_size=11, position="upper_left")
    pl.screenshot(path)
    pl.close()


def main():
    rows = []
    for name, blurb, bk, tk, ek in DEMOS:
        a, b = blade.make_complex_blade(nu=80, **bk)
        cadio.write_rails_csv(os.path.join(OUT, f"{name}_rails.csv"), a, b)
        step_ok = make_step(os.path.join(OUT, f"{name}.step"), a, b)
        pr = ProcessParams(**tk)
        p = Params(strategy="global", rails=(a, b), process=pr, **ek)
        r = compute(p)
        # exercise the rest of the operations as a debug pass
        ops = {}
        try:
            ops["rough_len_mm"] = rough_channel(p)["total_len_mm"]
            ops["fillet_gouge_free"] = fillet_machining(p)["gouge_free"]
            ops["rest_fraction"] = rest_machining(p, nx=32, ny=32)["rest_fraction"]
            ops["n_stacked"] = stacked_flank_passes(p)["n_passes"]
        except Exception as e:
            ops["op_error"] = f"{type(e).__name__}: {e}"
        R = pr.tool_dia / 2.0
        try:
            render_workflow(os.path.join(OUT, f"{name}_workflow.png"), r, name, R)
            png = f"{name}_workflow.png"
        except Exception as e:
            png = f"(render failed: {type(e).__name__}: {e})"
        finite = bool(np.all(np.isfinite(r["q0"])) and np.all(np.isfinite(r["dev"]))
                      and np.all(np.isfinite(r["swept_field"])))
        rows.append((name, blurb, r, ops, png, finite))
        L = np.linalg.norm(b - a, axis=1)
        print(f"{name:18s} ruling={L.min():.0f}-{L.max():.0f}mm "
              f"swept={r['swept_overcut']*1000:6.0f}µm cf={r['collision_free']} "
              f"reach={r['reachable']} finite={finite} ops={ops}")

    with open(os.path.join(OUT, "SUMMARY.md"), "w") as fh:
        fh.write("# BladeCAM super-complex demo gallery\n\n")
        fh.write("Realistic centrifugal / mixed-flow / blisk blades, each shown "
                 "as a 5-stage **workflow montage** (geometry → positioning → "
                 "kinematics → feed → verification).\n\n")
        for (name, blurb, r, ops, png, finite) in rows:
            cf = "✅" if r["collision_free"] else "⚠️ collision"
            fh.write(f"## {name}\n\n{blurb}\n\n")
            fh.write(f"- machined-surface error **{r['swept_overcut']*1000:.0f} µm**"
                     f" · cycle {r['cycle_time_s']:.1f} s · {cf}"
                     f" · {'reachable' if r['reachable'] else 'unreachable'}"
                     f" · finite={finite}\n")
            fh.write(f"- operations: {ops}\n\n")
            fh.write(f"![{name}]({png})\n\n")
    print(f"\nwrote {len(rows)} complex demos + SUMMARY.md to {OUT}")


if __name__ == "__main__":
    main()
