#!/usr/bin/env python3
"""Generate a gallery of demo impeller blades spanning a range of complexity,
run the full BladeCAM pipeline on each, and write for every part:

  out/<name>_rails.csv   hub/shroud rails (load via File > Import rails CSV)
  out/<name>.step        a STEP flank face (load via File > Load blade from STEP)
  out/<name>.png         a 3D render: flank coloured by the swept-envelope error
                         (µm), the optimised tool axes, and the contact path
  out/SUMMARY.md         a table of the headline numbers for every part

Run:  PYTHONPATH=python BLADECAM_LIB=build/core/libbladecam.so \\
          python3 demos/make_demos.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "python"))

from bladecam import blade, cadio                       # noqa: E402
from bladecam.pipeline import compute, Params, stacked_flank_passes  # noqa: E402
from bladecam.process import ProcessParams              # noqa: E402

OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)


# (name, blurb, Params kwargs, ProcessParams kwargs, extra)
DEMOS = [
    ("01_developable",
     "DEVELOPABLE flank (zero twist): a cylinder machines it essentially "
     "exactly — the ideal reference case (~10 µm).",
     dict(r_hub=30, r_shroud=55, z_span=20, z_offset=6, wrap=0.05, twist=0.0,
          R=4.0),
     dict(tool_dia=8, holder_dia=14), {}),
    ("02_mild_impeller",
     "A gently twisted, realistic impeller blade — modest envelope error.",
     dict(r_hub=30, r_shroud=55, z_span=20, z_offset=8, wrap=0.20, twist=0.15,
          R=4.0),
     dict(tool_dia=8, holder_dia=14), {}),
    ("03_twisted_cylinder",
     "A properly TWISTED blade with a plain cylinder: the swept-envelope error "
     "grows (≈ R·ℓ²/δ²) — the real flank-milling limit. (Before.)",
     dict(r_hub=30, r_shroud=55, z_span=22, z_offset=8, wrap=0.45, twist=0.45,
          R=4.0, swept_weight=0.0),
     dict(tool_dia=8, holder_dia=14), {}),
    ("04_twisted_optimised",
     "The SAME twisted blade, with the swept-overcut penalty on: the optimiser "
     "tilts the axes to minimise the real envelope error (≈20× lower). (After.)",
     dict(r_hub=30, r_shroud=55, z_span=22, z_offset=8, wrap=0.45, twist=0.45,
          R=4.0, swept_weight=0.6),
     dict(tool_dia=8, holder_dia=14), {}),
    ("05_tall_stacked",
     "A tall blade — taller than the usable flute, so it is split into stacked "
     "flank passes (each a thinner sub-band).",
     dict(r_hub=28, r_shroud=58, z_span=46, z_offset=10, wrap=0.35, twist=0.30,
          R=4.0),
     dict(tool_dia=8, holder_dia=14, flute_len=30), dict(stacked=True)),
    ("06_blisk",
     "A full blisk: several blades around the hub (load any one rail set, or the "
     "compound STEP, into the GUI).",
     dict(r_hub=26, r_shroud=48, z_span=20, z_offset=8, wrap=0.35, twist=0.30,
          n_blades=9, R=4.0),
     dict(tool_dia=8, holder_dia=14), dict(blisk=9)),
]


def make_step(path, A, B):
    """Write a STEP flank face interpolating the two rails (needs OpenCASCADE)."""
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
        for i, p in enumerate(pts):
            arr.SetValue(i + 1, gp_Pnt(float(p[0]), float(p[1]), float(p[2])))
        return GeomAPI_PointsToBSpline(arr).Curve()

    s = GeomFill_BSplineCurves(bspline(A), bspline(B), GeomFill_CoonsStyle).Surface()
    face = BRepBuilderAPI_MakeFace(s, 1e-6).Face()
    w = STEPControl_Writer(); w.Transfer(face, STEPControl_AsIs); w.Write(path)
    return True


def _rotz(ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def render(path, r, title, R, flute, n_blades=1):
    import pyvista as pv
    pv.OFF_SCREEN = True
    surf = r["surf"]
    nu, nv, _ = surf.shape
    p = pv.Plotter(off_screen=True, window_size=(1200, 850))
    # the machined blade, coloured by the real swept-envelope error
    g = pv.StructuredGrid()
    g.points = np.ascontiguousarray(surf.reshape(-1, 3))
    g.dimensions = (nv, nu, 1)
    g["overcut_um"] = (np.maximum(0.0, -r["swept_field"]) * 1000.0).reshape(-1)
    p.add_mesh(g, scalars="overcut_um", cmap="turbo",
               scalar_bar_args={"title": "swept overcut (µm)"}, smooth_shading=True)
    # the rest of the blisk (faint), rotated about the spin axis
    for k in range(1, n_blades):
        Rk = _rotz(k * 2.0 * np.pi / n_blades)
        gk = pv.StructuredGrid()
        gk.points = np.ascontiguousarray((surf.reshape(-1, 3) @ Rk.T))
        gk.dimensions = (nv, nu, 1)
        p.add_mesh(gk, color="lightsteelblue", opacity=0.45, smooth_shading=True)
    q0, al = r["q0"], r["alpha"]
    for i in range(0, nu, max(1, nu // 18)):
        a = al[i] / np.linalg.norm(al[i])
        p.add_mesh(pv.Line(q0[i] - a * 0.3 * R, q0[i] + a * flute),
                   color="black", line_width=2)
    p.add_mesh(pv.lines_from_points(np.ascontiguousarray(r["contact"])),
               color="red", line_width=4)
    p.add_text(title, font_size=11, position="upper_left")
    p.view_isometric()
    p.screenshot(path)
    p.close()


def main():
    rows = []
    for name, blurb, pk, tk, extra in DEMOS:
        pr = ProcessParams(**tk)
        p = Params(strategy="global", process=pr,
                   **{k: v for k, v in pk.items()})
        a, b = blade.make_blade(p.nu, p.r_hub, p.r_shroud, p.z_span,
                                p.z_offset, p.wrap, p.twist)
        cadio.write_rails_csv(os.path.join(OUT, f"{name}_rails.csv"), a, b)
        step_ok = make_step(os.path.join(OUT, f"{name}.step"), a, b)
        r = compute(p)
        R = pr.tool_dia / 2.0
        flute = min(pr.flute_len, float(np.linalg.norm(b - a, axis=1).max()))
        nb = int(extra.get("blisk", 1))
        try:
            render(os.path.join(OUT, f"{name}.png"), r, name, R, flute, n_blades=nb)
            png = f"{name}.png"
        except Exception as e:
            png = f"(render failed: {type(e).__name__})"
        extra_s = ""
        if extra.get("stacked"):
            st = stacked_flank_passes(p)
            extra_s = f" · {st['n_passes']} stacked passes"
        rows.append((name, blurb, r["swept_overcut"] * 1000.0,
                     r["dev"].max() * 1000.0, r["cycle_time_s"],
                     r["collision_free"], r["reachable"], png, step_ok, extra_s))
        print(f"{name:18s} swept={r['swept_overcut']*1000:7.1f}µm "
              f"cf={r['collision_free']} reach={r['reachable']}{extra_s}")

    with open(os.path.join(OUT, "SUMMARY.md"), "w") as fh:
        fh.write("# BladeCAM demo gallery\n\n")
        fh.write("Generated by `demos/make_demos.py`. Each `*_rails.csv` / "
                 "`*.step` loads straight into the GUI.\n\n")
        fh.write("| part | swept err (µm) | per-ruling dev (µm) | cycle (s) | "
                 "collision-free | reachable | render |\n")
        fh.write("|---|--:|--:|--:|:--:|:--:|---|\n")
        for (name, blurb, sw, dv, cyc, cf, re, png, step_ok, ex) in rows:
            fh.write(f"| **{name}**{ex} | {sw:.1f} | {dv:.1f} | {cyc:.2f} | "
                     f"{'✅' if cf else '❌'} | {'✅' if re else '❌'} | "
                     f"![{name}]({png}) |\n")
        fh.write("\n")
        for (name, blurb, *_rest) in rows:
            fh.write(f"- **{name}** — {blurb}\n")
    print(f"\nwrote {len(rows)} demos + SUMMARY.md to {OUT}")


if __name__ == "__main__":
    main()
