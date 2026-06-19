#!/usr/bin/env python3
"""Swept-envelope deviation (cross-station interference).

Found by external validation: the per-station deviation can read ~0 while the
swept tool actually overcuts the part by mm in twisted regions. These tests pin
down that the swept metric (a) reproduces a known exact case, (b) is never
smaller in magnitude than the per-station gouge, and (c) grows with blade twist.
"""
import sys

try:
    import numpy as np
    from bladecam import core
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP swept-envelope ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    # exact: one cylinder, axis +z at x=0, R=5; a point at radius 8 -> +3 mm,
    # at radius 3 -> -2 mm (gouge). Single station => swept == static.
    q0 = np.zeros((1, 3)); al = np.array([[0., 0, 1]]); L = np.array([20.0])
    pts = np.array([[8., 0, 10], [3., 0, 10]])
    g = core.swept_deviation(q0, al, L, 5.0, pts)
    check(abs(g[0] - 3.0) < 1e-9 and abs(g[1] + 2.0) < 1e-9,
          "swept exact single-cylinder distances", f"({g.round(2)})")

    # finite flute: a point beyond the flute end is NOT gouged by a far station
    pts2 = np.array([[0., 0, 100]])     # far above the 20 mm flute
    g2 = core.swept_deviation(q0, al, L, 5.0, pts2)
    check(g2[0] > 0, "finite flute does not gouge beyond its length")

    # pipeline: swept overcut >= per-station gouge, and grows with twist
    r_lo = compute(Params(strategy="global", twist=0.2))
    r_hi = compute(Params(strategy="global", twist=1.0))
    check(r_hi["swept_overcut"] >= r_hi["gouge_max"] - 1e-9,
          "swept overcut >= per-station gouge")
    check(r_hi["swept_overcut"] > r_lo["swept_overcut"],
          "swept overcut grows with twist",
          f"({r_lo['swept_overcut']*1000:.0f} -> {r_hi['swept_overcut']*1000:.0f} um)")

    # the global optimizer can trade per-ruling accuracy to reduce swept overcut:
    # with the swept penalty on, cross-station interference must drop (at some
    # cost to per-station deviation). Use a high-twist blade where the per-station
    # optimum genuinely overcuts neighbours.
    base = Params(strategy="global", twist=1.4)
    r_off = compute(base)
    r_on = compute(Params(strategy="global", twist=1.4, swept_weight=0.5))
    check(r_on["swept_overcut"] < r_off["swept_overcut"] - 1e-6,
          "swept penalty reduces swept overcut",
          f"({r_off['swept_overcut']*1000:.0f} -> {r_on['swept_overcut']*1000:.0f} um)")
    check(r_on["dev"].max() >= r_off["dev"].max(),
          "swept penalty trades per-ruling deviation")

    # swept_field (per-point machined-surface error map) is exposed for 3D
    # colouring and its overcut depth must equal the reported scalar overcut.
    r = compute(Params(strategy="global", twist=1.0))
    sf = r["swept_field"]
    check(sf.shape == r["surf"].shape[:2], "swept_field matches surface grid",
          f"({sf.shape} vs {r['surf'].shape[:2]})")
    oc = float(max(0.0, -sf.min()))
    check(abs(oc - r["swept_overcut"]) < 1e-9,
          "swept_field overcut depth == swept_overcut scalar")

    # true swept-envelope SURFACE: machined points displaced from the design by
    # exactly |swept deviation|, and lying on a cutter surface.
    env = r["envelope_surf"]
    check(env.shape == r["surf"].shape, "envelope surface matches design grid")
    check(np.all(np.isfinite(env)), "envelope surface is finite")
    design = r["surf"].reshape(-1, 3)
    g = core.swept_deviation(r["q0"], r["alpha"],
                             np.linalg.norm(r["b"] - r["a"], axis=1), 6.0, design)
    disp = np.linalg.norm(env.reshape(-1, 3) - design, axis=1)
    check(np.allclose(disp, np.abs(g), atol=1e-6),
          "machined displacement == |swept deviation|",
          f"(max diff {np.abs(disp - np.abs(g)).max():.2e})")

    # conical tool: the swept metric must use the LOCAL tool radius, not a
    # constant R. Points exactly on a cone surface must read ~0 overcut; the
    # cylinder formula (gamma omitted) wrongly reports up to lam*tan(gamma).
    Rc = 5.0; gam = np.radians(12.0)
    qc = np.array([[0., 0, 0]]); ac = np.array([[0., 0, 1.]]); Lfc = np.array([40.0])
    lam = np.array([0., 10., 20., 30.])
    on_cone = np.column_stack([Rc + lam*np.tan(gam), 0*lam, lam])
    gc = core.swept_deviation(qc, ac, Lfc, Rc, on_cone, gamma=gam)
    gcyl = core.swept_deviation(qc, ac, Lfc, Rc, on_cone)   # gamma=0 (cylinder)
    check(np.max(np.abs(gc)) < 1e-9, "conical swept ~0 on the cone surface",
          f"(max {np.max(np.abs(gc)):.2e})")
    check(np.max(np.abs(gcyl)) > 1.0, "cylinder formula mis-reads taper as overcut",
          f"({np.max(np.abs(gcyl)):.2f} mm)")
    envc = core.swept_surface(qc, ac, Lfc, Rc, on_cone, gamma=gam)
    check(np.max(np.linalg.norm(envc - on_cone, axis=1)) < 1e-9,
          "conical envelope surface lands on the cone")

    # barrel: points on a circle-segment tool read ~0 swept deviation, and the
    # swept envelope surface lands back on the barrel (exercises the barrel
    # branch of both tool_sdf and the tool_radius projection).
    Rb, lamc = 200.0, 15.0
    lamb = np.array([5., 10., 15., 20., 25.])
    on_barrel = np.column_stack([(Rc - Rb) + np.sqrt(Rb**2 - (lamb - lamc)**2),
                                 0*lamb, lamb])
    Lfb = np.array([30.0])
    gb = core.swept_deviation(qc, ac, Lfb, Rc, on_barrel, Rb=Rb, lamc=lamc)
    check(np.max(np.abs(gb)) < 1e-8, "barrel swept ~0 on the barrel surface",
          f"(max {np.max(np.abs(gb)):.2e})")
    envb = core.swept_surface(qc, ac, Lfb, Rc, on_barrel, Rb=Rb, lamc=lamc)
    check(np.max(np.linalg.norm(envb - on_barrel, axis=1)) < 1e-6,
          "barrel envelope surface lands on the barrel")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nSWEPT-ENVELOPE TESTS PASSED")


if __name__ == "__main__":
    main()
