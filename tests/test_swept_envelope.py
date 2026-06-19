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

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nSWEPT-ENVELOPE TESTS PASSED")


if __name__ == "__main__":
    main()
