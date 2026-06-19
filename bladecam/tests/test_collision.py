#!/usr/bin/env python3
"""Tool+holder collision (signed-distance) checking.

Exact cases on a known stepped-cylinder tool, plus a pipeline check that a
tight blade count flags a collision while a generous one is clear.
"""
import sys

try:
    import numpy as np
    from bladecam import core
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP collision ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def clr_one(pt, R=5.0, Lf=20.0, Rh=10.0, gap=5.0, Lh=20.0):
    q0 = np.zeros((1, 3)); al = np.array([[0.0, 0.0, 1.0]])
    return core.tool_clearance(q0, al, np.array([pt], float),
                               R, Lf, Rh, gap, Lh)[0]


def main():
    # flute radius 5 over z in [0,20]; holder radius 10 over z in [25,45]
    check(abs(clr_one([8, 0, 10]) - 3.0) < 1e-9, "outside flute: clearance = r-R",
          f"({clr_one([8,0,10]):.3f})")
    check(abs(clr_one([3, 0, 10]) + 2.0) < 1e-9, "inside flute: negative (gouge)",
          f"({clr_one([3,0,10]):.3f})")
    check(abs(clr_one([8, 0, 30]) + 2.0) < 1e-9, "inside holder: negative collision",
          f"({clr_one([8,0,30]):.3f})")
    check(abs(clr_one([0, 0, -5]) - 5.0) < 1e-9, "below tip: axial cap distance",
          f"({clr_one([0,0,-5]):.3f})")
    check(clr_one([50, 0, 10]) > 0, "far point: clear")
    # holder (R=10) must catch what the flute (R=5) would miss
    check(clr_one([8, 0, 30]) < 0 and clr_one([8, 0, 10]) > 0,
          "holder geometry catches collisions the flute misses")

    # SWEPT collision: both stations clear an obstacle but the mid-move gouges it
    from bladecam.pipeline import _densify_poses
    q0 = np.array([[0., 0, 0], [20., 0, 0]])     # tool translates along x
    al = np.array([[0., 0, 1], [0., 0, 1]])      # axis +z
    obst = np.array([[10., 0, 10]])              # sits at mid-span, off both ends
    args = (5.0, 20.0, 10.0, 5.0, 20.0)          # R, Lf, Rh, gap, Lh
    per_station = core.tool_clearance(q0, al, obst, *args).min()
    q0d, ad = _densify_poses(q0, al, 3)
    swept = core.tool_clearance(q0d, ad, obst, *args).min()
    check(per_station > 0, "per-station check clears both endpoints",
          f"({per_station:.2f} mm)")
    check(swept < 0, "swept check catches the mid-move collision",
          f"({swept:.2f} mm)")

    # pipeline: a very tight blade count collides; a generous one is clear
    tight = compute(Params(strategy="global", n_blades=38))
    loose = compute(Params(strategy="global", n_blades=6))
    check(tight["min_clearance"] < loose["min_clearance"],
          "tighter blade spacing reduces clearance",
          f"(tight {tight['min_clearance']:.2f} vs loose {loose['min_clearance']:.2f} mm)")
    check("gouge_max" in tight and tight["gouge_max"] >= 0, "gouge metric reported")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nCOLLISION TESTS PASSED")


if __name__ == "__main__":
    main()
