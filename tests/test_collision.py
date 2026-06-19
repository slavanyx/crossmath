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

    # CONTINUOUS swept-volume clearance: minimises over the whole motion, so it
    # must catch the same mid-move gouge AND be no less conservative than dense
    # pose sampling (continuous min <= sampled min).
    cont = core.swept_clearance(q0, al, obst, *args, nscan=12)
    check(cont[0] < 0, "continuous swept-volume catches mid-move gouge",
          f"({cont[0]:.2f} mm)")
    check(cont.min() <= swept + 1e-6,
          "continuous swept-volume no less conservative than sampling",
          f"(cont {cont.min():.3f} <= sampled {swept:.3f})")
    # last entry is the final static clearance (matches per-station there)
    stat_last = core.tool_clearance(q0[-1:], al[-1:], obst, *args)[0]
    check(abs(cont[-1] - stat_last) < 1e-6, "continuous last entry = static clearance")

    # HOLDER-only clearance vs the blade being machined (the flute is tangent to
    # that blade by design, so a full-tool check there is a false positive). The
    # fat holder (radius Rh=8 > flute R=5) hits a tall wall sitting at the flute
    # radius, but clears a wall set back beyond Rh.
    R, Lf, Rh, gap, Lh = 5.0, 20.0, 8.0, 2.0, 40.0
    base = Lf + gap
    qv = np.array([[0., 0, 0]]); up = np.array([[0., 0, 1.]])
    z = np.linspace(0, 60, 30)
    wall_at = lambda x: np.column_stack([np.full_like(z, x), np.zeros_like(z), z])
    h_near = core.holder_clearance(qv, up, wall_at(R), Rh, base, Lh)[0]   # within Rh
    h_far = core.holder_clearance(qv, up, wall_at(Rh + 3.0), Rh, base, Lh)[0]  # beyond Rh
    check(h_near < 0, "holder hits a wall inside its radius", f"({h_near:.2f})")
    check(h_far > 0, "holder clears a wall set back beyond its radius", f"({h_far:.2f})")
    # a steep lean that swings the holder over a tangent wall is detected
    th = np.radians(35.0); ax = np.array([[np.sin(th), 0, np.cos(th)]])
    overhang = np.array([[x, 0., 38.] for x in np.linspace(0, 30, 30)])  # wall top band
    check(core.holder_clearance(qv, ax, overhang, Rh, base, Lh).min() < 0,
          "steep lean swings the holder into an overhanging wall band")

    # pipeline: a very tight blade count collides; a generous one is clear
    tight = compute(Params(strategy="global", n_blades=38))
    loose = compute(Params(strategy="global", n_blades=6))
    check(tight["min_clearance"] < loose["min_clearance"],
          "tighter blade spacing reduces clearance",
          f"(tight {tight['min_clearance']:.2f} vs loose {loose['min_clearance']:.2f} mm)")
    check("gouge_max" in tight and tight["gouge_max"] >= 0, "gouge metric reported")
    check("holder_clearance" in tight, "pipeline reports holder clearance")
    check("assembly_clearance" in tight, "pipeline reports assembly clearance")

    # --- full tool-ASSEMBLY collision (flute+holder+spindle) + fixture plane ---
    q0a = np.array([[0., 0, 0], [20., 0, 0]]); ala = np.array([[0., 0, 1.], [0., 0, 1.]])
    segR = np.array([5., 10.]); segLo = np.array([0., 22.]); segHi = np.array([20., 62.])
    # a 2-segment assembly must reproduce the flute+holder swept clearance exactly
    obst = np.array([[10., 0, 10.]])
    a2 = core.assembly_clearance(q0a, ala, segR, segLo, segHi, obst).min()
    s2 = core.swept_clearance(q0a, ala, obst, 5., 20., 10., 2., 40.).min()
    check(abs(a2 - s2) < 1e-6, "assembly(2-seg) == flute+holder swept",
          f"({a2:.3f} vs {s2:.3f})")
    # a spindle segment catches a high obstacle the flute/holder miss
    s3R = np.array([5., 10., 15.]); s3Lo = np.array([0., 22., 64.]); s3Hi = np.array([20., 62., 104.])
    hi = np.array([[10., 0, 80.]])
    check(core.assembly_clearance(q0a, ala, segR, segLo, segHi, hi).min() > 0
          and core.assembly_clearance(q0a, ala, s3R, s3Lo, s3Hi, hi).min() < 0,
          "spindle segment catches a high obstacle flute+holder miss")
    # fixture half-space: the flute base (z=0) dips below a plane at z=2
    far = np.array([[100., 0, 100.]])
    cp = core.assembly_clearance(q0a, ala, segR, segLo, segHi, far,
                                 plane_pt=np.array([0., 0, 2.]),
                                 plane_n=np.array([0., 0, 1.])).min()
    check(abs(cp + 2.0) < 1e-6, "fixture plane catches the assembly base", f"({cp:.2f})")
    # tilted axis (nperp>0) on an EARLY segment dipping below the plane, with the
    # final station clear -- exercises the per-segment plane term AND the
    # segR*nperp tilt offset (a vertical-axis test masks both).
    q0t = np.array([[50., 0, 2.], [50., 0, 40.]])
    ax1 = np.array([0.6, 0, 0.8]); ax1 /= np.linalg.norm(ax1)
    alt = np.array([ax1, [0., 0, 1.]])
    far2 = np.array([[500., 0, 500.]])
    ct = core.assembly_clearance(q0t, alt, segR, segLo, segHi, far2,
                                 plane_pt=np.array([0., 0, 0.]),
                                 plane_n=np.array([0., 0, 1.])).min()
    check(ct < 0, "tilted per-segment assembly dips below the fixture plane",
          f"({ct:.2f})")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nCOLLISION TESTS PASSED")


if __name__ == "__main__":
    main()
