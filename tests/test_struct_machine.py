#!/usr/bin/env python3
"""Structural machine-model (kinematic-link collision) audit.

Validates the capsule-capsule clearance core against brute-force segment
sampling and closed-form geometric cases, checks that the structure is placed
in the part frame with the SAME rotation convention as the inverse kinematics,
verifies the swept (mid-motion) minimum is found, and exercises the full
pipeline integration (no false positives on the defaults; a real link
collision is caught)."""
import sys

try:
    import numpy as np
    from bladecam import core, machine
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP struct_machine ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def cap_clr(ca, cb):
    """Static clearance between two capsules via struct_clearance (nu=1)."""
    a = np.asarray(ca, float).reshape(1, 1, 7)
    b = np.asarray(cb, float).reshape(1, 1, 7)
    return float(core.struct_clearance(a, b, nscan=2)[0])


def brute_seg_dist(p1, q1, p2, q2, n=500):
    s = np.linspace(0.0, 1.0, n)
    A = np.asarray(p1) + s[:, None] * (np.asarray(q1) - np.asarray(p1))
    B = np.asarray(p2) + s[:, None] * (np.asarray(q2) - np.asarray(p2))
    return float(np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2).min())


def main():
    rng = np.random.default_rng(7)

    # 1) capsule core (radii 0) vs brute-force closest segment-segment distance
    worst = 0.0
    exact_le_brute = True
    for _ in range(300):
        p1, q1, p2, q2 = (rng.uniform(-3, 3, 3) for _ in range(4))
        ex = cap_clr([*p1, *q1, 0.0], [*p2, *q2, 0.0])   # = seg-seg distance
        bf = brute_seg_dist(p1, q1, p2, q2)
        worst = max(worst, abs(ex - bf))
        exact_le_brute &= ex <= bf + 1e-9               # exact min <= any sample
    check(worst < 2e-2 and exact_le_brute,
          "capsule core matches brute-force segment distance", f"(max err {worst:.1e})")

    # 2) closed-form geometric cases
    # parallel capsules offset by d perpendicular: clearance = d - ra - rb
    d, ra, rb = 5.0, 1.0, 0.5
    c = cap_clr([0, 0, 0, 4, 0, 0, ra], [0, d, 0, 4, d, 0, rb])
    check(abs(c - (d - ra - rb)) < 1e-9, "parallel-capsule clearance is exact",
          f"({c:.4f} vs {d-ra-rb:.4f})")
    # collinear, fully overlapping cores: distance 0 -> clearance = -(ra+rb)
    c = cap_clr([0, 0, 0, 4, 0, 0, ra], [1, 0, 0, 3, 0, 0, rb])
    check(abs(c - (-(ra + rb))) < 1e-9, "overlapping-core clearance is -(ra+rb)")
    # crossing capsules (cores intersect) -> negative
    c = cap_clr([-2, 0, 0, 2, 0, 0, 0.3], [0, -2, 0.0, 0, 2, 0.0, 0.3])
    check(c < 0, "crossing capsules report penetration", f"({c:.3f})")
    # point-capsule (p0==p1) vs a segment: distance to the segment - radii
    c = cap_clr([0, 0, 3, 0, 0, 3, 0.0], [-5, 0, 0, 5, 0, 0, 0.0])
    check(abs(c - 3.0) < 1e-9, "point-vs-segment degenerate case is exact")

    # 3) rotation convention matches kinematics: world->part = Rz(C)Rx(A), and
    #    the spindle axis (0,0,1) mapped that way must equal the IK tool axis.
    conv_ok = True
    for _ in range(50):
        O = rng.uniform(-1, 1, 3); O /= np.linalg.norm(O)
        if O[2] < 0:                                    # keep A in [0, pi]
            O = -O
        m = core.ik_path(np.zeros((1, 3)), O[None, :], np.zeros(3), kind=0)[0]
        A, C = m[3], m[4]
        O_back = machine._rotz(C) @ machine._rotx(A) @ np.array([0.0, 0.0, 1.0])
        conv_ok &= np.allclose(O_back, O, atol=1e-9)
    check(conv_ok, "machine._rotx/_rotz match the IK world->part convention")

    # 4) link placement: cradle moves with C only (A-invariant); column with both
    mc = machine.Machine(cradle_span=100.0, cradle_dia=40.0, cradle_drop=80.0,
                         column_dia=40.0, column_offset=150.0, column_height=300.0)
    piv = np.array([0.0, 0.0, -50.0])
    base = np.array([[0, 0, 0, 0.0, 0.0]])               # A=C=0
    pureA = np.array([[0, 0, 0, 0.6, 0.0]])              # tilt only
    pureC = np.array([[0, 0, 0, 0.0, 0.7]])              # rotate only
    s0 = machine.structure_capsules(mc, base, piv, mount_z=-20.0)[0]
    sA = machine.structure_capsules(mc, pureA, piv, mount_z=-20.0)[0]
    sC = machine.structure_capsules(mc, pureC, piv, mount_z=-20.0)[0]
    ncr = 3                                              # 2 posts + 1 beam
    check(np.allclose(s0[:ncr], sA[:ncr]),
          "cradle is A-invariant in the part frame (tilts with the part)")
    check(not np.allclose(s0[:ncr], sC[:ncr]),
          "cradle rotates with C in the part frame")
    check(not np.allclose(s0[ncr:], sA[ncr:]),
          "column (base-fixed) moves under a pure A tilt")
    # head-head: structure is static in the part frame
    mh = machine.Machine(kind=1, cradle_span=100.0, cradle_dia=40.0)
    h0 = machine.structure_capsules(mh, base, piv, mount_z=-20.0)[0]
    hA = machine.structure_capsules(mh, pureA, piv, mount_z=-20.0)[0]
    check(np.allclose(h0, hA), "head-head structure is static in the part frame")

    # 5) swept (mid-motion) minimum: a tool capsule passing THROUGH a fixed
    #    structure capsule at the segment midpoint -- endpoints clear, middle hits.
    tool = np.empty((2, 1, 7)); struct = np.empty((2, 1, 7))
    tool[0, 0] = [-5, 0, 0, -5, 0, 2, 0.3]
    tool[1, 0] = [5, 0, 0, 5, 0, 2, 0.3]
    struct[0, 0] = struct[1, 0] = [0, 0, 0, 0, 0, 2, 0.3]
    clr = core.struct_clearance(tool, struct, nscan=16)
    end_static = cap_clr(list(tool[1, 0]), list(struct[1, 0]))
    check(clr[0] < 0 and end_static > 0,
          "swept check catches a mid-motion collision endpoints miss",
          f"(swept {clr[0]:.2f}, endpoint {end_static:.2f})")

    # 6) pipeline integration: defaults clear; link_clearance reported & finite
    for nm in machine.DEFAULT_MACHINES:
        r = compute(Params(strategy="global", machine=machine.get_machine(nm)))
        lk = r["link_clearance"]
        check(np.isfinite(lk) or lk == float("inf"), f"'{nm}' reports link_clearance")
        check(lk > 0.0, f"'{nm}' has no false structural-link collision", f"({lk:.1f})")

    # 7) a real structural-link collision is caught: a fat column straight
    #    through the work zone must drive collision_free False and link_clr < 0.
    crash = machine.Machine(name="bad column", cradle_span=0.0,
                            column_dia=140.0, column_offset=0.0, column_height=200.0)
    rc = compute(Params(strategy="global", machine=crash))
    check(rc["link_clearance"] < 0.0 and not rc["collision_free"],
          "structural-link collision is caught and fails the collision gate",
          f"(link {rc['link_clearance']:.1f})")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nSTRUCT-MACHINE TESTS PASSED")


if __name__ == "__main__":
    main()
