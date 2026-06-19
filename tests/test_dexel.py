#!/usr/bin/env python3
"""Dexel material-removal simulation (gap-closer #1).

Verifies the ray/cylinder carve primitive against analytic chord lengths and
interval-union behaviour, and the volumetric removed-material measure against the
closed-form cylinder volume -- the commercial "verified MRR" check.
"""
import sys

try:
    import numpy as np
    from bladecam import core, verify
except ImportError as e:
    print(f"SKIP dexel ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    R = 5.0
    q0 = np.array([[0., 0, 0]]); al = np.array([[0., 0, 1.]]); Lf = np.array([20.0])

    # chord length: ray along +x at radial offset d crosses the cylinder
    for d in (0.0, 3.0, 4.9):
        rem, fc = core.dexel_carve(q0, al, R, Lf, np.array([[-20., d, 10.]]),
                                   np.array([[1., 0, 0]]), np.array([40.0]))
        chord = 2*np.sqrt(R**2 - d**2)
        check(abs(rem[0] - chord) < 1e-6, f"chord at offset {d}",
              f"({rem[0]:.3f} vs {chord:.3f})")
        # first cut enters at start(-20) + (20 - sqrt(R^2-d^2))
        check(abs(fc[0] - (20 - np.sqrt(R**2 - d**2))) < 1e-6, f"first-cut at {d}")

    # misses: outside radius, and above the flute cap
    rem, _ = core.dexel_carve(q0, al, R, Lf, np.array([[-20., 6., 10.]]),
                              np.array([[1., 0, 0]]), np.array([40.0]))
    check(rem[0] == 0.0, "ray outside radius removes nothing")
    rem, _ = core.dexel_carve(q0, al, R, Lf, np.array([[-20., 0., 25.]]),
                              np.array([[1., 0, 0]]), np.array([40.0]))
    check(rem[0] == 0.0, "ray above the flute cap removes nothing")

    # overlapping poses: union, not double-count
    q2 = np.array([[0., 0, 0], [3., 0, 0]]); a2 = np.array([[0., 0, 1.], [0., 0, 1.]])
    rem, _ = core.dexel_carve(q2, a2, R, np.array([20., 20.]),
                              np.array([[-20., 0., 10.]]), np.array([[1., 0, 0]]),
                              np.array([60.0]))
    check(abs(rem[0] - 13.0) < 1e-9, "overlapping poses union (13, not 20)",
          f"({rem[0]:.3f})")

    # DISJOINT poses: two cylinders far apart along the ray, with a clear gap.
    # removed must be the SUM of the two chords (union of two separate intervals),
    # NOT a single merged span across the gap. Exercises the union accumulation
    # and the (no-)merge logic that overlapping cases never reach.
    q3 = np.array([[0., 0, 0], [30., 0, 0]])      # axes 30 mm apart along x
    a3 = np.array([[0., 0, 1.], [0., 0, 1.]])
    rem, fc = core.dexel_carve(q3, a3, R, np.array([20., 20.]),
                               np.array([[-20., 0., 10.]]), np.array([[1., 0, 0]]),
                               np.array([80.0]))
    check(abs(rem[0] - 20.0) < 1e-9, "disjoint poses sum two chords (10+10)",
          f"({rem[0]:.3f})")          # each chord = 2R = 10; gap not counted
    check(abs(fc[0] - 15.0) < 1e-9, "first cut at the nearer interval",
          f"({fc[0]:.3f})")            # start -20 + (20-5) = 15

    # removed volume vs analytic pi R^2 Lf
    vol = verify.removed_volume(q0, al, R, Lf, [-7, -7, 0], [7, 7, 20], n=160)
    exact = np.pi*R**2*20
    check(abs(vol - exact)/exact < 0.01, "removed volume ~ pi R^2 Lf (<1%)",
          f"({vol:.1f} vs {exact:.1f})")

    # a tilted pose still gives a sane positive volume (Cavalieri holds)
    alt = np.array([[0.3, 0, 1.0]]); alt = alt/np.linalg.norm(alt)
    volt = verify.removed_volume(q0, alt, R, Lf, [-12, -12, 0], [12, 12, 25], n=160)
    check(volt > 0.5*exact, "tilted pose removes a sane volume", f"({volt:.1f})")

    # progressive carve (interactive simulation engine): removing material with
    # the first k poses must be monotone non-decreasing in k and converge to the
    # full removed volume. Two cylinders along x at 0 and 30.
    qp = np.array([[0., 0, 0], [30., 0, 0]]); apv = np.array([[0., 0, 1.], [0., 0, 1.]])
    Lfp = np.array([20., 20.])
    box_lo, box_hi = [-8, -8, 0], [40, 8, 20]
    vols = [0.0] + [verify.removed_volume(qp[:k], apv[:k], R, Lfp[:k],
                                          box_lo, box_hi, n=120)
                    for k in (1, 2)]
    check(vols[1] <= vols[2] + 1e-6 and vols[0] <= vols[1],
          "progressive carve volume is monotone in k",
          f"({[round(v,1) for v in vols]})")
    full = verify.removed_volume(qp, apv, R, Lfp, box_lo, box_hi, n=120)
    check(abs(vols[-1] - full) < 1e-6, "progressive carve converges to full volume")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nDEXEL TESTS PASSED")


if __name__ == "__main__":
    main()
