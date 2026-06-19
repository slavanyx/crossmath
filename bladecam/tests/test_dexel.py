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

    # removed volume vs analytic pi R^2 Lf
    vol = verify.removed_volume(q0, al, R, Lf, [-7, -7, 0], [7, 7, 20], n=160)
    exact = np.pi*R**2*20
    check(abs(vol - exact)/exact < 0.01, "removed volume ~ pi R^2 Lf (<1%)",
          f"({vol:.1f} vs {exact:.1f})")

    # a tilted pose still gives a sane positive volume (Cavalieri holds)
    alt = np.array([[0.3, 0, 1.0]]); alt = alt/np.linalg.norm(alt)
    volt = verify.removed_volume(q0, alt, R, Lf, [-12, -12, 0], [12, 12, 25], n=160)
    check(volt > 0.5*exact, "tilted pose removes a sane volume", f"({volt:.1f})")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nDEXEL TESTS PASSED")


if __name__ == "__main__":
    main()
