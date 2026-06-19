#!/usr/bin/env python3
"""Double-flank channel milling validation.

Exact case: two parallel planar walls a distance 2R apart -- a cylinder of
radius R centred in the channel is tangent to both, so both wall deviations
must be ~0. Also a curved channel must yield finite, bounded deviation.
"""
import sys

try:
    import numpy as np
    from bladecam import core
except ImportError as e:
    print(f"SKIP double-flank ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    R = 5.0
    nu = 30
    u = np.linspace(0, 1, nu)
    # left wall in plane y=0, right wall in plane y=2R; rulings along z
    aL = np.column_stack([40*u, 0*u, 0*u]);      bL = np.column_stack([40*u, 0*u, 12+0*u])
    aR = np.column_stack([40*u, 0*u + 2*R, 0*u]); bR = np.column_stack([40*u, 0*u + 2*R, 12+0*u])
    # mu=0 isolates the geometry: the cylinder is exactly tangent to both walls
    q0, al, devL, devR = core.optimize_double_flank(aL, bL, aR, bR, R, nv=31,
                                                    mu=0.0, nsweeps=4)
    check(devL.max() < 0.02 and devR.max() < 0.02,
          "planar channel: both walls mill to ~0",
          f"(L {devL.max()*1000:.1f} um, R {devR.max()*1000:.1f} um)")
    # tool axis should sit ~mid-channel (y ~ R)
    check(abs(np.mean(q0[:, 1]) - R) < 0.5, "axis centred in channel",
          f"(mean y {np.mean(q0[:,1]):.2f}, want {R})")

    # a gently curved channel: bounded deviation, both walls finite
    th = 0.4 * u
    aL = np.column_stack([40*u, 0*u, 0*u]); bL = np.column_stack([40*u, 0*u, 12+0*u])
    aR = np.column_stack([40*u, 2*R + 3*np.sin(th), 0*u])
    bR = np.column_stack([40*u, 2*R + 3*np.sin(th), 12 + 0*u])
    _, _, dL, dR = core.optimize_double_flank(aL, bL, aR, bR, R, nv=31, mu=0.0)
    check(np.all(np.isfinite(dL)) and np.all(np.isfinite(dR)),
          "curved channel: finite deviation",
          f"(maxL {dL.max()*1000:.0f} um, maxR {dR.max()*1000:.0f} um)")

    # pipeline-level channel helper runs end to end on the parametric blade
    from bladecam.pipeline import double_flank_channel, Params
    rr = double_flank_channel(Params(R=4.0, n_blades=14, mu=0.0))
    check(np.all(np.isfinite(rr["devL"])) and np.all(np.isfinite(rr["devR"])),
          "pipeline double-flank channel runs",
          f"(L {rr['devL'].max()*1000:.0f} um, R {rr['devR'].max()*1000:.0f} um)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nDOUBLE-FLANK TESTS PASSED")


if __name__ == "__main__":
    main()
