#!/usr/bin/env python3
"""Alternate machine kinematics through the pipeline (table-table vs head-head)."""
import sys

try:
    import numpy as np
    from bladecam.pipeline import compute, Params
    from bladecam.process import MachineLimits
except ImportError as e:
    print(f"SKIP kinematics ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    tt = compute(Params(strategy="global", machine=MachineLimits(kind=0)))
    hh = compute(Params(strategy="global", machine=MachineLimits(kind=1)))

    check(np.all(np.isfinite(hh["machine_path"])), "head-head IK is finite")
    # same tool orientation -> identical rotary (A,C) columns
    check(np.allclose(tt["machine_path"][:, 3:], hh["machine_path"][:, 3:]),
          "rotary axes identical (same tool orientation)")
    # different chain -> different linear (X,Y,Z) motion
    check(not np.allclose(tt["machine_path"][:, :3], hh["machine_path"][:, :3]),
          "linear axes differ between kinematics")
    # positioning (deviation) is kinematics-independent
    check(abs(tt["dev"].max() - hh["dev"].max()) < 1e-9,
          "deviation independent of machine kinematics")
    # head-head linear axes equal the contact point (part fixed)
    check(np.allclose(hh["machine_path"][:, :3], hh["contact"]),
          "head-head linear axes = contact point")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nKINEMATICS TESTS PASSED")


if __name__ == "__main__":
    main()
