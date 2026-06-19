#!/usr/bin/env python3
"""Headless end-to-end demo of the BladeCAM core.

Generates a twisted parametric blade, computes the distribution-parameter
(machinability) map, then compares cutter-positioning strategies:
two-point vs. min-max (Phase 2) vs. globally smoothed. Reports peak/mean
flank deviation and an orientation-jerk proxy for rotary-axis effort.
Writes a per-ruling CSV. No GUI required.
"""
from __future__ import annotations

import numpy as np

from bladecam import core, blade, optimize


def main(R: float = 6.0, nv: int = 41):
    a, b = blade.make_blade()
    ap, bp = blade.rail_tangents(a, b)
    nu = a.shape[0]

    delta, vstar, strict = core.distribution(a, b)
    finite = delta[np.isfinite(delta)]

    res = optimize.optimize_blade(a, b, ap, bp, R, nv=nv)

    print(f"blade stations           : {nu}")
    print(f"cutter radius R          : {R:.2f} mm")
    print(f"|delta| min (max twist)  : {np.min(np.abs(finite)):.3f} mm")
    print()
    print(f"{'strategy':10s} {'dev_max(um)':>12s} {'dev_mean(um)':>13s} "
          f"{'orient_jerk':>12s}")
    for name in ("two_point", "minmax", "smoothed"):
        dev = res[name]["dev"]
        jk = optimize.orientation_jerk(res[name]["alpha"])
        print(f"{name:10s} {dev.max()*1000:12.1f} {dev.mean()*1000:13.1f} "
              f"{jk:12.4f}")

    imp = (1.0 - res["minmax"]["dev"].max() / res["two_point"]["dev"].max())
    print(f"\nmin-max reduces peak deviation by {imp*100:.1f}% vs two-point")

    out = "bladecam_result.csv"
    np.savetxt(out,
               np.column_stack([np.arange(nu), delta,
                                res["two_point"]["dev"],
                                res["minmax"]["dev"],
                                res["smoothed"]["dev"]]),
               delimiter=",",
               header="u_index,delta,dev_two_point,dev_minmax,dev_smoothed",
               comments="")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
