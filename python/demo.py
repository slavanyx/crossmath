#!/usr/bin/env python3
"""Headless end-to-end demo of the full BladeCAM pipeline (Phases 1-4).

Geometry -> positioning (two-point/min-max/smoothed) -> 5-axis inverse
kinematics -> time-optimal feed (TOPP) -> cycle time, plus a neighbour-blade
collision check and a G-code dump. No GUI required.
"""
from __future__ import annotations

import numpy as np

from bladecam import pipeline, postproc
from bladecam.pipeline import Params


def main():
    print("=== positioning strategy comparison ===")
    print(f"{'strategy':10s} {'dev_max(um)':>12s} {'jerk':>8s} "
          f"{'cycle(s)':>9s} {'clear(mm)':>10s}")
    for strat in ("two_point", "minmax", "smoothed"):
        r = pipeline.compute(Params(strategy=strat))
        print(f"{strat:10s} {r['dev'].max()*1000:12.1f} "
              f"{r['orient_jerk']:8.3f} {r['cycle_time_s']:9.2f} "
              f"{r['min_clearance']:10.2f}")

    print("\n=== detail (min-max) ===")
    r = pipeline.compute(Params(strategy="minmax"))
    print(f"contact path length     : {r['path_len_mm']:.1f} mm")
    print(f"effective feed cap      : {r['feed_cap_mm_min']:.0f} mm/min")
    print(f"time-optimal cycle time : {r['cycle_time_s']:.2f} s")
    print(f"collision free          : {r['collision_free']} "
          f"(min clearance {r['min_clearance']:.2f} mm)")

    gcode = postproc.to_gcode(r["machine_path"], r["feed_cap_mm_min"])
    with open("bladecam.nc", "w") as f:
        f.write(gcode)
    print(f"wrote bladecam.nc ({gcode.count(chr(10))} lines)")


if __name__ == "__main__":
    main()
