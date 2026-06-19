#!/usr/bin/env python3
"""Stacked flank passes + roughing estimate (Tier 1 process gaps)."""
import sys

try:
    import numpy as np
    from bladecam.pipeline import (compute, stacked_flank_passes,
                                   roughing_time_estimate, Params)
    from bladecam.process import ProcessParams
except ImportError as e:
    print(f"SKIP stacked-passes ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    # short flute forces several stacked passes
    p = Params(strategy="global", z_span=40.0,
               process=ProcessParams(flute_len=12.0))
    st = stacked_flank_passes(p)
    check(st["n_passes"] >= 3, "tall blade is split into stacked passes",
          f"(n={st['n_passes']}, height {st['blade_height']:.0f} mm)")

    # stacking is for fitting the flute on tall blades; deviation stays bounded
    # and total cycle grows with the number of passes
    single = compute(p)
    check(st["dev_max"] < 0.1,
          "stacked per-band deviation bounded",
          f"({st['dev_max']*1000:.1f} um over {st['n_passes']} passes)")
    check(st["cycle_total_s"] > single["cycle_time_s"],
          "more passes -> more total cycle time",
          f"({st['cycle_total_s']:.1f} s vs single {single['cycle_time_s']:.1f} s)")
    check(len(st["passes"]) == st["n_passes"], "pass list length matches n_passes")

    # roughing estimate is positive and finite with a sane channel gap
    rg = roughing_time_estimate(p)
    check(rg["rough_time_s"] > 0 and np.isfinite(rg["rough_time_s"]),
          "roughing time estimate finite/positive",
          f"({rg['rough_time_s']:.0f} s, gap {rg['channel_gap_mm']:.1f} mm)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nSTACKED-PASSES TESTS PASSED")


if __name__ == "__main__":
    main()
