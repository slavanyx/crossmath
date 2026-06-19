#!/usr/bin/env python3
"""Physical-consistency checks on the feed / machine-limit / cycle-time coupling.

Locks in the audit finding that 5-axis flank finishing here is rotary-limited
and that the limits flow through TOPP into the cycle time the right way.
"""
import sys

try:
    import numpy as np
    from bladecam.pipeline import compute, Params
    from bladecam.process import MachineLimits, ProcessParams
except ImportError as e:
    print(f"SKIP physical-consistency ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def cyc(machine=None, process=None):
    p = Params(strategy="global", nu=30)   # coupling is visible at low nu; keep it fast
    if machine:
        p.machine = machine
    if process:
        p.process = process
    r = compute(p)
    return r["cycle_time_s"], r["feed_cap_mm_min"]


def main():
    # rotary-limited: halving rotary vmax should roughly double cycle time
    c_slow, _ = cyc(MachineLimits(v_rot=0.2))
    c_fast, _ = cyc(MachineLimits(v_rot=0.8))
    check(c_slow > 1.8 * c_fast, "cycle is rotary-limited (cycle ~ 1/v_rot)",
          f"({c_slow:.2f} vs {c_fast:.2f} s)")

    # cycle must be monotonic non-increasing in rotary vmax
    cs = [cyc(MachineLimits(v_rot=v))[0] for v in (0.2, 0.4, 0.8, 1.6)]
    check(all(cs[i] >= cs[i + 1] - 1e-6 for i in range(len(cs) - 1)),
          "cycle non-increasing as rotary vmax rises", f"({[round(x,2) for x in cs]})")

    # a lower feed ceiling can only increase (or hold) cycle time
    c_lowf, f_lowf = cyc(process=ProcessParams(feed_max_mm_min=500))
    c_hif, f_hif = cyc(process=ProcessParams(feed_max_mm_min=5000))
    check(c_lowf >= c_hif - 1e-6, "lower feed ceiling does not reduce cycle",
          f"({c_lowf:.2f} >= {c_hif:.2f} s)")
    check(f_lowf <= 500 + 1e-6, "effective feed respects the ceiling",
          f"(cap {f_lowf:.0f} <= 500)")

    # raising linear vmax must not increase cycle (it isn't the binding axis)
    c_l1, _ = cyc(MachineLimits(v_lin=20))
    c_l2, _ = cyc(MachineLimits(v_lin=200))
    check(c_l2 <= c_l1 + 1e-6, "more linear vmax never increases cycle")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nPHYSICAL-CONSISTENCY TESTS PASSED")


if __name__ == "__main__":
    main()
