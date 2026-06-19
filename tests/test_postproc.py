#!/usr/bin/env python3
"""G-code post-processor: inverse-time feed realises the TOPP cycle time."""
import sys

try:
    import numpy as np
    from bladecam.pipeline import compute, Params
    from bladecam import postproc
except ImportError as e:
    print(f"SKIP postproc ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    r = compute(Params(strategy="global", nu=40))
    mt = r["move_times_s"]

    # the per-move durations must sum to the reported cycle time
    check(abs(mt.sum() - r["cycle_time_s"]) < 1e-6,
          "per-move times sum to cycle time",
          f"({mt.sum():.3f} vs {r['cycle_time_s']:.3f} s)")

    g = postproc.to_gcode(r["machine_path"], r["feed_cap_mm_min"],
                          move_times=mt)
    check("G93" in g and "G94" in g, "inverse-time block emitted (G93/G94)")
    n_moves = len([ln for ln in g.splitlines() if ln.startswith("G1 X")])
    check(n_moves == len(r["machine_path"]) - 1, "one cutting move per segment",
          f"({n_moves})")

    # reconstruct cycle time from the inverse-time F values (F = 1/min)
    finv = [float(ln.split("F")[1]) for ln in g.splitlines()
            if ln.startswith("G1 X") and "F" in ln]
    recon = sum(60.0 / f for f in finv)
    check(abs(recon - r["cycle_time_s"]) < 1e-3,
          "G93 feeds reconstruct the cycle time",
          f"({recon:.3f} vs {r['cycle_time_s']:.3f} s)")

    # constant-feed fallback still works
    g2 = postproc.to_gcode(r["machine_path"], 3000.0)
    check("G93" not in g2 and "F3000" in g2, "constant-feed fallback (G94)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nPOSTPROC TESTS PASSED")


if __name__ == "__main__":
    main()
