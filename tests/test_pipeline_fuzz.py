#!/usr/bin/env python3
"""Property-based pipeline fuzz (audit angle F): for ANY sensible parameter set
and strategy, the full compute() must return finite outputs -- including the
verification fields added later (swept_field, envelope_surf, holder_clearance).
A NaN/Inf here would crash the 3D view or post a garbage toolpath.
"""
import sys

try:
    import numpy as np
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP pipeline-fuzz ({e})")
    sys.exit(0)

FAILED = []
KEYS = ["devfield", "swept_field", "envelope_surf", "holder_clearance",
        "min_clearance", "swept_overcut", "cycle_time_s", "aprof",
        "machine_path", "move_times_s", "dev"]


def main():
    rng = np.random.default_rng(3)
    bad = 0
    for t in range(30):
        p = Params(strategy=str(rng.choice(["global", "minmax", "two_point", "smoothed"])),
                   twist=rng.uniform(0, 1.8), wrap=rng.uniform(0, 1.5),
                   R=rng.uniform(1.5, 12), n_blades=int(rng.integers(4, 30)),
                   nu=int(rng.integers(20, 80)), gamma=float(rng.choice([0.0, 0.1])),
                   r_hub=rng.uniform(15, 40), r_shroud=rng.uniform(45, 70),
                   z_span=rng.uniform(10, 40))
        try:
            r = compute(p)
            for k in KEYS:
                if not np.all(np.isfinite(np.asarray(r[k]))):
                    print(f"  FAIL non-finite {k} (trial {t}, {p.strategy})")
                    bad += 1
                    break
        except Exception as e:
            print(f"  FAIL exception (trial {t}, {p.strategy}): {e!r}")
            bad += 1
    if bad:
        print(f"\nFAILED: {bad}/30 fuzz cases")
        sys.exit(1)
    print("  ok   30/30 fuzz cases produce finite outputs")
    print("\nPIPELINE FUZZ TESTS PASSED")


if __name__ == "__main__":
    main()
