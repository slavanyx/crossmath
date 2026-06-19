#!/usr/bin/env python3
"""Machine profiles + reachability: a toolpath must be flagged unreachable when
it exceeds a machine's travel/rotary envelope, and reachable on a roomy one."""
import sys

try:
    import numpy as np
    from bladecam import machine
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP machine ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    # default library is non-empty and each is a drop-in for TOPP (vmax/amax/kind)
    check(len(machine.DEFAULT_MACHINES) >= 3, "default machine library populated")
    for nm, m in machine.DEFAULT_MACHINES.items():
        check(len(m.vmax()) == 5 and len(m.amax()) == 5 and m.kind in (0, 1),
              f"'{nm}' is a valid TOPP-compatible profile")

    # reachability: a path inside the box is reachable; outside is flagged
    m = machine.Machine(x_range=(-100, 100), z_range=(-50, 50),
                        a_range=(-1.0, 1.0))
    inside = np.array([[10, 0, 10, 0.2, 0.0], [-20, 0, -10, -0.3, 1.0]], float)
    check(reach_ok := (len(machine.reachability(m, inside)) == 0),
          "path within envelope is reachable")
    outside = np.array([[10, 0, 10, 0.2, 0.0], [150, 0, -90, 1.6, 0.0]], float)
    v = machine.reachability(m, outside)
    check("X" in v and "Z" in v and "A" in v, "out-of-range axes are flagged",
          f"({sorted(v)})")
    check(abs(v["X"] - 50.0) < 1e-6, "X excess measured (150 vs +100 limit)")

    # through the pipeline: a roomy machine reaches the blade; a tiny one doesn't
    big = compute(Params(strategy="global", machine=machine.get_machine("Generic 5-axis trunnion")))
    check(big["reachable"] and "machine_name" in big,
          "blade reachable on the generic trunnion", f"({big['machine_name']})")
    tiny = compute(Params(strategy="global",
                          machine=machine.Machine(name="tiny", x_range=(-5, 5),
                                                  y_range=(-5, 5), z_range=(-5, 5))))
    check(not tiny["reachable"] and len(tiny["axis_violations"]) > 0,
          "blade unreachable on a 10 mm-cube machine",
          f"({sorted(tiny['axis_violations'])})")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nMACHINE TESTS PASSED")


if __name__ == "__main__":
    main()
