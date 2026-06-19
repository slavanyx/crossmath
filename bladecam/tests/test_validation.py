#!/usr/bin/env python3
"""Physics-validation tests (wired into ctest).

These check the optimizer against ground truth rather than just smoke-running:
  - a DEVELOPABLE ruled surface must flank-mill to ~zero error with a cylinder
    (exact case: the tool is tangent along the whole straight ruling);
  - optimization must be monotonic: min-max and global never worse than the
    two-point seed.
"""
import sys

try:
    import numpy as np
    from bladecam import core, blade
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP validation ({e})")
    sys.exit(0)

FAILED = []


def check(cond, name, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILED.append(name)


def test_developable_mills_to_zero():
    """Circular cylinder (constant ruling director => developable). A cylindrical
    flank tool is tangent along each straight ruling, so deviation must be ~0."""
    nu = 40
    u = np.linspace(0.0, 1.0, nu)
    a = np.column_stack([20*np.cos(u), 20*np.sin(u), 0*u])
    b = np.column_stack([20*np.cos(u), 20*np.sin(u), 12 + 0*u])
    ap, bp = blade.rail_tangents(a, b)
    _, _, dev = core.optimize_global(a, b, ap, bp, 6.0, nv=41, mu=10.0, nsweeps=4)
    check(dev.max() < 0.010, "developable surface mills to ~zero",
          f"(max {dev.max()*1000:.2f} um)")


def test_optimization_is_monotonic():
    """min-max and global must never be worse than the two-point seed."""
    tp = compute(Params(strategy="two_point", nu=40))["dev"].max()
    mm = compute(Params(strategy="minmax", nu=40))["dev"].max()
    gl = compute(Params(strategy="global", nu=40))["dev"].max()
    check(mm <= tp + 1e-9, "min-max <= two-point",
          f"({mm*1000:.1f} <= {tp*1000:.1f} um)")
    check(gl <= tp + 1e-9, "global <= two-point",
          f"({gl*1000:.1f} <= {tp*1000:.1f} um)")


def test_global_beats_naive_floor():
    """The global optimum must be far below the naive single-tangency twist
    error estimate eps ~ R*L^2/(8*delta^2) -- i.e. optimization genuinely helps."""
    p = Params(strategy="global", nu=60)
    r = compute(p)
    a, b = r["a"], r["b"]
    delta = r["delta"]
    L = np.linalg.norm(b - a, axis=1)
    finite = np.isfinite(delta) & (np.abs(delta) > 1e-6)
    naive = p.R * L[finite]**2 / (8.0 * delta[finite]**2)
    check(r["dev"].max() < 0.5 * np.median(naive),
          "global beats naive twist-error estimate",
          f"(global {r['dev'].max()*1000:.1f} um vs naive median {np.median(naive)*1000:.0f} um)")


def test_topp_respects_limits():
    """The time-optimal feed must not violate any axis velocity/acceleration
    limit (audit 3): reconstruct the trajectory and check |vel|<=vmax, |acc|<=amax."""
    from bladecam import core
    from bladecam.process import MachineLimits, ProcessParams
    r = compute(Params(strategy="global"))
    q = np.column_stack([r["machine_path"], r["seglen"]])
    ml = MachineLimits()
    feed = ProcessParams().effective_feed_mm_min() / 60.0
    vmax = np.array(ml.vmax() + [feed]); amax = np.array(ml.amax() + [1e4])
    aprof, _ = core.topp(q, vmax, amax)
    n = q.shape[0]; ds = 1.0 / (n - 1)
    qp = np.gradient(q, ds, axis=0); qpp = np.gradient(qp, ds, axis=0)
    sdd = np.gradient(aprof, ds) / 2.0
    vel = qp * np.sqrt(np.clip(aprof, 0, None))[:, None]
    acc = qpp * aprof[:, None] + qp * sdd[:, None]
    check((np.abs(vel) / vmax).max() <= 1.02, "TOPP respects velocity limits",
          f"({(np.abs(vel)/vmax).max():.3f})")
    check((np.abs(acc) / amax).max() <= 1.05, "TOPP respects acceleration limits",
          f"({(np.abs(acc)/amax).max():.3f})")


def main():
    for fn in (test_developable_mills_to_zero,
               test_optimization_is_monotonic,
               test_global_beats_naive_floor,
               test_topp_respects_limits):
        print(fn.__name__)
        fn()
    if FAILED:
        print(f"\nFAILED: {len(FAILED)} -> {FAILED}")
        sys.exit(1)
    print("\nALL VALIDATION TESTS PASSED")


if __name__ == "__main__":
    main()
