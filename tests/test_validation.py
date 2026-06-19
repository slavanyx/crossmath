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


def test_topp_handles_cusp():
    """At a velocity cusp (an axis reverses, q'->0 while q''!=0) the path
    acceleration is q''*a with no q'*sdd term to trade against, so feasibility
    requires |q''|a<=amax. The naive single forward/backward scheme left this
    unbounded and posted accelerations ~10x the limit. Check both an exact
    1-DOF cusp and the realistic case (rotary axis reverses while the arc-length
    DOF keeps advancing)."""
    from bladecam import core

    def accel_ratio(qq, vmax, amax):
        n = qq.shape[0]; ds = 1.0 / (n - 1)
        aprof, _ = core.topp(qq, vmax, amax)
        qp = np.gradient(qq, ds, axis=0); qpp = np.gradient(qp, ds, axis=0)
        # midpoint joint acceleration q''*a + q'*sdd over each segment
        r = 0.0
        for k in range(n - 1):
            sdd = (aprof[k+1] - aprof[k]) / (2*ds)
            am = 0.5*(aprof[k]+aprof[k+1])
            qpm = 0.5*(qp[k]+qp[k+1]); qppm = 0.5*(qpp[k]+qpp[k+1])
            r = max(r, np.max(np.abs(qppm*am + qpm*sdd) / amax))
        return r

    n = 121; s = np.linspace(0, 1, n)
    # realistic: rotary reversal + monotone arc length (what the pipeline emits)
    qq = np.column_stack([0.8*np.sin(2*np.pi*s), np.linspace(0, 50, n)])
    r1 = accel_ratio(qq, np.array([1.5, 200.]), np.array([5., 1e4]))
    check(r1 <= 2.0, "TOPP bounds acceleration through a rotary cusp",
          f"(ratio {r1:.2f}, was ~10x before the fix)")
    # exact 1-DOF cusp
    r2 = accel_ratio((2.0*np.sin(np.pi*s)).reshape(n, 1),
                     np.array([1.0]), np.array([1.0]))
    check(r2 <= 2.0, "TOPP bounds acceleration through an exact cusp",
          f"(ratio {r2:.2f})")


def test_topp_small_n():
    """TOPP must not read out of bounds for short paths. The 3-point curvature
    stencil reads q(:,3)/q(:,n-2); for n<3 that is out of bounds (UB; the Debug
    -fcheck=all build aborts). n=1/2 are degenerate but must return finite."""
    from bladecam import core
    for n in (2, 3, 4):
        q = np.column_stack([np.linspace(0, 5, n), np.linspace(0, 1, n)])
        aprof, T = core.topp(q, np.array([1., 1.]), np.array([1., 1.]))
        check(np.all(np.isfinite(aprof)) and np.isfinite(T),
              f"TOPP n={n} finite (no OOB)")


def test_chatter_matches_closed_form():
    """The single-DOF stability-lobe model must reproduce the closed-form
    asymptotic limit a_lim,min = 2 k zeta (1+zeta)/(Kt N) and the critical
    frequency ratio r* = sqrt(1+2 zeta) where Re[G] is most negative."""
    from bladecam import core
    wn, zeta, k, Kt, N = 800.0, 0.03, 2.0e7, 800.0, 4
    rpm, alim = core.stability_lobes(wn, zeta, k, Kt, N, nlobes=8, nptsper=4000)
    amin = np.nanmin(alim)
    analytic = 2 * k * zeta * (1 + zeta) / (Kt * N)
    check(abs(amin - analytic) / analytic < 1e-3,
          "chatter a_lim,min == 2 k zeta (1+zeta)/(Kt N)",
          f"({amin:.2f} vs {analytic:.2f})")
    # lobes ordered descending in rpm (k=0 is the high-speed lobe -> the eps
    # modulo-2pi reduction; regression for the high-speed-lobe audit fix)
    mx = [np.nanmax(rpm.reshape(8, 4000)[L]) for L in range(8)]
    check(all(mx[i] > mx[i+1] for i in range(7)),
          "stability lobes ordered high-speed-first")


def test_swept_penalty_finite_flute():
    """The swept-overcut penalty must use the finite flute (matching the
    swept_deviation metric): turning it on reduces real swept overcut, and the
    finite-flute form should not cost MORE per-ruling deviation than it buys."""
    from bladecam import blade, optimize, core
    a, b = blade.make_blade(60, 30, 55, 20, 8, 0.6, 1.4)   # high twist
    ap, bp = blade.rail_tangents(a, b); R = 6.0

    def overcut(q0, al):
        Lf = np.linalg.norm(b - a, axis=1)
        surf = blade.surface(a, b, 30).reshape(-1, 3)
        return float(max(0.0, -core.swept_deviation(q0, al, Lf, R, surf).min()))

    off = optimize.optimize_blade(a, b, ap, bp, R, strategy="global", swept_w=0.0)["global"]
    on = optimize.optimize_blade(a, b, ap, bp, R, strategy="global", swept_w=1.0)["global"]
    oc_off, oc_on = overcut(off["q0"], off["alpha"]), overcut(on["q0"], on["alpha"])
    check(oc_on < 0.1 * oc_off, "swept penalty cuts overcut >10x",
          f"({oc_off*1000:.0f} -> {oc_on*1000:.0f} um)")


def main():
    for fn in (test_developable_mills_to_zero,
               test_optimization_is_monotonic,
               test_global_beats_naive_floor,
               test_topp_respects_limits,
               test_topp_handles_cusp,
               test_topp_small_n,
               test_chatter_matches_closed_form,
               test_swept_penalty_finite_flute):
        print(fn.__name__)
        fn()
    if FAILED:
        print(f"\nFAILED: {len(FAILED)} -> {FAILED}")
        sys.exit(1)
    print("\nALL VALIDATION TESTS PASSED")


if __name__ == "__main__":
    main()
