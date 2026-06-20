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

    # --- mechanistic cutting-force model ---
    p = ProcessParams()
    f1 = p.cutting_forces(0.05); f2 = p.cutting_forces(0.10)
    check(f2["F_peak"] > f1["F_peak"] and f2["power_W"] > f1["power_W"],
          "forces & power increase with feed-per-tooth",
          f"(F {f1['F_peak']:.0f}->{f2['F_peak']:.0f} N)")
    # deeper / wider cut raises forces
    check(ProcessParams(ap=8).cutting_forces(0.05)["F_peak"]
          > p.cutting_forces(0.05)["F_peak"], "deeper cut raises force")
    # a low max-force cap lowers the feed
    cap_hi = ProcessParams(max_force_N=5000).mechanistic_feed_cap_mm_min()
    cap_lo = ProcessParams(max_force_N=300).mechanistic_feed_cap_mm_min()
    check(cap_lo < cap_hi, "tighter force limit lowers the feed cap",
          f"({cap_lo:.0f} < {cap_hi:.0f})")
    # an under-powered spindle on a heavy slot is flagged infeasible
    check(not ProcessParams(ap=20, ae=12, spindle_power_kW=2.0).feed_feasible(),
          "heavy cut on a 2 kW spindle is infeasible")
    # DIFFERENTIAL: reproduce F_peak/F_mean from the documented force model with
    # independent sampling -- kills any engagement-arc / coefficient drift.
    import math
    def forces_ref(pp, fz, ae):
        # independent helical reimplementation: different axial (32) and angular
        # (360) discretisation than the production model
        R = pp.tool_dia/2; ae = max(1e-6, min(ae, 2*R))
        phi_ex = math.acos(max(-1, min(1, 1 - ae/R)))
        Ktc, Krc, N = pp.Kt, pp.Kr*pp.Kt, pp.n_teeth
        beta = math.radians(pp.helix_deg); nz = 32; dz = pp.ap/nz
        zc = (np.arange(nz)+0.5)*dz; psi = zc*math.tan(beta)/R
        th = np.linspace(0, 2*math.pi/N, 360, endpoint=False)
        Sx = np.zeros_like(th); Sy = np.zeros_like(th)
        for k in range(N):
            phi = np.mod((th + k*2*math.pi/N)[:,None] - psi[None,:], 2*math.pi)
            eng = phi <= phi_ex; s = np.sin(phi)
            dFt = (Ktc*fz*s + pp.Kte)*dz*eng; dFr = (Krc*fz*s + pp.Kre)*dz*eng
            Sx += np.sum(-dFt*np.cos(phi) - dFr*np.sin(phi), axis=1)
            Sy += np.sum(dFt*np.sin(phi) - dFr*np.cos(phi), axis=1)
        F = np.hypot(Sx, Sy); return F.max(), F.mean()
    fp, fm = forces_ref(p, 0.07, p._ae())
    g = p.cutting_forces(0.07)
    check(abs(g["F_peak"] - fp) < 2.0 and abs(g["F_mean"] - fm) < 1.0,
          "force model matches independent helical engagement integral",
          f"(peak {g['F_peak']:.1f} vs {fp:.1f})")
    # wider radial engagement raises the mean force (ties force to phi_ex)
    check(p.cutting_forces(0.05, ae=2*p.tool_dia/2)["F_mean"]
          > p.cutting_forces(0.05, ae=0.2*p.tool_dia/2)["F_mean"],
          "wider radial engagement raises mean force")
    # deflection constraint binds: a tight deflection budget lowers the feed cap
    loose = ProcessParams(dev_allow_um=500).mechanistic_feed_cap_mm_min()
    tight = ProcessParams(dev_allow_um=2).mechanistic_feed_cap_mm_min()
    check(tight < loose, "tighter deflection budget lowers the feed cap",
          f"({tight:.0f} < {loose:.0f})")

    # pipeline surfaces the forces, finite and positive
    r = compute(Params(strategy="global"))
    check(r["cut_force_peak_N"] > 0 and r["cut_power_W"] > 0 and r["feed_feasible"],
          "pipeline reports cutting force/power", f"({r['cut_force_peak_N']:.0f} N)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nPHYSICAL-CONSISTENCY TESTS PASSED")


if __name__ == "__main__":
    main()
