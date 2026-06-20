#!/usr/bin/env python3
"""Mechanistic cutting-force model audit: helix lag, average-force invariants,
measured-coefficient calibration round-trip, and the material library."""
import sys
import math

try:
    import numpy as np
    from bladecam.process import (ProcessParams, identify_coefficients,
                                  coeffs_for_material, MATERIAL_COEFFS,
                                  _arc_integrals)
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP cutting_force ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def flat_ref(pp, fz, ae):
    """Independent zero-helix (flat ap) force reference."""
    R = pp.tool_dia / 2; ae = max(1e-6, min(ae, 2 * R))
    phi_ex = math.acos(max(-1, min(1, 1 - ae / R)))
    Ktc, Krc, N = pp.Kt, pp.Kr * pp.Kt, pp.n_teeth
    th = np.linspace(0, 2 * math.pi / N, 200, endpoint=False)
    Sx = np.zeros_like(th); Sy = np.zeros_like(th)
    for k in range(N):
        phi = np.mod(th + k * 2 * math.pi / N, 2 * math.pi); eng = phi <= phi_ex
        s = np.sin(phi)
        Ft = pp.ap * (Ktc * fz * s + pp.Kte) * eng
        Fr = pp.ap * (Krc * fz * s + pp.Kre) * eng
        Sx += -Ft * np.cos(phi) - Fr * np.sin(phi)
        Sy += Ft * np.sin(phi) - Fr * np.cos(phi)
    F = np.hypot(Sx, Sy)
    return F.max(), F.mean()


def main():
    # 1) helix=0 reduces EXACTLY to the flat ap*(...) model
    p0 = ProcessParams(helix_deg=0.0)
    g0 = p0.cutting_forces(0.06)
    fp, fm = flat_ref(p0, 0.06, p0._ae())
    check(abs(g0["F_peak"] - fp) < 1e-9 and abs(g0["F_mean"] - fm) < 1e-9,
          "helix=0 reduces exactly to the flat-tooth model")

    # 2) average force/torque/power are HELIX-INVARIANT (standard result); only
    #    the instantaneous waveform (hence the peak) changes
    means = {}
    for h in (0.0, 15.0, 30.0, 45.0):
        means[h] = ProcessParams(helix_deg=h).cutting_forces(0.06)
    tq0 = means[0.0]["torque_Nm"]; pw0 = means[0.0]["power_W"]
    inv = all(abs(means[h]["torque_Nm"] - tq0) < 0.01 * abs(tq0) and
              abs(means[h]["power_W"] - pw0) < 0.01 * abs(pw0) for h in means)
    check(inv, "mean torque & power are helix-invariant (<1%)",
          f"(torque {tq0:.3f} Nm)")
    fxinv = all(abs(means[h]["Fx_mean"] - means[0.0]["Fx_mean"]) <
                0.01 * (abs(means[0.0]["Fx_mean"]) + 1.0) for h in means)
    check(fxinv, "mean tool-frame Fx is helix-invariant")

    # 3) peak force DROPS with helix (the cut is spread axially)
    check(means[45.0]["F_peak"] < means[0.0]["F_peak"] < 1e9 and
          means[30.0]["F_peak"] < means[0.0]["F_peak"],
          "helix lowers the peak cutting force",
          f"({means[0.0]['F_peak']:.0f} -> {means[45.0]['F_peak']:.0f} N)")

    # 4) engagement-arc integrals vs numerical quadrature
    a, b = 0.3, 1.9
    x = np.linspace(a, b, 200001)
    num = (np.trapezoid(np.sin(x) * np.cos(x), x), np.trapezoid(np.sin(x) ** 2, x),
           np.trapezoid(np.cos(x), x), np.trapezoid(np.sin(x), x))
    ana = _arc_integrals(a, b)
    check(all(abs(n - m) < 1e-6 for n, m in zip(num, ana)),
          "arc integrals match numerical quadrature")

    # 5) measured-coefficient calibration round-trip: synth average forces from
    #    KNOWN coeffs (via the discrete model), recover them via the closed-form
    #    average-force inversion -- a real cross-check of the two paths.
    true = dict(Kt=2000.0, Kr=0.5, Kte=24.0, Kre=20.0)
    pp = ProcessParams(tool_dia=10.0, n_teeth=4, ap=5.0, ae=4.0, helix_deg=30.0,
                       **true)
    R = pp.tool_dia / 2
    phi_ex = math.acos(1 - pp.ae / R)
    fzs = np.array([0.02, 0.05, 0.08, 0.12])
    Fx = []; Fy = []
    for fz in fzs:
        g = pp.cutting_forces(fz)
        Fx.append(g["Fx_mean"]); Fy.append(g["Fy_mean"])
    rec = identify_coefficients(fzs, Fx, Fy, pp.ap, R, pp.n_teeth, phi_ex)
    rel = {k: abs(rec[k] - true[k]) / abs(true[k]) for k in true}
    check(all(v < 0.02 for v in rel.values()),
          "calibration recovers known coefficients (<2%)",
          f"({ {k: round(rec[k],1) for k in rec} })")

    # 6) material library: present, ordered hard>soft, usable in ProcessParams
    check(set(MATERIAL_COEFFS) >= {"Aluminium 7075", "Ti-6Al-4V", "Inconel 718"},
          "material coefficient library populated")
    al = coeffs_for_material("Aluminium 7075")["Kt"]
    ti = coeffs_for_material("Ti-6Al-4V")["Kt"]
    inc = coeffs_for_material("Inconel 718")["Kt"]
    check(inc > ti > al, "harder materials have higher Kt", f"({al}<{ti}<{inc})")
    pti = ProcessParams(**coeffs_for_material("Ti-6Al-4V"))
    check(pti.cutting_forces(0.05)["F_peak"] >
          ProcessParams(**coeffs_for_material("Aluminium 7075")).cutting_forces(0.05)["F_peak"],
          "titanium cuts harder than aluminium at equal feed")

    # 7) pipeline still reports finite, positive forces with the helical model
    r = compute(Params(strategy="global"))
    check(r["cut_force_peak_N"] > 0 and r["cut_power_W"] > 0,
          "pipeline force/power finite with helical model",
          f"({r['cut_force_peak_N']:.0f} N)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nCUTTING-FORCE TESTS PASSED")


if __name__ == "__main__":
    main()
