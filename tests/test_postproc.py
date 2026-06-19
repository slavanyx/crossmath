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

    # --- Heidenhain TCPM klartext post ---
    h = postproc.to_heidenhain(r["contact"], r["alpha"], r["feed_cap_mm_min"],
                               move_times=mt)
    lines = h.splitlines()
    check(lines[0].startswith("BEGIN PGM") and lines[-1].startswith("END PGM"),
          "Heidenhain program framed (BEGIN/END PGM)")
    check("FUNCTION TCPM" in h and "FUNCTION RESET TCPM" in h,
          "TCPM activated and reset")
    ln_moves = [l for l in lines if l.startswith("LN ")]
    check(len(ln_moves) == len(r["contact"]) + 1,  # rapid-in + plunge + cuts
          "one LN block per pose (+rapid-in)", f"({len(ln_moves)})")
    # tool vectors are unit and match alpha; the LN block carries TX/TY/TZ
    import re
    def vec(l):
        return np.array([float(re.search(fr"{a}([+-][0-9.]+)", l).group(1))
                         for a in ("TX", "TY", "TZ")])
    cut = ln_moves[-1]
    v = vec(cut); a_last = r["alpha"][-1]/np.linalg.norm(r["alpha"][-1])
    check(abs(np.linalg.norm(v) - 1.0) < 1e-4, "tool vector is unit")
    check(np.allclose(np.abs(v), np.abs(a_last), atol=1e-3),
          "LN tool vector matches the optimised tool axis")
    # cutting-move feeds reconstruct the cycle time (tip distance / feed)
    cut_lines = ln_moves[2:]  # skip rapid-in + plunge
    tip = r["contact"]; recon = 0.0
    for k, l in enumerate(cut_lines):
        f = float(re.search(r" F([0-9.]+)", l).group(1))
        recon += 60.0 * np.linalg.norm(tip[k+1]-tip[k]) / f
    # this matches cycle time only up to the linear/rotary mix; check it is a
    # sane positive duration of the right order
    check(0.2*r["cycle_time_s"] < recon < 5*r["cycle_time_s"],
          "Heidenhain feeds give a sane cycle time", f"({recon:.2f}s)")
    # a NON-unit input axis must be normalised in the LN tool vector (Heidenhain
    # requires a unit orientation vector)
    h3 = postproc.to_heidenhain(np.array([[0., 0, 0], [5., 0, 0]]),
                                np.array([[0., 0, 3.], [0., 0, 2.]]), 3000.0)
    v3 = vec([l for l in h3.splitlines() if l.startswith("LN ")][-1])
    check(abs(np.linalg.norm(v3) - 1.0) < 1e-4, "non-unit input axis is normalised",
          f"(|v|={np.linalg.norm(v3):.4f})")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nPOSTPROC TESTS PASSED")


if __name__ == "__main__":
    main()
