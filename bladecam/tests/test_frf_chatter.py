#!/usr/bin/env python3
"""Measured-FRF stability lobes must agree with the analytic modal model when
fed the receptance of that same single mode."""
import sys

try:
    import numpy as np
    from bladecam import core
except ImportError as e:
    print(f"SKIP frf-chatter ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    fn, zeta, k, Kt, teeth = 800.0, 0.03, 2.0e4, 800.0, 4

    # analytic modal lobes
    rpm_m, a_m = core.stability_lobes(fn, zeta, k, Kt, teeth, 6, 400)

    # synthesize the SAME mode's receptance G(f) = (1/k)/(1 - r^2 + i 2 zeta r)
    freq = np.linspace(fn, 2.0 * fn, 400)
    r = freq / fn
    denom = (1 - r**2) + 1j * (2 * zeta * r)
    G = (1.0 / k) / denom
    rpm_f, a_f = core.stability_lobes_frf(freq, G.real, G.imag, Kt, teeth, 6)

    amin_m = float(np.nanmin(a_m))
    amin_f = float(np.nanmin(a_f))
    check(np.isfinite(amin_f) and amin_f > 0, "FRF lobes produce finite depth",
          f"({amin_f:.3f} mm)")
    check(abs(amin_f - amin_m) / amin_m < 0.02,
          "FRF min depth matches modal model",
          f"(frf {amin_f:.3f} vs modal {amin_m:.3f} mm)")
    check(np.nanmax(rpm_f) > 3.0e4, "FRF high-speed lobe present")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nFRF CHATTER TESTS PASSED")


if __name__ == "__main__":
    main()
