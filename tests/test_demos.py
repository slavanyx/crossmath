#!/usr/bin/env python3
"""Smoke-test the demo gallery definitions (headless: no rendering / CAD): every
demo spec must build a blade and run the pipeline to a finite result, so the
gallery in demos/ can never silently rot."""
import sys
import os

try:
    import numpy as np
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "demos"))
    import make_demos
    from bladecam import blade
    from bladecam.pipeline import compute, Params
    from bladecam.process import ProcessParams
except Exception as e:
    print(f"SKIP demos ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    check(len(make_demos.DEMOS) >= 5, "demo gallery has several parts",
          f"({len(make_demos.DEMOS)})")
    names = set()
    for name, blurb, pk, tk, extra in make_demos.DEMOS:
        names.add(name)
        check(len(blurb) > 20, f"{name}: has a description")
        p = Params(strategy="global", process=ProcessParams(**tk), **pk)
        a, b = blade.make_blade(p.nu, p.r_hub, p.r_shroud, p.z_span,
                                p.z_offset, p.wrap, p.twist)
        check(a.shape == b.shape and np.all(np.isfinite(a)),
              f"{name}: builds finite rails")
        r = compute(p)
        check(np.all(np.isfinite(r["dev"])) and np.isfinite(r["swept_overcut"])
              and np.isfinite(r["cycle_time_s"]),
              f"{name}: pipeline runs to a finite result",
              f"(swept {r['swept_overcut']*1000:.0f} µm)")
    check(len(names) == len(make_demos.DEMOS), "demo names are unique")
    # the storyline: the 'optimised' twist demo beats the plain-cylinder one
    by = {n: (pk, tk) for n, _b, pk, tk, _e in make_demos.DEMOS}
    if "03_twisted_cylinder" in by and "04_twisted_optimised" in by:
        r3 = compute(Params(strategy="global",
                            process=ProcessParams(**by["03_twisted_cylinder"][1]),
                            **by["03_twisted_cylinder"][0]))
        r4 = compute(Params(strategy="global",
                            process=ProcessParams(**by["04_twisted_optimised"][1]),
                            **by["04_twisted_optimised"][0]))
        check(r4["swept_overcut"] < 0.5 * r3["swept_overcut"],
              "the optimised twisted demo beats the plain-cylinder one",
              f"({r4['swept_overcut']*1000:.0f} vs {r3['swept_overcut']*1000:.0f} µm)")

    # super-complex gallery: every spec must build a finite, machined result on
    # the richer make_complex_blade geometry (no NaN/crash on hard parts)
    try:
        import make_complex_demos as MC
        from bladecam import blade as _bl
    except Exception:
        MC = None
    if MC is not None:
        check(len(MC.DEMOS) >= 4, "complex gallery has several parts")
        for name, blurb, bk, tk, ek in MC.DEMOS:
            a, b = _bl.make_complex_blade(nu=60, **bk)
            r = compute(Params(strategy="global", rails=(a, b),
                               process=ProcessParams(**tk), **ek))
            check(np.all(np.isfinite(r["q0"])) and np.all(np.isfinite(r["swept_field"]))
                  and np.isfinite(r["cycle_time_s"]) and r["feed_feasible"],
                  f"complex {name}: finite, feed-feasible result",
                  f"(swept {r['swept_overcut']*1000:.0f} µm, cyc {r['cycle_time_s']:.1f} s)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nDEMO TESTS PASSED")


if __name__ == "__main__":
    main()
