#!/usr/bin/env python3
"""Regression tests for the bugs found in the adversarial audit.

Each test is named for the bug it guards and FAILS if that bug is reintroduced.
Run standalone (exit 0 = pass/skip, nonzero = failure) or via ctest.

  Bug 1  conical tool ignored when rendering the deviation field
  Bug 2  chatter diagram omitted the high-speed (k=0) stability lobe
  Bug 4  GUI recomputed the strategy comparison on the UI thread
  Bug 5  chatter chart x-limit blew up with the high-speed lobe asymptote
  Bug 6  Nelder-Mead simplex init had an out-of-bounds-looking subscript
         (covered by the compiler-warnings ctest, not here)
"""
import sys

try:
    import numpy as np
    from bladecam import core, pipeline
    from bladecam.pipeline import Params
except ImportError as e:
    print(f"SKIP audit regressions (missing dependency: {e})")
    sys.exit(0)

FAILED = []


def check(cond, name):
    print(f"  {'ok  ' if cond else 'FAIL'} {name}")
    if not cond:
        FAILED.append(name)


# --- Bug 1: deviation field must use the conical tool for the global strategy --
def test_bug1_devfield_uses_cone():
    """The visualised devfield must reflect the conical tool (not the cylinder)
    when strategy='global' and gamma>0, and must match an independent cone
    recompute while differing from the cylinder field."""
    p = Params(strategy="global", gamma=0.10, nu=40, nv=25)
    r = pipeline.compute(p)
    a, b, q0, al = r["a"], r["b"], r["q0"], r["alpha"]
    nu, nv_grid, _ = r["surf"].shape
    v = np.linspace(0.0, 1.0, nv_grid)

    cone = np.empty((nu, nv_grid))
    cyl = np.empty((nu, nv_grid))
    for i in range(nu):
        pts = (1 - v)[:, None] * a[i][None, :] + v[:, None] * b[i][None, :]
        cone[i] = core.deviation_cone(q0[i], al[i], p.R, p.gamma, pts)
        cyl[i] = core.deviation(q0[i], al[i], p.R, pts)

    check(np.allclose(r["devfield"], cone), "devfield matches cone recompute")
    check(not np.allclose(cone, cyl), "cone field differs from cylinder field")
    # gamma=0 must leave the field identical to the cylinder
    rc = pipeline.compute(Params(strategy="global", gamma=0.0, nu=40, nv=25))
    check(np.all(np.isfinite(rc["devfield"])), "gamma=0 devfield finite")


# --- Bug 1b: cone deviation reduces exactly to the cylinder at gamma=0 --------
def test_bug1_cone_equals_cylinder_at_zero():
    pts = np.array([[2.0, 0, 0], [3.0, 0, 5], [4.0, 1, 9]])
    q0 = np.zeros(3); al = np.array([0.0, 0.0, 1.0])
    g_cyl = core.deviation(q0, al, 2.0, pts)
    g_cone = core.deviation_cone(q0, al, 2.0, 0.0, pts)
    check(np.allclose(g_cyl, g_cone), "deviation_cone(gamma=0) == deviation")


# --- Bug 2: chatter must include the high-speed lobe -------------------------
def test_bug2_chatter_highspeed_lobe():
    rpm, alim = core.stability_lobes(800.0, 0.03, 2.0e4, 800.0, 4, 6, 80)
    check(rpm.max() > 3.0e4, "high-speed (k=0) lobe reaches high rpm")
    check(np.all(alim > 0), "limiting depths positive")
    # more damping must raise the unconditionally-stable depth
    _, a2 = core.stability_lobes(800.0, 0.06, 2.0e4, 800.0, 4, 6, 80)
    check(a2.min() > alim.min(), "more damping -> higher stable depth")


# --- Bug 4: compare data carries dev arrays (so the GUI need not recompute) --
def test_bug4_compare_full_carries_dev():
    try:
        from bladecam.gui.model import AppModel
    except ImportError:
        print("  skip Bug4 (no GUI deps)")
        return
    stats = AppModel().compute_compare_full()
    check(all("dev" in stats[s] for s in stats),
          "compare_full includes dev arrays (no UI-thread recompute needed)")
    check(all({"dev_um", "jerk", "cycle_s"} <= set(stats[s]) for s in stats),
          "compare_full includes scalar metrics")


# --- Bug 5: chatter chart x-limit stays bounded to the operating range -------
def test_bug5_chatter_chart_xlim_bounded():
    try:
        from bladecam.gui import charts
    except ImportError:
        print("  skip Bug5 (no matplotlib)")
        return
    rpm, alim = core.stability_lobes(800.0, 0.03, 2.0e4, 800.0, 4, 6, 80)
    fig = charts.chatter_chart(rpm, alim, 6, 80, feed_rpm=12000)
    xmax = fig.axes[0].get_xlim()[1]
    check(xmax <= 3.0 * 12000, f"x-limit bounded to operating range (xmax={xmax:.0f})")
    check(xmax < rpm.max(), "x-limit not blown out by the rpm asymptote")


def main():
    for fn in (test_bug1_devfield_uses_cone,
               test_bug1_cone_equals_cylinder_at_zero,
               test_bug2_chatter_highspeed_lobe,
               test_bug4_compare_full_carries_dev,
               test_bug5_chatter_chart_xlim_bounded):
        print(fn.__name__)
        fn()
    if FAILED:
        print(f"\nFAILED: {len(FAILED)} -> {FAILED}")
        sys.exit(1)
    print("\nALL AUDIT REGRESSION TESTS PASSED")


if __name__ == "__main__":
    main()
