#!/usr/bin/env python3
"""Persistent interval-dexel stock + rest-machining audit: carve correctness vs
analytic volume and vs the validated dexel_carve oracle, interior (middle)
removal (the interval dexel's reason to exist), sequential carry-across-ops, and
the rest-machining inequality (finishing after roughing removes only the rest)."""
import sys
import math

try:
    import numpy as np
    from bladecam import stock, core
    from bladecam.pipeline import rest_machining, Params
except ImportError as e:
    print(f"SKIP stock ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    # 1) block carve vs analytic cylinder volume
    s = stock.block([-10, -10, 0], [10, 10, 20], nx=120, ny=120)
    v0 = s.volume()
    rem = s.carve(np.array([[0., 0, 0]]), np.array([[0., 0, 1]]), 5.0, 20.0)
    check(abs(rem - math.pi * 25 * 20) / (math.pi * 25 * 20) < 0.01,
          "block carve matches analytic cylinder volume",
          f"({rem:.1f} vs {math.pi*25*20:.1f})")
    check(abs(s.removed_total() - rem) < 1e-9 and s.volume() < v0,
          "removed_total tracks the carve; volume drops")

    # 2) differential vs the validated dexel_carve removed-length oracle
    rng = np.random.default_rng(1)
    q0 = rng.uniform(-5, 5, (6, 3)); al = rng.uniform(-1, 1, (6, 3))
    R = 3.0; Lf = np.full(6, 15.0)
    orig = rng.uniform(-8, 8, (60, 3))
    dirv = rng.uniform(-1, 1, (60, 3)); dirv /= np.linalg.norm(dirv, axis=1, keepdims=True)
    rem_len, _ = core.dexel_carve(q0, al, R, Lf, orig, dirv, np.full(60, 1e9))
    rlo, rhi, rn = core.dexel_removed_intervals(q0, al, R, Lf, orig, dirv, maxseg=32)
    mine = np.array([sum(rhi[r, j] - rlo[r, j] for j in range(rn[r])) for r in range(60)])
    check(float(np.max(np.abs(mine - rem_len))) < 1e-9,
          "removed-interval lengths match the dexel_carve oracle exactly")
    # intervals are disjoint and ascending
    okint = all(all(rlo[r, j] < rhi[r, j] for j in range(rn[r])) and
                all(rhi[r, j] <= rlo[r, j+1] for j in range(rn[r]-1)) for r in range(60))
    check(okint, "removed intervals are ascending and disjoint")

    # 3) INTERIOR removal -- the interval dexel's reason to exist. A horizontal
    #    tool removes the MIDDLE of a vertical ray, leaving two solid intervals
    #    (a height field would wrongly remove nothing or the whole top).
    one = stock.Stock(np.array([[0., 0, 0]]), np.array([[0., 0, 1.]]),
                      np.array([20.0]), cell=1.0)
    one.carve(np.array([[-10., 0, 10]]), np.array([[1., 0, 0]]), 3.0, 20.0)
    segs = one.solid[0]
    check(len(segs) == 2 and abs(segs[0][1] - 7.0) < 1e-6
          and abs(segs[1][0] - 13.0) < 1e-6,
          "interior cut leaves two solid intervals", f"({segs})")
    check(abs(one.volume() - 14.0) < 1e-6, "interior-cut remaining length correct")

    # 4) sequential carry-across-ops: two overlapping tools remove their UNION,
    #    monotone, and less than the sum of the two separate volumes
    s2 = stock.block([-10, -10, 0], [10, 10, 20], nx=80, ny=80)
    r1 = s2.carve(np.array([[0., 0, 0]]), np.array([[0., 0, 1]]), 5.0, 20.0)
    vmid = s2.volume()
    r2 = s2.carve(np.array([[4., 0, 0]]), np.array([[0., 0, 1]]), 5.0, 20.0)
    check(s2.volume() <= vmid and r2 > 0, "second carve is monotone")
    check(r2 < math.pi * 25 * 20 * 0.999,
          "overlapping second carve removes only the union increment",
          f"({r2:.0f} < {math.pi*25*20:.0f})")
    check(abs(s2.removed_total() - (r1 + r2)) < 1e-9,
          "removed_total == sum of per-op removals (bookkeeping)")

    # 5) a tool that misses leaves the stock untouched
    s3 = stock.block([-10, -10, 0], [10, 10, 20], nx=30, ny=30)
    vb = s3.volume()
    miss = s3.carve(np.array([[100., 100, 0]]), np.array([[0., 0, 1]]), 2.0, 20.0)
    check(miss == 0.0 and s3.volume() == vb, "a missing tool removes nothing")

    # 6) rest-machining: finishing AFTER roughing removes only the rest material
    r = rest_machining(Params(strategy="global", nu=40))
    v0, vr, vf = r["stock_volume_mm3"], r["after_rough_mm3"], r["after_finish_mm3"]
    check(v0 >= vr >= vf - 1e-6, "stock volume is monotone through rough->finish",
          f"({v0:.0f} -> {vr:.0f} -> {vf:.0f})")
    check(r["rough_removed_mm3"] > 0 and r["finish_removed_mm3"] >= 0,
          "roughing clears the bulk; finishing removes the rest")
    check(r["finish_removed_mm3"] <= r["finish_from_raw_mm3"] + 1e-6
          and r["rest_fraction"] < 1.0,
          "finish-after-rough removes <= finish-from-raw (rest material)",
          f"(rest {r['finish_removed_mm3']:.0f} vs raw {r['finish_from_raw_mm3']:.0f}, "
          f"frac {r['rest_fraction']:.2f})")
    check(abs((r["rough_removed_mm3"] + r["finish_removed_mm3"]) - (v0 - vf)) < 1e-6,
          "rest-machining volume bookkeeping is exact")
    check(r["rest_field"].shape[0] > 0 and np.all(np.isfinite(r["rest_field"])),
          "per-ray rest-material field returned")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nSTOCK / REST-MACHINING TESTS PASSED")


if __name__ == "__main__":
    main()
