#!/usr/bin/env python3
"""Inter-stage (integration) consistency audit -- the SEAMS between pipeline
stages, where each stage is individually correct but their hand-off can drift:

  1. optimize -> IK -> post -> re-parse -> forward-kin reproduces the OPTIMISED
     tool-tip path (the whole CAM chain is a faithful round trip).
  2. forces -> feed -> TOPP: the time-optimal tip feed never exceeds the
     mechanistic (force/deflection/power) feed cap, and a heavier cut lowers it.
  3. recognition -> trim -> fillet: the flank trim offset and the fillet's
     flank-tangent contact COINCIDE, so the flank pass and the fillet pass meet
     with no uncut gap and no double-cut (closed-form on a 90 deg corner).
  4. geometry -> optimize -> envelope: the machined-surface error scales with the
     part (scale invariance across the geometry/optimiser seam).
"""
import sys
import re
import math

try:
    import numpy as np
    from bladecam import core, post, features
    from bladecam.pipeline import compute, fillet_machining, Params
except ImportError as e:
    print(f"SKIP integration ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def parse_fanuc(text, sign_p=1.0, sign_s=1.0):
    st = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "C": 0.0}
    pts = []; deg = math.pi / 180.0
    for line in text.splitlines():
        if not (line.startswith("G0") or line.startswith("G1")):
            continue
        toks = dict(re.findall(r"([XYZAC])(-?\d+\.?\d*)", line))
        for k, v in toks.items():
            st[k] = float(v)
        if line.startswith("G1") and "Z" in toks:
            pts.append([st["X"], st["Y"], st["Z"],
                        st["A"] * deg / sign_p, st["C"] * deg / sign_s])
    return np.array(pts)


def main():
    p = Params(strategy="global", nu=44)
    r = compute(p)

    # 1) FULL CHAIN round trip: optimise -> IK -> Fanuc post -> re-parse joints
    #    -> forward kinematics must reproduce the optimised contact path
    cfg = post.PostConfig(control="fanuc")
    prog = post.generate(cfg, r["contact"], r["alpha"], r["machine_path"],
                         r["feed_cap_mm_min"], r["move_times_s"])
    joints = parse_fanuc(prog)
    back = post.forward_kin(joints, p.pivot, 0)
    err = float(np.max(np.linalg.norm(back - r["contact"], axis=1)))
    check(joints.shape[0] == r["contact"].shape[0] and err < 5e-3,
          "optimise->IK->post->reparse->forward-kin reproduces the tool path",
          f"(max {err*1000:.2f} µm)")

    # 2) FORCES -> FEED -> TOPP: with a force-limited process the mechanistic cap
    #    BINDS (below the nominal feed), the TOPP tip feed respects it, and a
    #    heavier cut lowers it further -- the whole force->feed->schedule coupling
    from bladecam.process import ProcessParams
    pf = Params(strategy="global", nu=44,
                process=ProcessParams(ap=16.0, ae=11.0, max_force_N=700.0))
    rf = compute(pf)
    cap = rf["feed_cap_mm_min"]
    nominal = pf.process.nominal_feed_mm_min()
    check(cap < nominal,
          "the force-limited mechanistic cap binds below the nominal feed",
          f"(cap {cap:.0f} < nominal {nominal:.0f})")
    n = rf["aprof"].shape[0]; ds = 1.0 / (n - 1)
    feed = np.sqrt(np.clip(rf["aprof"], 0.0, None)) * np.gradient(rf["seglen"], ds) * 60.0
    check(float(feed.max()) <= cap * 1.02,
          "TOPP tip feed stays within the force-limited feed cap",
          f"({feed.max():.0f} <= {cap:.0f})")
    heavier = compute(Params(strategy="global", nu=44,
                             process=ProcessParams(ap=22.0, ae=12.0, max_force_N=700.0)))
    check(heavier["feed_cap_mm_min"] < cap
          and heavier["cut_force_peak_N"] > rf["cut_force_peak_N"],
          "a heavier cut raises the force and lowers the feed cap",
          f"(cap {cap:.0f}->{heavier['feed_cap_mm_min']:.0f})")

    # 3) TRIM <-> FILLET coverage: on a 90 deg corner the flank-trim offset and
    #    the fillet's flank-tangent contact coincide exactly -> the flank pass
    #    bottom meets the fillet pass top with no gap and no overlap.
    nu = 6
    a = np.column_stack([np.linspace(0, 10, nu), np.zeros(nu), np.zeros(nu)])
    b = a + np.array([0, 8.0, 0])                  # ruling +y (in-flank, perp to edge)
    nf = np.tile([0, 0, 1.], (nu, 1))              # flank = z=0 plane, normal +z
    nh = np.tile([0, 1, 0.], (nu, 1))              # hub  = y=0 plane, normal +y
    r_f = 3.0
    a_trim, _ = features.trim_root_fillet(a, b, r_f)          # flank starts r_f up
    fp = features.fillet_finish(a, b, nf, nh, r_f, 1.5, n_across=4)
    flank_tangent = fp["contacts"][0]              # t=0 contact = flank-tangent line
    check(np.allclose(a_trim, flank_tangent, atol=1e-9),
          "trim offset == fillet flank-tangent (flank & fillet passes meet, no gap)",
          f"(max {np.max(np.abs(a_trim-flank_tangent)):.2e})")

    # 4) GEOMETRY -> OPTIMISE -> ENVELOPE scale invariance: scaling the blade and
    #    the cutter by s scales the machined-surface error by s (same shape)
    a0, b0 = r["a"], r["b"]
    s = 2.0
    big = compute(Params(strategy="global", R=p.R * s, rails=(a0 * s, b0 * s)))
    base = compute(Params(strategy="global", R=p.R, rails=(a0, b0)))
    if base["swept_overcut"] > 1e-9:
        ratio = big["swept_overcut"] / base["swept_overcut"]
        check(abs(ratio - s) < 0.05 * s, "machined-surface error scales with the part",
              f"(ratio {ratio:.3f} vs {s})")
    else:
        check(big["swept_overcut"] < 1e-6, "developable case stays ~0 at both scales")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nINTEGRATION (inter-stage) TESTS PASSED")


if __name__ == "__main__":
    main()
