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
    from bladecam import core, post, features, machine as ml
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

    # 5) STOCK dexel surface <-> swept_surface envelope: two INDEPENDENT machined-
    #    surface computations (ray/cylinder carving vs envelope projection) must
    #    place the machined surface in the same place for a cylinder tool.
    R = 5.0; Lf = np.array([20.0]); q0 = np.array([[0., 0, 0]]); al = np.array([[0., 0, 1.]])
    gap = 3.0; th = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    P = np.column_stack([(R + gap) * np.cos(th), (R + gap) * np.sin(th), np.full(16, 10.0)])
    mp = core.swept_surface(q0, al, Lf, R, P)
    rad = np.hypot(mp[:, 0], mp[:, 1])
    dvec = -P / np.linalg.norm(P, axis=1, keepdims=True); dvec[:, 2] = 0
    dvec /= np.linalg.norm(dvec, axis=1, keepdims=True)
    _, fc = core.dexel_carve(q0, al, R, Lf, P, dvec, np.full(16, gap + 2 * R))
    check(np.allclose(rad, R, atol=1e-2) and np.allclose(fc, gap, atol=1e-2),
          "dexel carve and swept_surface agree on the machined surface")

    # 6) MACHINE swap <-> reachability <-> post certify: both use the SAME envelope
    #    check, so a tiny machine must flag the SAME axes in reachability and in
    #    the certification report.
    tiny = ml.Machine(name="tiny", x_range=(-5, 5), y_range=(-5, 5),
                      z_range=(-5, 5), a_range=(-0.2, 0.2))
    viol = ml.reachability(tiny, r["machine_path"])
    rep = post.certify(post.PostConfig(), r["machine_path"], r["contact"], p.pivot,
                       r["feed_cap_mm_min"], r["move_times_s"], machine=tiny)
    cert_axes = set(rep["travel_violations"]) | set(rep["rotary_violations"])
    check(len(viol) > 0 and set(viol) == cert_axes and not rep["certified"],
          "machine swap: reachability & post-certify flag the same axes",
          f"({sorted(viol)})")

    # 7) A/C UNWRAP <-> winding alarm: the pipeline unwraps the rotary axes, so the
    #    posted per-block rotary step is small (no spurious 2pi jump), and the
    #    certify winding metric equals the actual max step.
    m = r["machine_path"]
    step_deg = float(np.degrees(np.abs(np.diff(m[:, 3:5], axis=0))).max())
    rep2 = post.certify(post.PostConfig(), m, r["contact"], p.pivot,
                        r["feed_cap_mm_min"], r["move_times_s"])
    check(step_deg < 90.0 and abs(rep2["max_rotary_step_deg"] - step_deg) < 1e-6,
          "A/C unwrap keeps rotary steps small; certify winding == the max step",
          f"({step_deg:.1f}°)")

    # 8) BARREL tool identical across stages: a point ON the barrel surface reads
    #    ~0 in deviation_barrel AND swept_deviation (same tool model everywhere).
    Rb, lamc = 200.0, 12.0; lam = 8.0
    perp = R - Rb + np.sqrt(Rb**2 - (lam - lamc) ** 2)
    Pb = np.array([[perp, 0.0, lam]])
    g1 = core.deviation_barrel(np.array([0., 0, 0]), np.array([0., 0, 1.]), R, Rb, lamc, Pb)[0]
    g2 = core.swept_deviation(np.array([[0., 0, 0]]), np.array([[0., 0, 1.]]),
                              np.array([20.0]), R, Pb, Rb=Rb, lamc=lamc)[0]
    check(abs(g1) < 1e-6 and abs(g2) < 1e-6,
          "barrel surface point reads ~0 in deviation_barrel AND swept_deviation",
          f"({g1:.1e}, {g2:.1e})")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nINTEGRATION (inter-stage) TESTS PASSED")


if __name__ == "__main__":
    main()
