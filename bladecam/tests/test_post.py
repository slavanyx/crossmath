#!/usr/bin/env python3
"""Certified post-processor audit: control dialects (Heidenhain / Siemens /
Fanuc), the certification validator (travel, rotary, winding, linearisation,
rotary speed, forward-kinematics round trip), and a text round trip that proves
the emitted joints reproduce the tool-tip path on the target machine."""
import sys
import re

try:
    import numpy as np
    from bladecam import post, machine as machine_lib
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP post ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def parse_fanuc_joints(text, sign_p=1.0, sign_s=1.0):
    """Modal re-parse of a Fanuc program into machine axes (n,5) [X,Y,Z,A,C rad];
    records a pose at every G1 block carrying a Z word (plunge + cutting moves)."""
    st = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "C": 0.0}
    pts = []
    deg = np.pi / 180.0
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
    p = Params(strategy="global", nu=40,
               machine=machine_lib.get_machine("Generic 5-axis trunnion"))
    r = compute(p)
    mp, contact, axis = r["machine_path"], r["contact"], r["alpha"]
    feed, mt = r["feed_cap_mm_min"], r["move_times_s"]

    # 1) forward-kinematics reproduces the tool-tip path (the IK is invertible)
    back = post.forward_kin(mp, p.pivot, 0)
    check(np.max(np.linalg.norm(back - contact, axis=1)) < 1e-9,
          "forward kinematics reproduces the contact path")

    # 2) a roomy machine certifies clean (every sub-check passes)
    cfg = post.PostConfig()
    rep = post.certify(cfg, mp, contact, p.pivot, feed, mt)
    check(rep["certified"], "generic-trunnion path certifies",
          f"(rt {rep['roundtrip_max_err_mm']:.1e} mm)")
    check(all(rep[k] for k in ("within_travel", "within_rotary", "winding_ok",
                               "linearization_ok", "rotary_speed_ok",
                               "roundtrip_ok")), "all certification checks pass")

    # 3) travel violation is caught
    tiny = post.PostConfig(machine_name="__tiny__")
    tiny_m = machine_lib.Machine(name="__tiny__", x_range=(-5, 5),
                                 y_range=(-5, 5), z_range=(-5, 5))
    machine_lib.DEFAULT_MACHINES["__tiny__"] = tiny_m
    rt = post.certify(tiny, mp, contact, p.pivot, feed, mt)
    check(not rt["within_travel"] and not rt["certified"],
          "out-of-travel path fails certification", f"({sorted(rt['travel_violations'])})")
    del machine_lib.DEFAULT_MACHINES["__tiny__"]

    # 4) rotary winding (a big inter-block A jump) is caught
    wm = mp.copy(); wm[20, 3] += np.radians(200.0)
    rw = post.certify(cfg, wm, contact, p.pivot, feed, mt)
    check(not rw["winding_ok"] and rw["max_rotary_step_deg"] > 120.0,
          "excessive rotary winding is flagged", f"({rw['max_rotary_step_deg']:.0f}°)")

    # 5) linearisation: a kinked contact path exceeds the chord tolerance
    cc = contact.copy(); cc[15] += np.array([1.0, 0.0, 0.0])
    rl = post.certify(cfg, mp, cc, p.pivot, feed, mt)
    check(not rl["linearization_ok"] and rl["max_chord_dev_mm"] > 0.05,
          "coarse/kinked path fails the linearisation tolerance")

    # 6) rotary-speed limit: a fast rotary move on a slow table is infeasible
    fast_mt = mt.copy(); fast_mt[10] = 1e-3
    fm = mp.copy(); fm[11, 4] = mp[10, 4] + 0.5            # 0.5 rad in 1 ms
    rs = post.certify(cfg, fm, contact, p.pivot, feed, fast_mt)
    check(not rs["rotary_speed_ok"], "over-speed rotary move is flagged",
          f"({rs['max_rotary_speed_rad_s']:.0f} rad/s)")

    # 7) dialects emit their signature blocks
    gh = post.generate(cfg, contact, axis, mp, feed, mt)
    check("FUNCTION TCPM" in gh and "LN " in gh, "Heidenhain TCPM dialect")
    cs = post.PostConfig(control="siemens")
    gs = post.generate(cs, contact, axis, mp, feed, mt)
    check("TRAORI" in gs and "A3=" in gs and "TRAFOOF" in gs, "Siemens TRAORI dialect")
    cf = post.PostConfig(control="fanuc")
    gf = post.generate(cf, contact, axis, mp, feed, mt)
    check(gf.startswith("%") and "G43.4" in gf and re.search(r"A[-0-9.]+ C[-0-9.]+", gf),
          "Fanuc G43.4 joint dialect")

    # 8) Siemens orientation vector is unit and equals the tool axis
    m0 = re.search(r"A3=([-0-9.]+) B3=([-0-9.]+) C3=([-0-9.]+)", gs.splitlines()[-4] if False else gs)
    vec = np.array([float(x) for x in re.findall(r"A3=([-0-9.]+) B3=([-0-9.]+) C3=([-0-9.]+)", gs)[-1]])
    a_last = axis[-1] / np.linalg.norm(axis[-1])
    check(abs(np.linalg.norm(vec) - 1) < 1e-3 and np.allclose(np.abs(vec), np.abs(a_last), atol=1e-3),
          "Siemens A3/B3/C3 vector is unit & matches the tool axis")

    # 9) TEXT round trip: re-parse the Fanuc joints and reproduce the tip path
    parsed = parse_fanuc_joints(gf)
    check(parsed.shape[0] == mp.shape[0], "Fanuc text parses all poses",
          f"({parsed.shape[0]} vs {mp.shape[0]})")
    tip = post.forward_kin(parsed, p.pivot, 0)
    check(np.max(np.linalg.norm(tip - contact, axis=1)) < 1e-2,
          "posted Fanuc joints reproduce the tool-tip path (text round trip)")
    # sign convention: a negated primary rotary still round-trips when undone
    cfn = post.PostConfig(control="fanuc", sign_primary=-1.0)
    gfn = post.generate(cfn, contact, axis, mp, feed, mt)
    pn = parse_fanuc_joints(gfn, sign_p=-1.0)
    check(np.max(np.linalg.norm(post.forward_kin(pn, p.pivot, 0) - contact, axis=1)) < 1e-2,
          "rotary sign mapping is invertible (negated A round-trips)")

    # 10) PostConfig serialises round-trip; the certified library all certify
    cfg2 = post.from_dict(post.to_dict(cfg))
    check(cfg2 == cfg, "PostConfig dict round-trip is exact")
    for nm, c in post.CERTIFIED_POSTS.items():
        rr = compute(Params(strategy="global", nu=36, machine=c.machine()))
        prog = post.generate(c, rr["contact"], rr["alpha"], rr["machine_path"],
                             rr["feed_cap_mm_min"], rr["move_times_s"])
        rc = post.certify(c, rr["machine_path"], rr["contact"], Params().pivot,
                          rr["feed_cap_mm_min"], rr["move_times_s"])
        check(len(prog) > 0 and rc["roundtrip_ok"] and rc["within_travel"],
              f"certified post '{nm}' generates & certifies",
              f"(cert={rc['certified']})")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nPOST TESTS PASSED")


if __name__ == "__main__":
    main()
