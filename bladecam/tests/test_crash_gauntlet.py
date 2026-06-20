#!/usr/bin/env python3
"""CRASH GAUNTLET -- adversarial near-miss collision scenarios that the production
collision checks MUST catch, each with an independent ground-truth oracle.

The oracle for a swept check is the SAME core routine fed the dense, joint-space
forward-kinematics poses the machine actually traverses between two stations
(linear interpolation of the 5 joints, FK'd to the part frame, nsub very large).
The production call gets only the two stations and interpolates internally; where
the two disagree is a real gap (the machine crashes where the software said
clear). Scenarios:

  G1  joint-FK arc vs part-frame lerp: a far-up tool point swings on the real
      rotary arc into an obstacle the straight-chord lerp misses        (#1)
  G2  fast move with a deep narrow dive between the coarse scan samples  (#8)
  G3  the hub/shroud floor is hit -- must be in the obstacle world       (#3)
  G4  the holder swings into the blade being cut BETWEEN stations        (#4)

Run standalone: prints ok/FAIL per scenario. Scenarios for not-yet-built features
are expected to FAIL until their fix lands (that failure is the proof of the gap).
"""
import sys
import os

try:
    import numpy as np
    from bladecam import core, machine as ml
except Exception as e:                              # pragma: no cover
    print(f"SKIP crash_gauntlet ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


# ---- forward kinematics (machine joints -> part-frame contact point + axis) ----
def fk_pose(mrow, pivot, kind):
    X, Y, Z, A, C = mrow
    RR = ml._rotz(C) @ ml._rotx(A)
    O = RR @ np.array([0.0, 0.0, 1.0])
    Q = (np.array([X, Y, Z]) if kind == 1
         else RR @ (np.array([X, Y, Z]) - pivot) + pivot)
    return Q, O


def dense_fk(m, pivot, kind, nsub):
    """Dense part-frame (q0, alpha) by FK of joint-LINEAR interpolation between
    consecutive stations -- the path the machine truly runs."""
    q, a = [], []
    for i in range(len(m) - 1):
        for s in range(nsub):
            t = s / nsub
            Q, O = fk_pose((1 - t) * m[i] + t * m[i + 1], pivot, kind)
            q.append(Q); a.append(O)
    Q, O = fk_pose(m[-1], pivot, kind)
    q.append(Q); a.append(O)
    return np.array(q), np.array(a)


SEGR = np.array([4.0, 10.0, 30.0])             # flute / holder / spindle radii
SEGLO = np.array([0.0, 25.0, 90.0])
SEGHI = np.array([20.0, 65.0, 210.0])


def assembly_min(q0, alpha, pts, nscan, plane_pt=None):
    return core.assembly_clearance(q0, alpha, SEGR, SEGLO, SEGHI, pts,
                                   plane_pt=plane_pt,
                                   plane_n=np.array([0., 0, 1.]),
                                   nscan=nscan).min()


def oracle_min(m, pivot, kind, pts, plane_pt=None, nsub=2000):
    qd, ad = dense_fk(m, pivot, kind, nsub)
    return assembly_min(qd, ad, pts, 1, plane_pt)


def holder_clearance_swept_call(fn, q0, alpha, pts, hR, base, hL):
    return fn(q0, alpha, pts, hR, base, hL).min()


def swept_fk(m, pivot, kind, pts, plane_pt=None):
    """Production joint-FK + guaranteed sweep (#1 + #8). Falls back to the
    part-frame-lerp behaviour while unimplemented, so the gauntlet shows the gap
    until the real API lands."""
    fn = getattr(core, "assembly_clearance_fk", None)
    if fn is None:
        qS = np.array([fk_pose(m[i], pivot, kind)[0] for i in range(len(m))])
        aS = np.array([fk_pose(m[i], pivot, kind)[1] for i in range(len(m))])
        return assembly_min(qS, aS, pts, 8, plane_pt)
    return fn(m, pivot, kind, SEGR, SEGLO, SEGHI, pts,
              plane_pt=plane_pt, plane_n=np.array([0., 0, 1.])).min()


def main():
    pivot = np.array([0.0, 0.0, 0.0])
    kind = 0

    # ---- G1: joint-FK rotary arc vs part-frame straight-chord lerp ----------
    # Table-table: a C swing puts the CONTACT point (radius 120 from the C axis)
    # on a wide arc. Mid-move the contact bulges ~21 mm off the chord the part-
    # frame lerp follows. Put a thin obstacle at the flute on that arc bulge: the
    # true (FK) sweep drives the flute through it; the chord clears by ~17 mm.
    mA = np.array([[120.0, 0.0, 0.0, 0.0, -0.6],
                   [120.0, 0.0, 0.0, 0.0,  0.6]])
    qd, ad = dense_fk(mA, pivot, kind, 400)
    fmid = qd + 0.5 * SEGHI[0] * ad                         # flute mid-height track
    midc = fmid[len(fmid) // 2]                             # its arc midpoint
    chordc = 0.5 * (fmid[0] + fmid[-1])
    bulge = np.linalg.norm(midc - chordc)
    obs = midc + 0.001 * np.random.default_rng(0).standard_normal((20, 3))
    o_g1 = oracle_min(mA, pivot, kind, obs)
    qS = np.array([fk_pose(mA[0], pivot, kind)[0], fk_pose(mA[1], pivot, kind)[0]])
    aS = np.array([fk_pose(mA[0], pivot, kind)[1], fk_pose(mA[1], pivot, kind)[1]])
    p_g1 = assembly_min(qS, aS, obs, nscan=8)               # current: part-lerp
    p_g1_fk = swept_fk(mA, pivot, kind, obs)                # target: joint-FK
    check(bulge > 5.0, "G1 setup: real arc bulges off the chord", f"({bulge:.1f} mm)")
    check(o_g1 < 0 and p_g1 > 0,
          "G1 gap: chord-lerp clears, the true FK sweep collides",
          f"(lerp {p_g1:.1f} > 0, oracle {o_g1:.2f} < 0)")
    check(p_g1_fk < 0, "G1 production: joint-FK swept check catches it",
          f"(fk {p_g1_fk:.2f} vs oracle {o_g1:.2f} mm)")

    # ---- G2: curved FK path makes distance-to-obstacle BIMODAL ---------------
    # A big C swing: the real flute track is an arc that approaches an obstacle,
    # recedes, and the single golden bracket on the straight-chord seg_sdf lands
    # in the wrong basin. Even t-exact refinement of the WRONG (chord) path gives
    # the wrong depth. Only sweeping the dense FK path (#1) with a guaranteed
    # substep count (#8, which also tames the bimodality) matches the oracle.
    mB = np.array([[120.0, 0.0, 0.0, 0.0, -0.9],
                   [120.0, 0.0, 0.0, 0.0,  0.9]])
    qd2, ad2 = dense_fk(mB, pivot, kind, 2000)
    fmidB = qd2 + 0.5 * SEGHI[0] * ad2                      # flute mid track (arc)
    # obstacle on the arc at t=0.35 (off the chord, in a basin the chord-golden
    # misses because the chord's own nearest approach is elsewhere)
    kk = int(0.35 * (len(fmidB) - 1))
    obs2 = (fmidB[kk] + np.array([0.0, 0.0, 0.0]))[None, :]
    o_g2 = oracle_min(mB, pivot, kind, obs2)
    qS2 = np.array([fk_pose(mB[0], pivot, kind)[0], fk_pose(mB[1], pivot, kind)[0]])
    aS2 = np.array([fk_pose(mB[0], pivot, kind)[1], fk_pose(mB[1], pivot, kind)[1]])
    p_g2 = assembly_min(qS2, aS2, obs2, nscan=8)
    p_g2_ca = swept_fk(mB, pivot, kind, obs2)               # target: FK + guaranteed
    check(o_g2 < -0.5 and abs(p_g2 - o_g2) > 1.0,
          "G2 gap: the chord sweep mis-reports the curved-path collision depth",
          f"(chord {p_g2:.2f} vs oracle {o_g2:.2f} mm)")
    check(abs(p_g2_ca - o_g2) < 0.2,
          "G2 production: FK swept depth matches the dense oracle",
          f"(fk {p_g2_ca:.2f} vs oracle {o_g2:.2f} mm)")

    # ---- G3: the hub floor must be a collision obstacle (pipeline level) ------
    try:
        from bladecam.pipeline import compute, Params
        from bladecam.process import ProcessParams
        # a deep, stubby tool on a normal blade: the holder/spindle reaches below
        # the blade base toward the hub floor. If the hub is not an obstacle the
        # pipeline reports collision-free even though the assembly is in the hub.
        # a normal tool clears the hub (no false positive)
        rn = compute(Params(strategy="global", nu=24))
        check(rn.get("hub_clearance", -1) > 0 and rn["collision_free"],
              "G3a: a normal tool clears the hub (no false positive)",
              f"(hub {rn.get('hub_clearance', float('nan')):.1f} mm)")
        # a stubby deep tool drives the holder/spindle into the hub floor
        r = compute(Params(strategy="global", nu=24,
                           process=ProcessParams(flute_len=10.0, holder_len=40.0,
                                                 holder_dia=24.0)))
        check("hub_clearance" in r and r["hub_clearance"] < 0
              and not r["collision_free"],
              "G3b: a deep stubby tool is caught hitting the hub",
              f"(hub {r['hub_clearance']:.1f} mm)")
    except Exception as e:
        check(False, "G3: pipeline hub clearance", f"({e})")

    # ---- G4: holder swings into the blade being cut between stations ----------
    # Pure translation, vertical axis: the holder is a vertical cylinder at x=0
    # then x=40. An obstacle at x=20 (mid-move) sits >hR from BOTH station
    # cylinders, so the per-station holder_clearance clears it, but the swept
    # holder passes straight through. The holder check must be swept (#4).
    q0h = np.array([[0.0, 0.0, 0.0], [40.0, 0.0, 0.0]])
    az = np.array([0.0, 0.0, 1.0])
    alh = np.array([az, az])
    base, hR, hL = 12.0, 6.0, 30.0
    bl = np.array([[20.0, 0.0, base + hL * 0.5]]) \
        + 0.001 * np.random.default_rng(1).standard_normal((15, 3))
    per_station = core.holder_clearance(q0h, alh, bl, hR, base, hL).min()
    dense_q = np.array([[40.0 * s / 399.0, 0.0, 0.0] for s in range(400)])
    dense_a = np.tile(az, (400, 1))
    swept_holder = core.assembly_clearance(
        dense_q, dense_a, np.array([hR]), np.array([base]),
        np.array([base + hL]), bl, nscan=1).min()
    holder_prod = getattr(core, "holder_clearance_swept", None)
    p_g4 = (holder_clearance_swept_call(holder_prod, q0h, alh, bl, hR, base, hL)
            if holder_prod else per_station)
    check(swept_holder < 0 and per_station > 0,
          "G4 gap: per-station holder clears, the swept holder collides",
          f"(per-station {per_station:.1f} > 0, swept {swept_holder:.2f} < 0)")
    check(p_g4 < 0, "G4 production: swept holder check catches the mid-move swing",
          f"(prod {p_g4:.2f} vs swept {swept_holder:.2f} mm)")

    # ---- G5: a thin tool threads between point-cloud obstacle samples ---------
    # A neighbour flank as a COARSE point cloud (10 mm spacing) vs the same flank
    # as a triangle mesh. A thin flute (r=2) crossing the surface BETWEEN samples
    # is missed by the point cloud (clears +3) but the mesh, being continuous,
    # catches it exactly (#2: obstacle-side sampling gap).
    nvc = 5; nuc = 5
    grid = np.array([[[0.0, y, z] for y in np.linspace(-20, 20, nvc)]
                     for z in np.linspace(0, 40, nuc)])
    pts5 = grid.reshape(-1, 3)
    tris5 = core.tris_from_grid(grid)
    Rf = 2.0
    capf = np.zeros((2, 1, 7))
    capf[0, 0] = [0.0, 5.0, 15.0, 0.0, 5.0, 25.0, Rf]      # axis pierces the sheet
    capf[1, 0] = capf[0, 0]
    p_pts = core.assembly_clearance(np.array([[0.0, 5, 15.0], [0.0, 5, 15.0]]),
                                    np.array([[0.0, 0, 1.0], [0.0, 0, 1.0]]),
                                    np.array([Rf]), np.array([0.0]),
                                    np.array([10.0]), pts5, nscan=4).min()
    p_mesh = core.mesh_clearance(capf, tris5, nscan=4, signed=False).min()
    check(p_pts > 0 and p_mesh < 0,
          "G5 gap+fix: point cloud misses the thread, the mesh catches it",
          f"(points {p_pts:.1f} > 0, mesh {p_mesh:.1f} < 0)")

    # ---- G6: approach / retract moves are collision-checked -------------------
    try:
        from bladecam.pipeline import compute, Params
        r6 = compute(Params(strategy="global", nu=24))
        check("approach_clearance" in r6 and "retract_clearance" in r6,
              "G6: pipeline reports approach & retract clearances",
              f"(app {r6.get('approach_clearance', float('nan')):.1f}, "
              f"ret {r6.get('retract_clearance', float('nan')):.1f} mm)")
        check(np.isfinite(r6.get("approach_clearance", np.nan))
              and np.isfinite(r6.get("retract_clearance", np.nan)),
              "G6: lead-in/out clearances are finite (the moves were generated)")
    except Exception as e:
        check(False, "G6: approach/retract clearance", f"({e})")

    # ---- G7: the blade-INDEX (pass-linking) move is collision-checked ---------
    try:
        from bladecam.pipeline import compute, Params
        rb = compute(Params(strategy="global", nu=24, n_blades=11))
        rs = compute(Params(strategy="global", nu=24, n_blades=1))
        check("index_clearance" in rb and np.isfinite(rb["index_clearance"]),
              "G7: multi-blade wheel reports a finite index-move clearance",
              f"({rb.get('index_clearance', float('nan')):.1f} mm)")
        check(rs["index_clearance"] == float("inf"),
              "G7: a single blade has no index move (inf)")
    except Exception as e:
        check(False, "G7: index-move clearance", f"({e})")

    # ---- G8: the table is an EXACT mesh, not a threadable point cloud ----------
    # A thin flute dips just below the table top between the coarse point-cloud
    # samples: the point cloud clears it, the continuous table mesh catches it.
    try:
        from bladecam import machine as _ml
        from bladecam.machine import structure_obstacles, table_mesh
        mc = _ml.get_machine("Generic 5-axis trunnion")
        mz = 0.0
        tpts = structure_obstacles(mc, mz)                 # coarse point cloud
        ttris = table_mesh(mc, mz)                         # exact mesh
        # flute tip 2 mm below the table top, at a point between cloud samples
        rr = 0.55 * mc.table_radius
        tip = np.array([rr * np.cos(0.13), rr * np.sin(0.13), -2.0])
        capt = np.zeros((2, 1, 7))
        capt[0, 0] = [tip[0], tip[1], -2.0, tip[0], tip[1], 18.0, 3.0]
        capt[1, 0] = capt[0, 0]
        c_pts = core.assembly_clearance(
            np.array([[tip[0], tip[1], -2.0], [tip[0], tip[1], -2.0]]),
            np.array([[0.0, 0, 1.0], [0.0, 0, 1.0]]), np.array([3.0]),
            np.array([0.0]), np.array([20.0]), tpts, nscan=4).min()
        c_mesh = core.mesh_clearance(capt, ttris, nscan=4, signed=False).min()
        check(c_pts > 0 and c_mesh < 0,
              "G8 gap+fix: table point cloud misses the dip, the mesh catches it",
              f"(points {c_pts:.1f} > 0, mesh {c_mesh:.1f} < 0)")
    except Exception as e:
        check(False, "G8: table mesh", f"({e})")

    if FAILED:
        print(f"\nGAUNTLET GAPS (expected until fixed): {FAILED}")
        sys.exit(1)
    print("\nCRASH GAUNTLET PASSED")


if __name__ == "__main__":
    main()
