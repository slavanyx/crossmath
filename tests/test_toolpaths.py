#!/usr/bin/env python3
"""Point-milling (edge/fillet) and layered roughing toolpaths."""
import sys

try:
    import numpy as np
    from bladecam import pointmill, roughing, blade
    from bladecam.pipeline import edge_finish, rough_channel, Params
except ImportError as e:
    print(f"SKIP toolpaths ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    # --- point milling on a flat patch: exact scallop/offset ---
    nu, nv = 30, 20
    x = np.linspace(0, 40, nu); y = np.linspace(0, 20, nv)
    surf = np.zeros((nu, nv, 3))
    surf[:, :, 0] = x[:, None]; surf[:, :, 1] = y[None, :]
    R_ball, scal = 3.0, 0.01
    r = pointmill.point_mill(surf, R_ball, scal)
    check(r["scallop"] <= scal + 1e-9, "point-mill scallop within budget",
          f"({r['scallop']*1000:.2f} um <= {scal*1000:.0f})")
    check(abs(np.mean(r["cl"][:, :, 2]) - R_ball) < 1e-6,
          "ball centre offset = R along +z normal",
          f"(z {np.mean(r['cl'][:,:,2]):.3f})")
    p_exp = int(np.ceil(20.0 / np.sqrt(8 * R_ball * scal))) + 1
    check(r["n_rows"] == p_exp, "row count from scallop stepover",
          f"({r['n_rows']} vs {p_exp})")

    # REGRESSION (audit 1): non-uniform (fan) patch -- worst-case scallop, not
    # just the average, must stay within budget
    fan = np.zeros((nu, nv, 3))
    for i in range(nu):
        w = 5 + 25 * (i / (nu - 1))
        fan[i, :, 0] = x[i]; fan[i, :, 1] = np.linspace(0, w, nv)
    rf = pointmill.point_mill(fan, R_ball, scal)
    secw = np.sum(np.linalg.norm(np.diff(rf["contact"], axis=1), axis=2), axis=1)
    worst = (secw / (rf["n_rows"] - 1)).max() ** 2 / (8 * R_ball)
    check(worst <= scal + 1e-9, "worst-case scallop within budget on fan patch",
          f"({worst*1000:.2f} um)")

    # REGRESSION (audit 3): a patch whose raw normal points radially INWARD must
    # be flipped so the ball stays OUTSIDE the part (orient='radial').
    th = np.linspace(1.0, 0.0, nu)        # decreasing angle -> inward cross product
    z = np.linspace(0, 15, nv)
    cyl = np.stack([np.outer(30*np.cos(th), np.ones(nv)),
                    np.outer(30*np.sin(th), np.ones(nv)),
                    np.outer(np.ones(nu), z)], axis=2)
    rad_ct = np.linalg.norm(pointmill.point_mill(cyl, 2.0, 0.01,
             orient="radial")["cl"].reshape(-1, 3)[:, :2], axis=1).mean()
    rad_none = np.linalg.norm(pointmill.point_mill(cyl, 2.0, 0.01,
             orient=None)["cl"].reshape(-1, 3)[:, :2], axis=1).mean()
    check(rad_ct > 30.0, "orient='radial' keeps ball outside the part",
          f"(CL radius {rad_ct:.2f} > 30)")
    check(rad_none < 30.0, "raw cross-product would gouge inward (bug fixed)",
          f"(CL radius {rad_none:.2f} < 30)")

    # --- point milling drives off the blade leading edge ---
    ef = edge_finish(Params(), R_ball=2.0, scallop_allow=0.005)
    check(ef["n_rows"] >= 2 and np.all(np.isfinite(ef["cl"])),
          "edge finishing runs on the blade LE patch")

    # --- layered roughing: levels, engagement, volume ---
    a, b = blade.make_blade()
    pitch = 2 * np.pi / 11
    c, s = np.cos(pitch), np.sin(pitch)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    rg = roughing.adaptive_rough(a, b, a @ Rz.T, b @ Rz.T, ap=3.0,
                                 stepover=4.0, feed_mm_min=3000.0)
    check(rg["n_axial"] >= 1 and rg["n_radial"] >= 1, "roughing has layers")
    check(len(rg["passes"]) == rg["n_axial"] * (rg["n_radial"] + 1),
          "pass count = axial x radial")
    check(rg["channel_gap_max_mm"] / rg["n_radial"] <= 4.0 + 1e-9,
          "worst-case radial spacing within stepover (engagement bound)")
    check(rg["removed_volume_mm3"] > 0 and np.isfinite(rg["cycle_s"]),
          "roughing volume/cycle finite",
          f"(vol {rg['removed_volume_mm3']:.0f} mm3, {rg['cycle_s']:.0f} s)")

    # pipeline wrapper
    rc = rough_channel(Params(), ap=3.0)
    check(rc["total_len_mm"] > 0, "pipeline rough_channel runs")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nTOOLPATH TESTS PASSED")


if __name__ == "__main__":
    main()
