#!/usr/bin/env python3
"""Mesh-body collision audit: segment/capsule vs triangle-mesh distance against
brute force and closed forms (offset, piercing, edge), a watertight box, and the
pipeline fixture/machine-body collision check."""
import sys

try:
    import numpy as np
    from bladecam import core, machine
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP mesh_collide ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def pt_tri(p, tri9):
    """Distance from point p to one triangle, via a degenerate (point, r=0)
    capsule through mesh_clearance."""
    cap = np.array([[[p[0], p[1], p[2], p[0], p[1], p[2], 0.0]]])
    return float(core.mesh_clearance(cap, np.asarray(tri9, float).reshape(1, 9), nscan=1)[0])


def brute_pt_tri(p, a, b, c, n=160):
    s = np.linspace(0, 1, n)
    best = np.inf
    for u in s:
        for v in s:
            if u + v <= 1.0:
                q = a + u * (b - a) + v * (c - a)
                best = min(best, np.linalg.norm(p - q))
    return best


def box_mesh(lo, hi):
    """Watertight axis-aligned box as (verts, faces) -> (12,9) triangle array."""
    lo = np.asarray(lo, float); hi = np.asarray(hi, float)
    v = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
                  [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
                  [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
                  [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]]], float)
    f = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
                  [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2],
                  [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]])
    return v, f


def main():
    rng = np.random.default_rng(3)
    a, b, c = np.array([-10., -10, 0]), np.array([10., -8, 0]), np.array([0., 12, 0])
    tri = np.concatenate([a, b, c])

    # 1) point-triangle distance vs brute force (random points around it)
    worst = 0.0
    for _ in range(120):
        p = rng.uniform(-15, 15, 3)
        worst = max(worst, abs(pt_tri(p, tri) - brute_pt_tri(p, a, b, c)))
    check(worst < 0.2, "point-triangle distance matches brute force",
          f"(max err {worst:.2f})")

    # 2) closed forms: a capsule offset above the triangle face reads height - r;
    #    one piercing the face reads <0; one beside it reads the edge distance
    cap = lambda seg: np.asarray(seg, float).reshape(1, 1, 7)
    over = core.mesh_clearance(cap([0, 0, 3, 0, 0, 8, 1.0]), tri.reshape(1, 9), nscan=1)[0]
    check(abs(over - 2.0) < 1e-6, "capsule above face: clearance = height - radius",
          f"({over:.3f})")
    pierce = core.mesh_clearance(cap([0, 0, -2, 0, 0, 2, 0.5]), tri.reshape(1, 9), nscan=1)[0]
    check(pierce < 0, "piercing capsule reports penetration", f"({pierce:.3f})")
    # crossing the PLANE but outside the triangle is NOT a pierce (positive dist)
    out = core.mesh_clearance(cap([40, 0, -2, 40, 0, 2, 0.1]), tri.reshape(1, 9), nscan=1)[0]
    check(out > 0, "plane-crossing outside the triangle is not a collision", f"({out:.2f})")

    # 2b) randomised differential vs brute-force segment-triangle distance, which
    #     exercises ALL closest features (endpoint-face AND the three edge-edge
    #     terms). Segments are offset clear of the triangle so the true distance
    #     is positive and the sampled brute force is a tight upper bound.
    su = np.linspace(0, 1, 70)
    tu, tv = np.meshgrid(su, su)
    barymask = (tu + tv) <= 1.0
    tu, tv = tu[barymask], tv[barymask]

    def brute_seg_tri(p0, p1, a, b, c):
        seg = p0 + np.linspace(0, 1, 90)[:, None] * (p1 - p0)
        tp = a + tu[:, None]*(b - a) + tv[:, None]*(c - a)
        return float(np.min(np.linalg.norm(seg[:, None, :] - tp[None, :, :], axis=2)))

    worst_ee = 0.0; exact_le = True
    for _ in range(60):
        a3, b3, c3 = (rng.uniform(-6, 6, 3) for _ in range(3))
        p0 = rng.uniform(-6, 6, 3) + rng.uniform(2, 6) * rng.normal(size=3)
        p1 = p0 + rng.uniform(-5, 5, 3)
        ex = pt_tri([0, 0, 0], None) if False else core.mesh_clearance(
            np.array([[[*p0, *p1, 0.0]]]), np.concatenate([a3, b3, c3]).reshape(1, 9),
            nscan=1)[0]
        bf = brute_seg_tri(p0, p1, a3, b3, c3)
        if bf > 0.2:                                   # skip near-touching (sampling noise)
            # a lone triangle is not a closed mesh, so the signed sense is
            # undefined -- compare unsigned magnitude (the box cases test sign)
            worst_ee = max(worst_ee, abs(abs(ex) - bf))
            exact_le &= abs(ex) <= bf + 1e-9
    check(worst_ee < 0.1 and exact_le,
          "segment-triangle distance matches brute force over random configs",
          f"(max err {worst_ee:.3f})")

    # 3) watertight box: a point/seg inside is negative; far outside is positive
    bv, bf = box_mesh([-5, -5, -5], [5, 5, 5])
    tris = core.mesh_from_faces(bv, bf)
    inside = core.mesh_clearance(cap([0, 0, 0, 0, 0, 0, 0.0]), tris, nscan=1)[0]
    check(inside <= 0, "capsule centre inside the box collides", f"({inside:.2f})")
    far = core.mesh_clearance(cap([20, 20, 20, 20, 20, 25, 1.0]), tris, nscan=1)[0]
    check(far > 0, "capsule far from the box clears", f"({far:.2f})")

    # 4) swept: a capsule translating THROUGH the box is caught even if both
    #    endpoints' static poses are outside
    swept = np.empty((2, 1, 7))
    swept[0, 0] = [-20, 0, 0, -20, 0, 4, 1.0]
    swept[1, 0] = [20, 0, 0, 20, 0, 4, 1.0]
    cl = core.mesh_clearance(swept, tris, nscan=16)
    check(cl[0] < 0, "swept mesh check catches a pass-through the box", f"({cl[0]:.2f})")

    # 5) pipeline fixture/machine-body collision
    base = compute(Params(strategy="global"))
    check(base["mesh_clearance"] == float("inf"), "no fixture mesh -> inf mesh clearance")
    # a fixture box engulfing the tool region must fail the collision gate
    crash_v, crash_f = box_mesh([-80, -80, -50], [80, 80, 200])
    rc = compute(Params(strategy="global", fixture_mesh=(crash_v, crash_f)))
    check(rc["mesh_clearance"] < 0 and not rc["collision_free"],
          "fixture engulfing the tool fails the collision gate",
          f"({rc['mesh_clearance']:.1f})")
    # a tiny far-away fixture leaves the toolpath collision-free
    far_v, far_f = box_mesh([500, 500, 500], [510, 510, 510])
    rf = compute(Params(strategy="global", fixture_mesh=(far_v, far_f)))
    check(rf["mesh_clearance"] > 0, "distant fixture does not false-trigger",
          f"({rf['mesh_clearance']:.1f})")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nMESH-COLLISION TESTS PASSED")


if __name__ == "__main__":
    main()
