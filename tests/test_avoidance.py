#!/usr/bin/env python3
"""Collision-AWARE positioning (advanced avoidance) audit: it must (1) leave a
collision-free part untouched and fast, (2) CLEAR a part the swept-optimal axes
drive into a neighbour/hub, WITHOUT blowing the swept-envelope error, and (3)
honestly report residual collisions where no tilt within the swept budget clears
them (a too-tight channel / inflected flank)."""
import sys

try:
    import numpy as np
    from bladecam import blade
    from bladecam.pipeline import compute, Params
    from bladecam.process import ProcessParams
except Exception as e:                              # pragma: no cover
    print(f"SKIP avoidance ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    # 1) a collision-free part is unchanged by avoidance (no adjustments) and the
    #    swept error is identical with avoidance on vs off
    on = compute(Params(strategy="global", nu=36, avoid_collisions=True))
    off = compute(Params(strategy="global", nu=36, avoid_collisions=False))
    check(on["collision_free"] and on["avoidance_adjusted"] == 0,
          "collision-free part: avoidance makes no change",
          f"(adj {on['avoidance_adjusted']})")
    check(abs(on["swept_overcut"] - off["swept_overcut"]) < 1e-9,
          "collision-free part: swept error untouched by avoidance")

    # 2) a part whose swept-optimal axes collide gets CLEARED, swept preserved.
    # A tall, leaned, wrapped blade tilts the tool into a neighbour; avoidance
    # must restore positive clearance without wrecking the machined surface.
    a, b = blade.make_complex_blade(nu=60, rh0=34, rh1=50, rs0=44, rs1=58,
                                    z_span=30, z_offset=8, wrap=0.5, twist=0.45,
                                    lean=0.3)
    pr = ProcessParams(tool_dia=7, holder_dia=11, flute_len=32, holder_len=22)
    kw = dict(strategy="global", rails=(a, b), process=pr, n_blades=7, R=3.5,
              swept_weight=0.6)
    bad = compute(Params(avoid_collisions=False, **kw))
    good = compute(Params(avoid_collisions=True, **kw))
    check(not bad["collision_free"] and bad["min_clearance"] < -1.0,
          "setup: swept-optimal axes collide", f"(min {bad['min_clearance']:.2f})")
    check(good["collision_free"],
          "avoidance CLEARS the collision the swept-optimal path had",
          f"({bad['min_clearance']:.2f} -> {good['min_clearance']:.2f} mm, "
          f"adjusted {good['avoidance_adjusted']})")
    check(good["swept_overcut"] <= bad["swept_overcut"] + 0.12,
          "avoidance keeps the swept-envelope error within budget",
          f"({bad['swept_overcut']*1000:.0f} -> {good['swept_overcut']*1000:.0f} um)")

    # 3) an inflected (S-warp) flank cannot be cleared by ANY cylinder tilt -- the
    #    avoidance must report residual infeasible rulings, not silently gouge.
    a2, b2 = blade.make_complex_blade(nu=60, rh0=36, rh1=52, rs0=48, rs1=64,
                                      z_span=20, z_offset=6, wrap=0.55, twist=0.7,
                                      warp=1.4, lean=0.3)
    hard = compute(Params(strategy="global", rails=(a2, b2), avoid_collisions=True,
                          process=ProcessParams(tool_dia=8, holder_dia=14,
                                                flute_len=38, holder_len=26),
                          n_blades=9, R=4.0, swept_weight=0.6))
    check(len(hard["avoidance_infeasible"]) > 0 and not hard["collision_free"],
          "inflected flank: avoidance honestly reports it cannot be cleared",
          f"({len(hard['avoidance_infeasible'])} infeasible rulings)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nAVOIDANCE TESTS PASSED")


if __name__ == "__main__":
    main()
