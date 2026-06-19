#!/usr/bin/env python3
"""Accuracy/time benchmark across cutter radius and blade twist.

Compares the naive two-point positioning, the analytical single-tangency twist
error estimate (eps ~ R*L^2/(8*delta^2)), and the global optimum -- showing the
optimizer reaches far below the naive bound. No GUI required.
"""
from __future__ import annotations

import numpy as np

from bladecam.pipeline import compute, Params


def naive_floor_um(r, R):
    a, b, delta = r["a"], r["b"], r["delta"]
    L = np.linalg.norm(b - a, axis=1)
    m = np.isfinite(delta) & (np.abs(delta) > 1e-6)
    return float(np.median(R * L[m]**2 / (8.0 * delta[m]**2)) * 1000.0)


def main():
    print("Cutter-radius sweep (twist=0.7)")
    print(f"{'R(mm)':>6} {'two_point(um)':>13} {'naive(um)':>10} "
          f"{'global(um)':>11} {'cycle(s)':>9}")
    for R in (3.0, 4.0, 6.0, 8.0, 10.0):
        rtp = compute(Params(strategy="two_point", R=R))
        rgl = compute(Params(strategy="global", R=R))
        print(f"{R:6.1f} {rtp['dev'].max()*1000:13.1f} "
              f"{naive_floor_um(rtp, R):10.0f} "
              f"{rgl['dev'].max()*1000:11.1f} {rgl['cycle_time_s']:9.2f}")

    print("\nBlade-twist sweep (R=6)")
    print(f"{'twist':>6} {'two_point(um)':>13} {'global(um)':>11} {'cycle(s)':>9}")
    for tw in (0.2, 0.5, 0.9, 1.3):
        rtp = compute(Params(strategy="two_point", twist=tw))
        rgl = compute(Params(strategy="global", twist=tw))
        print(f"{tw:6.2f} {rtp['dev'].max()*1000:13.1f} "
              f"{rgl['dev'].max()*1000:11.1f} {rgl['cycle_time_s']:9.2f}")


if __name__ == "__main__":
    main()
