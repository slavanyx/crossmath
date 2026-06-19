#!/usr/bin/env python3
"""Headless end-to-end demo of the BladeCAM core.

Generates a twisted parametric blade, computes the distribution-parameter
(machinability) map, positions a cylindrical cutter per ruling with the
two-point method, and reports the resulting flank deviation. Writes a CSV
of the per-ruling result. No GUI required.
"""
from __future__ import annotations

import numpy as np

from bladecam import core, blade


def main(R: float = 6.0, nv: int = 41):
    a, b = blade.make_blade()
    ap, bp = blade.rail_tangents(a, b)
    nu = a.shape[0]

    delta, vstar, strict = core.distribution(a, b)

    v = np.linspace(0.0, 1.0, nv)
    emax = np.empty(nu)
    for i in range(nu):
        q0, alpha = core.two_point(a[i], ap[i], b[i], bp[i], R)
        pts = (1.0 - v)[:, None] * a[i][None, :] + v[:, None] * b[i][None, :]
        g = core.deviation(q0, alpha, R, pts)
        emax[i] = np.max(np.abs(g))

    finite = delta[np.isfinite(delta)]
    print(f"blade stations           : {nu}")
    print(f"cutter radius R          : {R:.2f} mm")
    print(f"|delta| min (max twist)  : {np.min(np.abs(finite)):.3f} mm")
    print(f"|delta| median           : {np.median(np.abs(finite)):.3f} mm")
    print(f"flank deviation max      : {emax.max()*1000:.1f} micron")
    print(f"flank deviation mean     : {emax.mean()*1000:.1f} micron")
    worst = int(np.argmax(emax))
    print(f"worst ruling index       : {worst}  (|delta|={abs(delta[worst]):.3f} mm)")

    out = "bladecam_result.csv"
    hdr = "u_index,delta,vstar,dev_max_mm"
    u_idx = np.arange(nu)
    np.savetxt(out, np.column_stack([u_idx, delta, vstar, emax]),
               delimiter=",", header=hdr, comments="")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
