#!/usr/bin/env python3
"""C-ABI / boundary differential tests (audit angle E).

Every closed-form core binding is reproduced with an INDEPENDENT NumPy
implementation and diffed to machine precision. This is the net against the
highest-severity failure class: a ctypes argtype mismatch or an (n,3) vs (3,n)
layout error corrupts memory silently and would never be caught by a "does it
run" check. A divergence here means the binding and the math have drifted apart.
"""
import sys

try:
    import numpy as np
    from bladecam import core
except ImportError as e:
    print(f"SKIP abi-differential ({e})")
    sys.exit(0)

FAILED = []


def chk(name, got, ref, tol=1e-9):
    got = np.asarray(got, float); ref = np.asarray(ref, float)
    e = float(np.max(np.abs(got - ref))) if got.size else 0.0
    print(f"  {'ok  ' if e < tol else 'FAIL'} {name}  max|diff|={e:.2e}")
    if e >= tol:
        FAILED.append(name)


def _unit(x):
    return x / np.linalg.norm(x, axis=-1, keepdims=True)


def _capped(a, r, h, rad):
    dx = r - rad; dy = abs(a - 0.5*h) - 0.5*h
    return np.hypot(max(dx, 0), max(dy, 0)) + min(max(dx, dy), 0)


def main():
    rng = np.random.default_rng(1)
    R = 4.3
    q0 = rng.normal(size=3); al = rng.normal(size=3); pts = rng.normal(size=(7, 3)) * 5
    ah = al / np.linalg.norm(al); w = pts - q0
    lam = w @ ah; perp = np.linalg.norm(w - np.outer(lam, ah), axis=1)

    chk("deviation", core.deviation(q0, al, R, pts), perp - R)
    g = 0.2
    chk("deviation_cone", core.deviation_cone(q0, al, R, g, pts),
        (perp - (R + lam*np.tan(g))) * np.cos(g))

    # distribution: delta=det(c',e,e')/|e'|^2 with the UNIT director e=unit(b-a)
    nu = 40; u = np.linspace(0, 1, nu)
    a = np.column_stack([30*np.cos(0.6*u), 30*np.sin(0.6*u), 20*u])
    b = np.column_stack([55*np.cos(1.3*u), 55*np.sin(1.3*u), 8 + 20*u])
    delta, vstar, strict = core.distribution(a, b)
    e = _unit(b - a); d = np.empty(nu); vs = np.empty(nu)
    for i in range(nu):
        im = max(0, i-1); ip = min(nu-1, i+1); du = ip - im
        cp = (a[ip]-a[im])/du
        ep = (_unit(b[ip]-a[ip]) - _unit(b[im]-a[im]))/du
        ee = ep @ ep
        d[i] = np.dot(cp, np.cross(e[i], ep))/ee; vs[i] = -cp @ ep/ee
    chk("distribution.delta", delta, d)
    chk("distribution.vstar", vstar, vs)
    chk("distribution.strict", strict, a + vstar[:, None]*e)

    # ik_path table-table (kind 0): A=acos(k), C=atan2(i,-j)
    Q = rng.normal(size=(5, 3)); O = _unit(rng.normal(size=(5, 3)))
    piv = np.array([0., 0, -100.])
    m = core.ik_path(Q, O, piv, kind=0)
    chk("ik_path.A", m[:, 3], np.arccos(np.clip(O[:, 2], -1, 1)))
    Cref = np.arctan2(O[:, 0], -O[:, 1])
    chk("ik_path.C(cos)", np.cos(m[:, 4]), np.cos(Cref))
    chk("ik_path.C(sin)", np.sin(m[:, 4]), np.sin(Cref))

    # stability_lobes a_lim = -1/(2 Kt N Re[G])
    rpm, alim = core.stability_lobes(800, 0.03, 2e7, 800, 4, nlobes=2, nptsper=50)
    j = np.arange(1, 51); r = 1 + j/50
    den = (1 - r*r)**2 + (2*0.03*r)**2; reg = (1/2e7)*(1 - r*r)/den
    chk("stability_lobes.alim", alim[:50], -1/(2*800*4*reg))

    # swept_deviation: min over stations of finite-flute dist - R
    n2 = 6; q0s = rng.normal(size=(n2, 3))*3; als = _unit(rng.normal(size=(n2, 3)))
    Lf = np.full(n2, 15.0); P = rng.normal(size=(10, 3))*8

    def sd_ref(P):
        out = []
        for p in P:
            best = 1e18
            for i in range(n2):
                wv = p - q0s[i]; l = np.clip(wv @ als[i], 0, Lf[i])
                best = min(best, np.linalg.norm(wv - l*als[i]) - R)
            out.append(best)
        return np.array(out)
    chk("swept_deviation", core.swept_deviation(q0s, als, Lf, R, P), sd_ref(P))

    # swept_surface: project onto the deepest-cutting cylinder
    env = core.swept_surface(q0s, als, Lf, R, P)
    mref = np.empty_like(P)
    for k, p in enumerate(P):
        best = 1e18; ib = 0; lb = 0.0
        for i in range(n2):
            wv = p - q0s[i]; l = np.clip(wv @ als[i], 0, Lf[i])
            dd = np.linalg.norm(wv - l*als[i]) - R
            if dd < best:
                best = dd; ib = i; lb = l
        axp = q0s[ib] + lb*als[ib]; rad = p - axp; rn = np.linalg.norm(rad)
        mref[k] = axp + R*rad/rn if rn > 1e-12 else p
    chk("swept_surface", env, mref)

    # holder_clearance: capped-cyl SDF of the holder cylinder only
    base, Lh, Rh = 17.0, 40.0, 8.0

    def hc_ref(P):
        out = []
        for i in range(n2):
            mn = 1e18
            for p in P:
                wv = p - q0s[i]; l = wv @ als[i]
                perp_ = np.linalg.norm(wv - l*als[i])
                mn = min(mn, _capped(l - base, perp_, Lh, Rh))
            out.append(mn)
        return np.array(out)
    chk("holder_clearance", core.holder_clearance(q0s, als, P, Rh, base, Lh), hc_ref(P))

    # tool_clearance: min(flute, holder) capped-cyl SDF
    gap = 2.0; Lfl = 15.0
    def tc_ref(P):
        out = []
        for i in range(n2):
            mn = 1e18
            for p in P:
                wv = p - q0s[i]; l = wv @ als[i]
                perp_ = np.linalg.norm(wv - l*als[i])
                mn = min(mn, min(_capped(l, perp_, Lfl, R),
                                 _capped(l - (Lfl+gap), perp_, Lh, Rh)))
            out.append(mn)
        return np.array(out)
    chk("tool_clearance", core.tool_clearance(q0s, als, P, R, Lfl, Rh, gap, Lh),
        tc_ref(P))

    # deviation_barrel: signed distance to the circular-arc flank profile
    Rb, lamc = 200.0, 12.0
    gb = core.deviation_barrel(q0, al, R, Rb, lamc, pts)
    cr = R - Rb
    gb_ref = np.sqrt((perp - cr)**2 + (lam - lamc)**2) - Rb
    chk("deviation_barrel", gb, gb_ref)

    # swept_deviation with a barrel profile (Rb>0) vs independent ref
    def sdb_ref(P):
        out = []
        for p in P:
            best = 1e18
            for i in range(n2):
                wv = p - q0s[i]; l = np.clip(wv @ als[i], 0, Lf[i])
                pp = np.linalg.norm(wv - l*als[i])
                best = min(best, np.sqrt((pp - cr)**2 + (l - lamc)**2) - Rb)
            out.append(best)
        return np.array(out)
    chk("swept_deviation(barrel)",
        core.swept_deviation(q0s, als, Lf, R, P, Rb=Rb, lamc=lamc), sdb_ref(P))

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nABI DIFFERENTIAL TESTS PASSED")


if __name__ == "__main__":
    main()
