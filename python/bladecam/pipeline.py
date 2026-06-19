"""End-to-end BladeCAM pipeline: geometry -> positioning -> kinematics ->
time-optimal feed -> cycle time, plus a neighbour-blade collision check.

`compute(params)` returns a single results dict consumed by both the headless
demo and the GUI, so the GUI stays a thin presentation layer.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from . import core, blade, optimize
from .process import MachineLimits, ProcessParams


@dataclass
class Params:
    # blade geometry
    nu: int = 60
    r_hub: float = 30.0
    r_shroud: float = 55.0
    z_span: float = 20.0
    z_offset: float = 8.0
    wrap: float = 0.6
    twist: float = 0.7
    n_blades: int = 11          # for neighbour-blade collision proxy
    # tool / strategy
    R: float = 6.0
    nv: int = 41
    strategy: str = "minmax"    # two_point | minmax | smoothed | global
    smooth_window: int = 5
    mu: float = 30.0            # global-optimizer smoothness weight
    gamma: float = 0.0          # tool taper half-angle (rad); 0 = cylinder
    nsweeps: int = 4
    rails: tuple = None         # optional (a, b) override for external blades
    # machine + process
    machine: MachineLimits = field(default_factory=MachineLimits)
    process: ProcessParams = field(default_factory=ProcessParams)
    pivot: tuple = (0.0, 0.0, -100.0)


def _seg_distance(points, p0, p1):
    """Min distance from each point to segment [p0,p1] (vectorized)."""
    d = p1 - p0
    L2 = float(d @ d)
    if L2 < 1e-12:
        return np.linalg.norm(points - p0, axis=1)
    t = np.clip((points - p0) @ d / L2, 0.0, 1.0)
    proj = p0[None, :] + t[:, None] * d[None, :]
    return np.linalg.norm(points - proj, axis=1)


def double_flank_channel(p: Params) -> dict:
    """Double-flank channel milling: one cylinder finishes both walls of the
    flow channel (this blade's wall and the adjacent blade's facing wall) in a
    single pass. Returns axes, per-wall deviation, and both wall surfaces."""
    if p.rails is not None:
        a, b = np.ascontiguousarray(p.rails[0]), np.ascontiguousarray(p.rails[1])
    else:
        a, b = blade.make_blade(p.nu, p.r_hub, p.r_shroud, p.z_span,
                                p.z_offset, p.wrap, p.twist)
    pitch = 2.0 * np.pi / p.n_blades
    c, s = np.cos(pitch), np.sin(pitch)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    aR, bR = a @ Rz.T, b @ Rz.T                      # adjacent blade's wall
    q0, alpha, devL, devR = core.optimize_double_flank(
        a, b, aR, bR, p.R, nv=p.nv, mu=p.mu, gamma=p.gamma, nsweeps=p.nsweeps)
    nvg = 30
    return dict(q0=q0, alpha=alpha, devL=devL, devR=devR,
                surfL=blade.surface(a, b, nvg), surfR=blade.surface(aR, bR, nvg),
                aL=a, bL=b, aR=aR, bR=bR)


def compute(p: Params) -> dict:
    if p.rails is not None:
        a, b = np.ascontiguousarray(p.rails[0]), np.ascontiguousarray(p.rails[1])
    else:
        a, b = blade.make_blade(p.nu, p.r_hub, p.r_shroud, p.z_span,
                                p.z_offset, p.wrap, p.twist)
    ap, bp = blade.rail_tangents(a, b)
    nu = a.shape[0]

    delta, vstar, strict = core.distribution(a, b)

    res = optimize.optimize_blade(a, b, ap, bp, p.R, nv=p.nv,
                                  smooth_window=p.smooth_window,
                                  mu=p.mu, gamma=p.gamma, nsweeps=p.nsweeps)
    sel = res[p.strategy]
    q0 = sel["q0"]; alpha = sel["alpha"]; dev = sel["dev"]

    # deviation field on the surface grid (for visualization).
    # gamma applies only to the conical "global" tool; other strategies are
    # cylindrical, so the displayed field stays consistent with `dev`.
    eff_gamma = p.gamma if p.strategy == "global" else 0.0
    nv_grid = 30
    surf = blade.surface(a, b, nv_grid)
    v = np.linspace(0.0, 1.0, nv_grid)
    devfield = np.empty((nu, nv_grid))
    for i in range(nu):
        pts = (1.0 - v)[:, None] * a[i][None, :] + v[:, None] * b[i][None, :]
        devfield[i] = core.deviation_cone(q0[i], alpha[i], p.R, eff_gamma, pts)

    # --- Phase 3: kinematics (contact point = mid-ruling) ---
    contact = 0.5 * (a + b)
    m = core.ik_path(contact, alpha, p.pivot)        # (nu, 5) [X,Y,Z,A,C]
    m[:, 3] = np.unwrap(m[:, 3])                      # unwrap A, C for TOPP
    m[:, 4] = np.unwrap(m[:, 4])

    # --- collision / reachability vs neighbour blade ---
    pitch = 2.0 * np.pi / p.n_blades
    cph, sph = np.cos(pitch), np.sin(pitch)
    Rz = np.array([[cph, -sph, 0.0], [sph, cph, 0.0], [0.0, 0.0, 1.0]])
    neigh = (surf.reshape(-1, 3) @ Rz.T)
    min_clear = np.inf
    for i in range(nu):
        seg0 = q0[i] - alpha[i] * 0.2 * p.process.flute_len
        seg1 = q0[i] + alpha[i] * p.process.flute_len
        min_clear = min(min_clear, _seg_distance(neigh, seg0, seg1).min())
    collision_free = bool(min_clear > p.R)

    # --- Phase 4: time-optimal feed ---
    # contact-path arc length as an extra DOF carrying the process feed cap
    seglen = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(contact, axis=0), axis=1))]
    feed_cap_mms = p.process.effective_feed_mm_min() / 60.0
    q = np.column_stack([m, seglen])                 # (nu, 6)
    vmax = p.machine.vmax() + [feed_cap_mms]
    amax = p.machine.amax() + [1.0e4]
    aprof, cycle_s = core.topp(q, vmax, amax)

    return dict(
        a=a, b=b, surf=surf, devfield=devfield, strict=strict,
        delta=delta, q0=q0, alpha=alpha, dev=dev,
        machine_path=m, aprof=aprof, cycle_time_s=cycle_s,
        min_clearance=min_clear, collision_free=collision_free,
        orient_jerk=optimize.orientation_jerk(alpha),
        contact=contact, seglen=seglen,
        feed_cap_mm_min=p.process.effective_feed_mm_min(),
        path_len_mm=float(seglen[-1]),
    )
