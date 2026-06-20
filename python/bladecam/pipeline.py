"""End-to-end BladeCAM pipeline: geometry -> positioning -> kinematics ->
time-optimal feed -> cycle time, plus a neighbour-blade collision check.

`compute(params)` returns a single results dict consumed by both the headless
demo and the GUI, so the GUI stays a thin presentation layer.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import numpy as np

from . import core, blade, optimize
from .process import MachineLimits, ProcessParams
from .machine import (reachability, table_mesh,
                      tool_branch_capsules, structure_capsules)


@dataclass
class Params:
    # blade geometry
    nu: int = 60
    r_hub: float = 30.0
    r_shroud: float = 55.0
    z_span: float = 20.0
    z_offset: float = 8.0
    # moderate, machinable default blade (a clean collision-free worked example);
    # crank wrap/twist up to see the envelope error grow.
    wrap: float = 0.3
    twist: float = 0.3
    n_blades: int = 11          # for neighbour-blade collision proxy
    # tool / strategy
    R: float = 6.0
    nv: int = 41
    strategy: str = "minmax"    # two_point | minmax | smoothed | global
    smooth_window: int = 5
    # Multi-objective weights for the global optimiser. The objective minimises
    # the REAL machined error (swept envelope) AND keeps the tool-axis motion
    # smooth, not just the (misleading) per-ruling residual: a higher mu is
    # required so the swept term does not wiggle the path (raising jerk / rotary
    # winding past the TOPP / post limits). mu=6, swept_weight=0.3 is the balanced
    # point -- low envelope error AND smooth motion (see AUDIT.md §D/§Q).
    mu: float = 6.0             # global-optimizer smoothness weight (dimensionless)
    gamma: float = 0.0          # tool taper half-angle (rad); 0 = cylinder
    barrel_R: float = 0.0       # barrel arc radius (0 = cylinder/cone tool)
    barrel_pos: float = 0.0     # barrel widest-point axial position (mm from q0)
    nsweeps: int = 3
    # On by default and balanced against mu above: the global optimiser minimises
    # the swept-envelope error it reports (optimise-what-you-report). 0 = off
    # (per-ruling fit only).
    swept_weight: float = 0.3   # global-optimizer swept-overcut penalty
    swept_window: int = 8       # neighbour index half-width for swept penalty
    collision_substeps: int = 2  # swept-motion sampling between stations
    fixture_z: float = None      # fixture/table plane z (None = no plane check)
    mount_clearance: float = 30.0  # blade base -> machine table top (mm)
    root_fillet_r: float = 0.0   # hub-fillet trim offset (mm); 0 = no trim
    rails: tuple = None         # optional (a, b) override for external blades
    fixture_mesh: tuple = None  # optional (verts, faces) fixture/machine-body mesh
    # operation parameters (were hardcoded call-site defaults; now config)
    rough_ap: float = 3.0          # roughing axial depth (mm)
    rough_ae_frac: float = 0.4     # roughing stepover as fraction of tool dia
    rough_stock_mm: float = 2.0    # stock left for the estimate
    troch_ae_frac: float = 0.15    # trochoidal target engagement / tool dia
    edge_R_ball: float = 3.0       # ball-nose radius for edge finishing (mm)
    edge_scallop: float = 0.005    # edge scallop allowance (mm)
    viz_grid: int = 30             # surface visualisation grid resolution
    # machine + process
    machine: MachineLimits = field(default_factory=MachineLimits)
    process: ProcessParams = field(default_factory=ProcessParams)
    pivot: tuple = (0.0, 0.0, -100.0)


def _blade_rails(p: Params):
    if p.rails is not None:
        a, b = (np.ascontiguousarray(p.rails[0]),
                np.ascontiguousarray(p.rails[1]))
    else:
        a, b = blade.make_blade(p.nu, p.r_hub, p.r_shroud, p.z_span,
                                p.z_offset, p.wrap, p.twist)
    if p.root_fillet_r > 0.0:
        from . import features
        a, b = features.trim_root_fillet(a, b, p.root_fillet_r)
    return a, b


def _rotz(ang: float) -> np.ndarray:
    """Rotation about the impeller (Z) axis."""
    c, s = np.cos(ang), np.sin(ang)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _densify_poses(q0, alpha, substeps):
    """Insert `substeps` interpolated tool poses between consecutive stations so
    collision checking covers the swept motion, not just the endpoints."""
    if substeps <= 0:
        return q0, alpha
    nu = q0.shape[0]
    qs, as_ = [], []
    for i in range(nu - 1):
        for s in range(substeps + 1):
            t = s / (substeps + 1)
            qs.append((1 - t) * q0[i] + t * q0[i + 1])
            a = (1 - t) * alpha[i] + t * alpha[i + 1]
            n = np.linalg.norm(a)
            as_.append(a / n if n > 0 else alpha[i])
    qs.append(q0[-1]); as_.append(alpha[-1])
    return np.ascontiguousarray(qs), np.ascontiguousarray(as_)


def _neighbour_walls(a, b, n_blades, k=1):
    """The k-th adjacent blade's wall, rotated by the blade pitch about Z."""
    Rz = _rotz(k * 2.0 * np.pi / n_blades)
    return a @ Rz.T, b @ Rz.T


def stacked_flank_passes(p: Params) -> dict:
    """Split a blade taller than the usable flute into stacked flank passes.

    Each pass machines a v-band [v0,v1] of the ruling (a thinner sub-strip), so
    the engaged ruling fits the flute AND the per-band twist error is smaller
    than a single full-height pass. Returns per-pass deviation/cycle and totals.
    """
    a, b = _blade_rails(p)
    height = float(np.mean(np.linalg.norm(b - a, axis=1)))
    n = max(1, int(np.ceil(height / p.process.flute_len)))
    vb = np.linspace(0.0, 1.0, n + 1)
    passes = []
    for k in range(n):
        v0, v1 = vb[k], vb[k + 1]
        ab = (1 - v0) * a + v0 * b
        bb = (1 - v1) * a + v1 * b
        r = compute(replace(p, rails=(ab, bb)))
        passes.append(dict(v0=float(v0), v1=float(v1),
                           dev=r["dev"], cycle_s=r["cycle_time_s"]))
    return dict(n_passes=n, blade_height=height, passes=passes,
                dev_max=max(pp["dev"].max() for pp in passes),
                cycle_total_s=sum(pp["cycle_s"] for pp in passes))


def roughing_time_estimate(p: Params, ap: float = None, ae_frac: float = None,
                           stock_mm: float = None) -> dict:
    """First-order channel-roughing cycle-time estimate: removed volume / MRR.
    Operation params default to the config fields on `p` (no hardcoding)."""
    ap = p.rough_ap if ap is None else ap
    ae_frac = p.rough_ae_frac if ae_frac is None else ae_frac
    a, b = _blade_rails(p)
    height = float(np.mean(np.linalg.norm(b - a, axis=1)))
    length = float(np.sum(np.linalg.norm(np.diff(0.5*(a + b), axis=0), axis=1)))
    rmid = 0.5 * (p.r_hub + p.r_shroud)
    gap = max(0.0, 2.0 * np.pi * rmid / p.n_blades - 2.0 * p.R)   # channel width
    volume = gap * height * length * 0.5                          # ~half = stock
    ae = ae_frac * 2.0 * p.R
    feed = p.process.effective_feed_mm_min()
    mrr = ap * ae * feed                                          # mm^3/min
    minutes = volume / mrr if mrr > 0 else float("inf")
    return dict(removed_volume_mm3=volume, mrr_mm3_min=mrr,
                rough_time_s=minutes * 60.0, channel_gap_mm=gap)


def edge_finish(p: Params, R_ball: float = None, scallop_allow: float = None):
    """Point-mill (ball-nose) finishing of the blade leading-edge patch."""
    from . import pointmill
    R_ball = p.edge_R_ball if R_ball is None else R_ball
    scallop_allow = p.edge_scallop if scallop_allow is None else scallop_allow
    a, b = _blade_rails(p)
    patch = pointmill.leading_edge_patch(a, b)
    return pointmill.point_mill(patch, R_ball, scallop_allow)


def rough_channel(p: Params, ap: float = None, stepover: float = None) -> dict:
    """Layered roughing toolpath for the flow channel (real passes, not an
    estimate)."""
    from . import roughing
    ap = p.rough_ap if ap is None else ap
    a, b = _blade_rails(p)
    a2, b2 = _neighbour_walls(a, b, p.n_blades)
    if stepover is None:
        stepover = p.rough_ae_frac * 2.0 * p.R
    return roughing.adaptive_rough(a, b, a2, b2, ap, stepover,
                                   p.process.effective_feed_mm_min())


def rough_channel_trochoidal(p: Params, ae_target: float = None) -> dict:
    """Engagement-controlled trochoidal roughing of the flow channel."""
    from . import roughing
    a, b = _blade_rails(p)
    a2, b2 = _neighbour_walls(a, b, p.n_blades)
    if ae_target is None:
        ae_target = p.troch_ae_frac * 2.0 * p.R
    return roughing.trochoidal_channel(a, b, a2, b2, p.R, ae_target,
                                       p.process.effective_feed_mm_min())


def _blade_rails_raw(p: Params):
    """The blade rails WITHOUT the root-fillet trim (the true root edge)."""
    if p.rails is not None:
        return (np.ascontiguousarray(p.rails[0]),
                np.ascontiguousarray(p.rails[1]))
    return blade.make_blade(p.nu, p.r_hub, p.r_shroud, p.z_span,
                            p.z_offset, p.wrap, p.twist)


def fillet_machining(p: Params, n_across: int = 5) -> dict:
    """Machine the recognised root fillet with a ball-nose tool: a dedicated
    concave-blend operation along the blade root, complementing the flank-trim
    (which keeps the flank pass OFF the fillet). The fillet runs along the
    untrimmed root edge; the ball rolls from the flank-tangent to the hub-tangent
    line. Returns the toolpath plus a no-gouge margin (the ball must stay >=
    r_ball from both walls)."""
    from . import features
    a, b = _blade_rails_raw(p)
    a2, b2 = _neighbour_walls(a, b, p.n_blades)
    t_edge = np.gradient(a, axis=0)
    t_edge = t_edge / (np.linalg.norm(t_edge, axis=1, keepdims=True) + 1e-12)
    u = b - a                                          # ruling (flank up-direction)
    n_flank = np.cross(u, t_edge)
    n_flank = n_flank / (np.linalg.norm(n_flank, axis=1, keepdims=True) + 1e-12)
    # orient the flank normal into the open channel (toward the neighbour wall)
    chan = 0.5 * (a2 + b2) - 0.5 * (a + b)
    flip = np.sum(n_flank * chan, axis=1) < 0.0
    n_flank[flip] = -n_flank[flip]
    # hub normal: flat-hub +Z approximation, made perpendicular to the edge
    z = np.tile(np.array([0.0, 0.0, 1.0]), (a.shape[0], 1))
    n_hub = z - np.sum(z * t_edge, axis=1, keepdims=True) * t_edge
    n_hub = n_hub / (np.linalg.norm(n_hub, axis=1, keepdims=True) + 1e-12)

    fillet_r = p.root_fillet_r if p.root_fillet_r > 0.0 else 3.0
    r_ball = min(p.edge_R_ball, 0.95 * fillet_r)
    fp = features.fillet_finish(a, b, n_flank, n_hub, fillet_r, r_ball, n_across)
    C = fp["centers"]
    # no-gouge margin: distance from each tool centre to the flank and hub planes
    df = np.einsum("kij,ij->ki", C - a[None], n_flank)
    dh = np.einsum("kij,ij->ki", C - a[None], n_hub)
    min_wall = float(min(df.min(), dh.min()))
    plen = float(np.sum(np.linalg.norm(np.diff(C, axis=1), axis=2)))
    return dict(centers=C, contacts=fp["contacts"], axis=fp["axis"],
                fillet_r=fillet_r, r_ball=r_ball, n_passes=n_across,
                path_len_mm=plen, min_wall_dist_mm=min_wall,
                gouge_free=bool(min_wall >= r_ball - 1e-6))


def rest_machining(p: Params, nx: int = 44, ny: int = 44) -> dict:
    """Stock-aware rest-machining: carry a persistent dexel (Z-map) stock of the
    flow channel through roughing THEN finishing, so the finish pass sees only
    the material roughing left -- not the raw block.

    Roughing clears the channel bulk (layered vertical-cylinder passes); the
    flank-finish tool then removes the rest material on the wall. Returns the
    stock volumes at each step plus the key rest-machining comparison: how much
    the finish removes after roughing vs finishing the RAW stock (the latter is
    larger -- roughing already took the overlap)."""
    from . import stock as stock_mod
    from . import roughing
    a, b = _blade_rails(p)
    a2, b2 = _neighbour_walls(a, b, p.n_blades)
    zspan = float(max(a[:, 2].max(), b[:, 2].max(),
                      a2[:, 2].max(), b2[:, 2].max())
                  - min(a[:, 2].min(), b[:, 2].min(),
                        a2[:, 2].min(), b2[:, 2].min()))
    Lf = np.linalg.norm(b - a, axis=1)

    st = stock_mod.channel_stock(a, b, a2, b2, nx, ny)
    v0 = st.volume()
    # roughing: every layered pass point is a vertical-cylinder tool centre
    rough = roughing.adaptive_rough(a, b, a2, b2, p.rough_ap,
                                    p.rough_ae_frac * 2.0 * p.R,
                                    p.process.effective_feed_mm_min())
    rq0 = np.vstack(rough["passes"])
    raxis = np.tile(np.array([0.0, 0.0, 1.0]), (rq0.shape[0], 1))
    rough_removed = st.carve(rq0, raxis, p.R, zspan)
    v_rough = st.volume()
    # finishing: the optimised flank tool removes the rest material on the wall
    r = compute(p)
    finish_removed = st.carve(r["q0"], r["alpha"], p.R, Lf)
    v_finish = st.volume()
    # reference: finishing the RAW channel (no roughing) removes strictly more
    raw = stock_mod.channel_stock(a, b, a2, b2, nx, ny)
    finish_from_raw = raw.carve(r["q0"], r["alpha"], p.R, Lf)
    return dict(stock_volume_mm3=v0, after_rough_mm3=v_rough,
                after_finish_mm3=v_finish, rough_removed_mm3=rough_removed,
                finish_removed_mm3=finish_removed,
                finish_from_raw_mm3=finish_from_raw,
                rest_fraction=(finish_removed / finish_from_raw
                               if finish_from_raw > 0 else 0.0),
                rest_field=st.rest_per_ray())


def double_flank_channel(p: Params) -> dict:
    """Double-flank channel milling: one cylinder finishes both walls of the
    flow channel (this blade's wall and the adjacent blade's facing wall) in a
    single pass. Returns axes, per-wall deviation, and both wall surfaces."""
    a, b = _blade_rails(p)
    aR, bR = _neighbour_walls(a, b, p.n_blades)      # adjacent blade's wall
    q0, alpha, devL, devR = core.optimize_double_flank(
        a, b, aR, bR, p.R, nv=p.nv, mu=p.mu, gamma=p.gamma, nsweeps=p.nsweeps)
    nvg = p.viz_grid
    return dict(q0=q0, alpha=alpha, devL=devL, devR=devR,
                surfL=blade.surface(a, b, nvg), surfR=blade.surface(aR, bR, nvg),
                aL=a, bL=b, aR=aR, bR=bR)


def compute(p: Params) -> dict:
    a, b = _blade_rails(p)
    ap, bp = blade.rail_tangents(a, b)
    nu = a.shape[0]
    pr = p.process
    feed_cap = pr.effective_feed_mm_min()
    # mechanistic cutting forces at the planned feed-per-tooth
    fz_eff = feed_cap / max(1.0, pr.n_teeth * pr.rpm)
    forces = pr.cutting_forces(fz_eff)
    feed_feasible = pr.feed_feasible()

    delta, vstar, strict = core.distribution(a, b)

    res = optimize.optimize_blade(a, b, ap, bp, p.R, nv=p.nv,
                                  smooth_window=p.smooth_window,
                                  mu=p.mu, gamma=p.gamma, nsweeps=p.nsweeps,
                                  strategy=p.strategy,
                                  swept_w=p.swept_weight,
                                  swept_window=p.swept_window,
                                  barrel_R=p.barrel_R, barrel_pos=p.barrel_pos)
    sel = res[p.strategy]
    q0 = sel["q0"]; alpha = sel["alpha"]; dev = sel["dev"]

    # deviation field on the surface grid (for visualization). The tool family
    # (cone gamma / barrel) applies only to the "global" strategy; the others are
    # cylindrical, so the displayed field stays consistent with `dev`.
    eff_gamma = p.gamma if p.strategy == "global" else 0.0
    eff_Rb = p.barrel_R if p.strategy == "global" else 0.0
    nv_grid = p.viz_grid
    surf = blade.surface(a, b, nv_grid)
    v = np.linspace(0.0, 1.0, nv_grid)
    devfield = np.empty((nu, nv_grid))
    for i in range(nu):
        pts = (1.0 - v)[:, None] * a[i][None, :] + v[:, None] * b[i][None, :]
        if eff_Rb > 0.0:
            devfield[i] = core.deviation_barrel(q0[i], alpha[i], p.R, eff_Rb,
                                                p.barrel_pos, pts)
        else:
            devfield[i] = core.deviation_cone(q0[i], alpha[i], p.R, eff_gamma, pts)

    # --- Phase 3: kinematics (contact point = mid-ruling) ---
    contact = 0.5 * (a + b)
    m = core.ik_path(contact, alpha, p.pivot, kind=p.machine.kind)  # (nu,5) X,Y,Z,A,C
    m[:, 3] = np.unwrap(m[:, 3])                      # unwrap A, C for TOPP
    m[:, 4] = np.unwrap(m[:, 4])

    # machine reachability: does this toolpath fit the machine's travel/rotary
    # envelope? (only when a full Machine profile is supplied, not bare limits)
    axis_violations = (reachability(p.machine, m)
                       if hasattr(p.machine, "x_range") else {})
    reachable = len(axis_violations) == 0
    machine_name = getattr(p.machine, "name", "custom limits")

    # --- collision: full tool ASSEMBLY (flute+holder+spindle) vs the part and
    # machine, swept over the whole motion, plus an optional fixture/table plane.
    flat = surf.reshape(-1, 3)
    pitch = 2.0 * np.pi / p.n_blades
    # NEIGHBOUR FLANKS as EXACT triangle meshes (#2): a point cloud cannot detect
    # a tool thinner than its sample spacing threading between samples, so the
    # neighbour blades are checked as continuous unsigned meshes (a thin flank is
    # an OPEN sheet -- the tool crossing it drives the seg-triangle distance below
    # the radius). A modest grid keeps the chord error far below the cut tolerance.
    ui = np.unique(np.linspace(0, nu - 1, min(nu, 18)).round().astype(int))
    vi = np.unique(np.linspace(0, nv_grid - 1, min(nv_grid, 16)).round().astype(int))
    ngrid = surf[np.ix_(ui, vi)]
    obstacle_tris = np.vstack([core.tris_from_grid(ngrid @ _rotz(pitch).T),
                               core.tris_from_grid(ngrid @ _rotz(-pitch).T)])
    # structural machine model: the trunnion TABLE the workpiece sits on, now an
    # EXACT mesh (#2) appended to the unsigned obstacle mesh -- continuous, so a
    # thin tool cannot thread between samples as it could with the point cloud.
    structural = hasattr(p.machine, "table_radius")
    mount_z = float(min(a[:, 2].min(), b[:, 2].min())) - p.mount_clearance
    if structural:
        obstacle_tris = np.vstack([obstacle_tris, table_mesh(p.machine, mount_z)])
    # stacked assembly segments (axial extents from q0 along the tool axis)
    hbase = pr.flute_len + pr.holder_gap
    sbase = hbase + pr.holder_len + pr.spindle_gap
    seg_R = np.array([p.R, pr.holder_dia*0.5, pr.spindle_dia*0.5])
    seg_lo = np.array([0.0, hbase, sbase])
    seg_hi = np.array([pr.flute_len, hbase + pr.holder_len, sbase + pr.spindle_len])
    plane_pt = None if p.fixture_z is None else np.array([0.0, 0.0, p.fixture_z])
    plane_n = np.array([0.0, 0.0, 1.0])
    nscan_c = max(4, 2 * p.collision_substeps)
    reach = float(seg_hi.max() + seg_R.max())
    kind = p.machine.kind
    # hub / shroud ENDWALLS (#3): the rotating disk and outer band as surfaces of
    # revolution of the root/tip rails, sampled just dense enough that the holder
    # cannot thread between points (azimuthal & axial spacing < holder radius).
    hgap = max(2.0, 0.8 * pr.holder_dia * 0.5)
    r_out = float(np.hypot(b[:, 0], b[:, 1]).max())
    nazi = max(16, int(np.ceil(2.0 * np.pi * r_out / hgap)))
    rail_len = float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())
    stride = max(1, nu // max(2, int(np.ceil(rail_len / hgap))))
    aw, bw = a[::stride], b[::stride]
    endwall = np.vstack([np.vstack([aw @ _rotz(2.0*np.pi*k/nazi).T,
                                    bw @ _rotz(2.0*np.pi*k/nazi).T])
                         for k in range(nazi)])

    def _obstacle_clr(dqL, daL, owL, nuL):
        """Min clearance of a (densified) pose set to the whole obstacle world:
        the unsigned obstacle MESH (neighbour flanks + table, full assembly), the
        fixture plane (full assembly), and hub/shroud endwalls (holder+spindle
        only). Returns a per-original-segment array (length nuL)."""
        capsL = tool_branch_capsules(dqL, daL, seg_R, seg_lo, seg_hi)
        c = core.reduce_to_segments(
            core.mesh_clearance(capsL, obstacle_tris, nscan=nscan_c,
                                signed=False), owL, nuL)
        if plane_pt is not None:
            c = np.minimum(c, core.reduce_to_segments(
                core.assembly_clearance(dqL, daL, seg_R, seg_lo, seg_hi,
                                        np.zeros((0, 3)), plane_pt=plane_pt,
                                        plane_n=plane_n, nscan=nscan_c), owL, nuL))
        c = np.minimum(c, core.reduce_to_segments(
            core.assembly_clearance(dqL, daL, seg_R[1:], seg_lo[1:], seg_hi[1:],
                                    endwall, nscan=nscan_c), owL, nuL))
        return c, capsL

    # JOINT-SPACE swept collision (#1+#8): the machine moves the 5 joints linearly
    # between stations, so the tool sweeps the FK of the interpolated joints -- a
    # curve, not the part-frame chord. Densify each segment in joint space (with a
    # substep count that bounds the chord-vs-arc residual, the conservative-
    # advancement guarantee) ONCE and run every swept check on the real path. The
    # IK is run on q0 itself (the tool reference the assembly is built from); its
    # rotary axes equal the posted path's, so the FK sweep is rotary-exact.
    m_tool = core.ik_path(q0, alpha, p.pivot, kind=kind)
    m_tool[:, 3] = np.unwrap(m_tool[:, 3]); m_tool[:, 4] = np.unwrap(m_tool[:, 4])
    dense_m, dq, da, owner = core.densify_fk(m_tool, p.pivot, kind, reach)
    clr, tool_caps = _obstacle_clr(dq, da, owner, nu)
    hub_clearance = float(core.reduce_to_segments(
        core.assembly_clearance(dq, da, seg_R[1:], seg_lo[1:], seg_hi[1:],
                                endwall, nscan=nscan_c), owner, nu).min())
    # holder vs the blade BEING machined: the flute is tangent to this blade by
    # design, but the holder must still clear it -- SWEPT (#4), not per-station.
    holder_base = pr.flute_len + pr.holder_gap
    holder_self = core.reduce_to_segments(
        core.holder_clearance_swept(dq, da, flat, pr.holder_dia * 0.5,
                                    holder_base, pr.holder_len, nscan=nscan_c),
        owner, nu)
    holder_min = float(holder_self.min())
    min_clear = float(min(clr.min(), holder_min, hub_clearance))
    # LEAD-IN / LEAD-OUT (#approach/retract): the tool plunges into the first cut
    # and retracts from the last along its own axis (sliding out of the passage).
    # These non-cutting moves are where many real crashes happen, so check them
    # with the same swept obstacle world. Dret pulls the whole flute clear of the
    # part before any lateral rapid takes over.
    Dret = float(pr.flute_len + (b[:, 2].max() - a[:, 2].min()) + 10.0)
    nlead = 6

    def _lead_clr(qend, axend):
        ts = np.linspace(1.0, 0.0, nlead)
        ql = qend[None, :] + ts[:, None] * Dret * axend[None, :]
        al = np.tile(axend, (nlead, 1))
        ml = core.ik_path(ql, al, p.pivot, kind=kind)
        ml[:, 3] = np.unwrap(ml[:, 3]); ml[:, 4] = np.unwrap(ml[:, 4])
        _, dql, dal, owl = core.densify_fk(ml, p.pivot, kind, reach)
        cl, _ = _obstacle_clr(dql, dal, owl, nlead)
        return float(cl.min())

    approach_clearance = _lead_clr(q0[0], alpha[0])
    retract_clearance = _lead_clr(q0[-1], alpha[-1])
    min_clear = min(min_clear, approach_clearance, retract_clearance)
    # PASS-LINKING / blade-index move (#1): on a multi-blade wheel the machine
    # retracts off the end of one blade, indexes the C table by one pitch to the
    # next, and re-approaches. By rotational symmetry every blade's lead-in/out is
    # the one already checked above; the NEW motion is the index itself -- a joint
    # move from the retracted end of this blade to the retracted (pitch-rotated)
    # start of the next, swept against the same obstacle world. The approach/
    # retract are along the axis; the index sweeps the tool across a passage.
    # routed via a CLEARANCE PLANE above the wheel (the standard safe sequence a
    # post emits): lift the retracted tool to clear_z, traverse + index C at that
    # height, then descend to the next blade's retracted start (its lead-in is the
    # one already checked, by symmetry). Modelling the lift/traverse/descend
    # rather than one diagonal removes the pessimism of cutting the corner.
    index_clearance = float("inf")
    if p.n_blades > 1:
        Rp = _rotz(pitch)
        clear_z = float(max(a[:, 2].max(), b[:, 2].max())) + max(20.0, 0.5 * Dret)
        q_end = q0[-1] + Dret * alpha[-1]                 # retracted end (lead-out)
        q_lift = np.array([q_end[0], q_end[1], max(q_end[2], clear_z)])
        app0 = q0[0] + Dret * alpha[0]                    # this blade's approach pt
        q_trav = np.array([app0[0], app0[1], max(app0[2], clear_z)]) @ Rp.T
        q_desc = app0 @ Rp.T                              # next blade retracted start
        ql = np.array([q_end, q_lift, q_trav, q_desc])
        al = np.array([alpha[-1], alpha[-1], alpha[0] @ Rp.T, alpha[0] @ Rp.T])
        ml = core.ik_path(ql, al, p.pivot, kind=kind)
        ml[:, 3] = np.unwrap(ml[:, 3]); ml[:, 4] = np.unwrap(ml[:, 4])
        _, dqi, dai, owi = core.densify_fk(ml, p.pivot, kind, reach)
        ci, _ = _obstacle_clr(dqi, dai, owi, ql.shape[0])
        index_clearance = float(ci.min())
        min_clear = min(min_clear, index_clearance)
    # structural machine model: tool-assembly capsules vs the trunnion cradle yoke
    # + machine column, on the DENSE FK poses + dense machine joints.
    link_clearance = float("inf")
    mesh_clear = float("inf")
    if structural and getattr(p.machine, "cradle_span", 0.0) >= 0.0:
        struct_caps = structure_capsules(p.machine, dense_m, p.pivot, mount_z)
        if struct_caps.shape[1] > 0:
            link_clr = core.reduce_to_segments(
                core.struct_clearance(tool_caps, struct_caps, nscan=nscan_c),
                owner, nu)
            link_clearance = float(link_clr.min())
            min_clear = min(min_clear, link_clearance)
    # mesh-accurate fixture / machine-body collision: the tool assembly vs an
    # imported (closed) triangle mesh -- signed, so a tool buried in the body is a
    # collision even without touching a face.
    if p.fixture_mesh is not None:
        tris = core.mesh_from_faces(p.fixture_mesh[0], p.fixture_mesh[1])
        mclr = core.reduce_to_segments(
            core.mesh_clearance(tool_caps, tris, nscan=nscan_c, signed=True),
            owner, nu)
        mesh_clear = float(mclr.min())
        min_clear = min(min_clear, mesh_clear)
    collision_free = bool(min_clear > 0.0)
    gouge_max = float(max(0.0, -devfield.min()))   # per-station depth past the design surface
    # swept-envelope overcut: cross-station interference the per-station model
    # misses (real flank-milling overcut in twisted LE/TE regions)
    Lflute = np.linalg.norm(b - a, axis=1)
    swept = core.swept_deviation(q0, alpha, Lflute, p.R, surf.reshape(-1, 3),
                                 gamma=eff_gamma, Rb=eff_Rb, lamc=p.barrel_pos)
    swept_overcut = float(max(0.0, -swept.min()))
    # full per-point swept (machined-surface) error field, so the 3D view can
    # colour the surface by the REAL envelope error rather than the per-station
    # residual (which is ~0 for a cylinder on an exact ruled surface)
    swept_field = swept.reshape(surf.shape[0], surf.shape[1])
    # true swept-envelope SURFACE: the actual machined geometry (design grid
    # projected onto the nearest swept cutter), renderable as a (nu,nv,3) mesh
    envelope_surf = core.swept_surface(q0, alpha, Lflute, p.R,
                                       surf.reshape(-1, 3), gamma=eff_gamma,
                                       Rb=eff_Rb, lamc=p.barrel_pos).reshape(surf.shape)

    # --- Phase 4: time-optimal feed ---
    # contact-path arc length as an extra DOF carrying the process feed cap
    seglen = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(contact, axis=0), axis=1))]
    feed_cap_mms = feed_cap / 60.0
    q = np.column_stack([m, seglen])                 # (nu, 6)
    vmax = p.machine.vmax() + [feed_cap_mms]
    amax = p.machine.amax() + [1.0e4]
    aprof, cycle_s = core.topp(q, vmax, amax)
    # per-move durations from the TOPP profile (sum == cycle time); used to post
    # inverse-time (G93) feedrates so the G-code realises the optimal schedule.
    ds_s = 1.0 / (nu - 1)
    sq = np.sqrt(np.clip(aprof, 0.0, None))
    move_times = 2.0 * ds_s / (sq[:-1] + sq[1:] + 1e-12)

    return dict(
        a=a, b=b, surf=surf, devfield=devfield, strict=strict,
        delta=delta, q0=q0, alpha=alpha, dev=dev,
        machine_path=m, aprof=aprof, cycle_time_s=cycle_s,
        min_clearance=min_clear, collision_free=collision_free,
        holder_clearance=holder_min, assembly_clearance=float(clr.min()),
        link_clearance=link_clearance, mesh_clearance=mesh_clear,
        hub_clearance=hub_clearance,
        approach_clearance=approach_clearance, retract_clearance=retract_clearance,
        index_clearance=index_clearance,
        reachable=reachable, axis_violations=axis_violations,
        machine_name=machine_name, structural_check=structural,
        cut_force_peak_N=forces["F_peak"], cut_force_mean_N=forces["F_mean"],
        cut_power_W=forces["power_W"], cut_torque_Nm=forces["torque_Nm"],
        feed_feasible=feed_feasible,
        gouge_max=gouge_max, swept_overcut=swept_overcut, clearance=clr,
        swept_field=swept_field, envelope_surf=envelope_surf,
        orient_jerk=optimize.orientation_jerk(alpha),
        contact=contact, seglen=seglen, move_times_s=move_times,
        feed_cap_mm_min=feed_cap,
        path_len_mm=float(seglen[-1]),
    )
