"""Certified post-processors: a control-specific G-code dialect bound to a
SPECIFIC machine, plus a certification validator that proves the program is safe
to run on that machine/control pairing.

Where `postproc.py` holds bare emitters, this layer adds:
  - PostConfig : a named control+machine binding (axis letters/signs, limits,
                 TCP mode, linearisation tolerance) -- the "post hardened for a
                 machine/control".
  - generate() : dispatch to the Heidenhain (klartext TCPM), Siemens 840D
                 (TRAORI / A3 B3 C3 vector) or Fanuc 30i (G43.4 TCP, joint A/C)
                 dialect.
  - certify()  : independently re-check the program against the machine's travel
                 & rotary envelope, per-block rotary winding, the linearisation
                 chord tolerance, the rotary-speed limit, AND a forward-kinematics
                 round trip (the posted joints must reproduce the tool-tip path).
"""
from __future__ import annotations

from dataclasses import dataclass, asdict, fields
import numpy as np

from . import machine as machine_lib
from . import postproc


@dataclass
class PostConfig:
    name: str = "Heidenhain TNC640 — generic trunnion AC"
    control: str = "heidenhain"          # heidenhain | siemens | fanuc
    machine_name: str = "Generic 5-axis trunnion"
    rotary_primary: str = "A"            # tilt-axis letter (about X)
    rotary_secondary: str = "C"          # rotary-table letter (about Z)
    sign_primary: float = 1.0            # machine sign of the primary rotary
    sign_secondary: float = 1.0          # machine sign of the secondary rotary
    tcp: bool = True                     # emit tool-centre-point / TCPM mode
    decimals: int = 3                    # linear-axis decimals
    rotary_decimals: int = 4             # rotary-axis decimals
    safe_z: float = 50.0
    max_rotary_step_deg: float = 120.0   # per-block rotary winding alarm
    max_lin_dev_mm: float = 0.05         # linearisation chord tolerance
    rotary_speed_margin: float = 0.05    # tolerance on the reconstructed rotary speed
    program: str = "BLADECAM"
    tool: int = 1
    rpm: float = 12000.0

    def machine(self):
        return machine_lib.get_machine(self.machine_name)


def to_dict(cfg: PostConfig) -> dict:
    return asdict(cfg)


def from_dict(d: dict) -> PostConfig:
    valid = {f.name for f in fields(PostConfig)}
    return PostConfig(**{k: v for k, v in d.items() if k in valid})


# --- forward kinematics (matches kinematics.f90: world->part = Rz(C)Rx(A)) ----
def forward_kin(machine_path: np.ndarray, pivot, kind: int) -> np.ndarray:
    """Tool-tip contact path (n,3) reconstructed from machine axes (n,5)."""
    m = np.asarray(machine_path, float)
    piv = np.asarray(pivot, float)
    out = np.empty((m.shape[0], 3))
    for i, (X, Y, Z, A, C) in enumerate(m):
        if kind == 1:                                  # head-head: part fixed
            out[i] = (X, Y, Z)
        else:
            RR = machine_lib._rotz(C) @ machine_lib._rotx(A)
            out[i] = RR @ (np.array([X, Y, Z]) - piv) + piv
    return out


# --- dialect emitters --------------------------------------------------------
def _feed_for_move(move_times, k, p0, p1, fallback):
    if move_times is None:
        return fallback
    dist = float(np.linalg.norm(np.asarray(p1) - np.asarray(p0)))
    return min(max(60.0 * dist / max(float(move_times[k]), 1e-9), 1.0), 1e5)


def post_heidenhain(cfg: PostConfig, contact, axis, feed, move_times=None) -> str:
    """Klartext TCPM (kinematics-independent vector orientation)."""
    header = f"{cfg.program} | {cfg.name}"
    return postproc.to_heidenhain(contact, axis, feed, rpm=cfg.rpm, header=header,
                                  move_times=move_times, tool=cfg.tool,
                                  prog=cfg.program, clear_z=cfg.safe_z)


def post_siemens(cfg: PostConfig, contact, axis, feed, move_times=None) -> str:
    """Siemens 840D TRAORI with the tool orientation as an A3/B3/C3 vector."""
    c = np.asarray(contact, float)
    t = np.asarray(axis, float)
    t = t / np.linalg.norm(t, axis=1, keepdims=True)
    dl = cfg.decimals
    out = [
        f"; {cfg.program} - {cfg.name}",
        "G90 G54 G71",                                  # absolute, metric
        f"T{cfg.tool} D1",
        f"S{cfg.rpm:.0f} M3",
        f"G0 Z{cfg.safe_z:.{dl}f}",
        "TRAORI",                                       # activate 5-axis transform
        "ORIWKS",                                       # orientation in workpiece frame
        f"G0 X{c[0,0]:.{dl}f} Y{c[0,1]:.{dl}f} "
        f"A3={t[0,0]:.6f} B3={t[0,1]:.6f} C3={t[0,2]:.6f}",
        f"G1 Z{c[0,2]:.{dl}f} F{feed:.0f}",
    ]
    for k in range(1, c.shape[0]):
        f = _feed_for_move(move_times, k - 1, c[k - 1], c[k], feed)
        out.append(f"G1 X{c[k,0]:.{dl}f} Y{c[k,1]:.{dl}f} Z{c[k,2]:.{dl}f} "
                   f"A3={t[k,0]:.6f} B3={t[k,1]:.6f} C3={t[k,2]:.6f} F{f:.0f}")
    out += [f"G0 Z{cfg.safe_z:.{dl}f}", "TRAFOOF", "M5", "M30"]
    return "\n".join(out) + "\n"


def post_fanuc(cfg: PostConfig, machine_path, feed, move_times=None) -> str:
    """Fanuc 30i TCP (G43.4) with machine rotary JOINTS A/C -- this dialect bakes
    the machine kinematics into the joint values, so it is genuinely machine-
    specific (the control must hold the matching kinematic model)."""
    m = np.asarray(machine_path, float)
    deg = 180.0 / np.pi
    dl, rd = cfg.decimals, cfg.rotary_decimals
    ap, cp = cfg.rotary_primary, cfg.rotary_secondary

    def rot(A, C):
        return (f"{ap}{cfg.sign_primary*A*deg:.{rd}f} "
                f"{cp}{cfg.sign_secondary*C*deg:.{rd}f}")

    out = [
        "%",
        f"O0001 ({cfg.program} - {cfg.name})",
        "G21 G90 G94",
        f"S{cfg.rpm:.0f} M3",
        f"G0 Z{cfg.safe_z:.{dl}f}",
    ]
    X, Y, Z, A, C = m[0]
    out.append(f"G0 X{X:.{dl}f} Y{Y:.{dl}f} {rot(A, C)}")
    if cfg.tcp:
        out.append(f"G43.4 H{cfg.tool}")               # tool-centre-point control
    out.append(f"G1 Z{Z:.{dl}f} F{feed:.0f}")
    for k in range(1, m.shape[0]):
        X, Y, Z, A, C = m[k]
        f = _feed_for_move(move_times, k - 1, m[k - 1, :3], m[k, :3], feed)
        out.append(f"G1 X{X:.{dl}f} Y{Y:.{dl}f} Z{Z:.{dl}f} {rot(A, C)} F{f:.0f}")
    out += ["G49", f"G0 Z{cfg.safe_z:.{dl}f}", "M5", "M30", "%"]
    return "\n".join(out) + "\n"


def generate(cfg: PostConfig, contact, axis, machine_path, feed,
             move_times=None) -> str:
    """Emit the program for cfg's control dialect."""
    if cfg.control == "heidenhain":
        return post_heidenhain(cfg, contact, axis, feed, move_times)
    if cfg.control == "siemens":
        return post_siemens(cfg, contact, axis, feed, move_times)
    if cfg.control == "fanuc":
        return post_fanuc(cfg, machine_path, feed, move_times)
    raise ValueError(f"unknown control dialect {cfg.control!r}")


# --- certification -----------------------------------------------------------
def certify(cfg: PostConfig, machine_path, contact, pivot, feed,
            move_times=None, machine=None) -> dict:
    """Independently validate a posted toolpath for cfg's machine/control.

    Checks: linear travel & rotary envelope, per-block rotary winding, the
    linearisation chord tolerance, the rotary-speed limit (if move_times given),
    and a forward-kinematics round trip (the posted joints reproduce the tool-tip
    contact path). `machine` overrides cfg's named machine (e.g. an edited live
    profile). Returns a report dict with `certified` = all checks pass."""
    m = np.asarray(machine_path, float)
    c = np.asarray(contact, float)
    mach = machine if machine is not None else cfg.machine()
    deg = 180.0 / np.pi
    rep = {"control": cfg.control, "machine": cfg.machine_name}

    # 1) travel & rotary envelope
    viol = machine_lib.reachability(mach, m)
    rep["travel_violations"] = {k: v for k, v in viol.items() if k in "XYZ"}
    rep["rotary_violations"] = {k: v for k, v in viol.items() if k in "AC"}
    rep["within_travel"] = not rep["travel_violations"]
    rep["within_rotary"] = not rep["rotary_violations"]

    # 2) per-block rotary winding (a discontinuity / unsafe rapid wind)
    dA = np.abs(np.diff(m[:, 3])) * deg
    dC = np.abs(np.diff(m[:, 4])) * deg
    rep["max_rotary_step_deg"] = float(max(dA.max(initial=0.0), dC.max(initial=0.0)))
    rep["winding_ok"] = rep["max_rotary_step_deg"] <= cfg.max_rotary_step_deg

    # 3) linearisation: each interior contact point's distance to the chord of
    #    its neighbours must stay within tolerance (blocks dense enough)
    if c.shape[0] >= 3:
        p0, p1, p2 = c[:-2], c[1:-1], c[2:]
        d = p2 - p0
        L2 = np.sum(d * d, axis=1)
        tt = np.where(L2 > 0, np.sum((p1 - p0) * d, axis=1) / np.where(L2 > 0, L2, 1), 0)
        proj = p0 + tt[:, None] * d
        rep["max_chord_dev_mm"] = float(np.max(np.linalg.norm(p1 - proj, axis=1)))
    else:
        rep["max_chord_dev_mm"] = 0.0
    rep["linearization_ok"] = rep["max_chord_dev_mm"] <= cfg.max_lin_dev_mm

    # 4) rotary-speed feasibility against the machine's v_rot (rad/s)
    if move_times is not None:
        mt = np.maximum(np.asarray(move_times, float), 1e-9)
        wA = np.abs(np.diff(m[:, 3])) / mt
        wC = np.abs(np.diff(m[:, 4])) / mt
        rep["max_rotary_speed_rad_s"] = float(max(wA.max(initial=0.0),
                                                  wC.max(initial=0.0)))
        # the inverse-time reconstruction (move_times from the TOPP a-profile) is
        # a finite-difference average, so allow a small margin over the continuous
        # v_rot limit the planner actually respects.
        rep["rotary_speed_ok"] = (rep["max_rotary_speed_rad_s"]
                                  <= mach.v_rot * (1.0 + cfg.rotary_speed_margin))
    else:
        rep["max_rotary_speed_rad_s"] = 0.0
        rep["rotary_speed_ok"] = True

    # 5) forward-kinematics round trip: the posted joints reproduce the tip path
    back = forward_kin(m, pivot, mach.kind)
    rep["roundtrip_max_err_mm"] = float(np.max(np.linalg.norm(back - c, axis=1)))
    rep["roundtrip_ok"] = rep["roundtrip_max_err_mm"] < 1e-6

    rep["certified"] = bool(rep["within_travel"] and rep["within_rotary"]
                            and rep["winding_ok"] and rep["linearization_ok"]
                            and rep["rotary_speed_ok"] and rep["roundtrip_ok"])
    return rep


# --- library of certified machine/control pairings ---------------------------
CERTIFIED_POSTS = {
    "Heidenhain TNC640 — generic trunnion AC": PostConfig(),
    "Siemens 840D — compact blisk cell AC": PostConfig(
        name="Siemens 840D — compact blisk cell AC", control="siemens",
        machine_name="Compact blisk cell", rpm=18000.0),
    "Fanuc 31i — large gantry AC": PostConfig(
        name="Fanuc 31i — large gantry AC", control="fanuc",
        machine_name="Large gantry 5-axis", max_rotary_step_deg=90.0),
}


def get_post(name: str) -> PostConfig:
    return CERTIFIED_POSTS.get(name, PostConfig())
