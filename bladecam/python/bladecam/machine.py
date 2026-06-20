"""Machine profiles: kinematic limits, travel/rotary envelopes and structural
geometry for full-machine verification, plus a library of default machines.

A Machine is a drop-in for the older MachineLimits (same kind / vmax() / amax()
used by TOPP) but adds the axis-travel and rotary ranges needed to answer "will
this toolpath actually run on THIS machine?" (reachability) and the structural
envelope (spindle housing, table) for full-machine collision.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class Machine:
    name: str = "Generic 5-axis trunnion"
    kind: int = 0                      # 0 = table-table (A-C), 1 = head-head
    # linear travel ranges (mm) and rotary ranges (rad)
    x_range: tuple = (-400.0, 400.0)
    y_range: tuple = (-400.0, 400.0)
    z_range: tuple = (-350.0, 350.0)
    a_range: tuple = (math.radians(-120.0), math.radians(120.0))
    c_range: tuple = (math.radians(-360.0), math.radians(360.0))
    # kinematic limits for the time-optimal feed (linear mm/s, rotary rad/s)
    v_lin: float = 50.0
    a_lin: float = 800.0
    v_rot: float = 0.6
    a_rot: float = 6.0
    # structural envelope (for full-machine collision)
    spindle_dia: float = 60.0          # mm, spindle-nose / housing diameter
    spindle_len: float = 120.0         # mm, modelled housing length
    table_radius: float = 300.0        # mm, trunnion table the part sits on
    # structural KINEMATIC links (capsule model) for full-machine self-collision:
    # the trunnion cradle yoke (two side posts straddling the A axis + a bottom
    # cross-beam) tilts/rotates with the table; the machine column is base-fixed.
    cradle_span: float = 220.0         # mm, half-spacing of the trunnion posts (0 = no cradle)
    cradle_dia: float = 90.0           # mm, trunnion post/beam diameter
    cradle_drop: float = 160.0         # mm, post length below the table top
    column_dia: float = 0.0            # mm, machine-column diameter (0 = no column)
    column_offset: float = 250.0       # mm, column distance behind the work (+Y)
    column_height: float = 500.0       # mm, modelled column height above the table

    # --- drop-in for MachineLimits (TOPP) ---
    def vmax(self):
        return [self.v_lin, self.v_lin, self.v_lin, self.v_rot, self.v_rot]

    def amax(self):
        return [self.a_lin, self.a_lin, self.a_lin, self.a_rot, self.a_rot]


def reachability(m: "Machine", machine_path: np.ndarray, tol: float = 1e-6):
    """Check an IK machine path (n,5)=[X,Y,Z,A,C] against the machine envelope.

    Returns dict axis -> excess (how far outside its limit, mm or rad); empty
    means the whole toolpath is reachable on this machine."""
    p = np.asarray(machine_path, float)
    ranges = {"X": m.x_range, "Y": m.y_range, "Z": m.z_range,
              "A": m.a_range, "C": m.c_range}
    viol = {}
    for j, ax in enumerate(("X", "Y", "Z", "A", "C")):
        lo, hi = ranges[ax]
        col = p[:, j]
        excess = max(0.0, lo - float(col.min()), float(col.max()) - hi)
        if excess > tol:
            viol[ax] = excess
    return viol


# --- default machine library (select / edit in the GUI) ---
DEFAULT_MACHINES = {
    "Generic 5-axis trunnion": Machine(),
    "Compact blisk cell": Machine(
        name="Compact blisk cell", kind=0,
        x_range=(-150.0, 150.0), y_range=(-150.0, 150.0), z_range=(-200.0, 120.0),
        a_range=(math.radians(-30.0), math.radians(120.0)),
        c_range=(math.radians(-360.0), math.radians(360.0)),
        v_lin=80.0, a_lin=1500.0, v_rot=1.2, a_rot=20.0,
        spindle_dia=45.0, spindle_len=90.0, table_radius=120.0,
        cradle_span=110.0, cradle_dia=60.0, cradle_drop=110.0),
    "Large gantry 5-axis": Machine(
        name="Large gantry 5-axis", kind=1,
        x_range=(-1500.0, 1500.0), y_range=(-1000.0, 1000.0), z_range=(-800.0, 200.0),
        a_range=(math.radians(-110.0), math.radians(110.0)),
        c_range=(math.radians(-360.0), math.radians(360.0)),
        v_lin=40.0, a_lin=500.0, v_rot=0.4, a_rot=4.0,
        spindle_dia=90.0, spindle_len=180.0, table_radius=800.0,
        cradle_span=0.0,            # head-head: workpiece fixed, no tilting cradle
        column_dia=0.0),
}


def get_machine(name: str) -> "Machine":
    return DEFAULT_MACHINES.get(name, Machine())


def structure_obstacles(m: "Machine", mount_z: float, depth: float = 80.0,
                        n: int = 24) -> np.ndarray:
    """Point cloud of the machine's structural envelope in the PART frame: the
    trunnion TABLE the workpiece is mounted on -- a disc of radius table_radius
    whose top sits at z=mount_z (below the part), extending downward by `depth`.

    In table-table A-C kinematics the table tilts/rotates WITH the part, so in
    part coordinates it is a static obstacle: the tool assembly (holder/spindle)
    must clear it, which it may not at a steep lead/lean tilt or deep reach.
    Returns (npts,3); sample the top disc + the cylindrical rim.
    """
    R = m.table_radius
    th = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    pts = []
    # top disc (rings) at z = mount_z
    for r in np.linspace(0.15 * R, R, max(3, n // 3)):
        pts.append(np.column_stack([r*np.cos(th), r*np.sin(th),
                                    np.full_like(th, mount_z)]))
    # cylindrical rim down to mount_z - depth
    for z in np.linspace(mount_z - depth, mount_z, max(3, n // 4)):
        pts.append(np.column_stack([R*np.cos(th), R*np.sin(th),
                                    np.full_like(th, z)]))
    return np.ascontiguousarray(np.vstack(pts))


# --- structural kinematic-link collision (capsule model) ---------------------
# Rotation matrices MATCHING kinematics.f90's convention (world->part = Rz(C)Rx(A)
# about the pivot), so the structure is placed in the part frame exactly as the
# inverse kinematics places the tool.
def _rotx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rotz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tool_branch_capsules(q0: np.ndarray, alpha: np.ndarray,
                         seg_R, seg_lo, seg_hi) -> np.ndarray:
    """Tool-assembly stack as part-frame capsules, (nu, nseg, 7) with the last
    axis [p0(3), p1(3), radius]. Segment k spans axial [seg_lo[k], seg_hi[k]]
    from q0 along the unit tool axis with radius seg_R[k]."""
    q0 = np.asarray(q0, float); alpha = np.asarray(alpha, float)
    seg_R = np.asarray(seg_R, float)
    seg_lo = np.asarray(seg_lo, float); seg_hi = np.asarray(seg_hi, float)
    nu = q0.shape[0]; nseg = seg_R.shape[0]
    nrm = np.linalg.norm(alpha, axis=1, keepdims=True)
    ah = np.divide(alpha, nrm, out=np.zeros_like(alpha), where=nrm > 0)
    caps = np.empty((nu, nseg, 7))
    for k in range(nseg):
        caps[:, k, 0:3] = q0 + seg_lo[k] * ah
        caps[:, k, 3:6] = q0 + seg_hi[k] * ah
        caps[:, k, 6] = seg_R[k]
    return np.ascontiguousarray(caps)


def _structure_nominal(m: "Machine", mount_z: float):
    """Nominal structural capsules. Cradle posts/beam in the CRADLE frame (= part
    frame at C=0); the column in the BASE frame. Each row is [p0(3),p1(3),r]."""
    cradle = []
    if m.cradle_span > 0.0 and m.cradle_dia > 0.0:
        rr = 0.5 * m.cradle_dia
        top = mount_z                       # cradle top just under the table top
        bot = mount_z - m.cradle_drop
        for sgn in (-1.0, 1.0):             # two trunnion posts straddling the A axis
            x = sgn * m.cradle_span
            cradle.append([x, 0.0, top, x, 0.0, bot, rr])
        cradle.append([-m.cradle_span, 0.0, bot,    # bottom cross-beam (along X)
                        m.cradle_span, 0.0, bot, rr])
    column = []
    if m.column_dia > 0.0:
        rr = 0.5 * m.column_dia
        y = m.column_offset
        column.append([0.0, y, mount_z + m.column_height, 0.0, y, mount_z, rr])
    return (np.array(cradle, float).reshape(-1, 7),
            np.array(column, float).reshape(-1, 7))


def structure_capsules(m: "Machine", machine_path: np.ndarray, pivot,
                       mount_z: float) -> np.ndarray:
    """Structural links as PART-frame capsules over the toolpath, (nu, nb, 7).

    Table-table (kind 0): the cradle is fixed in the cradle frame, which relates
    to the part frame by Rz(C) about the pivot (the C-table spins the part on the
    cradle; both tilt together in A, so there is no relative A motion). The
    column is base-fixed, placed in the part frame by Rz(C)Rx(A). Head-head
    (kind 1): the workpiece is fixed, so the structure is static in the part
    frame (the tool tilts instead, already captured by q0/alpha)."""
    cradle0, column0 = _structure_nominal(m, mount_z)
    nb = cradle0.shape[0] + column0.shape[0]
    p = np.asarray(machine_path, float)
    nu = p.shape[0]
    piv = np.asarray(pivot, float)
    caps = np.empty((nu, nb, 7))
    if nb == 0:
        return np.ascontiguousarray(caps)
    head_head = (m.kind == 1)
    for i in range(nu):
        if head_head:
            Rcr = np.eye(3); Rcol = np.eye(3)
        else:
            Rz = _rotz(p[i, 4])             # C
            Rcr = Rz
            Rcol = Rz @ _rotx(p[i, 3])      # Rz(C) Rx(A)
        row = 0
        for cap in cradle0:
            caps[i, row, 0:3] = Rcr @ (cap[0:3] - piv) + piv
            caps[i, row, 3:6] = Rcr @ (cap[3:6] - piv) + piv
            caps[i, row, 6] = cap[6]
            row += 1
        for cap in column0:
            caps[i, row, 0:3] = Rcol @ (cap[0:3] - piv) + piv
            caps[i, row, 3:6] = Rcol @ (cap[3:6] - piv) + piv
            caps[i, row, 6] = cap[6]
            row += 1
    return np.ascontiguousarray(caps)
