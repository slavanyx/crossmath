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
        spindle_dia=45.0, spindle_len=90.0, table_radius=120.0),
    "Large gantry 5-axis": Machine(
        name="Large gantry 5-axis", kind=1,
        x_range=(-1500.0, 1500.0), y_range=(-1000.0, 1000.0), z_range=(-800.0, 200.0),
        a_range=(math.radians(-110.0), math.radians(110.0)),
        c_range=(math.radians(-360.0), math.radians(360.0)),
        v_lin=40.0, a_lin=500.0, v_rot=0.4, a_rot=4.0,
        spindle_dia=90.0, spindle_len=180.0, table_radius=800.0),
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
