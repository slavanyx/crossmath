"""Minimal 5-axis G-code post-processor (table-table A-C, angles in degrees)."""
from __future__ import annotations

import numpy as np


def to_gcode(machine_path: np.ndarray, feed_mm_min: float,
             rpm: float = 12000.0, header: str = "BladeCAM flank finish",
             move_times=None) -> str:
    """Emit a 5-axis G-code program from a machine path (n,5)=[X,Y,Z,A,C], A,C
    in radians.

    If `move_times` (seconds per cutting move, length n-1) is given, the cutting
    moves use inverse-time feed (G93) so each move realises its TOPP-optimised
    duration regardless of the linear/rotary mix; otherwise a constant
    units/min (G94) feed is used.
    """
    deg = 180.0 / np.pi
    lines = [
        f"(% {header})",
        "G21 (mm)",
        "G90 (absolute)",
        f"S{rpm:.0f} M3",
        "G0 Z50.0",
    ]
    x, y, z, A, C = machine_path[0]
    lines.append(f"G0 X{x:.4f} Y{y:.4f} A{A*deg:.4f} C{C*deg:.4f}")
    lines.append("G94 (units/min)")
    lines.append(f"G1 Z{z:.4f} F{feed_mm_min:.0f}")            # plunge
    if move_times is not None:
        lines.append("G93 (inverse time)")
        for k, (x, y, z, A, C) in enumerate(machine_path[1:]):
            f_inv = 60.0 / max(float(move_times[k]), 1e-9)     # 1/min
            lines.append(f"G1 X{x:.4f} Y{y:.4f} Z{z:.4f} "
                         f"A{A*deg:.4f} C{C*deg:.4f} F{f_inv:.2f}")
        lines.append("G94 (units/min)")
    else:
        for x, y, z, A, C in machine_path[1:]:
            lines.append(f"G1 X{x:.4f} Y{y:.4f} Z{z:.4f} "
                         f"A{A*deg:.4f} C{C*deg:.4f} F{feed_mm_min:.0f}")
    lines += ["G0 Z50.0", "M5", "M30"]
    return "\n".join(lines) + "\n"


def to_heidenhain(contact: np.ndarray, axis: np.ndarray, feed_mm_min: float,
                  rpm: float = 12000.0, header: str = "BladeCAM flank finish",
                  move_times=None, tool: int = 1, prog: str = "BLADECAM",
                  clear_z: float = 50.0) -> str:
    """Heidenhain klartext (conversational) 5-axis program with TCPM.

    Posts the tool-tip path `contact` (n,3) and the tool-axis unit vectors
    `axis` (n,3) directly, and lets the control resolve the rotary axes via
    FUNCTION TCPM. This is the correct, machine-kinematics-independent way to
    post 5-axis simultaneous motion (vs baking A/C joint values into ISO):
    LN blocks carry the orientation as a TX/TY/TZ vector, so the SAME program
    runs on any TCPM-capable Heidenhain regardless of head/table layout.

    If `move_times` (s per cutting move, length n-1) is given, each move's feed
    is the tool-tip distance over its TOPP-optimised duration (mm/min), so the
    realised schedule matches the time-optimal plan.
    """
    contact = np.asarray(contact, float)
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis, axis=1, keepdims=True)
    n = contact.shape[0]

    def ln(p, t, f=None, fmax=False):
        s = (f"LN X{p[0]:+.3f} Y{p[1]:+.3f} Z{p[2]:+.3f} "
             f"TX{t[0]:+.6f} TY{t[1]:+.6f} TZ{t[2]:+.6f}")
        return s + (" FMAX" if fmax else f" F{f:.0f}")

    out = [
        f"BEGIN PGM {prog} MM",
        f"; {header}",
        f"TOOL CALL {tool} Z S{rpm:.0f}",
        "M3",
        f"L Z+{clear_z:.3f} R0 FMAX",
        "FUNCTION TCPM F TCP AXIS POS PATHCTRL AXIS",
    ]
    p0, t0 = contact[0], axis[0]
    # rapid to the start above the part, then engage along the tool vector
    out.append(ln([p0[0], p0[1], clear_z], t0, fmax=True))
    out.append(ln(p0, t0, f=feed_mm_min))                 # feed in (plunge)
    for k in range(1, n):
        if move_times is not None:
            dist = float(np.linalg.norm(contact[k] - contact[k-1]))
            f = 60.0 * dist / max(float(move_times[k-1]), 1e-9)
            f = min(max(f, 1.0), 1e5)                      # clamp to sane range
        else:
            f = feed_mm_min
        out.append(ln(contact[k], axis[k], f=f))
    out += [
        "FUNCTION RESET TCPM",
        f"L Z+{clear_z:.3f} R0 FMAX",
        "M5",
        "M30",
        f"END PGM {prog} MM",
    ]
    return "\n".join(out) + "\n"
