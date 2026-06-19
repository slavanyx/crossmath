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
