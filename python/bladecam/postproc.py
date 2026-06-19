"""Minimal 5-axis G-code post-processor (table-table A-C, angles in degrees)."""
from __future__ import annotations

import numpy as np


def to_gcode(machine_path: np.ndarray, feed_mm_min: float,
             rpm: float = 12000.0, header: str = "BladeCAM flank finish") -> str:
    """Emit a simple G-code program from a machine path (n,5) = [X,Y,Z,A,C]
    with A,C in radians. Linear 5-axis moves at the given feed."""
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
    lines.append(f"G1 Z{z:.4f} F{feed_mm_min:.0f}")
    for x, y, z, A, C in machine_path[1:]:
        lines.append(f"G1 X{x:.4f} Y{y:.4f} Z{z:.4f} "
                     f"A{A*deg:.4f} C{C*deg:.4f} F{feed_mm_min:.0f}")
    lines += ["G0 Z50.0", "M5", "M30"]
    return "\n".join(lines) + "\n"
