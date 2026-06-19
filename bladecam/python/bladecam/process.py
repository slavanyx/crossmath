"""Machine limits and cutting-process feed caps (Phase 4 technological layer)."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class MachineLimits:
    """Per-axis kinematic limits. Linear in mm, rotary in radians."""
    v_lin: float = 50.0      # mm/s
    a_lin: float = 800.0     # mm/s^2
    v_rot: float = 0.6       # rad/s  (~34 deg/s -- typical 5-axis rotary)
    a_rot: float = 6.0       # rad/s^2
    kind: int = 0            # 0 = table-table, 1 = head-head (spindle tilt)

    def vmax(self):
        return [self.v_lin, self.v_lin, self.v_lin, self.v_rot, self.v_rot]

    def amax(self):
        return [self.a_lin, self.a_lin, self.a_lin, self.a_rot, self.a_rot]


@dataclass
class ProcessParams:
    """Cutting-process parameters for feed-cap estimation."""
    n_teeth: int = 4
    fz: float = 0.05          # feed per tooth, mm
    rpm: float = 12000.0
    Kt: float = 800.0         # tangential cutting coeff, N/mm^2
    Kr: float = 0.3           # radial/tangential ratio
    tool_dia: float = 12.0    # mm
    flute_len: float = 35.0   # mm (cantilever length)
    holder_dia: float = 25.0  # mm (shank/holder diameter)
    holder_gap: float = 2.0   # mm (clearance below holder)
    holder_len: float = 40.0  # mm (modelled holder length)
    spindle_dia: float = 60.0 # mm (spindle-nose diameter, full-machine check)
    spindle_gap: float = 5.0  # mm (clearance below the spindle nose)
    spindle_len: float = 80.0 # mm (modelled spindle-nose length)
    ap: float = 4.0           # axial depth of cut, mm
    E: float = 600.0e3        # Young's modulus, N/mm^2 (carbide ~600 GPa)
    dev_allow_um: float = 50.0   # allowed deflection-induced error, micron
    feed_max_mm_min: float = 6000.0  # user/machine feed ceiling

    def nominal_feed_mm_min(self) -> float:
        return self.fz * self.n_teeth * self.rpm

    def deflection_feed_cap_mm_min(self) -> float:
        """Cap feed so cutting-force tool deflection stays under dev_allow.

        F = Kt*ap*fz*sqrt(1+Kr^2); cantilever delta = F*L^3/(3 E I),
        I = 0.8*pi*d^4/64. Solve for the fz that yields delta = dev_allow,
        then feed = fz * n_teeth * rpm.
        """
        d = self.tool_dia
        I = 0.8 * math.pi * d**4 / 64.0
        L = self.flute_len
        dev_allow = self.dev_allow_um * 1e-3  # mm
        # delta = (Kt*ap*fz*sqrt(1+Kr^2)) * L^3 / (3 E I)  -> fz_max
        k = self.Kt * self.ap * math.sqrt(1.0 + self.Kr**2) * L**3 / (3.0 * self.E * I)
        if k <= 0:
            return self.feed_max_mm_min
        fz_max = dev_allow / k
        return fz_max * self.n_teeth * self.rpm

    def effective_feed_mm_min(self) -> float:
        return min(self.feed_max_mm_min,
                   self.deflection_feed_cap_mm_min(),
                   self.nominal_feed_mm_min())


def read_frf_csv(path: str):
    """Read a measured tool-tip receptance CSV (columns: freq_hz, re, im) in
    m/N or mm/N. Returns (freq_hz, reG_mm_per_N, imG_mm_per_N).

    Values are auto-scaled to mm/N if they look like m/N (|G| < 1e-3)."""
    import numpy as np
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    freq, re, im = d[:, 0], d[:, 1], d[:, 2]
    if np.nanmax(np.abs(re)) < 1e-3:     # looks like m/N -> mm/N
        re = re * 1e3; im = im * 1e3
    return freq, re, im


def chatter_alim_mm(Kt: float, n_teeth: int, re_frf_min: float = -2.0e-6) -> float:
    """Single-DOF regenerative chatter limiting depth (display proxy).

    a_lim = -1 / (2 * Kt * Re[FRF]_min), with Re[FRF]_min the most negative
    real part of the tool-tip frequency response (m/N -> mm/N here).
    """
    if re_frf_min >= 0:
        return float("inf")
    return -1.0 / (2.0 * Kt * re_frf_min)
