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
    ae: float = 0.0           # radial width of cut, mm (<=0 -> auto 0.5*tool_dia)
    Kte: float = 20.0         # tangential EDGE coeff, N/mm
    Kre: float = 18.0         # radial edge coeff, N/mm
    spindle_power_kW: float = 15.0   # available spindle power
    max_force_N: float = 2500.0      # max allowed resultant cutting force
    E: float = 600.0e3        # Young's modulus, N/mm^2 (carbide ~600 GPa)
    dev_allow_um: float = 50.0   # allowed deflection-induced error, micron
    feed_max_mm_min: float = 6000.0  # user/machine feed ceiling

    def nominal_feed_mm_min(self) -> float:
        return self.fz * self.n_teeth * self.rpm

    def _ae(self) -> float:
        return self.ae if (self.ae and self.ae > 0.0) else 0.5 * self.tool_dia

    def cutting_forces(self, fz: float, ae: float = None) -> dict:
        """Mechanistic milling-force model (Altintas): integrate the per-tooth
        tangential/radial forces (cutting coeff Kt/Kr*Kt + edge Kte/Kre) over the
        engaged arc and teeth across one tooth pitch. Returns peak & mean
        resultant force (N), mean spindle power (W) and torque (N*m).

        Chip thickness h(phi)=fz*sin(phi); engagement arc phi in [0, acos(1-ae/R)]
        for radial width ae and cutter radius R; teeth at phi_k = theta+k*2pi/N.
        """
        import numpy as np
        R = 0.5 * self.tool_dia
        ae = self._ae() if ae is None else ae
        ae = max(1e-6, min(ae, 2.0 * R))
        phi_ex = math.acos(max(-1.0, min(1.0, 1.0 - ae / R)))   # exit angle
        Ktc, Krc = self.Kt, self.Kr * self.Kt
        N = self.n_teeth
        th = np.linspace(0.0, 2.0 * math.pi / N, 200, endpoint=False)
        Sx = np.zeros_like(th); Sy = np.zeros_like(th); St = np.zeros_like(th)
        for k in range(N):
            phi = np.mod(th + k * 2.0 * math.pi / N, 2.0 * math.pi)
            eng = phi <= phi_ex                                   # engaged teeth
            s = np.sin(phi)
            Ft = self.ap * (Ktc * fz * s + self.Kte) * eng
            Fr = self.ap * (Krc * fz * s + self.Kre) * eng
            Sx += -Ft * np.cos(phi) - Fr * np.sin(phi)           # tool-frame Fx
            Sy += Ft * np.sin(phi) - Fr * np.cos(phi)            # tool-frame Fy
            St += Ft                                             # tangential (torque)
        Fmag = np.hypot(Sx, Sy)
        omega = 2.0 * math.pi * self.rpm / 60.0                   # rad/s
        torque = St * (R * 1e-3)                                  # N*m (R in m)
        power = torque * omega                                    # W
        return dict(F_peak=float(Fmag.max()), F_mean=float(Fmag.mean()),
                    power_W=float(power.mean()), torque_Nm=float(torque.mean()),
                    phi_ex=phi_ex)

    def mechanistic_feed_cap_mm_min(self, ae: float = None) -> float:
        """Largest feed (mm/min) whose mechanistic forces respect ALL of: tool
        deflection <= dev_allow, resultant force <= max_force_N, and spindle power
        <= spindle_power_kW. Forces grow monotonically with fz, so bisect fz."""
        d = self.tool_dia
        I = 0.8 * math.pi * d**4 / 64.0
        L = self.flute_len
        dev_allow = self.dev_allow_um * 1e-3                      # mm
        pmax = self.spindle_power_kW * 1000.0

        def ok(fz):
            f = self.cutting_forces(fz, ae)
            defl = f["F_peak"] * L**3 / (3.0 * self.E * I)        # cantilever, mm
            return (defl <= dev_allow and f["F_peak"] <= self.max_force_N
                    and f["power_W"] <= pmax)

        if not ok(1e-6):
            return 0.0                                            # even a sliver overloads
        lo, hi = 1e-6, 1.0
        if ok(hi):
            fz = hi
        else:
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                if ok(mid):
                    lo = mid
                else:
                    hi = mid
            fz = lo
        return fz * self.n_teeth * self.rpm

    # backward-compatible alias (now mechanistic)
    def deflection_feed_cap_mm_min(self) -> float:
        return self.mechanistic_feed_cap_mm_min()

    def effective_feed_mm_min(self) -> float:
        # floor at 1 mm/min: a 0 cap means the cut is infeasible on this
        # spindle/tool (forces alone overload it) -- surfaced via cutting_forces
        # / feed_feasible, while keeping the downstream feed schedule numerically
        # well-posed.
        return max(1.0, min(self.feed_max_mm_min,
                            self.mechanistic_feed_cap_mm_min(),
                            self.nominal_feed_mm_min()))

    def feed_feasible(self) -> bool:
        """False when the mechanistic forces overload the spindle/tool even at a
        vanishing feed (the cut itself is infeasible, not just feed-limited)."""
        return self.mechanistic_feed_cap_mm_min() > 0.0


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
