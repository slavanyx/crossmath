"""Application model: parameter schema, strategy registry, and compute glue.

This layer is Qt-free and fully headless-testable. The Qt views (params panel,
3D view, plots, results) are thin and read from here; adding a parameter or a
strategy is a one-line change in PARAM_SPEC / STRATEGIES.
"""
from __future__ import annotations

from dataclasses import asdict
import numpy as np

from ..pipeline import Params, compute
from ..process import MachineLimits, ProcessParams


# Strategy registry -- extend by adding the key here and in optimize.optimize_blade
STRATEGIES = ["global", "smoothed", "minmax", "two_point"]

# Parameter schema drives auto-generated editors. Each entry:
#   (key, label, lo, hi, step, kind, group)
PARAM_SPEC = [
    ("R",         "Cutter radius (mm)",     0.5,  30.0, 0.5,  "float", "Tool / strategy"),
    ("gamma",     "Tool taper γ (rad)",     0.0,  0.5,  0.01, "float", "Tool / strategy"),
    ("mu",        "Smoothness weight µ",    0.0,  20.0, 0.5,  "float", "Tool / strategy"),
    ("nsweeps",   "Global sweeps",          1,    12,   1,    "int",   "Tool / strategy"),
    ("smooth_window", "Smooth window",      1,    21,   2,    "int",   "Tool / strategy"),
    ("twist",     "Blade twist (rad)",      0.0,  2.0,  0.05, "float", "Geometry"),
    ("wrap",      "Blade wrap (rad)",       0.0,  2.0,  0.05, "float", "Geometry"),
    ("r_hub",     "Hub radius (mm)",        5.0, 100.0, 1.0,  "float", "Geometry"),
    ("r_shroud",  "Shroud radius (mm)",     5.0, 150.0, 1.0,  "float", "Geometry"),
    ("z_span",    "Blade height (mm)",      5.0, 100.0, 1.0,  "float", "Geometry"),
    ("nu",        "Stations",              20,   200,   5,    "int",   "Geometry"),
    ("n_blades",  "Blade count",            3,    40,    1,   "int",   "Geometry"),
]

# Machine / process editors (these map to nested objects, not Params fields).
MACHINE_SPEC = [
    ("v_rot",     "Rotary vmax (rad/s)",    0.05, 3.0,  0.05, "float", "Machine / process"),
    ("feed_max",  "Feed ceiling (mm/min)",  200, 20000, 200,  "int",   "Machine / process"),
    ("dev_allow", "Deflection budget (µm)", 5,   500,   5,    "int",   "Machine / process"),
]


class AppModel:
    """Holds parameter values and runs the pipeline. No Qt imports."""

    def __init__(self):
        d = asdict(Params())
        self.values = {k: d[k] for (k, *_rest) in PARAM_SPEC}
        self.strategy = "global"
        # machine/process kept as nested objects, exposed via a few keys
        self.values["v_rot"] = MachineLimits().v_rot
        self.values["feed_max"] = ProcessParams().feed_max_mm_min
        self.values["dev_allow"] = ProcessParams().dev_allow_um
        self.rails = None  # optional external (a, b)
        self.frf = None    # optional measured FRF (freq, reG, imG)

    def build_params(self, strategy=None) -> Params:
        v = self.values
        return Params(
            nu=int(v["nu"]), r_hub=v["r_hub"], r_shroud=v["r_shroud"],
            z_span=v["z_span"], wrap=v["wrap"], twist=v["twist"],
            n_blades=int(v["n_blades"]), R=v["R"],
            strategy=strategy or self.strategy,
            smooth_window=int(v["smooth_window"]),
            mu=v["mu"], gamma=v["gamma"], nsweeps=int(v["nsweeps"]),
            machine=MachineLimits(v_rot=v["v_rot"]),
            process=ProcessParams(feed_max_mm_min=v["feed_max"],
                                  dev_allow_um=v["dev_allow"]),
            rails=self.rails,
        )

    def compute_current(self) -> dict:
        return compute(self.build_params())

    def compute_compare(self) -> dict:
        """Peak-deviation array per strategy (for the deviation chart)."""
        out = {}
        for s in STRATEGIES:
            out[s] = compute(self.build_params(strategy=s))["dev"]
        return out

    def compute_compare_full(self) -> dict:
        """Per strategy: deviation array plus scalar metrics (µm, jerk, cycle).

        One pipeline run per strategy; reused for both the deviation and
        comparison charts so the GUI never recomputes on the UI thread.
        """
        out = {}
        for s in STRATEGIES:
            r = compute(self.build_params(strategy=s))
            out[s] = dict(dev=r["dev"],
                          dev_um=float(r["dev"].max() * 1000.0),
                          jerk=float(r["orient_jerk"]),
                          cycle_s=float(r["cycle_time_s"]))
        return out
