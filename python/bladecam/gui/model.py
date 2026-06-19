"""Application model: parameter schema, strategy registry, and compute glue.

This layer is Qt-free and fully headless-testable. The Qt views (params panel,
3D view, plots, results) are thin and read from here; adding a parameter or a
strategy is a one-line change in PARAM_SPEC / STRATEGIES.
"""
from __future__ import annotations

from dataclasses import asdict
import numpy as np

from dataclasses import replace
from ..pipeline import Params, compute
from ..process import MachineLimits, ProcessParams
from .. import machine as machine_lib
from .. import presets as preset_lib


# Strategy registry -- extend by adding the key here and in optimize.optimize_blade
STRATEGIES = ["global", "smoothed", "minmax", "two_point"]

# Parameter schema drives auto-generated editors. Each entry:
#   (key, label, lo, hi, step, kind, group)
PARAM_SPEC = [
    ("R",         "Cutter radius (mm)",     0.5,  30.0, 0.5,  "float", "Tool / strategy"),
    ("gamma",     "Tool taper γ (rad)",     0.0,  0.5,  0.01, "float", "Tool / strategy"),
    ("mu",        "Smoothness weight µ",    0.0,  20.0, 0.5,  "float", "Tool / strategy"),
    ("barrel_R",  "Barrel arc radius (0=off)", 0.0, 2000.0, 10.0, "float", "Tool / strategy"),
    ("barrel_pos", "Barrel widest pos (mm)", 0.0, 60.0, 1.0,  "float", "Tool / strategy"),
    ("swept_weight", "Swept-overcut weight", 0.0,  5.0,  0.1,  "float", "Tool / strategy"),
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
    ("kind",      "Kinematics 0=table 1=head", 0, 1,    1,    "int",   "Machine / process"),
]


class AppModel:
    """Holds parameter values and runs the pipeline. No Qt imports."""

    def __init__(self):
        d = asdict(Params())
        self.values = {k: d[k] for (k, *_rest) in PARAM_SPEC}
        self.strategy = "global"
        # machine/process kept as nested objects, exposed via a few keys
        self.values["v_rot"] = MachineLimits().v_rot
        self.values["kind"] = MachineLimits().kind
        self.values["feed_max"] = ProcessParams().feed_max_mm_min
        self.values["dev_allow"] = ProcessParams().dev_allow_um
        self.values["nv"] = Params().nv               # strategy-preset fields with
        self.values["swept_window"] = Params().swept_window   # no dedicated editor
        self.rails = None  # optional external (a, b)
        self.frf = None    # optional measured FRF (freq, reG, imG)
        # selected machine profile (a Machine; editable via the config editor)
        self.machine_name = "Generic 5-axis trunnion"
        self.machine = replace(machine_lib.get_machine(self.machine_name))
        # full tool (ProcessParams) preset; feed/dev editors override on build
        self.tool = ProcessParams()
        # OrcaSlicer-style preset store + the active preset name per category
        self.presets = preset_lib.PresetStore()
        self.preset_names = {"machine": self.machine_name,
                             "tool": "12 mm 4FL carbide",
                             "strategy": "Flank finish (global)"}

    def select_machine(self, name):
        """Switch to a default machine profile (resets any edits)."""
        self.machine_name = name
        self.machine = replace(machine_lib.get_machine(name))

    # ---- preset application / capture (OrcaSlicer-style) ----
    def apply_preset(self, kind, name):
        """Load a saved/built-in preset into the live model state."""
        d = self.presets.load(kind, name)
        if kind == "machine":
            self.machine = preset_lib.machine_from_dict(d)
            self.machine_name = self.machine.name
            self.values["v_rot"] = self.machine.v_rot
            self.values["kind"] = self.machine.kind
        elif kind == "tool":
            self.tool = preset_lib.tool_from_dict(d)
            self.values["feed_max"] = self.tool.feed_max_mm_min
            self.values["dev_allow"] = self.tool.dev_allow_um
        elif kind == "strategy":
            for k, val in d.items():
                if k == "strategy":
                    self.strategy = val
                elif k in self.values:
                    self.values[k] = val
        self.preset_names[kind] = name

    def capture_preset(self, kind) -> dict:
        """Serialise the current live state for `kind` into a preset dict."""
        if kind == "machine":
            return preset_lib.machine_to_dict(self._live_machine())
        if kind == "tool":
            return preset_lib.tool_to_dict(self._live_tool())
        if kind == "strategy":
            v = self.values
            out = {}
            for k in preset_lib.STRATEGY_FIELDS:
                out[k] = self.strategy if k == "strategy" else v[k]
            return out
        raise ValueError(kind)

    def save_preset(self, kind, name) -> str:
        path = self.presets.save(kind, name, self.capture_preset(kind))
        self.preset_names[kind] = name
        return path

    def _live_machine(self):
        v = self.values
        return replace(self.machine, v_rot=v["v_rot"], kind=int(v["kind"]),
                       name=self.machine.name)

    def _live_tool(self):
        v = self.values
        return replace(self.tool, feed_max_mm_min=v["feed_max"],
                       dev_allow_um=v["dev_allow"])

    def build_params(self, strategy=None) -> Params:
        v = self.values
        return Params(
            nu=int(v["nu"]), r_hub=v["r_hub"], r_shroud=v["r_shroud"],
            z_span=v["z_span"], wrap=v["wrap"], twist=v["twist"],
            n_blades=int(v["n_blades"]), R=v["R"],
            strategy=strategy or self.strategy,
            smooth_window=int(v["smooth_window"]),
            nv=int(v["nv"]),
            mu=v["mu"], gamma=v["gamma"], nsweeps=int(v["nsweeps"]),
            barrel_R=v["barrel_R"], barrel_pos=v["barrel_pos"],
            swept_weight=v["swept_weight"], swept_window=int(v["swept_window"]),
            # the selected machine profile drives reachability + limits; the
            # v_rot/kind editors fine-tune the active profile
            machine=self._live_machine(),
            process=self._live_tool(),
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
