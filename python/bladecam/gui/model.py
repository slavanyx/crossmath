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
from .. import post as post_lib


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
    ("z_offset",  "Shroud z-offset (mm)",   0.0,  50.0, 1.0,  "float", "Geometry"),
    ("nu",        "Stations",              20,   200,   5,    "int",   "Geometry"),
    ("n_blades",  "Blade count",            3,    40,    1,   "int",   "Geometry"),
    ("collision_substeps", "Collision substeps", 0, 8,   1,   "int",   "Setup"),
    ("mount_clearance", "Mount clearance (mm)", 0.0, 200.0, 5.0, "float", "Setup"),
    ("root_fillet_r", "Root fillet trim (mm)", 0.0, 20.0, 0.5, "float", "Setup"),
    ("pivot_x",   "Pivot X (mm)",          -500.0, 500.0, 5.0, "float", "Setup"),
    ("pivot_y",   "Pivot Y (mm)",          -500.0, 500.0, 5.0, "float", "Setup"),
    ("pivot_z",   "Pivot Z (mm)",          -500.0, 500.0, 5.0, "float", "Setup"),
]

# Tool / cutting editors -- map 1:1 to ProcessParams fields (no hardcoding).
TOOL_SPEC = [
    ("tool_dia",   "Tool diameter (mm)",    1.0, 50.0, 0.5,  "float", "Tool / cutting"),
    ("n_teeth",    "Teeth",                 1,   12,   1,    "int",   "Tool / cutting"),
    ("fz",         "Feed/tooth (mm)",       0.005, 0.5, 0.005, "float", "Tool / cutting"),
    ("rpm",        "Spindle rpm",           500, 60000, 500, "float", "Tool / cutting"),
    ("ap",         "Axial depth ap (mm)",   0.1, 50.0, 0.5,  "float", "Tool / cutting"),
    ("ae",         "Radial width ae (0=auto)", 0.0, 50.0, 0.5, "float", "Tool / cutting"),
    ("Kt",         "Kt (N/mm²)",            100, 4000, 50,   "float", "Tool / cutting"),
    ("Kr",         "Kr ratio",              0.05, 1.0, 0.05, "float", "Tool / cutting"),
    ("Kte",        "Edge Kte (N/mm)",       0.0, 200.0, 1.0, "float", "Tool / cutting"),
    ("Kre",        "Edge Kre (N/mm)",       0.0, 200.0, 1.0, "float", "Tool / cutting"),
    ("helix_deg",  "Helix angle (deg)",     0.0, 65.0, 1.0,  "float", "Tool / cutting"),
    ("n_axial",    "Force axial slices",    1,   80,   1,    "int",   "Tool / cutting"),
    ("flute_len",  "Flute length (mm)",     5.0, 120.0, 1.0, "float", "Tool / cutting"),
    ("holder_dia", "Holder dia (mm)",       5.0, 100.0, 1.0, "float", "Tool / cutting"),
    ("holder_gap", "Holder gap (mm)",       0.0, 50.0, 0.5,  "float", "Tool / cutting"),
    ("holder_len", "Holder length (mm)",    5.0, 150.0, 1.0, "float", "Tool / cutting"),
    ("spindle_dia","Spindle dia (mm)",      10.0, 200.0, 1.0, "float", "Tool / cutting"),
    ("spindle_gap","Spindle gap (mm)",      0.0, 50.0, 0.5,  "float", "Tool / cutting"),
    ("spindle_len","Spindle length (mm)",   10.0, 400.0, 5.0, "float", "Tool / cutting"),
    ("spindle_power_kW", "Spindle power (kW)", 0.5, 60.0, 0.5, "float", "Tool / cutting"),
    ("max_force_N", "Max force (N)",        100, 10000, 100, "float", "Tool / cutting"),
    ("E",          "Tool E (N/mm²)",        50e3, 1e6, 10e3, "float", "Tool / cutting"),
    ("dev_allow_um", "Deflection budget (µm)", 1, 500, 5,   "float", "Tool / cutting"),
    ("feed_max_mm_min", "Feed ceiling (mm/min)", 200, 20000, 200, "float", "Tool / cutting"),
]
_TOOL_KEYS = [s[0] for s in TOOL_SPEC]

# Machine quick editors (the full envelope is in the Machine config dialog).
MACHINE_SPEC = [
    ("v_rot",     "Rotary vmax (rad/s)",    0.05, 3.0,  0.05, "float", "Machine / process"),
    ("kind",      "Kinematics 0=table 1=head", 0, 1,    1,    "int",   "Machine / process"),
]


class AppModel:
    """Holds parameter values and runs the pipeline. No Qt imports."""

    def __init__(self):
        d = asdict(Params())
        # most editor keys mirror a Params field; a few (pivot_x/y/z) are derived
        self.values = {k: d[k] for (k, *_rest) in PARAM_SPEC if k in d}
        self.strategy = "global"
        # tool / cutting params -> editable, 1:1 with ProcessParams fields
        td = asdict(ProcessParams())
        for k in _TOOL_KEYS:
            self.values[k] = td[k]
        # machine quick editors + strategy-preset fields without a dedicated editor
        self.values["v_rot"] = MachineLimits().v_rot
        self.values["kind"] = MachineLimits().kind
        self.values["nv"] = Params().nv
        self.values["swept_window"] = Params().swept_window
        px, py, pz = Params().pivot
        self.values["pivot_x"], self.values["pivot_y"], self.values["pivot_z"] = px, py, pz
        self.rails = None  # optional external (a, b)
        self.frf = None    # optional measured FRF (freq, reG, imG)
        # selected machine profile (a Machine; editable via the config editor)
        self.machine_name = "Generic 5-axis trunnion"
        self.machine = replace(machine_lib.get_machine(self.machine_name))
        # selected certified post (control + machine binding)
        self.post_name = "Heidenhain TNC640 — generic trunnion AC"
        self.post = post_lib.get_post(self.post_name)
        # OrcaSlicer-style preset store + the active preset name per category
        self.presets = preset_lib.PresetStore()
        self.preset_names = {"machine": self.machine_name,
                             "tool": "12 mm 4FL carbide",
                             "strategy": "Flank finish (global)",
                             "blade": "Default impeller blade",
                             "post": self.post_name}

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
            for k in _TOOL_KEYS:
                if k in d and d[k] is not None:
                    self.values[k] = d[k]
        elif kind == "strategy":
            for k, val in d.items():
                if k == "strategy":
                    self.strategy = val
                elif k in self.values:
                    self.values[k] = val
        elif kind == "blade":
            for k in preset_lib.BLADE_FIELDS:
                if k in d:
                    self.values[k] = d[k]
        elif kind == "post":
            self.post = preset_lib.post_from_dict(d)
            self.post_name = self.post.name
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
        if kind == "blade":
            return {k: self.values[k] for k in preset_lib.BLADE_FIELDS}
        if kind == "post":
            return preset_lib.post_to_dict(self._live_post())
        raise ValueError(kind)

    def save_preset(self, kind, name) -> str:
        path = self.presets.save(kind, name, self.capture_preset(kind))
        self.preset_names[kind] = name
        return path

    def preset_dirty(self, kind) -> bool:
        """True if the live state for `kind` differs from its saved preset (the
        OrcaSlicer 'modified ●' state)."""
        name = self.preset_names.get(kind)
        if not name:
            return True
        try:
            saved = self.presets.load(kind, name)
        except KeyError:
            return True
        return not preset_lib.presets_equal(self.capture_preset(kind), saved)

    def dirty_kinds(self):
        return [k for k in preset_lib.KINDS if self.preset_dirty(k)]

    def _live_machine(self):
        v = self.values
        return replace(self.machine, v_rot=v["v_rot"], kind=int(v["kind"]),
                       name=self.machine.name)

    def _live_post(self):
        """The active certified post, retargeted to the live machine name so the
        post and the selected machine stay consistent."""
        return replace(self.post, machine_name=self.machine.name)

    def post_program(self, results):
        """Generate the certified-post program for `results` plus its
        certification report. Returns (text, report)."""
        cfg = replace(self._live_post(), machine_name=self._live_machine().name)
        text = post_lib.generate(cfg, results["contact"], results["alpha"],
                                 results["machine_path"], results["feed_cap_mm_min"],
                                 results.get("move_times_s"))
        rep = post_lib.certify(cfg, results["machine_path"], results["contact"],
                               self.build_params().pivot,
                               results["feed_cap_mm_min"],
                               results.get("move_times_s"), machine=self._live_machine())
        return text, rep

    def _live_tool(self) -> ProcessParams:
        v = self.values
        kw = {k: v[k] for k in _TOOL_KEYS}
        kw["n_teeth"] = int(kw["n_teeth"])
        kw["n_axial"] = int(kw["n_axial"])
        return ProcessParams(**kw)

    def build_params(self, strategy=None) -> Params:
        v = self.values
        return Params(
            nu=int(v["nu"]), r_hub=v["r_hub"], r_shroud=v["r_shroud"],
            z_span=v["z_span"], z_offset=v["z_offset"], wrap=v["wrap"],
            twist=v["twist"], n_blades=int(v["n_blades"]), R=v["R"],
            strategy=strategy or self.strategy,
            smooth_window=int(v["smooth_window"]),
            nv=int(v["nv"]),
            mu=v["mu"], gamma=v["gamma"], nsweeps=int(v["nsweeps"]),
            barrel_R=v["barrel_R"], barrel_pos=v["barrel_pos"],
            swept_weight=v["swept_weight"], swept_window=int(v["swept_window"]),
            collision_substeps=int(v["collision_substeps"]),
            mount_clearance=v["mount_clearance"],
            root_fillet_r=v["root_fillet_r"],
            pivot=(v["pivot_x"], v["pivot_y"], v["pivot_z"]),
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
