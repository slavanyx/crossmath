"""OrcaSlicer-style preset system: named Machine / Tool / Strategy profiles.

Three independent preset categories (mirroring Orca's Printer / Filament /
Process):
  - machine  : a Machine profile (kinematics, travel/rotary envelope, structure)
  - tool     : a ProcessParams set (tool geometry, holder/spindle, cutting coeffs)
  - strategy : the optimisation/tool-shape parameters (strategy, R, mu, gamma,
               barrel, swept weight, sweeps, ...)

Each category has built-in (code-defined, read-only) presets plus user presets
saved as JSON under a presets root (env BLADECAM_PRESETS or ~/.bladecam/presets).
The store is Qt-free and headless-testable; the GUI is a thin selector on top.
"""
from __future__ import annotations

import dataclasses
import json
import os

from . import machine as machine_lib
from . import post as post_lib
from .process import ProcessParams, coeffs_for_material


KINDS = ("machine", "tool", "strategy", "blade", "post")

# fields that make up a "strategy" preset (a subset of pipeline.Params)
STRATEGY_FIELDS = ("strategy", "R", "nv", "mu", "gamma", "barrel_R", "barrel_pos",
                   "nsweeps", "smooth_window", "swept_weight", "swept_window")
# fields that make up a "blade"/job preset (the part geometry)
BLADE_FIELDS = ("nu", "r_hub", "r_shroud", "z_span", "z_offset", "wrap",
                "twist", "n_blades")
_MACHINE_RANGE_FIELDS = ("x_range", "y_range", "z_range", "a_range", "c_range")


# ---- serialization (object <-> plain JSON-able dict) ----------------------
def machine_to_dict(m) -> dict:
    return dataclasses.asdict(m)


def machine_from_dict(d: dict):
    d = dict(d)
    for k in _MACHINE_RANGE_FIELDS:
        if k in d and d[k] is not None:
            d[k] = tuple(d[k])
    valid = {f.name for f in dataclasses.fields(machine_lib.Machine)}
    return machine_lib.Machine(**{k: v for k, v in d.items() if k in valid})


def tool_to_dict(p: ProcessParams) -> dict:
    return dataclasses.asdict(p)


def tool_from_dict(d: dict) -> ProcessParams:
    valid = {f.name for f in dataclasses.fields(ProcessParams)}
    return ProcessParams(**{k: v for k, v in d.items() if k in valid})


def post_to_dict(c) -> dict:
    return post_lib.to_dict(c)


def post_from_dict(d: dict):
    return post_lib.from_dict(d)


# ---- built-in preset libraries -------------------------------------------
def _builtin_machines() -> dict:
    return {n: machine_to_dict(m) for n, m in machine_lib.DEFAULT_MACHINES.items()}


def _builtin_tools() -> dict:
    return {
        "12 mm 4FL carbide": tool_to_dict(ProcessParams()),
        "8 mm 3FL finishing": tool_to_dict(ProcessParams(
            tool_dia=8.0, n_teeth=3, fz=0.03, flute_len=28.0, holder_dia=20.0,
            ap=2.0, ae=2.0)),
        "16 mm 5FL roughing": tool_to_dict(ProcessParams(
            tool_dia=16.0, n_teeth=5, fz=0.08, flute_len=45.0, holder_dia=32.0,
            ap=8.0, ae=9.0, max_force_N=4000.0)),
        "Long-reach tapered shank": tool_to_dict(ProcessParams(
            tool_dia=10.0, flute_len=50.0, holder_dia=22.0, holder_len=60.0,
            spindle_len=140.0)),
        # measured-coefficient tools (material-calibrated Kt/Kr/Kte/Kre)
        "Ti-6Al-4V finisher": tool_to_dict(ProcessParams(
            tool_dia=8.0, n_teeth=4, fz=0.04, helix_deg=38.0, flute_len=28.0,
            holder_dia=20.0, ap=3.0, ae=1.5, **coeffs_for_material("Ti-6Al-4V"))),
        "Inconel 718 rougher": tool_to_dict(ProcessParams(
            tool_dia=12.0, n_teeth=5, fz=0.06, helix_deg=30.0, flute_len=35.0,
            holder_dia=25.0, ap=4.0, ae=6.0, max_force_N=4000.0,
            **coeffs_for_material("Inconel 718"))),
    }


def _builtin_strategies() -> dict:
    base = dict(strategy="global", R=6.0, nv=41, mu=1.0, gamma=0.0,
                barrel_R=0.0, barrel_pos=0.0, nsweeps=3, smooth_window=5,
                swept_weight=0.0, swept_window=8)
    return {
        "Flank finish (global)": dict(base),
        "Min-max accuracy": dict(base, strategy="minmax"),
        "Smoothed orientation": dict(base, strategy="smoothed", mu=2.0),
        "Barrel finish (R200)": dict(base, barrel_R=200.0, barrel_pos=15.0),
        "Low swept overcut": dict(base, swept_weight=0.5),
    }


def _builtin_blades() -> dict:
    base = dict(nu=60, r_hub=30.0, r_shroud=55.0, z_span=20.0, z_offset=8.0,
                wrap=0.6, twist=0.7, n_blades=11)
    return {
        "Default impeller blade": dict(base),
        "Tall twisted blade": dict(base, z_span=40.0, twist=1.3, wrap=0.9),
        "Compact blisk blade": dict(base, r_hub=20.0, r_shroud=40.0, z_span=15.0,
                                    n_blades=17),
    }


def _builtin_posts() -> dict:
    return {n: post_to_dict(c) for n, c in post_lib.CERTIFIED_POSTS.items()}


_BUILTINS = {"machine": _builtin_machines, "tool": _builtin_tools,
             "strategy": _builtin_strategies, "blade": _builtin_blades,
             "post": _builtin_posts}


class PresetStore:
    """Built-in + on-disk user presets for each category."""

    def __init__(self, root: str = None):
        self.root = (root or os.environ.get("BLADECAM_PRESETS")
                     or os.path.join(os.path.expanduser("~"), ".bladecam", "presets"))
        self.builtins = {k: fn() for k, fn in _BUILTINS.items()}

    # -- discovery --
    def _user_dir(self, kind: str) -> str:
        return os.path.join(self.root, kind)

    def user_names(self, kind: str):
        d = self._user_dir(kind)
        if not os.path.isdir(d):
            return []
        return sorted(f[:-5] for f in os.listdir(d) if f.endswith(".json"))

    def names(self, kind: str):
        """Built-in names first, then user names (de-duplicated)."""
        bi = list(self.builtins[kind].keys())
        return bi + [n for n in self.user_names(kind) if n not in bi]

    def is_builtin(self, kind: str, name: str) -> bool:
        return name in self.builtins[kind]

    # -- load / save / delete --
    def load(self, kind: str, name: str) -> dict:
        path = os.path.join(self._user_dir(kind), name + ".json")
        if os.path.isfile(path):                 # user preset shadows nothing here
            with open(path) as fh:
                return json.load(fh)
        if name in self.builtins[kind]:
            return dict(self.builtins[kind][name])
        raise KeyError(f"no {kind} preset named {name!r}")

    def save(self, kind: str, name: str, data: dict) -> str:
        if kind not in KINDS:
            raise ValueError(f"unknown preset kind {kind!r}")
        d = self._user_dir(kind)
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, name + ".json")
        with open(path, "w") as fh:
            json.dump(data, fh, indent=2, sort_keys=True)
        return path

    def delete(self, kind: str, name: str) -> bool:
        """Delete a USER preset (built-ins are read-only)."""
        path = os.path.join(self._user_dir(kind), name + ".json")
        if os.path.isfile(path):
            os.remove(path)
            return True
        return False

    # -- import / export of preset bundles (share whole configs) --
    def export_bundle(self, path: str, include_builtins: bool = False) -> int:
        """Write all USER presets (optionally + built-ins) of every kind to a
        single JSON bundle. Returns the number of presets written."""
        bundle = {"format": "bladecam-presets", "version": 1, "presets": {}}
        n = 0
        for kind in KINDS:
            names = (self.names(kind) if include_builtins
                     else self.user_names(kind))
            bundle["presets"][kind] = {}
            for nm in names:
                bundle["presets"][kind][nm] = self.load(kind, nm)
                n += 1
        with open(path, "w") as fh:
            json.dump(bundle, fh, indent=2, sort_keys=True)
        return n

    def import_bundle(self, path: str, overwrite: bool = True) -> int:
        """Import presets from a bundle into the user store. Skips entries whose
        name collides with a built-in (those are read-only). Returns the count
        actually imported."""
        with open(path) as fh:
            bundle = json.load(fh)
        if bundle.get("format") != "bladecam-presets":
            raise ValueError("not a BladeCAM preset bundle")
        n = 0
        for kind, items in bundle.get("presets", {}).items():
            if kind not in KINDS:
                continue
            for nm, data in items.items():
                if self.is_builtin(kind, nm):
                    continue                         # never shadow a built-in
                if not overwrite and nm in self.user_names(kind):
                    continue
                self.save(kind, nm, data)
                n += 1
        return n


def presets_equal(a: dict, b: dict, tol: float = 1e-9) -> bool:
    """Tolerant comparison of two preset dicts (numbers within tol; nested
    lists/tuples compared element-wise). Used for the dirty-state indicator."""
    if set(a.keys()) != set(b.keys()):
        return False
    for k in a:
        x, y = a[k], b[k]
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            if abs(float(x) - float(y)) > tol:
                return False
        elif isinstance(x, (list, tuple)) and isinstance(y, (list, tuple)):
            if len(x) != len(y) or any(abs(float(p) - float(q)) > tol
                                       for p, q in zip(x, y)):
                return False
        elif x != y:
            return False
    return True
