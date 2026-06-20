"""Workflow stage model: turns one pipeline result into an ordered sequence of
renderer-agnostic 3D scenes, so the GUI can let the user *step through* the CAM
process (geometry -> positioning -> kinematics -> feed -> verification) with the
view flowing from one stage to the next.

The pipeline (pipeline.compute) stays the single source of truth; this module
only *re-presents* its output. Scenes are plain dicts of primitive specs
(surface / polyline / lines / points / tube) plus a metrics list, so they are
fully testable headlessly and the GUI (PyVista) is a thin translator. No Qt or
PyVista import here.
"""
from __future__ import annotations

import numpy as np


# ordered pipeline stages shown in the workflow stepper
STAGES = [
    ("geometry",     "1 · Blade geometry",
     "Ruled design surface, hub/shroud rails and the line of striction. "
     "Colour = local non-developability (|distribution parameter δ|)."),
    ("positioning",  "2 · Cutter positioning",
     "Optimised cylinder axis at every station fitted to each ruling. "
     "Black segments are the tool axes."),
    ("kinematics",   "3 · 5-axis kinematics",
     "Tool-tip contact path with the machine A/C orientation realised at each "
     "station (orientation fans show the rotary motion)."),
    ("feed",         "4 · Time-optimal feed",
     "Contact path coloured by the time-optimal feed schedule (TOPP). "
     "Warm = fast, cool = slowed by an axis / process limit."),
    ("verification", "5 · Verification",
     "Machined-surface (swept-envelope) error and tool/holder clearance to the "
     "neighbouring blades. Red = overcut, the real finishing error."),
]

STAGE_KEYS = tuple(k for k, *_ in STAGES)

# stages where animating the cutter sweeping along the path is meaningful
# (geometry is the static design surface; everything after involves the tool)
STAGE_ANIMATE = {
    "geometry":     False,
    "positioning":  True,
    "kinematics":   True,
    "feed":         True,
    "verification": True,
}

# the bottom analysis tab most relevant to each stage, so stepping the workflow
# also brings the matching chart forward (names match the GUI's tab labels)
STAGE_CHART = {
    "geometry":     "Machinability",   # delta / machinability index vs station
    "positioning":  "Deviation",       # per-strategy flank deviation
    "kinematics":   "Kinematics",      # A/C rotary orientation along the path
    "feed":         "Feed",            # the time-optimal feed schedule itself
    "verification": "Chatter",         # process-stability (chatter) check
}


def _axis_segments(q0, alpha, every, back, fwd):
    """Tool-axis line segments (one per sampled station)."""
    nu = q0.shape[0]
    segs = []
    for i in range(0, nu, max(1, every)):
        p = q0[i]; a = alpha[i]
        segs.append((p - a * back, p + a * fwd))
    return segs


def _norm_scalar(field, lo=None, hi=None):
    """Flatten a per-(u,v) field to a finite array for colouring."""
    f = np.asarray(field, dtype=float)
    f = np.nan_to_num(f, nan=0.0, posinf=0.0, neginf=0.0)
    return f


def stage_scene(r, key, R=6.0):
    """Build the 3D scene + metrics for one workflow stage from a compute() result.

    Returns dict(title, blurb, meshes=[...], metrics=[(label, value), ...]).
    Each mesh spec has a 'type' the GUI knows how to render.
    """
    titles = {k: (t, b) for k, t, b in STAGES}
    if key not in titles:
        raise KeyError(f"unknown stage {key!r}; expected one of {STAGE_KEYS}")
    title, blurb = titles[key]
    surf = r["surf"]
    meshes = []
    metrics = []

    if key == "geometry":
        # surface coloured by |delta| (non-developability), rails + striction
        delta = np.abs(np.asarray(r["delta"]))
        meshes.append(dict(type="surface", points=surf,
                           scalar=np.repeat(delta, surf.shape[1]),
                           title="|δ| (mm)", cmap="viridis", opacity=1.0))
        meshes.append(dict(type="polyline", points=r["a"], color="#1f77b4", width=4))
        meshes.append(dict(type="polyline", points=r["b"], color="#ff7f0e", width=4))
        meshes.append(dict(type="polyline", points=r["strict"], color="lime", width=3))
        metrics = [
            ("stations", f"{surf.shape[0]}"),
            ("min |δ| (most warped)", f"{float(np.min(delta)):.2f} mm"),
            ("mean |δ|", f"{float(np.mean(delta)):.2f} mm"),
        ]

    elif key == "positioning":
        # surface (faint) + tool axes; colour by machined-surface error if present
        sf = r.get("swept_field")
        if sf is not None:
            sc = _norm_scalar(np.maximum(0.0, -sf) * 1000.0)
            meshes.append(dict(type="surface", points=surf, scalar=sc.reshape(-1),
                               title="surface err (µm)", cmap="coolwarm", opacity=0.85))
        else:
            meshes.append(dict(type="surface", points=surf, scalar=None,
                               color="lightgray", opacity=0.6))
        meshes.append(dict(type="lines",
                           segments=_axis_segments(r["q0"], r["alpha"],
                                                   max(1, surf.shape[0] // 16),
                                                   0.2 * R, 5.0 * R),
                           color="black", width=2))
        metrics = [
            ("machined-surface error", f"{r.get('swept_overcut', 0.0)*1000:.1f} µm"),
            ("contact-line residual", f"{r['dev'].max()*1000:.1f} µm"),
            ("orientation jerk", f"{r['orient_jerk']:.3f}"),
        ]

    elif key == "kinematics":
        meshes.append(dict(type="surface", points=surf, scalar=None,
                           color="lightgray", opacity=0.4))
        meshes.append(dict(type="polyline", points=r["contact"],
                           color="#d62728", width=4))
        meshes.append(dict(type="lines",
                           segments=_axis_segments(r["q0"], r["alpha"],
                                                   max(1, surf.shape[0] // 12),
                                                   0.0, 8.0 * R),
                           color="black", width=2))
        m = r["machine_path"]
        metrics = [
            ("A travel", f"{np.degrees(np.ptp(m[:,3])):.1f}°"),
            ("C travel", f"{np.degrees(np.ptp(m[:,4])):.1f}°"),
            ("path length", f"{r['path_len_mm']:.1f} mm"),
        ]

    elif key == "feed":
        # contact path as a tube coloured by feed (sqrt of TOPP pseudo-accel
        # profile is proportional to path speed)
        aprof = np.asarray(r["aprof"], dtype=float)
        speed = np.sqrt(np.clip(aprof, 0.0, None))
        meshes.append(dict(type="surface", points=surf, scalar=None,
                           color="lightgray", opacity=0.3))
        meshes.append(dict(type="tube", points=r["contact"],
                           scalar=speed, title="feed (rel.)", cmap="turbo",
                           radius=0.25 * R))
        mt = np.asarray(r["move_times_s"], dtype=float)
        metrics = [
            ("cycle time", f"{r['cycle_time_s']:.2f} s"),
            ("feed cap", f"{r['feed_cap_mm_min']:.0f} mm/min"),
            ("slowest move", f"{float(mt.max())*1000:.1f} ms"),
        ]

    elif key == "verification":
        sf = r.get("swept_field")
        if sf is not None:
            sc = _norm_scalar(np.maximum(0.0, -sf) * 1000.0)
            meshes.append(dict(type="surface", points=surf, scalar=sc.reshape(-1),
                               title="overcut (µm)", cmap="coolwarm", opacity=1.0))
        else:
            meshes.append(dict(type="surface", points=surf, scalar=None,
                               color="lightgray", opacity=0.8))
        coll = "OK" if r["collision_free"] else "COLLISION"
        metrics = [
            ("machined-surface error", f"{r.get('swept_overcut', 0.0)*1000:.1f} µm"),
            ("gouge depth (per station)", f"{r.get('gouge_max', 0.0)*1000:.1f} µm"),
            ("min clearance", f"{r['min_clearance']:.2f} mm"),
            ("collision check", coll),
        ]

    return dict(key=key, title=title, blurb=blurb, meshes=meshes, metrics=metrics)


def all_scenes(r, R=6.0):
    """Every stage scene in order (the full visual workflow)."""
    return [stage_scene(r, k, R) for k in STAGE_KEYS]
