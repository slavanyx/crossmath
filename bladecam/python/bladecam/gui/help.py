"""Learn-as-you-go help content for the BladeCAM GUI.

Qt-free so it is unit-testable and reusable (the GUI is a thin presenter, exactly
like charts.py / workflow.py). The content is intentionally COMPLETE: every
parameter editor, every results metric and every workflow stage has an entry, so
a user can hover/click anything and learn what it does. test_help.py enforces
that completeness as an invariant.
"""
from __future__ import annotations


# One-line help for every parameter editor (tooltip + status tip). Keys match
# PARAM_SPEC / TOOL_SPEC / MACHINE_SPEC in gui/model.py.
PARAM_HELP = {
    # --- tool / strategy ---
    "R": "Cutter radius (mm). The flank tool is fitted tangent to each ruling; "
         "a larger R averages over more of the blade but raises the twist error.",
    "gamma": "Tool taper half-angle (rad). 0 = cylinder; >0 fits a conical flank "
             "tool, which can match a fanning ruled flank better.",
    "mu": "Smoothness weight in the global optimiser. Higher µ trades a little "
          "per-ruling accuracy for a smoother, lower-jerk tool-axis motion.",
    "barrel_R": "Barrel (circle-segment) tool arc radius (mm). 0 = off. A barrel "
                "tool hugs a curved flank with far less scallop than a cylinder.",
    "barrel_pos": "Axial position (mm from the tool tip) of the barrel's widest "
                  "point. Only used when Barrel arc radius > 0.",
    "swept_weight": "Weight on the swept-overcut penalty. >0 pulls the axes to "
                    "reduce cross-station interference (real envelope overcut).",
    "nsweeps": "Number of block coordinate-descent sweeps in the global "
               "optimiser. More sweeps = closer to the joint optimum, slower.",
    "smooth_window": "Half-window (stations) for the 'smoothed' strategy's "
                     "moving-average of the tool axis.",
    # --- geometry ---
    "twist": "Blade twist (rad) of the parametric test blade: how much the "
             "shroud rail rotates relative to the hub. Drives the twist error.",
    "wrap": "Blade wrap (rad): how far the blade sweeps around the impeller axis.",
    "r_hub": "Hub (inner) radius of the parametric blade root (mm).",
    "r_shroud": "Shroud (outer) radius of the parametric blade tip (mm).",
    "z_span": "Blade height (mm) from hub to shroud along the spin axis.",
    "z_offset": "Axial offset (mm) of the shroud rail relative to the hub rail.",
    "nu": "Number of stations sampled along the blade (toolpath resolution). "
          "More stations = finer path and analysis, slower compute.",
    "n_blades": "Blade count of the impeller. Sets the neighbour-blade pitch "
                "used by the channel collision and roughing.",
    # --- setup ---
    "collision_substeps": "Extra interpolated tool poses inserted between "
                          "stations for the swept collision check (0 = endpoints).",
    "mount_clearance": "Distance (mm) from the blade base to the machine table "
                       "top, for the structural table-collision check.",
    "root_fillet_r": "Root-fillet trim offset (mm). >0 lifts the flank pass off "
                     "the hub fillet so it is left for the fillet operation.",
    "pivot_x": "Machine rotary pivot X (mm) used by the 5-axis inverse kinematics.",
    "pivot_y": "Machine rotary pivot Y (mm) used by the 5-axis inverse kinematics.",
    "pivot_z": "Machine rotary pivot Z (mm) used by the 5-axis inverse kinematics.",
    # --- tool / cutting ---
    "tool_dia": "Cutter diameter (mm). Note R = tool_dia/2 drives the flank fit.",
    "n_teeth": "Number of cutter teeth/flutes (sets the tooth-passing forces).",
    "fz": "Feed per tooth (mm). Chip load; the nominal feed = fz·teeth·rpm.",
    "rpm": "Spindle speed (rev/min).",
    "ap": "Axial depth of cut (mm) for the force/roughing model.",
    "ae": "Radial width of cut (mm); 0 = auto (half the diameter). Sets the "
          "engagement arc and hence the cutting force.",
    "Kt": "Tangential cutting coefficient (N/mm²). Use the material library / "
          "calibration for a measured value.",
    "Kr": "Radial/tangential force ratio (dimensionless).",
    "Kte": "Tangential EDGE (ploughing) coefficient (N/mm) — non-zero force at "
           "zero chip load.",
    "Kre": "Radial edge coefficient (N/mm).",
    "helix_deg": "Cutter helix angle (deg). Spreads the cut axially: same mean "
                 "torque, lower peak force.",
    "n_axial": "Axial slices for the helical force integration (accuracy vs speed).",
    "flute_len": "Cutting-flute length (mm) — also the cantilever length for the "
                 "deflection limit and the flute extent for collision.",
    "holder_dia": "Tool-holder diameter (mm) for the collision model.",
    "holder_gap": "Gap (mm) between the flute top and the holder.",
    "holder_len": "Modelled holder length (mm).",
    "spindle_dia": "Spindle-nose diameter (mm) for the full-machine collision.",
    "spindle_gap": "Gap (mm) below the spindle nose.",
    "spindle_len": "Modelled spindle-nose length (mm).",
    "spindle_power_kW": "Available spindle power (kW); caps the feed via the "
                        "mechanistic power limit.",
    "max_force_N": "Maximum allowed resultant cutting force (N); caps the feed.",
    "E": "Tool Young's modulus (N/mm², carbide ≈ 600 GPa) for tip deflection.",
    "dev_allow_um": "Allowed deflection-induced error (µm); caps the feed.",
    "feed_max_mm_min": "User/machine feed ceiling (mm/min).",
    # --- machine quick editors ---
    "v_rot": "Rotary axis max speed (rad/s) used by the time-optimal feed.",
    "kind": "Kinematics: 0 = table-table (A-C, workpiece rotates), "
            "1 = head-head (spindle tilts).",
}


# What each results-table metric means (tooltip on the row).
METRIC_HELP = {
    "strategy": "The active positioning strategy.",
    "machined-surface error (swept)": "THE headline accuracy: peak depth the "
        "swept tool envelope cuts past the design flank. The real finishing error.",
    "gouge depth (per station)": "Per-station overcut depth past the design "
        "surface (a per-ruling proxy, < the swept value).",
    "contact-line residual (peak)": "Peak distance from the design surface to "
        "the tool's contact line. ≈0 for a cylinder on a ruled surface — a "
        "diagnostic, NOT the real error (see machined-surface error).",
    "contact-line residual (mean)": "Mean contact-line residual (diagnostic).",
    "orientation jerk": "Smoothness of the tool-axis motion (lower = gentler "
        "5-axis moves, less servo stress).",
    "cycle time": "Time-optimal (TOPP) traversal time of the finishing pass.",
    "path length": "Tool-tip contact path length (mm).",
    "feed cap": "Effective feed ceiling (mm/min) = min(machine ceiling, "
        "mechanistic force/deflection/power cap, nominal feed).",
    "min clearance": "Smallest of all collision clearances (mm); the governing "
        "safety number. Negative = a collision somewhere.",
    "assembly clearance": "Tool-assembly (flute+holder+spindle) clearance to the "
        "neighbour blades + table/fixture.",
    "holder clearance": "Holder-only clearance to the blade being machined "
        "(the flute is tangent by design, so only the holder can hit it).",
    "structural-link clearance": "Tool assembly vs the trunnion cradle posts and "
        "machine column (kinematic links). '—' if the machine has none.",
    "fixture/body clearance": "Tool assembly vs the imported fixture/machine "
        "triangle mesh. '—' if no fixture is loaded.",
    "collision-free": "True only if every clearance above is positive.",
    "machine": "The active machine profile driving limits + reachability.",
    "reachable": "Whether the toolpath fits the machine's travel/rotary envelope "
        "(shows the offending axis + excess if not).",
    "cut force (peak)": "Peak resultant cutting force (N) from the mechanistic "
        "model at the planned feed.",
    "cut power": "Mean spindle power (kW) at the planned feed.",
    "feed feasible": "False if the cut overloads the spindle/tool even at a "
        "vanishing feed (the cut itself is infeasible, not just feed-limited).",
}


# Glossary of the domain terms the UI uses.
GLOSSARY = {
    "Flank milling": "Finishing a ruled blade with the SIDE of the tool tangent "
        "to the wall along a whole ruling at once — far faster than point milling.",
    "Ruled surface": "A surface swept by a straight line (the ruling). An "
        "impeller blade flank is (nearly) ruled between its hub and shroud rails.",
    "Distribution parameter δ": "Measures how non-developable (twisted) the "
        "ruled flank is. Small |δ| = highly twisted = harder to flank-mill "
        "(the twist error grows as R·ℓ²/δ²).",
    "Striction curve": "The locus of the points of closest approach of "
        "neighbouring rulings — the 'spine' of a twisted ruled surface.",
    "Swept envelope / overcut": "The actual surface the moving tool sweeps. The "
        "machined error is the overcut of this envelope past the design flank — "
        "the number that matters, unlike the (≈0) contact-line residual.",
    "Contact-line residual": "Distance from the design surface to the tool's "
        "single contact line. Misleadingly ≈0 for a cylinder on a ruled surface.",
    "TOPP": "Time-Optimal Path Parameterisation: the fastest feed schedule along "
        "the fixed path that respects every axis velocity and acceleration limit.",
    "TCPM": "Tool-Centre-Point Management: the control keeps the tool tip on the "
        "path while the rotary axes orient it — the kinematics-independent post.",
    "Dexel / Z-map": "A field of rays storing the remaining material along each. "
        "Used to verify material removal and carry stock across operations.",
    "Rest machining": "Finishing that knows what roughing already removed, so it "
        "cuts only the leftover (rest) material, not the raw block.",
    "Barrel tool": "A cutter whose side is a large-radius circular arc; it hugs "
        "a curved flank with much less scallop than a straight cylinder.",
    "Reachability": "Whether the toolpath's machine axes stay inside the "
        "machine's travel and rotary limits.",
    "Chatter / stability lobes": "Self-excited vibration; the lobe diagram shows "
        "the depth of cut that is stable at each spindle speed.",
}


# Per workflow-stage guidance: what you are looking at and the next step. Keys
# match workflow.STAGE_KEYS.
STAGE_HELP = {
    "geometry": "You are looking at the ruled design surface coloured by twist "
                "(|δ|). Warmer = more twisted = harder to flank-mill. Next: step "
                "to Cutter positioning to see the fitted tool axes.",
    "positioning": "The optimised tool axis at each ruling, with the surface "
                   "coloured by the real machined error. Try a different strategy "
                   "or a barrel tool to lower it. Next: Kinematics.",
    "kinematics": "The tool-tip path with the A/C rotary orientation realised at "
                  "each station. Check 'reachable' in Results. Next: Feed.",
    "feed": "The contact path coloured by the time-optimal feed (warm = fast). "
            "Cool spots are slowed by an axis or force limit. Next: Verification.",
    "verification": "The machined surface (red = overcut) and the collision "
                    "clearances. When this is clean, post the G-code (File ▸ Save "
                    "certified G-code).",
}


QUICK_START = [
    "Pick a blade source: keep the parametric blade, or File ▸ Load blade/blisk "
    "from STEP/IGES.",
    "Choose a Strategy preset (toolbar) — 'Flank finish (global)' is a good start.",
    "Set the cutter in the Parameters panel (R / tool diameter) — hover any field "
    "for help.",
    "Press Recompute. Read the headline 'machined-surface error' in Results.",
    "Switch to Preview and step the workflow (Geometry → … → Verification) to "
    "understand each stage; the matching chart follows below.",
    "If 'collision-free' and 'reachable' are good, File ▸ Save certified G-code.",
]


GETTING_STARTED = (
    "BladeCAM positions a 5-axis flank-milling cutter on impeller / blisk blades "
    "and posts the G-code.\n\n"
    "The workflow runs left to right: GEOMETRY (the ruled blade) → POSITIONING "
    "(fit the tool to each ruling) → KINEMATICS (5-axis A/C motion) → FEED "
    "(time-optimal schedule) → VERIFICATION (envelope error + collisions).\n\n"
    "Two modes: PREPARE (edit parameters) and PREVIEW (step through the stages "
    "and inspect). Presets (toolbar) save Blade / Machine / Tool / Strategy / "
    "Post configurations; a whole job saves to a single .bladecam project.\n\n"
    "The number to watch is 'machined-surface error (swept)' — the real finishing "
    "error. The per-ruling 'contact-line residual' looks tiny but is only a "
    "diagnostic.\n\n"
    "Hover any parameter or results row for an explanation, or use Help ▸ "
    "Glossary for the domain terms."
)


def param_tip(key: str) -> str:
    return PARAM_HELP.get(key, "")


def metric_tip(label: str) -> str:
    return METRIC_HELP.get(label, "")


def stage_help(key: str) -> str:
    return STAGE_HELP.get(key, "")
