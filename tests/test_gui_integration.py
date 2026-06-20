#!/usr/bin/env python3
"""GUI-integration audit: every parameter, strategy, Operation and chart path
the (Qt-free) model/charts layers expose must run headlessly. Guards against a
feature being added to the core but not wired into the interface."""
import sys

try:
    import numpy as np
    from bladecam.gui.model import (AppModel, STRATEGIES, PARAM_SPEC,
                                    TOOL_SPEC, MACHINE_SPEC)
    from bladecam.gui import charts
    from bladecam import pipeline, core
except ImportError as e:
    print(f"SKIP gui-integration ({e})")
    sys.exit(0)

FAILED = []


def check(c, name):
    print(f"  {'ok  ' if c else 'FAIL'} {name}")
    if not c:
        FAILED.append(name)


def main():
    m = AppModel()

    # every auto-generated editor (incl. the full tool/cutting set) maps to a
    # real model value -- so every config parameter is GUI-editable
    check(all(spec[0] in m.values for spec in PARAM_SPEC + TOOL_SPEC + MACHINE_SPEC),
          "all parameter editors map to model values")
    # every ProcessParams field is exposed as a tool editor (no hidden tool config)
    import dataclasses
    from bladecam.process import ProcessParams
    tool_keys = {s[0] for s in TOOL_SPEC}
    pp_fields = {f.name for f in dataclasses.fields(ProcessParams)}
    check(pp_fields <= tool_keys,
          f"every ProcessParams field has a GUI editor (missing {pp_fields - tool_keys})")
    # tool edits flow into the built ProcessParams
    m.values["fz"] = 0.077; m.values["ap"] = 6.5
    p = m.build_params()
    check(p.process.fz == 0.077 and p.process.ap == 6.5,
          "tool editors flow into Params.process")

    # every strategy is reachable via the model
    good = True
    for s in STRATEGIES:
        m.strategy = s
        good &= bool(np.all(np.isfinite(m.compute_current()["dev"])))
    check(good, "all strategies compute via the model")

    # OrcaSlicer-style presets flow through the model into compute()
    m.apply_preset("strategy", "Barrel finish (R200)")
    m.apply_preset("tool", "16 mm 5FL roughing")
    m.apply_preset("machine", "Compact blisk cell")
    m.apply_preset("blade", "Tall twisted blade")
    m.apply_preset("post", "Siemens 840D — compact blisk cell AC")
    p = m.build_params()
    check(p.barrel_R == 200.0 and p.process.tool_dia == 16.0
          and p.machine.name == "Compact blisk cell" and p.z_span == 40.0,
          "machine/tool/strategy/blade presets drive build_params")
    check(m._live_post().control == "siemens",
          "post preset selects the control dialect")
    # the active post generates + certifies a program for the configured machine
    _r = m.compute_current()
    _text, _rep = m.post_program(_r)
    check(len(_text) > 0 and "roundtrip_ok" in _rep,
          "model posts a program and a certification report")
    check(p.pivot[2] == m.values["pivot_z"], "pivot is configurable via editors")
    check(np.all(np.isfinite(m.compute_current()["dev"])),
          "preset-configured model computes")
    check(m.capture_preset("strategy")["barrel_R"] == 200.0,
          "capture_preset round-trips the live state")

    # comparison data + its two charts
    m.strategy = "global"
    stats = m.compute_compare_full()
    check(all("dev" in stats[s] for s in stats), "compare-full carries dev arrays")
    charts.deviation_chart({s: stats[s]["dev"] for s in stats})
    charts.compare_chart(stats)

    # every Operations-menu compute path
    p = m.build_params()
    check(np.all(np.isfinite(pipeline.double_flank_channel(p)["devL"])),
          "operation: double-flank channel")
    check(pipeline.rough_channel(p)["total_len_mm"] > 0, "operation: roughing")
    check(pipeline.edge_finish(p)["n_rows"] >= 2, "operation: edge finishing")
    check(pipeline.stacked_flank_passes(p)["n_passes"] >= 1, "operation: stacked passes")

    # analysis charts incl. modal + measured-FRF chatter
    r = m.compute_current()
    charts.machinability_chart(r["delta"], r["dev"])
    charts.feed_chart(r["seglen"], r["aprof"])
    # per-stage Preview binding: the kinematics chart + station cursor on each
    nu = r["q0"].shape[0]
    charts.kinematics_chart(r["machine_path"])
    charts.machinability_chart(r["delta"], r["dev"], mark=nu // 2)
    charts.deviation_chart({m.strategy: r["dev"]}, mark=nu // 2)
    charts.kinematics_chart(r["machine_path"], mark=nu // 2)
    charts.feed_chart(r["seglen"], r["aprof"], mark=nu // 2)
    # every workflow stage maps to a buildable chart (or the standalone Chatter)
    from bladecam import workflow
    chart_builders = {"Machinability", "Deviation", "Kinematics", "Feed", "Chatter"}
    check(all(c in chart_builders for c in workflow.STAGE_CHART.values()),
          "every workflow stage binds to a known analysis chart")
    rpm, al = core.stability_lobes(800, 0.03, 2e4, 800, 4, 6, 80)
    charts.chatter_chart(rpm, al, 6, 80, 12000)
    freq = np.linspace(800, 1600, 200)
    G = (1 / 2e4) / ((1 - (freq / 800) ** 2) + 1j * 2 * 0.03 * (freq / 800))
    rpmf, alf = core.stability_lobes_frf(freq, G.real, G.imag, 800, 4, 6)
    charts.chatter_chart(rpmf, alf, 6, 200, 12000)
    check(True, "analysis charts: machinability / feed / chatter (modal+FRF)")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nGUI INTEGRATION TESTS PASSED")


if __name__ == "__main__":
    main()
