#!/usr/bin/env python3
"""`.bladecam` project file: a self-contained snapshot of the whole job (live
parameters, strategy, machine + certified-post configs, and embedded CAD rails)
must round-trip exactly and reopen to an identical build_params()/post."""
import sys
import os
import tempfile

try:
    import numpy as np
    from dataclasses import replace
    from bladecam.gui.model import AppModel, save_project, load_project
    from bladecam import blade
except ImportError as e:
    print(f"SKIP project ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def main():
    m = AppModel()
    # edit a spread of state across every category
    m.values["R"] = 7.5
    m.values["fz"] = 0.066
    m.values["root_fillet_r"] = 2.0
    m.values["helix_deg"] = 42.0
    m.values["pivot_z"] = -123.0
    m.strategy = "minmax"
    m.apply_preset("post", "Fanuc 31i — large gantry AC")
    m.machine = replace(m.machine, cradle_span=137.0, name="Custom cell")
    a, b = blade.make_blade(40, 28.0, 52.0, 22.0, 6.0, 0.7, 0.9)
    m.rails = (a, b)

    # in-memory snapshot round-trip into a FRESH model
    d = m.to_project()
    m2 = AppModel()
    m2.load_project(d)
    check(m2.values["R"] == 7.5 and m2.values["fz"] == 0.066
          and m2.values["helix_deg"] == 42.0 and m2.values["pivot_z"] == -123.0,
          "live parameter values restored")
    check(m2.strategy == "minmax", "strategy restored")
    check(m2.machine.cradle_span == 137.0 and m2.machine.name == "Custom cell",
          "full machine config (incl. structural links) restored")
    check(m2._live_post().control == "fanuc", "certified post restored")
    check(m2.rails is not None and np.allclose(m2.rails[0], a)
          and np.allclose(m2.rails[1], b), "embedded CAD rails restored exactly")

    # self-containment: build_params is identical without any preset library help
    p1, p2 = m.build_params(), m2.build_params()
    check(p1.R == p2.R and p1.process.fz == p2.process.fz
          and p1.root_fillet_r == p2.root_fillet_r
          and p1.process.helix_deg == p2.process.helix_deg
          and p1.pivot == p2.pivot
          and p1.machine.cradle_span == p2.machine.cradle_span
          and p1.strategy == p2.strategy,
          "reopened project rebuilds an identical job")
    check(np.all(np.isfinite(m2.compute_current()["dev"])),
          "reopened project computes")

    # the posted program + certification match across the round trip
    r1 = m.compute_current(); r2 = m2.compute_current()
    t1, _ = m.post_program(r1); t2, _ = m2.post_program(r2)
    check(t1 == t2, "reopened project posts the identical program")

    # file IO round-trip + format guard
    with tempfile.TemporaryDirectory() as dd:
        path = os.path.join(dd, "job.bladecam")
        save_project(path, m)
        m3 = AppModel()
        loaded = load_project(path, m3)
        check(loaded["format"] == "bladecam-project", "file carries the format tag")
        check(m3.machine.cradle_span == 137.0 and m3.values["R"] == 7.5
              and np.allclose(m3.rails[0], a), "project file reloads the full state")
        # a non-project JSON is rejected
        bad = os.path.join(dd, "bad.json")
        with open(bad, "w") as fh:
            fh.write('{"format": "something-else"}')
        try:
            load_project(bad, AppModel()); raised = False
        except ValueError:
            raised = True
        check(raised, "a non-project file is rejected")

    # a project with no CAD (parametric blade) round-trips with rails=None
    mp = AppModel()
    dp = mp.to_project()
    check(dp["rails"] is None, "parametric (no-CAD) project stores rails=None")
    mp2 = AppModel(); mp2.load_project(dp)
    check(mp2.rails is None and np.all(np.isfinite(mp2.compute_current()["dev"])),
          "parametric project reopens and computes")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nPROJECT TESTS PASSED")


if __name__ == "__main__":
    main()
