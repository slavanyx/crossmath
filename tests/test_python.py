#!/usr/bin/env python3
"""Python-level smoke test wired into ctest. Skips cleanly if numpy is absent.

Verifies the Fortran<->Python pipeline (global strategy) and CAD I/O round
trips. Exit 0 = pass/skip, nonzero = failure.
"""
import sys
import tempfile
import os

try:
    import numpy as np
    from bladecam import pipeline, cadio
    from bladecam.pipeline import Params
except ImportError as e:
    print(f"SKIP python test (missing dependency: {e})")
    sys.exit(0)


def main():
    r = pipeline.compute(Params(strategy="global", nu=40))
    assert np.all(np.isfinite(r["dev"])), "non-finite deviation"
    assert r["dev"].max() < 0.5, f"deviation too large: {r['dev'].max()}"
    assert r["cycle_time_s"] > 0.0, "non-positive cycle time"
    assert r["machine_path"].shape[1] == 5, "machine path must be 5-axis"

    # conical deviation must reduce exactly to the cylinder at gamma=0
    from bladecam import core
    pts = r["surf"][0]
    g_cyl = core.deviation(r["q0"][0], r["alpha"][0], 6.0, pts)
    g_con = core.deviation_cone(r["q0"][0], r["alpha"][0], 6.0, 0.0, pts)
    assert np.allclose(g_cyl, g_con), "cone(gamma=0) must equal cylinder"

    # conical "global" run is a smoke test of the gamma path through the pipeline
    rg = pipeline.compute(Params(strategy="global", gamma=0.08, nu=40))
    assert np.all(np.isfinite(rg["devfield"])), "cone devfield non-finite"

    # chatter lobes: high-speed (k=0) lobe present after the mod-2pi fix
    rpm, alim = core.stability_lobes(800.0, 0.03, 2.0e4, 800.0, 4, 6, 80)
    assert rpm.max() > 2.0e4, "high-speed stability lobe missing"
    assert np.all(alim > 0), "non-physical limiting depth"

    # GUI model + charts are Qt-free and must work headless
    try:
        from bladecam.gui.model import AppModel, PARAM_SPEC, STRATEGIES
        from bladecam.gui import charts
        gm = AppModel()
        assert set(STRATEGIES) and len(PARAM_SPEC) > 0
        gr = gm.compute_current()
        stats = gm.compute_compare_full()
        assert all("dev" in stats[s] for s in stats), "compare-full missing dev arrays"
        charts.deviation_chart({s: stats[s]["dev"] for s in stats})
        charts.compare_chart(stats)
        charts.machinability_chart(gr["delta"], gr["dev"])
        charts.feed_chart(gr["seglen"], gr["aprof"])
        charts.chatter_chart(rpm, alim, 6, 80, 12000)
    except ImportError:
        print("  (skipping GUI-model/charts check: matplotlib unavailable)")

    with tempfile.TemporaryDirectory() as d:
        csv = os.path.join(d, "rails.csv")
        stl = os.path.join(d, "blade.stl")
        cadio.write_rails_csv(csv, r["a"], r["b"])
        a2, b2 = cadio.read_rails_csv(csv)
        assert np.allclose(a2, r["a"]) and np.allclose(b2, r["b"]), "rail round-trip"

        verts, faces = cadio.surface_to_triangles(r["surf"])
        cadio.write_stl(stl, verts, faces, binary=True)
        v2, f2 = cadio.read_stl(stl)
        assert len(f2) == len(faces), "STL triangle count mismatch"

    print("PYTHON SMOKE TEST PASSED")


if __name__ == "__main__":
    main()
