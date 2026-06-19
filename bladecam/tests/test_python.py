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
