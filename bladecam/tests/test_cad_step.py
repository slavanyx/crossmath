#!/usr/bin/env python3
"""STEP/IGES import round-trip test (skips if OpenCASCADE is unavailable)."""
import sys
import os
import tempfile

try:
    import numpy as np
    from bladecam import cadio
except ImportError as e:
    print(f"SKIP STEP test ({e})")
    sys.exit(0)

try:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
except ImportError:
    print("SKIP STEP test (OpenCASCADE / cadquery-ocp not installed)")
    sys.exit(0)


def main():
    with tempfile.TemporaryDirectory() as d:
        step = os.path.join(d, "part.step")
        shp = BRepPrimAPI_MakeCylinder(10.0, 30.0).Shape()
        w = STEPControl_Writer(); w.Transfer(shp, STEPControl_AsIs); w.Write(step)

        verts, faces = cadio.read_step(step)
        assert verts.shape[1] == 3 and faces.shape[1] == 3, "mesh shape"
        assert len(faces) > 0, "no triangles read from STEP"
        assert np.all(np.isfinite(verts)), "non-finite vertices"
        # round-trip through STL writer/reader as well
        stl = os.path.join(d, "part.stl")
        cadio.write_stl(stl, verts, faces)
        v2, f2 = cadio.read_stl(stl)
        assert len(f2) == len(faces), "STL round-trip triangle mismatch"
        # dispatcher
        v3, f3 = cadio.read_cad(step)
        assert len(f3) == len(faces), "read_cad dispatch mismatch"

    print(f"STEP import OK ({len(faces)} triangles)")


if __name__ == "__main__":
    main()
