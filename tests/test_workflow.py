#!/usr/bin/env python3
"""Workflow stage model: the GUI steps through these scenes, so every stage must
produce a renderable, finite scene from a pipeline result. Guards against a
stage referencing a result key the pipeline stopped emitting, or emitting NaN
geometry/scalars that would crash the 3D view.
"""
import sys

try:
    import numpy as np
    from bladecam import workflow
    from bladecam.pipeline import compute, Params
except ImportError as e:
    print(f"SKIP workflow ({e})")
    sys.exit(0)

FAILED = []


def check(c, name, d=""):
    print(f"  {'ok  ' if c else 'FAIL'} {name} {d}")
    if not c:
        FAILED.append(name)


def _finite_mesh(m):
    pts = np.asarray(m.get("points")) if "points" in m else None
    if pts is not None and not np.all(np.isfinite(pts)):
        return False
    sc = m.get("scalar")
    if sc is not None and not np.all(np.isfinite(np.asarray(sc))):
        return False
    if m.get("type") == "lines":
        for p0, p1 in m["segments"]:
            if not (np.all(np.isfinite(p0)) and np.all(np.isfinite(p1))):
                return False
    return True


def main():
    r = compute(Params(strategy="global", twist=1.0))

    # stage keys are stable and unique
    check(len(workflow.STAGE_KEYS) == len(set(workflow.STAGE_KEYS)) >= 5,
          "five+ unique stages defined")

    scenes = workflow.all_scenes(r, R=6.0)
    check(len(scenes) == len(workflow.STAGE_KEYS),
          "all_scenes covers every stage")

    # every stage maps to a real analysis-tab name (these drive the GUI tabs)
    valid_tabs = {"Deviation", "Machinability", "Feed", "Compare", "Chatter"}
    check(set(workflow.STAGE_CHART) == set(workflow.STAGE_KEYS),
          "STAGE_CHART covers exactly the stages")
    check(all(v in valid_tabs for v in workflow.STAGE_CHART.values()),
          "STAGE_CHART points at real analysis tabs")

    for sc in scenes:
        ok = bool(sc["title"]) and bool(sc["blurb"]) and len(sc["meshes"]) >= 1
        check(ok, f"stage '{sc['key']}' has title/blurb/meshes")
        check(all(_finite_mesh(m) for m in sc["meshes"]),
              f"stage '{sc['key']}' meshes are finite")
        check(len(sc["metrics"]) >= 1 and all(len(t) == 2 for t in sc["metrics"]),
              f"stage '{sc['key']}' has (label,value) metrics")
        # surface scalar, when present, must match the surface point count
        for m in sc["meshes"]:
            if m.get("type") == "surface" and m.get("scalar") is not None:
                npts = np.asarray(m["points"]).reshape(-1, 3).shape[0]
                check(np.asarray(m["scalar"]).reshape(-1).size == npts,
                      f"stage '{sc['key']}' surface scalar matches points")

    # a bad stage key is rejected, not silently empty
    try:
        workflow.stage_scene(r, "nonsense")
        check(False, "unknown stage raises")
    except KeyError:
        check(True, "unknown stage raises")

    if FAILED:
        print(f"\nFAILED: {FAILED}")
        sys.exit(1)
    print("\nWORKFLOW TESTS PASSED")


if __name__ == "__main__":
    main()
