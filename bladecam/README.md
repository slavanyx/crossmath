# BladeCAM

Cross-platform (desktop) toolkit for **5-axis flank-milling tool positioning
of impeller blades** with a cylindrical cutter, optimized for minimum machining
time. Heavy numerics in a **modern Fortran** core (C ABI); GUI in **Python
(PySide6 + PyVista)**.

> Staged inside the `crossmath` repo under `bladecam/` for now (tooling could
> not attach a separate repo in this session). It is self-contained and can be
> moved to a dedicated repository with its history intact.

## Architecture

```
bladecam/
  core/          Fortran numeric core (-> libbladecam shared library, C ABI)
    src/vec3.f90            vector primitives
    src/ruled_surface.f90   distribution parameter / striction (machinability)
    src/flank_geom.f90      two-point positioning + envelope deviation g
    src/bladecam_capi.f90   iso_c_binding wrappers (bc_*)
  tests/         Fortran unit tests (ctest)
  python/
    bladecam/core.py    ctypes bindings to libbladecam
    bladecam/blade.py   parametric twisted-blade generator
    bladecam/viewer.py  PySide6 + PyVista GUI
    demo.py             headless end-to-end demo (no GUI)
```

The core is platform-independent; the Python layer is the thin desktop shell.

## Build the core

```bash
cd bladecam
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Produces `build/core/libbladecam.{so,dylib}` (or `bladecam.dll` on Windows).

## Run

```bash
pip install -r python/requirements.txt        # numpy is enough for the demo
cd python
PYTHONPATH=. python demo.py                    # headless, writes CSV
PYTHONPATH=. python -m bladecam.viewer         # 3D GUI
```

Set `BLADECAM_LIB=/path/to/libbladecam.so` to point at the library explicitly.

## Implemented

**Phase 0-1 (geometry core)**
- Ruled-surface invariants: distribution parameter δ, striction (machinability
  map that predicts where cylindrical flank error will be worst).
- Two-point (Bedi/Mann/Menzel) cutter-axis positioning per ruling.
- Envelope deviation `g = dist(point, axis) − R` (over/undercut field).
- Verified Fortran↔Python pipeline; headless demo + interactive 3D viewer.

**Phase 2 (positioning optimization)**
- Per-ruling **min–max (Chebyshev)** refinement of the cutter axis (4-DOF
  Nelder–Mead in the Fortran core). On the demo blade this cuts peak flank
  deviation by ~89% vs two-point (789 → 85 µm).
- **Tolerance-constrained global smoothing**: low-pass-filters the cutter-axis
  field within a deviation budget, reducing orientation jerk (rotary-axis
  effort / cycle time) ~70% with no loss of worst-case accuracy.

## Roadmap

- **Phase 3** 5-axis inverse kinematics / post-processor; collision &
  reachability in the blade channel.
- **Phase 4** Time-optimal feed (TOPP-RA) + chatter/deflection caps; cycle-time
  report and rotary-axis smoothing within the tolerance band.
- **Phase 5** CAD import (STEP/IGES/STL), G-code/APT export, packaging.

See the design notes for the governing formulas behind each phase.
