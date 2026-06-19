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
  effort / cycle time) with no loss of worst-case accuracy.

**Phase 3 (kinematics / post / collision)**
- 5-axis **inverse kinematics** for a table-table A–C machine (`kinematics.f90`),
  verified by forward/inverse round-trip.
- Neighbour-blade **collision / reachability** check (tool-axis vs. rotated
  adjacent blade) with min-clearance reporting.
- Minimal **G-code post-processor** (`postproc.py`).

**Phase 4 (time-optimal feed)**
- **TOPP** time-optimal path parameterization (`topp.f90`): forward/backward
  integration under per-axis velocity/acceleration limits; verified against the
  analytic trapezoidal-profile time.
- **Process feed caps** (`process.py`): tool-deflection and feed-ceiling limits
  folded into TOPP as a tool-tip feed constraint; **cycle-time** report.
- Demonstrates smoothing → shorter cycle time at equal accuracy (3.90 → 3.38 s).

**Phase 5 (global optimization, CAD I/O, packaging)**
- **Global envelope optimization** (`optimize_global` in `flank_opt.f90`):
  Gauss–Seidel block coordinate descent minimising
  `J = Σ max_v|g_i| + μ·Σ ‖axis_i − neighbour_avg‖²`, optimising accuracy and
  orientation smoothness jointly. On the demo blade it reaches ~13 µm peak
  deviation with the smoothest axis field *and* the fastest cycle time —
  dominating min-max / smoothed on every metric.
- **CAD I/O** (`cadio.py`): STL mesh read/write (ASCII + binary), hub/shroud
  rail-polyline CSV import/export for externally-designed blades. (STEP/IGES
  needs a B-rep kernel; the rail/STL path is the bridge.)
- **Packaging**: `pyproject.toml`, `build.sh`, and a Python pipeline smoke
  test wired into `ctest`.

**GUI** — architected desktop app in `bladecam/gui/` (PySide6 + PyVista +
matplotlib):
- `model.py` — Qt-free application state, **parameter schema** (`PARAM_SPEC`)
  and **strategy registry** (`STRATEGIES`); add a parameter or strategy in one
  line. Fully headless-testable (covered by the ctest Python suite).
- `worker.py` — **background compute** (QThreadPool) so the UI never blocks.
- `charts.py` — Qt-free matplotlib panels (deviation-by-strategy, machinability
  map, TOPP feed profile); reusable for reports/CI thumbnails.
- `main.py` — **dockable** main window (Parameters · 3D view · Results ·
  Analysis), menus/toolbar/status bar, STL & rails import, STL/G-code export.

  Run: `PYTHONPATH=. python -m bladecam.viewer` (or `-m bladecam.gui.main`).

## Roadmap

- STEP/IGES import via OpenCASCADE (pythonocc).
- Chatter stability-lobe model from a measured tool-tip FRF.
- Conical/barrel tool support for strongly warped blades.
- Double-flank milling of thin blades (one tool, both sides).

See the design notes for the governing formulas behind each phase.
