# BladeCAM demo gallery

Six worked example blades spanning the difficulty range, so you can see the
system run and visualise its output. See [`out/SUMMARY.md`](out/SUMMARY.md) for
the rendered gallery and the headline numbers.

The story the gallery tells:

| part | what it shows |
|---|---|
| **01 developable** | A zero-twist flank a cylinder machines almost exactly (~tens of µm) — the ideal reference. |
| **02 mild impeller** | A gently twisted real blade — modest envelope error. |
| **03 twisted (cylinder)** | A properly twisted blade with a plain cylinder: the swept-envelope error grows (≈ R·ℓ²/δ²) — the real flank-milling limit. **Before.** |
| **04 twisted (optimised)** | The *same* blade with the swept-overcut penalty on: the optimiser tilts the axes and the real error drops ~60×. **After.** |
| **05 tall (stacked)** | A blade taller than the flute, split into stacked flank passes. |
| **06 blisk** | A full multi-blade wheel. |

Each render colours the flank by the **swept-envelope overcut (µm)** — the real
machined error — with the optimised tool axes (black) and the contact path (red).

## Use the parts in the GUI

Every part is exported two ways:

- `out/<name>_rails.csv` — **File ▸ Import rails CSV**
- `out/<name>.step` — **File ▸ Load blade from STEP/IGES**

Then press **Recompute** and step through **Preview** (the Guide panel explains
each stage). Try **Operations ▸ Minimize swept overcut** on `03_twisted` to
reproduce the `04` result yourself.

## Regenerate

```bash
PYTHONPATH=python BLADECAM_LIB=build/core/libbladecam.so \
    python3 demos/make_demos.py
```

(Rendering needs the `gui` extras — pyvista; STEP export needs the `cad` extra —
cadquery-ocp. The script degrades gracefully if either is missing.)

## Super-complex parts (`make_complex_demos.py`)

Five realistic centrifugal / mixed-flow / blisk blades (`blade.make_complex_blade`
— backsweep, S-warp, lean, mixed-flow radial bulge), each rendered as a **5-stage
workflow montage** (geometry → positioning → kinematics → feed → verification +
a summary card) so every step is visualised. See [`cout/SUMMARY.md`](cout/SUMMARY.md).

| part | what it stresses |
|---|---|
| 01 mixed-flow | axial→radial sweep with twist — machined well (~80 µm) |
| 02 backswept | trailing-edge lean (advance ∝ u²) + S-warp |
| 03 high-twist | strong non-developability; the swept penalty keeps it in check |
| 04 s-warp turbine | an inflected flank a cylinder **cannot** fit — the verification panel shows where it overcuts (the honest limit; use a barrel/point-mill) |
| 05 tall leaned | a z-bowed shroud, machined as stacked passes |

```bash
PYTHONPATH=python BLADECAM_LIB=build/core/libbladecam.so \
    python3 demos/make_complex_demos.py
```

