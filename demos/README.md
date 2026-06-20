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
