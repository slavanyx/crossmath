# BladeCAM super-complex demo gallery

Realistic centrifugal / mixed-flow / blisk blades, each shown as a 5-stage **workflow montage** (geometry → positioning → kinematics → feed → verification).

## 01_mixed_flow

Mixed-flow impeller: the flank sweeps from axial to radial (the radius bulges mid-span) with real twist — and a cylinder still machines it well.

- machined-surface error **76 µm** · cycle 3.3 s · ✅ · reachable · finite=True
- operations: {'rough_len_mm': 5592.539658222864, 'fillet_gouge_free': True, 'rest_fraction': 0.40416086358962944, 'n_stacked': 1}

![01_mixed_flow](01_mixed_flow_workflow.png)

## 02_backswept

Backswept centrifugal blade: the trailing edge leans back (advance ∝ u²) with an S-warp — a strongly 3-D flank.

- machined-surface error **296 µm** · cycle 3.9 s · ✅ · reachable · finite=True
- operations: {'rough_len_mm': 4880.480865765845, 'fillet_gouge_free': True, 'rest_fraction': 0.2931712225625443, 'n_stacked': 1}

![02_backswept](02_backswept_workflow.png)

## 03_high_twist

A strongly twisted blisk blade — high non-developability; the swept penalty tilts the axes to keep the envelope error in check.

- machined-surface error **133 µm** · cycle 5.9 s · ✅ · reachable · finite=True
- operations: {'rough_len_mm': 4884.1052979657, 'fillet_gouge_free': True, 'rest_fraction': 0.24491512541333768, 'n_stacked': 1}

![03_high_twist](03_high_twist_workflow.png)

## 04_s_warp_turbine

An S-warped turbine flank (inflected camber): a cylinder fundamentally cannot fit it — the render shows WHERE the envelope overcuts (the honest flank-milling limit; use a barrel/point-mill here).

- machined-surface error **3967 µm** · cycle 5.4 s · ⚠️ collision · reachable · finite=True
- operations: {'rough_len_mm': 7962.402781946339, 'fillet_gouge_free': True, 'rest_fraction': 0.35771913316487536, 'n_stacked': 1}

![04_s_warp_turbine](04_s_warp_turbine_workflow.png)

## 05_tall_leaned

A tall, leaned blade (z-bowed shroud) — machined as stacked flank passes (each pass a thinner sub-band of the ruling).

- machined-surface error **363 µm** · cycle 4.0 s · ✅ · reachable · finite=True
- operations: {'rough_len_mm': 5976.25933386986, 'fillet_gouge_free': True, 'rest_fraction': 0.30725287749704105, 'n_stacked': 1}

![05_tall_leaned](05_tall_leaned_workflow.png)

