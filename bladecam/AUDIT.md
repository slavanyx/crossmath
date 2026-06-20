# BladeCAM — Systemic Audit Prompt

A reusable, exhaustive audit driver for the BladeCAM 5-axis flank-milling system
(Fortran numeric core + Python/PySide6/PyVista GUI). Hand this whole file to a
capable agent. The goal is **coverage of every failure class**, not another
ad-hoc pass — most real bugs here have hidden in categories the previous pass
didn't think to look at.

---

## 0. Mission

Audit the entire system for correctness and fidelity along **all** the angles in
§3, fixing every confirmed defect. Work the matrix top to bottom; do not stop at
the first clean area. A pass is finished only when the §6 exit criteria hold.

## 1. Operating principles (these are what actually find bugs here)

1. **External oracles over self-consistency.** Self-referential audits converged
   to "clean" while real bugs remained; every real bug was caught by comparing
   against something *outside* the code: a closed-form result, an analytic
   scaling law, a fuzzer, an independent reconstruction. Prefer those.
2. **Falsify, don't confirm.** For every property, construct an input designed to
   *break* it. Sweep parameters and check power laws (e.g. ε ∝ R¹·ℓ²·δ⁻²); a
   wrong exponent is a bug. Test the boundary, the degenerate, the extreme.
3. **Differential testing.** Compute the same quantity two independent ways and
   diff them: per-station vs swept, sampled vs continuous, modal vs measured FRF,
   Fortran vs a throwaway NumPy reimplementation, forward∘inverse round-trips.
4. **Distrust the headline number.** Ask what a metric *physically means* and
   whether it can be trivially satisfied. (Per-ruling deviation is ~0 for any
   ruled surface — it looked great and meant nothing; the real error was the
   swept envelope.)
5. **The metric you optimize must equal the metric you report.** Penalty terms,
   objective functions, and reported KPIs must use the *same* geometry (e.g. the
   swept penalty must use the finite flute, like the swept metric — not an
   infinite line).
6. **Every finding is not done until it has a mutation-verified regression
   test.** Write the test, confirm it FAILS on the unfixed code (revert the fix
   or mutate the constant), then fix and confirm it passes.

## 2. Rules of engagement (per finding)

For each confirmed defect:
1. Minimal reproduction (a few lines, ideally pure-numeric).
2. Root-cause statement: which assumption was wrong, and the bug *class* (§3).
3. Fix at the right layer (don't patch a symptom in Python if the core is wrong).
4. Regression test that fails without the fix (mutation-verified) and is added to
   the appropriate `tests/` suite + registered in `tests/CMakeLists.txt`.
5. `cmake --build build && (cd build && ctest)` — all suites green, including the
   `-Werror` warnings gate and the Debug `-fcheck=all` build.
6. One commit per coherent fix, with the root cause and the before/after numbers
   in the message. Push to the working branch only.

If an area is audited and *clean*, say so explicitly and state **what oracle**
established it — "looks right" is not a result.

## 3. The audit matrix — all angles

For each angle: the question, the techniques, and BladeCAM-specific targets.

### A. Mathematical correctness
- Are formulas, signs, and conventions right? Compare to closed form wherever one
  exists.
- Oracles already in play (re-verify, extend): distribution parameter δ on a
  helicoid (δ = h/ω); flank twist law ε ∝ R·ℓ²/δ²; chatter a_lim,min =
  2kζ(1+ζ)/(KtN) and r*=√(1+2ζ); cone signed distance = (perp−ρ)·cos γ.
- Targets: `ruled_surface.f90` (δ, striction, vstar), `flank_geom.f90`
  (deviation, deviation_cone, two_point), `kinematics.f90` (A/C convention),
  `chatter.f90`, `topp.f90` (the MVC/accel feasibility math).

### B. Physical fidelity (does the model match reality?)
- Does the model capture the real phenomenon, or a convenient proxy? This is
  where the biggest bugs lived (per-station vs swept-envelope; holder vs the
  blade being cut; continuous vs per-pose collision).
- Check: is the cutting/engagement model right? Are forces/deflection coupled to
  feed? Is the envelope (not just the contact line) the error metric? Conical AND
  barrel tools (swept_surface/swept_deviation are currently cylinder-only —
  audit whether cone/barrel envelopes are handled). Scallop between flank passes.
- Targets: `flank_opt.f90` objective + penalties, `collision.f90`, `pipeline.py`
  metric assembly.

### C. Numerical robustness & degeneracies
- Zero-length vectors (`unit3(0)`), parallel/antiparallel directions (cross→0:
  a'∥ruling, axis tilt reference), 180° axis reversals in interpolation,
  coincident points, single-station paths, empty clouds.
- Finite-difference accuracy at domain boundaries (np.gradient edge_order),
  convergence floors vs true optima, conditioning of det/normalization,
  catastrophic cancellation.
- atan2/acos domain edges; modulo/branch choices (chatter lobe eps); NaN/Inf
  propagation (verify guards actually fire — feed NaN-inducing inputs).

### D. Algorithmic correctness
- Optimizer: does it actually reach the optimum (not a seed)? Scale invariance
  (non-dimensionalized objective); rigid-motion invariance; monotonicity (more
  weight ⇒ monotone trade). Convergence vs iteration budget.
- TOPP: feasibility GUARANTEE — reconstruct the trajectory and check |v|≤vmax and
  |a|≤amax (per-axis), especially at cusps; verify the output profile slope lies
  in the sdd interval everywhere.
- Numerical optimizers (Nelder-Mead, golden-section): unimodality assumptions,
  bracketing, restart logic.

### E. API / boundary integrity (C ABI ↔ ctypes ↔ NumPy)
- Array layout: NumPy (n,3) C-contiguous ≡ Fortran (3,n). Every `bc_*` wrapper:
  do `argtypes` match the Fortran signature (count, by-value vs pointer, int vs
  double)? A mismatch is silent memory corruption.
- Units consistency across the boundary (mm, rad, mm/min vs mm/s, rpm).
- `intent(out)` arrays sized correctly on the Python side; contiguity enforced.
- Differential test: call each `bc_*` and reproduce it in NumPy; diff.

### F. Edge cases & fuzzing
- Random/extreme parameters: tiny/huge R, twist→0 and large, n_blades extremes,
  nu=2, degenerate blades, near-zero δ.
- CAD fuzzing (OCC): non-manifold, sphere/torus, open shells, tiny faces — the
  source of a real StdFail crash. Re-run with new shape families.
- Property-based: feasibility/clearance/deviation invariants must hold for ALL
  sampled inputs, not a chosen few.

### G. Consistency & defaults
- Optimized metric == reported metric == GUI-displayed metric (geometry, units,
  finite vs infinite extent).
- Are the shipped defaults a *valid* setup (e.g. flute length covering the
  ruling), or do they trip the collision/feasibility checks out of the box?
- Strategy/operation selectors compute what they claim; selected vs displayed.

### H. Presentation / GUI correctness
- Does the GUI lead with the *physically meaningful* number, not a degenerate
  proxy? Colour maps keyed to the right field with sane ranges (far-field
  clearance shouldn't swamp the overcut scale). Workflow stages show the right
  3D artefacts and the matching analysis tab. No work on the UI thread.
- Targets: `gui/main.py` (`_fill_results`, `_draw_3d`, `_render_scene`),
  `gui/model.py`, `gui/charts.py`, `workflow.py`.

### I. Performance & complexity
- Hotspots (optimizer is ~96% — verify), big-O of new routines (golden-section ×
  points × segments), redundant recompute, accidental O(n²) in Python loops.
  Time a full `compute()`; profile if >a few seconds.

### J. Test-suite integrity (audit the tests themselves)
- Mutation testing: flip a sign / perturb a constant in each core routine and
  confirm SOME test fails. Untested mutations reveal blind spots.
- Are assertions falsifiable and tight, or tautological / too loose?
- Do tests encode *wrong* hypotheses (past examples: "thinner bands reduce
  deviation", "penalty ⇒ smoother")? Re-derive the expected value independently.

### K. Machine model, reachability & structural collision
- **Reachability:** does the IK path respect the profile's X/Y/Z travel and A/C
  rotary ranges? Construct a path that pokes just past each limit and confirm the
  exact axis + excess is flagged; a path inside is clean. Check A/C unwrap doesn't
  spuriously exceed C range.
- **Profiles:** every default machine is a valid TOPP drop-in (vmax/amax len 5,
  kind ∈ {0,1}); editing limits in the config editor flows into Params.
- **Structural collision:** the trunnion TABLE is a static obstacle in the PART
  frame. Oracle: a tool assembly tilted toward the table is caught, one pointing
  away clears; the near-vertical optimised path must NOT false-trigger (table
  sits below the flute base by mount_clearance). Verify the frame assumption
  (table moves with the part in table-table A-C) — it is WRONG for head-head.
- **Structural kinematic links (capsule model):** the tool assembly vs the
  trunnion cradle yoke (two posts + cross-beam) and machine column, each a rigid
  link placed in the PART frame by the SAME convention as the IK (cradle: Rz(C)
  about pivot — A-invariant relative to the part; column base-fixed: Rz(C)Rx(A);
  head-head: structure static). Oracles: capsule-capsule clearance = segment-
  segment distance − ra − rb, cross-checked vs brute-force sampling and closed
  forms (parallel / collinear-overlap / crossing / point-vs-segment); the swept
  scan must catch a mid-motion hit endpoints miss; `_rotx/_rotz` must reproduce
  the IK world→part axis map; defaults must NOT false-trigger; a fat column
  through the work zone must fail the collision gate. Mutation: drop the radius
  subtraction / unclamp seg-seg / endpoints-only scan / swap Rx·Rz — all killed.
- Targets: `machine.py` (Machine struct fields, tool_branch_capsules,
  structure_capsules, _rotx/_rotz), `struct_machine.f90` (seg_seg_dist,
  capsule_clearance, struct_clearance), `collision.f90` (assembly_*),
  `pipeline.py` (obstacle assembly, mount_z, link_clearance).

### L. Cutting-force & process physics (mechanistic model)
- Force scaling: F_peak, F_mean, power, torque must rise monotonically with fz,
  ap, ae; power ≈ mean-tangential·R·ω (cross-check torque·ω). Edge terms (Kte,
  Kre) give a non-zero force at fz→0. Engagement arc φ_ex=acos(1−ae/R) correct at
  ae=0, R, 2R (slot → π).
- Feed caps: deflection, max-force and power caps each BIND when tightened
  (monotone); `effective_feed` is the min; `feed_feasible` is False when forces
  overload at fz→0. Floor at 1 mm/min keeps TOPP well-posed — verify no 0-feed
  NaN downstream.
- **Helix lag:** forces integrate over the axial depth with each slice lagged by
  ψ(z)=z·tanβ/R. Oracles: β=0 reduces EXACTLY to the flat-tooth model; mean
  torque/power/Fx are helix-INVARIANT (<1%) while the peak DROPS with β (cut
  spread axially). Mutation: zero the lag → peak-drop test must fail.
- **Measured-coefficient calibration:** per-rev average forces are linear in fz;
  `identify_coefficients` inverts the slopes/intercepts through the arc integrals
  (I1..I4, validated vs quadrature) to recover Kt/Kr/Kte/Kre. Oracle: synth
  averages from KNOWN coeffs via the discrete model → recover within <2%
  (cross-checks the two paths). Material library (Al/SS/Ti/Inconel) ordered
  Kt: Inconel>Ti>SS>Al, usable in ProcessParams and as measured-coeff tools.
- Targets: `process.py` (cutting_forces helix, identify_coefficients,
  _arc_integrals, MATERIAL_COEFFS), `presets.py` (material tools), `pipeline.py`.

### M. Material-removal / dexel verification
- Carve primitive vs brute force: removed length = analytic chord; misses outside
  R / beyond the flute cap; OVERLAPPING poses union (single interval) AND
  DISJOINT poses (sum, gap not counted) — both. first_cut = nearest entry.
- Removed volume vs closed form (π R² Lf, <1%); tilted poses (Cavalieri holds);
  progressive carve over poses 0..k monotone and converges to the full volume.
- Targets: `dexel.f90` (ray_cyl, union/merge), `verify.py` (removed_volume).

### N. Tooling families (cylinder / cone / barrel consistency)
- The SAME tool family must be used everywhere for a given run: optimizer
  objective, per-station devfield, swept deviation, swept surface, GUI render.
  Oracle: a point ON the cone/barrel surface reads ~0 in EACH of those paths;
  cylinder defaults (gamma=0, Rb=0) reproduce the old results byte-for-byte.
- Barrel-aware optimisation fits the arc flank better than a cylinder-optimised
  axis; the optimiser's reported dev uses the active tool model.
- Targets: `flank_geom.f90` (tool_sdf, deviation_*), `flank_opt.f90` (tool_dev),
  `pipeline.py` (eff_gamma/eff_Rb threading).

### O. Post-processors (G-code / Heidenhain TCPM)
- ISO: inverse-time (G93) feeds reconstruct the TOPP cycle time; one move per
  segment; constant-feed fallback. Heidenhain: framed BEGIN/END PGM, TCPM
  activate+reset, one LN per pose, tool vectors UNIT and equal to the optimised
  axis, sane reconstructed feed. Units (mm, mm/min) and signs.
- Edge cases: single-pose path, non-unit input axis. Targets: `postproc.py`.

### P. CAD I/O & feature extraction
- Rail extraction: edge-based (trimmed faces) and UV fallback recover the rails;
  ruling-direction tie-break is deterministic; orientation normalised hub-first /
  low-Z-first across faces. Blisk: one rail pair per blade. Fuzz with new shape
  families (sphere/torus/open shells) — must degrade gracefully, never crash.
- **Feature recognition (`features.py`):** splitter blades classified from
  streamwise length (largest-significant-gap split; uniform set → all main; the
  short blade IS the splitter). Root fillets/blends recognised by radius of
  curvature — analytic (cylinder/torus/sphere via OCP isinstance) or sampled-grid
  Menger curvature; oracle recovers a known fillet radius (≈3 mm), flags a tight
  blend, clears a gentle flank, ∞ on a flat patch; ruled flanks have lower
  characteristic curvature than blends. Fillet-aware trim moves the HUB rail up
  the ruling by the offset (shroud unchanged, clamped, never inverted), wired to
  `pipeline.root_fillet_r` (shortens the machined rulings). End-to-end: a STEP
  blisk of 2 main + 1 splitter + a cylinder fillet face → fillet excluded, 3
  flanks kept, classified 2 main + 1 splitter. Mutation: always-split / inverted
  curvature / trim-shroud — all killed.
- Targets: `cadio.py` (`_rails_from_face*`, `_orient_hub_first`,
  `rails_from_all_faces`, `_face_min_radius`, `blades_from_cad`), `features.py`,
  `pipeline.py` (`_blade_rails` trim).

## 4. Module-by-module sweep (don't skip any)

Core (`core/src`): `vec3.f90`, `ruled_surface.f90`, `flank_geom.f90`,
`flank_opt.f90`, `kinematics.f90`, `topp.f90`, `chatter.f90`, `collision.f90`,
`dexel.f90`, `bladecam_capi.f90`.
Python (`python/bladecam`): `core.py` (bindings), `blade.py`, `optimize.py`,
`pipeline.py`, `process.py`, `machine.py`, `verify.py`, `cadio.py`,
`pointmill.py`, `roughing.py`, `postproc.py`, `workflow.py`, `gui/*`.
For each: list its assumptions/preconditions explicitly, then attack each one.

## 5. Suggested execution order (cheap signal first)

1. Test-suite mutation sweep (J) — finds blind spots fast.
2. API/boundary differential tests (E) — silent corruption is highest-severity.
3. Math oracles + scaling laws (A) and physical-fidelity diffs (B).
4. Degeneracy/fuzz probes (C, F) — include new modules (K–P).
5. Algorithmic guarantees (D), then consistency/defaults/GUI/perf (G, H, I).
6. Subsystem angles K–P (machine, forces, dexel, tooling, posts, CAD) —
   each with its own external oracle as listed.

## 6. Exit criteria (when the pass is genuinely done)

- Every angle A–P has been exercised with at least one *external* oracle or
  adversarial input, and the result (clean / fixed) is stated with its oracle.
- Every confirmed defect has a mutation-verified regression test, committed.
- A full mutation sweep of the core leaves no surviving untested mutation in the
  audited routines (or survivors are explicitly justified).
- `ctest` fully green incl. the warnings gate and Debug `-fcheck=all`.
- A short report: per angle — what was checked, the oracle used, findings, fixes,
  and any *residual* known limitations stated honestly (e.g. grid-TOPP at exact
  cusps is bounded to ~1.75× at the singular station, not 1.0×).

## 7. Known limitations / watch-list (don't "re-discover" — verify or extend)

RESOLVED (verify they haven't regressed; don't re-report as new):
- Conical/barrel swept envelope — DONE (swept_deviation/surface take gamma, Rb,
  lamc; on-surface reads 0). Barrel-aware optimisation — DONE.
- TOPP short-path OOB (n<3) — guarded. Two-point degenerate normal — guarded.
- Per-station vs swept, per-pose vs continuous collision — DONE.

OPEN (real residual limitations — state honestly, improve if in scope):
- TOPP at an exact velocity cusp: bounded but ~1.75× amax at the singular grid
  station (discretization).
- Holder-vs-current-blade check is per-station while the assembly/neighbour and
  fixture checks are swept; audit whether a between-station holder swing can slip.
- Structural collision is tool-assembly + table/fixture, NOT a full kinematic
  machine model (no ram/column/trunnion link geometry); table frame assumes
  table-table A-C (wrong for head-head kind=1 — verify or guard).
- Mechanistic force coefficients (Kt/Kr/Kte/Kre) are nominal, not measured;
  helix lag is ignored (instantaneous engagement). Treat outputs as indicative.
- Dexel machined-error-along-normals was dropped (unreliable on coarse normals);
  only removed_volume is trusted. Z-dexel volume is Cavalieri (single direction).
- Default tight blisk (n_blades=11) is not collision-free; barrel is verify+opt
  but the per-station devfield uses the barrel only for the global strategy.
- swept_clearance "hit slack" mutation survives (provably benign via the hi>lo
  guard) — do not "fix".
