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
7. **Constrain/verify the REALIZED quantity, discretized like the machine
   moves — and identically across every check.** A surprising family of bugs is
   the *discretization seam*: the code bounds or measures a sampled proxy that
   the machine never realizes. Three shapes of it, all found here:
   (a) **Wrong stencil.** TOPP bounded the *central-difference* axis slope at a
   station, but between two stations the machine traverses a straight joint
   segment whose slope is the *forward difference* — at a curvature kink that is
   larger, so the posted C-axis ran 0.63 rad/s on a 0.6 rad/s table. Fix: bound
   the forward-difference segment slope (the realized one). Rule: derivatives in
   a constraint must use the stencil of the motion the machine performs (segment
   midpoints for acceleration, segment forward-differences for velocity), not the
   convenient station-centred one.
   (b) **Between-sample blindness.** A constraint enforced only at sample points
   says nothing between them unless you bound it there — golden-section refine,
   midpoint enforcement, or a provable interpolation bound.
   (c) **Inconsistent fidelity for the same quantity.** Two checks of the *same*
   physical thing must use the *same* fidelity: the fixture-mesh clearance did a
   coarse scan while the obstacle-cloud clearance scanned *and refined*, so the
   mesh check silently verified a coarser quantity. Fix: share the refinement.
   Operationally, for every constraint/metric ask: *what continuous quantity does
   the machine realize, how is it discretized here, does that match the motion,
   and does every other check of it use the same discretization?*

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
- **Project files (`.bladecam`):** `to_project`/`load_project` snapshot the
  COMPLETE editable state (live values, strategy, full machine + post configs,
  embedded CAD rails / FRF) — self-contained, so a reopened job rebuilds an
  identical `build_params()`, computes, and posts the IDENTICAL program without
  any preset library or the original CAD. Oracles: full round-trip equality,
  file format guard, parametric (no-CAD) rails=None case. Mutation: drop
  rails / ignore values / skip the format guard — killed.
- Targets: `gui/main.py` (`_fill_results`, `_draw_3d`, `_render_scene`,
  `save_project`/`open_project`), `gui/model.py` (`to_project`/`load_project`),
  `gui/charts.py`, `workflow.py`.

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
- **Mesh-body collision (`mesh_collide.f90`):** tool-assembly capsules vs an
  imported TRIANGLE MESH (fixture / clamp / machine casting), the sub-mm check
  the capsule links approximate. seg-triangle distance = 0 if it pierces, else
  min(2 endpoint point-triangle, 3 edge-edge) -- validated vs brute-force sampled
  distance over random configs (catches any dropped feature). SIGNED: a capsule
  inside the closed solid is a collision (parity ray-cast inside test), so a tool
  buried in a fixture body is caught even when touching no face. Oracles: offset
  = height-r, pierce <0, plane-cross-outside not a hit, box inside <0 / outside
  >0, swept pass-through caught; pipeline fixture engulfing the tool fails the
  gate, a distant one does not. Mutation: pierce guard / inside sign / parity
  even-odd / each edge / endpoint term -- all killed.
- Targets: `machine.py` (Machine struct fields, tool_branch_capsules,
  structure_capsules, _rotx/_rotz), `struct_machine.f90` (seg_seg_dist,
  capsule_clearance, struct_clearance), `mesh_collide.f90` (pt_tri_dist,
  seg_tri_dist, point_in_mesh, mesh_clearance), `collision.f90` (assembly_*),
  `pipeline.py` (obstacle assembly, mount_z, link_clearance, fixture_mesh).

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
- **Persistent interval-dexel stock (rest-machining):** `Stock` carries solid
  intervals per ray ACROSS operations. `dexel_removed_intervals` returns the
  merged removed intervals (validated: total length == the `dexel_carve` oracle
  EXACTLY; intervals ascending+disjoint). Interval subtraction handles INTERIOR
  removal (a tilted tool cutting a ray's middle leaves TWO solid intervals — the
  reason a height field is insufficient). Oracles: block carve vs analytic
  cylinder, sequential union/monotone/bookkeeping, missing tool = no-op, and the
  rest-machining inequality finish-after-rough ≤ finish-from-raw (rest fraction
  < 1). Mutation: lo≥0 clamp / interval merge / subtraction clip+tail — killed.
- Targets: `dexel.f90` (ray_cyl, union/merge, dexel_removed_intervals),
  `verify.py` (removed_volume), `stock.py` (Stock, _subtract, channel_stock),
  `pipeline.py` (rest_machining).

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
- **Certified posts (`post.py`):** a PostConfig binds a control dialect
  (Heidenhain klartext TCPM / Siemens 840D TRAORI A3·B3·C3 / Fanuc 30i G43.4
  joint A·C) to a SPECIFIC machine, with axis letters/signs, limits and
  tolerances. `certify()` independently re-checks travel + rotary envelope,
  per-block rotary winding, the linearisation chord tolerance, the rotary-speed
  limit (with a documented reconstruction margin), and a forward-kinematics
  ROUND TRIP (posted joints reproduce the tool-tip path). Oracles: a roomy
  machine certifies clean; out-of-travel / over-wind / kinked / over-speed paths
  each fail their check; the Fanuc TEXT re-parses and reproduces the tip path
  (incl. an inverted rotary sign); every library pairing generates + certifies.
  Mutation: forward-kin order, dropped rotary sign, faked travel pass — killed.
  Post is a 5th preset KIND (round-trips, GUI post-config editor + "Save
  certified G-code" with the certification report).

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
  `pipeline.py` (`_blade_rails` trim, fillet_machining).
- **Fillet machining (recognised fillet -> ball-nose path):** `fillet_finish`
  rolls a ball (r_ball<=fillet_r) along the concave root fillet. On a closed-form
  90 deg corner EVERY invariant is exact: ball tangent to the fillet
  (|center-contact|=r_ball), contacts on the arc (|contact-Of|=fillet_r) spanning
  flank-tangent to hub-tangent, and the centre never gouges either wall
  (>= r_ball from both planes, equality at the end passes). A smaller ball reaches
  deeper, still gouge-free. Pipeline `fillet_machining` builds oriented flank/hub
  normals and reports a no-gouge margin. Mutation: ball offset / contact radius /
  Of bisector sign -- killed.

### Q. Inter-stage / integration consistency (the SEAMS between stages)
Each stage can be individually correct yet hand off wrong data to the next. Audit
the seams, not just the boxes. Concrete scenarios (in `test_integration.py`):
- **Full CAM chain round trip:** optimise → IK (`ik_path`) → post (`generate`) →
  re-parse the emitted joints → `forward_kin` must reproduce the OPTIMISED
  tool-tip contact path (currently <0.1 µm with 4-decimal posting). Catches any
  IK-convention / axis-letter / sign / units drift across the whole chain.
- **forces → feed → TOPP:** with a force-limited process the mechanistic cap must
  BIND below the nominal feed, the TOPP tip feed (√aprof·dL/ds) must stay within
  that cap, and a heavier cut must lower it. Ties `process.cutting_forces` →
  `effective_feed` → `topp` together.
- **recognition → trim → fillet coverage:** the flank-trim offset and the
  fillet's flank-tangent contact must COINCIDE (closed-form r_f·√((1−g)/(1+g)),
  = r_f at 90°), so the flank pass bottom meets the fillet pass top with no uncut
  gap and no double-cut. Ties `trim_root_fillet` ↔ `fillet_finish`.
- **geometry → optimise → envelope scale invariance:** scaling the blade and the
  cutter by s scales the machined-surface error by s.
- **stock ↔ envelope:** the dexel-carved machined surface and the `swept_surface`
  envelope (two independent computations) agree on the machined surface.
- **machine swap ↔ reachability ↔ certify:** a tiny machine flags the SAME axes
  in `reachability` and in the post `certify` report (both use one envelope
  check); mutation: certify ignoring the machine override — killed.
- **A/C unwrap ↔ winding:** the pipeline unwrap keeps per-block rotary steps
  small (no 2π jump) and the certify winding metric equals the actual max step.
- **barrel across stages:** a point ON the barrel surface reads ~0 in
  `deviation_barrel` AND `swept_deviation` (same tool model everywhere).
- Targets: the whole `pipeline.compute` assembly + `post.py` + `features.py`.

### R. Real-time interaction & GUI performance (responsiveness, thread safety)
- **No work on the UI thread:** the pipeline runs in `ComputeWorker`; every HEAVY
  operation (rest-machining ~1 s, fillet/mesh) must run via `OpWorker` off the UI
  thread and deliver results through a signal -- never block the 3D interactor.
  (Found & fixed: rest/fillet ops were synchronous.)
- **Realtime 3D:** `QtInteractor` (VTK) gives true mouse orbit/zoom/pan; the
  animation `QTimer` (~16 fps) advances the slider and `_show_tool_at` updates
  NAMED actors (tool/contact/ghosts) -- incremental, no actor accumulation, no
  full redraw. Param edits are debounced (single-shot QTimer) before recompute.
- **Currency:** every result flows through `_on_results` → 3D redraw + results
  table + the stage-bound chart cursor (§H Preview binding). Verify the displayed
  field is the live result, colour-keyed to the swept-envelope error.
- Targets: `gui/worker.py`, `gui/main.py` (`_run_bg`, `_anim_step`,
  `_show_tool_at`, `_bind_stage_chart`).

### S. Numerical precision & industrial tolerance budget
- **Error budget, CAD → optimise → post → machine.** Account every contributor:
  core math (double, validated to closed form ~1e-15), optimiser convergence
  floor, swept-envelope residual (the PHYSICAL flank-milling limit ε∝R·ℓ²/δ²),
  posting quantisation (now 1e-4 mm linear / 1e-5 deg rotary → sub-0.1 µm), and
  the dexel/mesh GRID tolerances (verification only, not the cut surface).
- **The dominant term is physical, not numeric:** a cyl/cone/barrel tool cannot
  exactly machine a non-developable twisted flank; the residual is reported by
  `swept_overcut` and minimised (barrel + global + stacked passes). Numerics are
  orders of magnitude below industrial profile tolerances (impeller ±10–50 µm;
  aero blisk ±10–25 µm + Ra 0.4–0.8 µm).
- Audit: is the posted decimal precision ≤ 0.1× the tolerance? Does any stage
  quantise/round below the budget (G-code decimals, viz grid mistaken for the cut
  surface)? Does the dexel resolution bias volumes (Z-map edge, ~0.3%)?
- Targets: `post.py` (decimals), `topp.f90` (feasibility), `flank_opt.f90`
  (convergence), `stock.py`/`dexel.f90` (grid), `verify.py`.

### T. Realized-quantity & discretization-consistency (the proxy-sample seam)
- **Per constraint/metric, fill the four-column table** (operating principle 7):
  | quantity | what the MACHINE realizes (continuous) | how the code discretizes it | match? |
  Flag any row where the discretization is not the motion's own stencil, leaves
  the between-sample value unbounded, or differs from another check of the same
  quantity.
- **Stencil targets:** `topp.f90` velocity limit (forward-difference SEGMENT
  slope — the straight joint move; FIXED) vs acceleration limit (segment
  MIDPOINT — the realized accel; already correct); `post.py` rotary-speed check
  (`|ΔA|/move_time`, the same forward difference — must agree with what TOPP
  bounds, oracle: posted speed ≤ v_rot, now 0.600 on a 0.6 table).
- **Between-sample targets:** a swept check of a NON-concave quantity (distance
  to a point/triangle) must bound the worst time, not sample it —
  `collision.f90 swept_clearance`, `struct_machine.f90 struct_clearance`, and
  `mesh_collide.f90 mesh_clearance` all do coarse-scan + golden-section refine
  (mesh FIXED to match; the others already did). The per-station
  `holder_clearance` is the known exception (watch-list). NOT every check needs
  sweeping: the fixture HALF-SPACE term in `assembly_clearance` is endpoint-exact
  and was kept simple — proof: the half-space clearance q0·n + L(ah·n) −
  R√(1−(ah·n)²) is concave over a normalize-lerp segment (q0 linear; the short
  great-circle arc makes ah·n bulge up; −R√(…) rises with ah·n), so its min is at
  an endpoint. Don't add a swept loop where concavity already guarantees it (the
  oracle is a refine-invariance test, not a golden search). Surface-error fields
  (`dev`, `devfield`, `swept_field`) are sampled on the `nu×nv` grid —
  interference strictly between grid lines is not a sample; the grid is the bound.
- **Same-quantity-same-fidelity targets:** mesh vs obstacle-cloud clearance
  (FIXED); barrel/cone/cylinder deviation used identically in the optimiser
  objective, the reported `dev`, and the swept metric (principle 5/§N).
- **Hidden-clamp targets (a clamp that turns an infeasible input into a
  valid-looking output):** `roughing.py` engagement `arccos(clip(1-ae/R,-1,1))`
  (ae>2R is impossible — now flagged via `engagement_feasible`); `flank_geom.f90`
  barrel `sqrt(max(d2,0))` (benign: never active for realistic Rb≫flute, but a
  point past the arc should read material-left, not a clamped radius); any
  `max(0,·)`/`clip`/`+1e-12` that could swallow a real violation.
- **Oracle:** differential test the proxy against a fine reference — refine the
  scan/grid 10× and confirm the metric does not move (if it does, the coarse
  discretization was the bug); round-trip the posted motion through the machine
  kinematics and confirm the realized velocities/accels are within limits.

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
- TOPP velocity-limit stencil (§T.a): bounded the central-difference station
  slope; now bounds the forward-difference SEGMENT slope the machine traverses,
  so posted rotary speed lands at v_rot (0.600 on a 0.6 table), not 0.63.
- mesh_clearance fidelity (§T.c): coarse-scan only → coarse-scan + golden-section
  refine, matching obstacle-cloud swept_clearance (oracle: recovers a -1.0 mm
  interpenetration whose worst time is between scan samples).
- Multi-objective default (principle 5): swept term on by default (mu=6.0,
  swept_weight=0.3) — the optimiser minimises the swept envelope it reports
  (~40 µm vs ~1400), not the misleading per-ruling residual.
- roughing engagement hidden clamp (§T): ae>2R now flagged (engagement_feasible),
  not silently posted as a 180° slot.
- JOINT-SPACE swept collision (#1+#8): every swept check (assembly/holder/struct/
  mesh) now runs on the FK of the linearly-interpolated joints — the path the
  machine TRULY traverses — densified with a substep count that bounds the chord-
  vs-arc residual (uniform conservative advancement). Replaces the part-frame
  chord lerp that cleared real rotary-arc collisions. Oracle: the crash gauntlet
  (G1/G2) vs a dense-FK ground truth.
- Swept holder-vs-cut-blade (#4): was per-station; now swept+refined over the
  motion (gauntlet G4 — a holder translating into the blade between stations).
- Hub/shroud endwalls (#3): the rotating disk + outer band, modelled as the
  surfaces of revolution of the hub/shroud rails, are now collision obstacles for
  the holder+spindle (reported as hub_clearance); gauntlet G3 catches a deep
  stubby tool diving into the hub.

- Neighbour-flank obstacle as an EXACT mesh (#2): neighbour blades were point
  clouds at the viz-grid spacing — a tool thinner than the spacing could thread
  between samples. Now checked as continuous UNSIGNED triangle meshes
  (mesh_clearance signed=False; a thin flank is an open sheet, so the closed-solid
  parity inside-test is skipped — any crossing already drives the seg-triangle
  distance below r). Gauntlet G5: a thin flute the point cloud cleared (+3) is
  caught by the mesh (-2). Endwalls/table stay point clouds (spacing bounded <
  holder radius / large flat solid).
- Lead-in / lead-out collision (approach & retract): the plunge to the first cut
  and retract from the last are now swept along the tool axis against the full
  obstacle world (approach_clearance / retract_clearance). Gauntlet G6.
- PASS-LINKING / blade-index move: on a multi-blade wheel the retract-index-
  reapproach between consecutive blades is now swept (index_clearance); every
  blade's lead-in/out is the checked one by rotational symmetry. Gauntlet G7.
- Table as an EXACT mesh (#2): the trunnion table is a continuous fan-disc + side
  mesh appended to the unsigned obstacle mesh, not the structure_obstacles point
  cloud (which a thin tool could thread between, ~78 mm spacing at the rim).
  Gauntlet G8 (point cloud misses a dip the mesh catches).

OPEN (real residual limitations — state honestly, improve if in scope):
- TOPP at an exact velocity cusp: bounded but ~1.75× amax at the singular grid
  station (discretization).
- The blade-index move is modelled as ONE joint-linear G0 (retract-end ->
  reapproach-start); a real post may break it into retract-plane / rapid / plunge
  sub-moves. The imported-fixture mesh is checked as a closed solid (signed); the
  neighbour flanks + table are exact unsigned meshes; the hub/shroud endwalls
  remain point clouds (spacing bounded < holder radius).
- Structural collision is tool-assembly + table/fixture/hub/shroud, NOT a full
  kinematic machine model (no ram/column/spindle-housing link geometry); table
  frame assumes table-table A-C (wrong for head-head kind=1 — verify or guard).
- Mechanistic force coefficients (Kt/Kr/Kte/Kre) are nominal, not measured;
  helix lag is ignored (instantaneous engagement). Treat outputs as indicative.
- Dexel machined-error-along-normals was dropped (unreliable on coarse normals);
  only removed_volume is trusted. Z-dexel volume is Cavalieri (single direction).
- Default tight blisk (n_blades=11) is not collision-free; barrel is verify+opt
  but the per-station devfield uses the barrel only for the global strategy.
- swept_clearance "hit slack" mutation survives (provably benign via the hi>lo
  guard) — do not "fix".
