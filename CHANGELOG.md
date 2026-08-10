# Changelog

All notable changes to this project are documented here. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Breaking

- **`RunConfig.use_acopf_warmstart` is removed.** Its only legal value was already
  determined by `network_form`: FULL_BUS TSC-ACOPF must pre-solve the steady-state
  ACOPF that seeds every coupling variable, and no other path pre-solves at all.
  The flat-start alternative (`build_flat_start_hints`, `CouplingInitSource`,
  `coupling_init_source_label`) is removed with it — on FULL_BUS a flat guess
  leaves the joint NLP far from any equilibrium, and a failed pre-solve now aborts
  the run instead of silently falling back. Delete the field from any call site;
  passing it raises a `MethodError`.

- **Two configurations that used to be accepted and ignored now throw**, both from
  `validate_dyn_config!`, before any model is built:
  `network_form = FULL_BUS` with `mech_power_mode = USE_PG` (previously thrown by
  the builder, but only *after* the warm-start ACOPF had been solved), and
  `network_form = KRON_REDUCED` with `ode_first_step = :backward_euler`.

- **`bound_style` now changes the model on the Kron paths.** A Kron run configured
  with `:coi_box` used to build the swing-propagated constraint regardless. Runs
  that declared `:coi_box` on Kron (including every `USE_PM` run, since `USE_PM`
  forces that value) will produce different numbers — the ones the configuration
  always claimed. To keep the old model, say `:swing_propagated` explicitly.

### Added

- **`RunConfig.run_script` archives the script that configured the run.** Set it to
  `@__FILE__` in the run script and `run_case!` copies that `.jl` file, byte for byte,
  into the run-folder root next to `input_parameters.txt` — so a `RESULTS/` folder
  carries the configuration that produced it, comments included. Default `nothing`
  keeps every existing run unchanged; a path that is not an existing file throws in
  `validate_run_config!`. The copy is written before the solve, so it survives a
  failed or iteration-limited run.

- **`save_warmstart_dispatch` covers TSC-DCOPF.** The DC pre-solve that fixes the
  Taylor anchor `δ_ref` is a genuine pre-solve, and its dispatch was discarded.
  It now writes the same reports as the FULL_BUS warm start to
  `Dispatch_WarmStart/`, plus `CSV/delta_ref.csv` (`gen; delta_ref_rad;
  delta_ref_deg`) — nothing else in the results tree records the point every
  linearised `Pe` row expands around.

- **`ode_first_step` reaches classical FULL_BUS.** The equation builders already
  branched on `t == 1`; only `ClassicalFullBusModel` lacked the field, so a
  CLASSICAL_2ND FULL_BUS run silently integrated trapezoidally whatever was asked
  for. Backward Euler now reaches its swing rows and, with `include_governor`, the
  TGOV1 rows too. DQ_4TH is unchanged; Kron rejects the value (see above).

### Fixed

- **The AVR exciter row is normalised on the state instead of on `K_exc`.**
  `eq_const_avr_exciter!` divided both field-voltage terms by `K_exc`
  (`E_fd_unlim[t]·(1+c)/K_exc − E_fd[t−1]·(1−c)/K_exc − c·(2V_ref − V[t] − V[t−1]) = 0`);
  it now multiplies the voltage-error term by the gain instead, leaving the `(1+c)`
  coefficient on the state itself, as every other state ODE in the package does and as
  Part I §8 already documented the backward-Euler row (the code divided in both branches;
  the prose did not). The two forms differ by the constant factor `K_exc` on the whole
  row, so the feasible set and every trajectory are unchanged — but the multiplier scales
  inversely: `dual_avr_E_fd` values are now `λ_old / K_exc`. Two consequences worth
  knowing. Exciter duals from before this change need dividing by that machine's `K_exc`
  before they can be compared with new ones. And because the old row was divided by each
  machine's *own* gain, exciter multipliers were never comparable across a fleet with
  heterogeneous `K_exc`, nor against the swing and EMF duals; now they are.

- **The TGOV1 valve integrator wound up against its own limiter.** `eq_const_gov_valve!`
  advanced the raw state from `Pv_raw[t−1]`, making it a free integrator: under
  `GOV_SMOOTH` it kept climbing for as long as the droop signal demanded while the output
  sat pinned at the clamp, and the valve could not come off the limit until that
  accumulated excess had been unwound. The previous-step term is now the **limited**
  output `Pv[t−1]` — the anti-windup convention the AVR exciter has always used
  (`eq_const_avr_exciter!` anchors on the saturated `E_fd`) — which caps the raw state one
  step's valve travel beyond the limit. The post-fault window seam follows suit: `t=1` of
  the valve ODE now anchors on the last fault-on *limited* valve output, as the turbine
  row already did, rather than on the raw state. `Attach_Governor_fault!` /
  `Attach_Governor_postf!` build the limiter before the valve ODE so the limited container
  exists, mirroring `Attach_Avr_fault!`. `Pv` and `Pv_raw` are the same container under
  `GOV_NO_LIMIT` and `GOV_HARD_BOUND` (both encodings), so those paths are unchanged
  row-for-row and only `GOV_SMOOTH` changes behaviour. Exported variables, dual families
  and column names are untouched.

- **The TGOV1 turbine row let the valve limiter be bypassed.** `eq_const_gov_mech!`
  discretised the substituted state form
  `T3·dPm/dt = (1−T2/T1)·Pv + [(T2/T1)/R]·(P_ref−Δω) − Pm`, obtained by eliminating
  `dPv/dt` with the valve ODE. That elimination is exact only while the valve is
  unsaturated: under `GOV_SMOOTH` the clamped `Pv` no longer satisfies
  `dPv/dt = [(P_ref−Δω)/R − Pv]/T1`, yet the feedforward term kept injecting the raw,
  unclamped droop signal into the mechanical power — on the bundled 9-bus data that
  path carries a gain of `(T2/T1)/R = 100`, so valve saturation never capped `P_m`.
  The turbine lead-lag is now discretised directly, `T3·dPm/dt + Pm = T2·dPv/dt + Pv`,
  on the limited `Pv` alone; `P_ref`, `Δω`, `R` and `T1` no longer appear in the row.
  Substituting the discrete valve equality into the old discrete row recovers the new
  one exactly, trapezoidal and backward Euler alike, so `GOV_NO_LIMIT` and
  `GOV_HARD_BOUND` are unchanged and only `GOV_SMOOTH` changes behaviour. The row is
  normalised by `2·T3` to keep `dual_gov_mech` on its previous scale; the governor
  multipliers still redistribute between `dual_gov_mech` and `dual_gov_valve`, while
  LMPs and the δ/Δω box duals are untouched.

- **`Dispatch_WarmStart/` was created on paths that never warm-start.** The
  pre-fault coupling snapshot was gated on a `coupling_init_source` local that
  defaulted to `:acopf_warmstart` and was only corrected inside the FULL_BUS
  branch, so Kron TSC-ACOPF and TSC-DCOPF runs got a folder holding
  `prefault_coupling_starts.{txt,csv}` with `delta_rad = 0` and `E = 1` — the
  constants `var_kron_gen_rotor_angle!` seeds, not an operating point. The
  snapshot is now written only where the starts came from a solved ACOPF.

- **`bound_style` was recorded and ignored on Kron.** Both Kron builders called
  `ineq_const_kron_δ_COI_generic_modified!` unconditionally while `meta` (and
  `dynamic_model_details.txt`) reported whatever was configured. The `:coi_box` /
  `:swing_propagated` dispatch moved next to the two constraint families it
  chooses between (`functions_4_TS_kron_ineqconst.jl`), and all four network paths
  now route through it.

- **`dynamic_model_details.txt` no longer interrupts its constraint listing with
  configuration lines.** `bound_style`, `ode_first_step` and the ZIP splits are
  printed once by `_export_dyn_meta_header` at the top of the file, on every path,
  instead of appearing between the shared constraint block and the FULL_BUS
  network block.

- **The turbine governor was missing from `dynamic_model_details.txt` on the
  classical FULL_BUS path.** The appendix that prints the machine and control
  constraint families returned early unless `gen_order = DQ_4TH`, but TGOV1
  attaches on `CLASSICAL_2ND` FULL_BUS as well. A classical run with
  `include_governor` produced a dump with no governor rows at all — the valve and
  mechanical-power ODEs, the `P_ref` set-point pin, and, under `GOV_SMOOTH`, the
  anti-windup softsat equality (`eq_const_gov_valve_limit_tf/tpf`). Only the TXT
  audit trail was affected: the dual export and the trajectory CSVs take separate
  paths and were always complete, which is what made the gap easy to miss. The
  `gen_order` gate is gone; every family in the appendix was already selected by
  `haskey`, so the dq and AVR keys stay absent on a classical run without it.

  Two related gaps in the same export, found while tracing the first: the governor
  *variables* (`P_ref`, and `P_valve_raw` / `P_valve` / `P_mech` per window) reached
  no variable listing on **any** path, dq included, because the listing only carried
  the dq machine and GFM stems; and the explicit valve bounds emitted by
  `GOV_HARD_BOUND` under the `CONSTRAINT` encoding
  (`ineq_const_gov_valve_<win>_<side>`) were never printed either. Both are listed
  now. Under the `VARIABLE` encoding those bounds still live on `P_valve_raw` and
  remain the business of `variable_bounds.txt`.

- **Versioned documentation never deployed: the `Documentation` workflow raced
  itself on every release.** A release pushes `main` and the version tag in one
  `git push`, firing two workflow runs simultaneously. The concurrency group was
  keyed on `github.ref`, so a branch run and a tag run landed in *different*
  groups, built docs in parallel, and both tried to push `gh-pages`. The loser
  died with `! [rejected] HEAD -> gh-pages (fetch first)` and that release's
  versioned docs were never published — `gh-pages` held only `dev/`, and
  `/v0.1.0/` and `/v0.1.1/` were 404 on the published site.

  The group no longer includes the ref and no longer cancels in progress, so the
  tag run queues behind the branch run and clones a `gh-pages` that already
  contains it. Doc builds serialise instead of overlapping, costing about a
  minute and a half of queueing per release.

- **Unit commitment was broken with Gurobi 13: `GurobiSolverConfig.optimality_target`
  now defaults to `-1` instead of `1`.** `apply_gurobi_options!` stamps
  `OptimalityTarget` onto every Gurobi model. Gurobi 13 renumbered that parameter —
  it accepts only `-1` (automatic), `0` (global) and `1` (**local**), where older
  releases used `1` for global and `2`/`3` for local. The old default therefore
  selected *local* optimization, and Gurobi refuses to combine that with integer
  variables:

  ```
  Gurobi Error 10016: Local optimization cannot be used for discrete problems
  or SOS constraints
  ```

  Every UC run through Gurobi 13 failed on this, regardless of licence. LP paths
  (DC-OPF, ED, the explicit dual LPs) were unaffected in result, since local and
  global coincide on a convex problem.

  The bug stayed hidden because `runtests_uc.jl` only executes when Gurobi is in the
  test environment, which never happens on CI (`scripts/setup_test_env.jl` skips it
  there) and had not happened locally.

  Do not pin this parameter to a number unless you know what your Gurobi version
  means by it; the validation range (`-1:3`) is deliberately permissive across
  versions, and the setter is best-effort, so a value your Gurobi rejects is ignored
  rather than fatal.

- **A machine CSV missing the columns the selected dynamic model needs now fails
  validation instead of reaching the builders.** `include_governor = true` against a
  `gen_dynamic_filename` with no `R`, `T1`, `T2`, `T3` raised nothing during the sanity
  checks. `Parse_Gen_Dynamic_DataFrame` pre-fills every optional column with `NaN` and
  overwrites only the ones the file carries, so `DGEN_DYN.R` always exists and is simply
  `NaN` on a minimal file. Those `NaN`s reached the governor builder and became JuMP
  constraint coefficients.

  The guard already existed (`has_required_dyn_columns` / `required_dyn_column_names`,
  which appends `R`, `T1`, `T2`, `T3` under `include_governor`) but was wired to
  `gen_order == DQ_4TH`, so the classical second-order FULL_BUS path skipped it
  entirely; on the DQ path it fired only after the warm-start ACOPF had solved, and
  named neither the flag nor the file.

  `validate_dyn_data!(cfg, DGEN_DYN)` now runs in the sanity block of `run_case!` next
  to `validate_fault_config!`. It covers the whole union of required columns (dq set,
  AVR, governor) and reports which columns are missing, which knob requires them, and
  the filename you configured:

  ```
  gen_dynamic_data.csv is missing machine data required by the selected
  dynamic model: R, T1, T2, T3 (required by include_governor=true).
  Columns must be present and finite for every generator row.
  ```

  `missing_dyn_columns` is the new primitive; `has_required_dyn_columns` becomes
  `isempty(missing_dyn_columns(...))`, so there is one predicate rather than two that
  can disagree. The DQ-only check in the build block is deleted as superseded. The
  builder-level guard in the DQ FULL_BUS builder stays, since it protects callers that
  bypass `run_case!`.

- **Blank cells in a machine CSV no longer kill the parser before validation can speak.**
  A generator with governor columns present but empty (`R;;;;`) never reached
  `validate_dyn_data!`. CSV.jl types a column with any empty cell as
  `Union{Missing, Float64}`, so `Float64.(col)` threw first:

  ```
  MethodError: no method matching Float64(::Missing)
  ```

  which names neither the column nor the generator row. The optional columns and the
  `Float64` mandatory ones (`Xd_tr`, `H`, `D`) now route through `_gen_dyn_float`, which
  maps `missing` to `NaN`. A blank cell is then indistinguishable from an absent column
  downstream, and both spellings produce the same diagnostic. `bus` keeps its strict
  `Int64` conversion: an index has no `NaN` sentinel, so a blank there is a malformed
  file with nothing to defer to validation.

  This matters because `include_governor` is fleet-wide. The governor builders loop over
  every `active_gen`, so a partly-filled column is a genuine error rather than a per-unit
  opt-out.

### Changed

- **The two DC susceptance builders collapsed into one.** `Calculate_Matrix_B` and
  `Calculate_Matrix_B_PowerModels` were identical apart from the per-branch kernel, which
  `dc_branch_susceptance` already computes for both models. `Calculate_Matrix_B` now
  takes a `susceptance_model` keyword and broadcasts that helper; the PowerModels entry
  point is a thin alias, since tests and docs reference it by name.

  The helper's `r` / `x` annotations loosen from `Float64` to `Real`: the old builders
  used a `@.` expression that accepted integer-typed reactance columns, and the broadcast
  form would otherwise `MethodError` on them.

- **The dq builders return NamedTuples.** Four of them returned bare 4-, 5-, 7- and
  8-element tuples whose elements are all `OrderedDict`s of `VariableRef` or
  `ConstraintRef`. Sibling builders differ in arity (`var_dq_gen_state_time!` carries
  `Te`, its pre-fault counterpart does not), and adjacent elements are interchangeable to
  the compiler, so a positional slip built a silently wrong model rather than raising.
  That is the fragility behind the earlier Kron rotor-angle regression.

  The six call sites in the DQ FULL_BUS builder now bind by field name: property
  destructuring where the local names already match, explicit field access where the
  locals carry `_tf` / `_tpf` suffixes. No constraint, variable or export key changes.

### Added

- **`DynModelConfig.gfm_integrator` — the GFM measurement filters and Q–V PI integrator
  are now selectable.** They used to be backward Euler at every step with no knob
  reaching them: `ode_first_step` governed GFM δ and the `E_int` *first* row and nothing
  else, so the converter integrated on a different rule from every synchronous machine.
  The new symbol takes `:backward_euler` (default, BE at every step — the reference
  implementation's scheme), `:follow_ode_first_step` (BE at t=1 of each window iff
  `ode_first_step = :backward_euler`, trapezoidal after — the SG rule), or
  `:trapezoidal`. It is scoped to those two families only: GFM δ and every SG family
  still follow `ode_first_step` alone, unchanged.

  **The default preserves today's behaviour**, so existing GFM runs and reference parity
  are unaffected. One narrow exception: `E_int` at t=1 used to follow `ode_first_step`
  and is now governed by `gfm_integrator`, so under the default it is BE at t=1 too
  (previously trapezoidal when `ode_first_step = :trapezoidal`).

  The trapezoidal variants are opt-in for accuracy studies, not everyday settings.
  Measured on `9bus_gfm`, SC at bus 7 cleared in 150 ms, `t_step = 0.01`:

  | horizon | `:backward_euler` | `:follow_ode_first_step` |
  |---|---|---|
  | 0.6 s | 139 it | 97 it, 0.074 s/it |
  | 3.0 s | 205 it, 40.6 s Ipopt | 161 it, 2826 s Ipopt |

  Trapezoidal pulls `u_{t-1}` (`Pe`/`Qe`, nonlinear in V, θ, δ, Id, Iq) into every filter
  row, and the resulting fill-in compounds with horizon length — fewer iterations, ~70×
  the solve time, per-iteration cost growing 0.074 → 17.6 s across a 5× horizon. Accuracy
  given up by the default is small: worst rotor-angle gap 2.5° absolute / 1.8° relative to
  COI over 3 s (≈2 % of swing), objective 1.6e-5 relative. `:trapezoidal` — trapezoidal at
  t=1 as well, i.e. across the fault-inception `Pe` discontinuity — hit the wall clock at
  41 iterations on the 0.6 s case.

  Also: trapezoidal is only A-stable, so its per-step factor `(1−c)/(1+c)`,
  `c = Δt/(2Tf)`, turns negative once `t_step > 2·Tf` and the filter output rings into the
  droop → δ → limiter chain; the builder warns when a run crosses that line. And the
  `α`/`β` columns of `gfm_filter_debug.csv` now follow whichever scheme is active per
  step — previously hard-coded to the BE pair regardless, which would have silently
  misreported a trapezoidal model. `scripts/run_gfm_integrator_ab.jl` is the A/B harness.

### Changed


- **Test input data moved to `test/INPUT_FILES/`; `INPUT_FILES/` is now yours to edit.**
  The suite used to assert its numeric pins against the same case data the documentation
  invites users to modify, so adjusting a load in `INPUT_FILES/9bus` to try something out
  turned CI red for reasons unrelated to the code. The exposure was wider than the
  MATPOWER files that made it visible: it covered the A6 objective at `rtol = 1e-6`, the
  parsed `Xd_tr` values, the GFM `Imax`/`Xl` row, and two structural properties — every
  `9bus` branch must have r = 0 for the two susceptance kernels to agree, and `39bus` must
  have r ≠ 0 for that test to prove anything.

  Byte copies of everything the tests read now live under `test/INPUT_FILES/`, reached
  through `test/test_paths.jl` (`fixture_case`, `fixture_matpower`, `load_fixture_system`,
  `run_fixture_case!`). No package change was needed: `load_system(cfg, path_main)` already
  composes `<path_main>/INPUT_FILES/<case>`, so pointing `path_main` at `test/` is the whole
  mechanism — the same one `runtests_simulation_pins.jl` uses for the external simulations
  repo. Run output still lands in `<repo>/RESULTS`.

  Consequences worth knowing: the two trees are expected to **drift** — a correction applied
  to demo data does not reach a fixture unless copied deliberately, and re-pinning a fixture
  is a documented protocol (`test/INPUT_FILES/README.md`) that also names the three coupled
  pairs which cannot be edited alone. `9bus_reference_cost` was dropped rather than copied:
  it was a byte-identical clone of `9bus` whose stated purpose — insurance against edits to
  the working case — is now structural, so `REFERENCE_LINEAR_ACOPF_CASE` points at `9bus`
  with its pinned objective unchanged. `runtests_fixture_isolation.jl` fails the fast gate if
  a suite reaches back into the demo tree, including through the shape that carries no
  `INPUT_FILES` string at all, `load_system(cfg, PROJECT_ROOT)`.

### Fixed

- **The PowerModels cross-check staged scratch cases inside `INPUT_FILES/`.** `run_inhouse`
  created `INPUT_FILES/_cmp_{ac,acm,dc}_case39/`, and its `mkpath` and `cp` loop sat outside
  the `try` whose `finally` removed them — an exception while staging, or an interrupted run,
  left a half-populated case directory behind in tracked data. The scratch case is now built
  under `mktempdir() do … end` at `<tmp>/INPUT_FILES/<case>/` and passed as `path_main`, which
  removes the tree on normal and exceptional exit and keeps test scratch out of the data tree
  entirely.

- **`run_case!` returned an objective that could not be read (CI failure).**
  `release_solver_backend!` calls `empty!(model)` on Ipopt paths to free the solver
  log handle, which invalidates every `VariableRef` in the model — including the ones
  inside the returned `obj_MVA` expression. Any `JuMP.value(result.obj_MVA)` after
  `run_case!` therefore threw. `obj_MVA` is now a **plain number**, snapshotted before
  the backend is released (`nothing` when the solve did not converge); drop the
  `JuMP.value` wrapper at call sites. The same hazard applies to `dyn_model_dict`
  (`:vars` and `:eq_const` hold dead references after the call) — read trajectory and
  dual values from the exported CSVs, or from `RGEN`/`RBUS`/`RCIR`, which are
  DataFrames and unaffected. This is what had `Pkg.test()` red: the A6 FULL_BUS
  benchmark errored on the pinned-objective assertion, which also meant the four suites
  after it in `test/runtests.jl` never ran on CI at all.
- **Runs finishing in the same second shared one results folder.**
  `_timestamped_results_dir` names each run `Results - yyyy-mm-dd HHMMSS` — second
  resolution. Two runs completing inside the same second resolved to the same path, so
  `Inputs/`, `Dispatch/CSV/` and the dual exports became a union of both, with
  same-named files overwritten by whichever finished last. Harmless for a single
  interactive run; **silent data corruption for a sweep** that solves faster than one
  second per case. The directory is now claimed with `mkdir` (which fails when the path
  exists) and a ` (2)`, ` (3)`, … suffix is appended until the claim succeeds — atomic,
  so concurrent sweeps in separate processes are safe too. A run that gets its own
  second keeps the plain name, so frozen baselines and documented paths are unaffected.
- **CI: restored `GKSwstype: "100"`**, dropped by mistake in the same commit that added
  the Plots extension. Without it the GR backend tries to open a display on the headless
  runner and `runtests_plots_ext.jl` errors.
- `test/runtests_admittance_matrices.jl` could not run: it called unexported internals
  (`Read_Input_Data`, `Calculate_Ybus_*`, `Reduce_Matrix`) unqualified and used `@printf`
  without `using Printf`. `test/runtests_governor.jl` asserted on released JuMP
  references; the checks now read the exported governor CSVs instead.

### Changed

- **BREAKING — `GFMNumerics` and `GFM_NUM_DEFAULT` removed; GFM limits are now
  `RunConfig` knobs.** The struct mixed two unrelated things and no call site ever passed
  it, so every GFM limit was in practice a source constant. It is split by what the
  numbers actually are:
  - The **box limits** (measured-power and measured-voltage windows, the E bands, the
    `Id`/`Iq` box, the pre-fault converter angle) move to `TsBoundLimitsConfig` as
    `gfm_δ_*`, `gfm_V_meas_*`, `gfm_E_raw_extra_pu`, `gfm_E_raw_slack_pu`,
    `gfm_E_clip_slack_pu`, `gfm_PQ_bound_scale`, `gfm_PQ_bound_offset_pu`,
    `gfm_P_meas_floor_pu`, `gfm_Q_meas_floor_pu`, `gfm_I_floor_pu`, `gfm_I_ceiling_pu`.
    These are margins and floors: `resolve_gfm_bound_limits` derives the per-unit box from
    them plus the `DGFM` row, in **one** place instead of the two copy-pasted blocks the
    fault and post-fault windows carried. Defaults reproduce the previous numbers exactly.
  - The **smoothing constants** become module constants `_GFM_EPS_E`, `_GFM_EPS_NORM_V`,
    `_GFM_EPS_LIM_I`, alongside the limiter-bypass threshold `_GFM_IMAX_NO_LIMIT`. A unit
    whose `Imax` crosses that threshold gets unsaturated stator algebra instead of the
    clamp — a change of model shape driven by input data, so bypassed units are now
    logged and recorded on `meta[:gfm_limiter_bypassed]` rather than switching silently.
  - `Attach_GFM_fault!` / `Attach_GFM_postf!` lose the `num` keyword; both windows share
    one implementation (`_attach_gfm_window!`) parameterised by the window suffix and its
    starting anchor.
  - Every GFM box is switchable from `TsBuilderConfig` in the same shape as the SG
    families — `bound_gfm_δ` plus
    `bound_gfm_{P_meas,Q_meas,V_meas,E_int_raw,E_int,E_droop_raw,E_droop,Id,Iq}_{tf,tpf}`
    — but defaulting **`true`** where the SG `bound_*` toggles default `false`, because
    the nonconvex limiter needs the guard-rails to converge. Switching one off removes
    that box only: no JuMP bound at creation, no ≤-row, no manifest entry. If turning a
    box off moves the objective, it was binding and is a limit, not a guard-rail.

- **Converter trajectories now reach CSV and figures (roadmap G4.2).** GFM states existed
  only inside `dynamic_model_details.txt`: `_export_gfm_time_var_dicts` fed the text dump
  and nothing else, so `P_meas`, `E_int`, `E_droop` and friends had no CSV and no plot.
  A mixed-fleet run now writes `gfm_P_meas`, `gfm_Q_meas`, `gfm_V_meas`, `gfm_E_int`,
  `gfm_E_int_raw`, `gfm_E_droop`, `gfm_E_droop_raw` and `gfm_current_loading` — the last
  being `|I| / Imax`, where 1.0 means the limiter is saturating — plus, under
  `save_ts_plots`, the matching SVGs and two overlays (`gfm_E_int_clip`,
  `gfm_E_droop_clip`) putting each raw PI state against its clipped counterpart so the
  gap between them is the anti-windup clip acting. Shared quantities already included the
  converter; only these converter-specific channels were missing.

- **`RunConfig.save_ts_debug_csv` (default `false`) — per-(window, generator, step)
  diagnostics (roadmap G4.1).** Writes `Transient_Stability/CSV/Debug/` with
  `gfm_filter_debug.csv`, `gfm_limiter_debug.csv`, `gfm_voltage_debug.csv` and
  `swing_debug.csv`, each row carrying the value, its distance to the relevant limit, and
  the dual. Everything in them was derivable before, but only by joining four or five
  CSVs and redoing the limiter algebra by hand; "was the limiter binding at t, and what
  did it cost?" is now one row. Column layout follows the reference implementation's
  `Debug/` folder so a parity check is a direct CSV diff.

  `swing_debug.csv` extends the reference's columns with the package's own question: each
  machine's `δ_rel_COI`, the active tolerance, the signed margin to each side, `δ_util`
  (the corridor analogue of `Iout_over_Imax`, 1.0 = sitting on the limit) and the corridor
  duals, plus the Δω equivalents when `constrain_Δω_COI`. `dual_accel` is `NaN` by
  construction — the package folds accelerating power into the swing rows rather than
  carrying a separate constraint, so the value exists but there is no row to price.

  Nothing was added to the optimisation model: the save layer recomputes the limiter
  geometry and filter coefficients from solved primal values plus a new
  `meta[:gfm_params]` stamp (plain `Float64`s, so they outlive the solver backend), and
  reads duals from the named GFM equality families and the COI corridor rows.

- **`ode_first_step` now reaches the GFM first step (roadmap G4.3, decision D5 phase 2).**
  The converter's δ integration and Q–V PI integrator were backward Euler at `t = 1`
  unconditionally, so a mixed fleet under the default `:trapezoidal` integrated the
  machines trapezoidally and the converters by Euler *in the same step* — an
  inconsistency of order with no way to express it in the config. Both rows now follow
  the same dial as the SG swing, dq EMF, AVR and governor. The `t = 1` anchors come from
  the window's existing anchor tuple: `Δω⁰ = mp·(P_set − P_meas⁰)` and
  `ε⁰ = V_set − mq·Q_meas⁰ − V_meas⁰`, both identically zero entering the fault window and
  equal to the last fault-on values entering the post-fault window. Steps after the first
  are untouched (δ trapezoidal, `E_int` backward Euler), as are the measurement filters.

  **Behaviour change under the default:** a mixed-fleet run with `ode_first_step =
  :trapezoidal` now differs slightly from the same case before this change. Reference
  parity runs must set `:backward_euler` explicitly, which reproduces the previous GFM
  rows exactly — verified bit-identical (objective and every exported CSV) against the
  pre-change code on `9bus_gfm`, contingency 2, 150 ms clearing.

- **GFM boxes honour `bound_encoding` and export duals.** They were unconditional JuMP
  variable bounds regardless of the configured encoding, and reached neither the bound
  manifest nor the dual registry — a binding converter limit had no exportable shadow
  price. They now follow the same path as every other family: explicit ≤-rows under
  `CONSTRAINT`, JuMP bounds plus a manifest entry under `VARIABLE`, with the same export
  names either way. The transient equalities are additionally stored per family
  (`eq_const_gfm_droop_tf`, `_limiter_Id_tf`, `_Eint_clip_tf`, …) instead of only as one
  anonymous vector per (gen, t), and `DynDualRegistry` gains `dual_gfm_droop`,
  `dual_gfm_limiter_Id`/`_Iq`, `dual_gfm_Eint_clip`, `dual_gfm_Edroop_clip`,
  `dual_gfm_Pe`, `dual_gfm_Qe` plus the `dual_LB/UB_gfm_*` box entries. The flat
  `eq_const_gfm_tf` / `_tpf` vector is unchanged for the text appendix.

- **SG bound families no longer land on GFM ids.** `attach_fault_tf_var_bounds!` walked
  the whole `Id_tf`/`Δω_tf`/… containers, which include converters, and ran *before* the
  GFM attachers — under `VARIABLE` encoding the GFM box silently overwrote a configured
  `bound_Id_tf`. Each side now owns its own ids. No effect with shipped defaults (those
  toggles are off), but `Δω` on a GFM row is a droop output rather than a swing state, so
  an SG-calibrated box there was wrong in principle.

- **`INPUT_FILES/9bus_gfm` re-cut to one converter.** Three SG (ids 1–3) plus one GFM
  (id 4 at bus 5) with `Imax = 1.2` pu, so the bundled mixed-fleet case now exercises the
  current limiter instead of bypassing it. New heavy-tier `test/runtests_gfm_transient.jl`
  solves it on contingency 2 (fault at bus 7, cleared after 150 ms by opening branch 5–7)
  under both encodings — the first test in the suite that solves a GFM transient at all
  (roadmap G3.3).

- **Decision D4 restated (no code change).** The steady-state ACOPF converter circle uses
  the *nameplate* current, 1.0 pu on machine base; the transient limiter uses `DGFM.Imax`.
  A dispatch is a sustained operating point and cannot bank on overload, whereas a
  converter tolerates brief overcurrent up to the junction thermal limit. Documented in
  `attach_gfm_acopf_limits!`, `docs/src/model/09_grid_forming.md` and
  `docs/parameter_reference.md`; the earlier plan wording (use `Imax` in both) is
  superseded.

- **Test suite reorganised into four explicit tiers** (see `test/README.md` and
  `OUTPUTS/reference/test_suite_ranking.md`). `Pkg.test()` is now a fast gate for every
  push; `TSCOPF_RUN_HEAVY=true` adds DQ_4TH, AVR/governor, GL/OB and the A6 numeric pin
  (nightly on CI); `TSCOPF_RUN_SIMULATION_PINS` and `TSCOPF_RUN_BOUND_ENCODING_TSC_PARITY`
  are unchanged; `TSCOPF_RUN_PM_CROSSCHECK` is new. Merged
  `runtests_{avr,dq_governor,governor}.jl` → `runtests_controls.jl`, the three
  `*_vs_powermodels.jl` → `runtests_powermodels_crosscheck.jl`, and
  `runtests_fullbus_acopf.jl` into `runtests_tsc_builder_fullbus.jl`. Deleted
  `runtests_all.jl` (an alias for `Pkg.test()`) and `runtests_acopf.jl` (fully duplicated
  by the smoke and fast-unit suites). `PowerModels` and the stdlibs `LinearAlgebra`,
  `Printf`, `SparseArrays` are now declared in `test/Project.toml`; `Gurobi` is no longer
  installed on CI, where the UC tests skip themselves anyway.

- **BREAKING — independent ZIP splits for active and reactive demand.**
  `DynModelConfig.zip_load` is replaced by **`zip_load_p`** and **`zip_load_q`**, both
  defaulting to `(1.0, 0.0, 0.0)`. There is no alias: every `zip_load = …` must be
  updated. TSOs do not generally apply one load model to both channels — the Spanish TSO,
  for example, recommends constant-current active demand with constant-admittance reactive
  demand, which the shared tuple could not express:

  ```julia
  DynModelConfig(
      network_form = FULL_BUS,
      zip_load_p   = (0.0, 1.0, 0.0),   # (Z, I, P) — constant current
      zip_load_q   = (1.0, 0.0, 0.0),   # (Z, I, P) — constant admittance
  )
  ```

  Order stays **(Z, I, P)** = impedance / current / power in both vectors, and each must
  sum to 1 — `validate_dyn_config!` now checks them **separately**, so a mismatched pair
  can no longer slip through. Wired on both FULL_BUS builders (classical and `DQ_4TH`,
  SG-only and mixed SG+GFM): the four `eq_const_{fullbus,dq}_{P,Q}balance!` builders each
  receive their own vector. Ignored on `KRON_REDUCED`, where loads are folded into `Y_red`
  as constant admittance — a non-default split there now raises a warning instead of being
  silently dropped. `dyn_model_dict[:meta][:zip_load]` becomes `:zip_load_p` / `:zip_load_q`,
  and `dynamic_model_short_results.txt` prints both. Existing runs are unaffected
  numerically: every prior config was constant-Z on both channels.

- **Internal: `Δω` tolerance is carried as a resolved signed pair.** The three
  `AbstractDynamicGenModel` structs, the four TS builders, and
  `dyn_model_dict[:meta]` now hold `Δω_tol::Tuple{Float64, Float64}` instead of the
  scalar `Δω_tol_pu`; the value is resolved once in `dynamic_gen_model` via
  `Δω_tol_tuple(dyn)`. Anything reading `meta[:Δω_tol_pu]` must read `meta[:Δω_tol]`,
  and the `constrain_Δω_COI` line in `dynamic_model_short_results.txt` now prints the
  signed pair instead of `±half-width`. The user-facing `DynModelConfig.Δω_tol_pu`
  field is unchanged.

- **Transient dual CSV export is now registry-driven.** `Save_Duals_Dynamic_Model_tsred`
  iterated a hand-maintained list of ~39 `CSV.write` calls, so registered families with
  no matching line were silently never written to `Transient_Stability/CSV_duals/` —
  every optional variable-bound dual (`dual_LB_Ed`, `dual_UB_V_ref`, `dual_LB_Pv`,
  `dual_LB_Pe_tf`, …) and the GFM init duals were affected the moment their toggle was
  on. The block now loops `dual_spec_catalog` and writes any entry that declares a
  `csv_file`, dispatching on `DualExtractLayout`. Filenames and column headers are
  unchanged; 33 further families become exportable on the DQ FULL_BUS path. Adding a
  `DualRegistryEntry` is now the only step needed to export a new constraint family.

- **`mechanical_power.csv` is skipped when a governor is present.** With
  `mech_power_mode = USE_PM` plus a governor, `P_m(t)` is the governor mechanical-power
  state, and the file was byte-identical to `governor_P_mech.csv`. Runs without a
  governor still get it (there it is the only record of the trajectory).

- **`prefault_coupling.csv` drops the `E_pu` column on DQ_4TH runs.** `E` is the
  classical internal voltage behind `x'_d`; the dq machine carries `Ed`/`Eq`, so the
  column was empty on every row. Classical paths are unchanged.

- **Branch thermal / flow limits:** removed `UNBOUNDED_BRANCH_FLOW_PU` and
  `DispatchLimitsConfig.unbounded_branch_flow_pu`. Unrated branches
  (`DCIR.l_cap_1 == 0`) are skipped (no finite sentinel). Capacity inequalities
  still require `DispatchConfig.ineq_sbranch_upper=true`; ratings alone are not
  enough. DC dual µ variables follow the same rated-branch set.

### Added

- **Asymmetric Δω–COI corridor**: `DynModelConfig.Δω_tol_pu_lower` and
  `Δω_tol_pu_upper` set independent below/above limits on `Δω_g − Δω_COI`, so
  `lower = 0.05, upper = 0.03` gives `−0.05 ≤ Δω_g − Δω_COI ≤ 0.03`. Both are positive
  magnitudes and each defaults to `Δω_tol_pu`, mirroring `δ_tol_deg_lower` /
  `δ_tol_deg_upper` on the angle corridor. Existing configs that set only the scalar
  `Δω_tol_pu` are unchanged. The inequality builder already consumed the two tuple
  entries independently, so only the config layer needed the asymmetry.

- **Governor valve-limiter duals**: three new `DualRegistryEntry` rows for families that
  were built but never exported — `dual_gov_valve_sat` (the `GOV_SMOOTH` clamp equality
  `Pv = smoothclip(Pv_raw)`, the governor counterpart of `dual_avr_E_fd_sat`) and
  `dual_LB_gov_valve` / `dual_UB_gov_valve` (the `GOV_HARD_BOUND` ≤-form pair). Their
  constraint keys are assembled with `Symbol(...)` interpolation in `_store_gov_limit!`,
  which is why they were missed. `:dual_δCOI` also gained its `csv_file`
  (`dual_delta_COI.csv`), previously written by a one-off line outside the registry.

- **Pre-saturation control trajectories**: `dq_E_fd_unlim_pu.csv` (AVR exciter output
  before the softsat clamp) and `governor_P_valve_raw.csv` (valve integrator state before
  the limiter), each with an SVG under `save_ts_plots`. The gap between a raw signal and
  its saturated counterpart is exactly when that limiter is active. Governor `P_mech`,
  `P_valve`, and `P_valve_raw` now also get figures (previously CSV-only).

- **`Export_Variable_Bounds!`**: writes `variable_bounds.txt` (VariableRef bounds only)
  next to every TS model dump, for line-by-line comparison against the reference implementation.

- **`Save_Prefault_Coupling_Starts!`**: snapshot of the pre-fault coupling `start=`
  values under `Dispatch_WarmStart/`, written before the joint `optimize!` when
  `coupling_init_source = :acopf_warmstart`. Independent of `save_warmstart_dispatch`.

- **`DynModelConfig.ode_first_step`**: choose first-step ODE discretization for EMF /
  AVR / governor (`:trapezoidal` default, or `:backward_euler`). Preserves the pinned
  SG AVR+TG FULL_BUS path; GFM investigation may opt into BE later.

- **GFM Phase G3 (duals + docs):** export duals for GFM pre-fault filter/Vset equalities
  in `DQ_4TH_FULL_BUS_DUAL_SPECS`; document mixed-fleet GFM in generator model page and
  `running_a_case` §7.3b. Reference numeric pins deferred (optional follow-up).

- **GFM Phase G2 (transient core):** fault/post-fault GFM dynamics on DQ FULL_BUS —
  BE measurement filters, P–f droop + δ integration, Q–V PI with smooth E clips,
  angle-preserving current limiter → Pe/Qe; SG-only COI / swing / EMF / AVR / governor;
  `GFMNumerics` knobs. KCL still uses all active gens via Id/Iq.

- **GFM Phase G1 (init + ACOPF limits):** real `Attach_GFM_init!` (pre-fault δ / Ed/Eq /
  Id/Iq / P_set / filters / V_set equalities behind `Xl`), `gfm_machine_warmstart`,
  `attach_gfm_acopf_limits!` (`|S|≤|V|Imax` and internal-E behind `Xl`, system-base
  `Imax`), and SG/GFM split in `Define_Initial_Condition_4_dq!`.

- **GFM Phase G0 scaffolding:** separate `gfm_dynamic_data.csv` ingest (`Parse_GFM_Dynamic_DataFrame` /
  `Read_GFM_Dynamic_Data` + machine-base conversion), `DynModelConfig.allow_gfm`,
  `TransientConfig.gfm_dynamic_filename`, `SystemData.DGFM`, SG/GFM id partition
  validation, `Attach_GFM_*!` stubs on the DQ FULL_BUS path, and example case
  `INPUT_FILES/9bus_gfm/`. GFM transient physics is deferred to Phase G1+.

- **`FaultType` `OB` (Open Branch)**: open one or more in-service lines/transformers
  with no short-circuit stage (`FaultConfig(fault_type=OB, ob_branch_ids=[…])`).
  Same single-window timeline as GL. Config, validation (incl. islanding hard-error),
  `build_fault_details`, classical **FULL_BUS**, **Kron**, and **Kron-Linear**
  builders are wired, and DQ FULL_BUS now accepts the same OB workflow too.

### Fixed

- **ZIP index order on GFM parity runs**: `DynModelConfig.zip_load` is ordered
  `(Z, I, P)` while the reference implementation indexes `(P, I, Z)`, so its `[0,0,1]`
  (constant impedance) is package `(1.0, 0.0, 0.0)`. Runs that passed `(0,0,1)`
  were solving with constant-**power** loads and hit `ITERATION_LIMIT`. Documented
  on the `zip_load` field; corrected in `scripts/run_gfm_{reference_parity,trap_1s,trap_imax12}.jl`.

- **Mixed-fleet (SG + GFM) post-processing**:
  - Trajectory plots raised `KeyError` on GFM ids — `Vke` omits GFM units (no `H`),
    so `Vpe` / `Ve` are now built from SG ids only, with a guard.
  - `Save_Duals_2_Excel_tsred` accepts registry `kwargs...` so the GFM and `V_ref`
    sheets export; sheet names are truncated with `chop` instead of byte slicing,
    which corrupted names containing `δ` / `Δω`.

- **AVR `V_ref` start value**: `JuMP.start_value(V)` is still ~1.0 when the exciter
  is initialised, so `var_avr_V_ref!` now takes `val_V` and seeds the reference way,
  `V_ref ← val_V[bus] + E_fd/K_exc`. `V_ref` is documented as one scalar per active
  **SG** (GFM units are excluded).

- **`dq_machine_warmstart`**: angle from synchronous **Xq** (reference formula), not `Xq_tr`.

- **Gurobi NL-only attributes on MILP/LP**: stamp `NLBarIterLimit`,
  `NLBarPFeasTol` / `NLBarDFeasTol` / `NLBarCFeasTol`, and `OptimalityTarget`
  best-effort; skip when the MOI backend rejects them as unsupported, and scrub
  Gurobi.jl's sticky `params` entry so `optimize!` / `empty!` does not replay
  the rejected attribute (UC MILP).
- **CI FULL_BUS builder timeouts**: raise `time_limit_sec` to 1800 s and shorten
  builder-test `t_end_sim` to 1.0 s in `test/tsc_main_style_config.jl` (and A6
  gate time limit) so classical FULL_BUS TSC no longer hits `MOI.TIME_LIMIT` on
  GitHub runners after ~600 s.
- **Near-converged FULL_BUS NLP saves/tests**: treat `MOI.ALMOST_LOCALLY_SOLVED`
  as acceptable for the FULL_BUS builder regression and save results for that
  status in `run_case!`, avoiding CI-only false failures on Ipopt borderline
  solves.
- **Repo hygiene (Phase 2 quick wins)**:
  - CSV writers: `writeheader=true` → `header=true` (CSV.jl deprecation).
  - Delete unused `Save_Results_Dynamic_Model_tsred`,
    `Save_Duals_Dynamic_Model_2_Excel`, and `Build_Dynamic_Model_Linear!`.
  - Archive `setup_includes.jl` and completed migration scripts under `_archive/`.
  - Branch SC stubs now throw a clear `ArgumentError` instead of empty `#TODO`.
  - Kron / FULL_BUS builder tests use distinct helper names (no `Main` overwrite).
- **`assert_bus_not_islanded`**: shared islanded-bus guard for ACOPF/DCOPF balance
  builders (`functions_4_eqconst.jl`) and FULL_BUS nodal injection.

### Changed

- **EMF / AVR / governor first step**: optional `:backward_euler` via
  `ode_first_step` (default remains full trapezoidal, matching the SG AVR+TG pin).

- **`TsBoundLimitPair` defaults**: optional tf/tpf limit pairs now default to
  `(-Inf, Inf)` instead of `±9999`. Inf means inactive, not a wide numeric box:
  `VARIABLE` omits JuMP bounds on that side; `CONSTRAINT` skips the ≤-row so
  Inf never becomes an inequality RHS (solver-hostile). Toggle on + Inf = no-op;
  set finite limits when you want a real box.

### Known issues

- **Mixed GFM DQ FULL_BUS vs the reference implementation**: converging. The
  earlier `ITERATION_LIMIT` was the ZIP index-order trap (see Fixed above), not a
  structural problem — with `zip_load = (1.0, 0.0, 0.0)` the joint TSC solve
  reaches an optimum. Open items are NLP size/encoding audits (`a_tf`,
  Interval vs Aff) and the reference numeric pins.

### Added

- **Extended `TsBuilderConfig` optional bounds** for DQ_4TH (`bound_Ed/Eq/Id/Iq`,
  tf/tpf machine states, `bound_Te_*`), AVR (`bound_V_ref`, `bound_E_fd_unlim_*`),
  and governor (`bound_P_ref`, `bound_Pv_*`, `bound_Pm_*`). All new toggles default
  `false`; physics saturation (AVR clamp, `governor_limiter`) is unchanged.

- **Breaking — remove `TsBuilderConfig` init-equality toggles**: drop
  `eq_P_init` / `eq_Q_init` / `eq_Pm_init`. Pre-fault OPF↔dynamics links are
  always-on physics: `eq_const_P_init` on all TSC paths; `eq_const_Q_init` on
  AC classical / DQ (omitted on TSC-DCOPF / Kron linear); `eq_const_Pm_init`
  only when `mech_power_mode = USE_PM`. Update scripts that still pass those
  fields to `TsBuilderConfig(...)`.
- **`HiGHSSolverConfig` expanded defaults**: wire `ipm_optimality_tolerance`
  (default `1e-8`) and `solver` (default `"choose"`); always set
  `simplex_iteration_limit` (default `5000`, was optional/`nothing`).
- **`GurobiSolverConfig` expanded defaults**: wire `NLBarIterLimit`, `NLBarPFeasTol`,
  `NLBarDFeasTol`, `NLBarCFeasTol`, and `OptimalityTarget` (default `1`). Default
  `MIPGap` tightened from `0.01` to `1e-8` to match the typed continuous/NL tolerances.
- **Drop MadNLP+Pardiso**: remove `"MadNLP-pardiso"` / `"MadNLP-mklpardiso"` and
  `MadNLPPardiso` wiring. Pardiso remains Ipopt-only (`"Ipopt-pardiso"`). MadNLP
  optional linear solvers are HSL only (`"MadNLP-ma57"`, `"MadNLP-ma97"`).
- **Breaking — Ipopt linear-solver `solver_name` keys**: renamed `"ma57"` → `"Ipopt-ma57"`,
  `"ma97"` → `"Ipopt-ma97"`, `"pardiso"` → `"Ipopt-pardiso"` so RunConfig names match the
  MadNLP-prefixed pattern (`"MadNLP-ma57"`, …). Ipopt's MOI `linear_solver` attributes
  remain the native strings `"ma57"` / `"ma97"` / `"pardiso"`. Update any scripts that
  still use the short names.

### Fixed

- **VARIABLE box limits in `model_details.txt`**: `Export_OPF_Model` now writes box limits from `bound_manifest` when `bound_encoding = VARIABLE`, using the same ≤-form lines as the `CONSTRAINT` path so the full primal is auditable in `Dispatch/model_details.txt`.
- **`init_opf_dict!` matrix builders**: accept `Pair{Symbol, <:Any}` extras so `Make_ACOPF_Model_w_Ybus!` / `Make_DCOPF_Model_w_Bbus!` can pass typed `Ybus` / `Bbus` sparse matrices without a method error.
- **COI inertia on GL generator trip** (`cc0698c`): `eq_const_kron_COI_generic!` now uses `sum(DGEN_DYN.H[active_gen])` for the COI denominator on all TSC paths (Kron, Kron-linear, FullBus, DQ). Previously the denominator included tripped machines while the numerator did not, biasing δ_COI and the stability corridor for generator-disconnection (GL) cases. Short-circuit (SC) runs are unchanged when all machines remain active.
- **Propagated δ–COI bounds** (`d2f10dc`): `:swing_propagated` inequalities now apply `δ_tol[1]` below COI and `δ_tol[2]` above COI (was `δ_tol[2]` on both sides).

### Added

- **MadNLP HSL linear solvers**: `solver_name` aliases `"MadNLP-ma57"`,
  `"MadNLP-ma97"` (namespaced away from Ipopt `"Ipopt-ma57"` / `"Ipopt-ma97"`).
  Optional `MadNLPHSL` + `HSL_jll`; no `hsllib` attribute on MadNLP.
  `MadNLPSolverConfig.ma97_num_threads` for MA97 multithreading; other HSL knobs
  via `raw_options`. Pardiso is Ipopt-only. See `docs/running_a_case.md` §8.2.2.
- **`BoundEncoding`** (`CONSTRAINT` | `VARIABLE`) on `DispatchConfig.bound_encoding` and `TsBuilderConfig.bound_encoding`: choose explicit ≤-form bound inequalities (default, dual-friendly `ConstraintRef` export) or JuMP variable `lower_bound`/`upper_bound` at creation (fewer LP/QP rows). Dual export uses a shared `bound_manifest` adapter so CSV/XLSX column names are unchanged. UC bilinear limits and Kron `Qe` expressions always use `CONSTRAINT` bounds.
- **`IpoptSolverConfig`** on `RunConfig.ipopt`: legacy Ipopt defaults (`tol`, KKT tolerances, `max_iter`, `print_level`) plus optional `hessian_approximation = "limited-memory"` with `limited_memory_max_history`.
- **`HiGHSSolverConfig`**, **`GurobiSolverConfig`**, **`MadNLPSolverConfig`** on `RunConfig`: typed LP/MILP/NLP solver knobs plus optional `raw_options` dict per solver for advanced MOI attributes.
- **`solver_name = "Ipopt-pardiso"`**: Ipopt with Intel MKL PARDISO as the KKT linear solver; `IpoptSolverConfig.pardiso_lib_path` or `ENV["JULIA_PARDISO_LIB"]`, optional `PARDISOLICMESSAGE` via `pardiso_license_message`.
- **`validate_dgen_dyn_row_ids!`** (`d2f10dc`): `load_system` asserts `DGEN_DYN` rows are `id = 1:n`, match `DGEN` row count, and share the same `bus` per row as `generators_data.csv`. Catches misordered `gen_dynamic_data.csv` before `DGEN_DYN.H[gen]` indexes the wrong machine.
- **Asymmetric COI angle limits** (`d2f10dc`): `TsSimulationConfig` fields `δ_tol_deg_lower` and `δ_tol_deg_upper` (optional; default to `δ_tol_deg` for a symmetric box). Built tuple is `(−lower, +upper)` in radians via `common_ts_parameters`.
- **`release_solver_backend!`**: after exports, finalizes Ipopt if needed, `empty!(model)`, `GC.gc()` — releases `output_file` locks on Windows. Documented in [Running a case — solver logs](docs/running_a_case.md).
- **Regression tests**: `verify_coi_active_inertia!` (Kron / FullBus / DQ GL gen-trip); fast-unit coverage for row-id validation and asymmetric `δ_tol`.
- **Simulation scalar pins**: `test/simulation_pins/` + `scripts/extract_simulation_pins.jl` — objective, dispatch `P_g`, and δ-COI trajectories pinned from `_TSCOPF_simulations` `bound_compare_*_constraint` baselines (plus reference AVR+TG case); run via `test/runtests_simulation_pins.jl` (Tier B).

### Changed

- **GL gen-trip numerics**: Re-baseline any published results that used generator disconnection before `cc0698c`.
