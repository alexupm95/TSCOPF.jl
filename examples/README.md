# `examples/` — one runnable script per simulation avenue

Each file is a **complete, standalone reference** for exactly one avenue: every
field of every config struct that avenue constructs is written out and commented,
with no shared helper file and no `include`. Copy one, change the values you care
about, run it. The duplication between files is deliberate — it is what lets a
single file answer "what can I set here?" without cross-referencing.

Run any of them from the repository root:

```bash
julia --project=. examples/03_acopf.jl
```

Scripts read from `INPUT_FILES/<case>/` and write to `RESULTS/Results - <timestamp>/`,
resolved through the exported `project_root()` / `default_results_dir()` helpers.

## The scripts

| # | File | Avenue | Case | Extra input files | Solver |
|---|---|---|---|---|---|
| 01 | `01_ed.jl` | Economic dispatch | 9bus | — | HiGHS |
| 02 | `02_dcopf.jl` | DC-OPF | 9bus | — | HiGHS |
| 03 | `03_acopf.jl` | AC-OPF | 9bus | — | Ipopt |
| 04 | `04_tsc_dcopf_kron.jl` | TSC-DCOPF, Kron, 2nd order | 9bus | `gen_dynamic_data.csv`, `contingencies.csv` | HiGHS |
| 05 | `05_tsc_acopf_kron.jl` | TSC-ACOPF, Kron, 2nd order | 9bus | `gen_dynamic_data.csv`, `contingencies.csv` | Ipopt |
| 06 | `06_tsc_acopf_fullbus_sg2nd.jl` | TSC-ACOPF, FULL_BUS, 2nd order | 9bus | `gen_dynamic_data.csv`, `contingencies.csv` | Ipopt |
| 07 | `07_tsc_acopf_fullbus_sg2nd_tg.jl` | + turbine governor | 9bus | `gen_dynamic_data_full.csv`, `contingencies.csv` | Ipopt |
| 08 | `08_tsc_acopf_fullbus_sg4th.jl` | TSC-ACOPF, FULL_BUS, 4th order | 9bus | `gen_dynamic_data_full.csv`, `contingencies.csv` | Ipopt |
| 09 | `09_tsc_acopf_fullbus_sg4th_avr.jl` | + AVR | 9bus | `gen_dynamic_data_full.csv`, `contingencies.csv` | Ipopt |
| 10 | `10_tsc_acopf_fullbus_sg4th_tg.jl` | + turbine governor | 9bus | `gen_dynamic_data_full.csv`, `contingencies.csv` | Ipopt |
| 11 | `11_tsc_acopf_fullbus_sg4th_avr_tg.jl` | + AVR + governor | 9bus | `gen_dynamic_data_full.csv`, `contingencies.csv` | Ipopt |
| 12 | `12_tsc_acopf_fullbus_sg4th_avr_tg_gfm.jl` | + grid-forming converters | **9bus_gfm** | `gen_dynamic_data_full.csv`, `gfm_dynamic_data.csv`, `contingencies.csv` | Ipopt |
| 13 | `13_tsc_acopf_fullbus_sg4th_gl_gentrip.jl` | GL disturbance (generator trip) | 9bus | `gen_dynamic_data_full.csv`, `contingencies.csv` | Ipopt |
| 14 | `14_tsc_acopf_kron_ob_openbranch.jl` | OB disturbance (open branch) | 9bus | `gen_dynamic_data.csv`, `contingencies.csv` | Ipopt |

## Parameter sweeps

Multi-scenario scripts. Each writes one timestamped `RESULTS/` folder per scenario plus a single
summary CSV, and prints a table with the objective spread across the range — a flat spread means
the swept limit never bound, which is a result about the case, not a failure.

| # | File | Swept quantity | Range | Scenarios | Base model |
|---|---|---|---|---|---|
| 15 | `15_sweep_delta_tol_kron.jl` | `δ_tol_deg` — δ–COI corridor half-width | 90° … 100°, step 1° | 11 | example 05 (Kron, 2nd order) |
| 16 | `16_sweep_gfm_imax.jl` | GFM `Imax` — converter current limit | 1.1 … 1.3 pu, step 0.05 | 5 | example 12 (FULL_BUS, 4th order, AVR + TG + GFM) |

The two sweeps differ in *where* the swept quantity lives, and the scripts are shaped accordingly:

- **δ_tol is a config field**, so 15 varies it through `reconfigure_transient`. It rebuilds the
  whole `TsSimulationConfig` each iteration rather than passing one field, because
  `reconfigure_transient(tc; simulation = TsSimulationConfig(δ_tol_deg = x))` replaces the entire
  block and silently reverts every field you did not restate — which would sweep two things at once.
- **`Imax` is case data**, a column of `gfm_dynamic_data.csv` with no config field behind it, so 16
  reloads the system each iteration and writes `sys.DGFM.Imax`. It affects the transient current
  limiter only: no steady-state constraint reads that column, so the dispatch feasible set is
  identical across scenarios.

Unit commitment is not covered here — it needs a Gurobi licence and a different
result path. See `test/runtests_uc.jl`.

`contingencies.csv` must be present for **every** `trans_stab = true` run, including
the GL and OB examples that never read it: the input-archival step copies it
unconditionally.

## Which combinations are illegal

These are enforced at runtime and throw rather than silently degrading. Sources in
`src/engine.jl` (`validate_run_config!`, `validate_dyn_config!`) and
`src/_common/functions_4_sanity_checks.jl` (`Check_Coherence_Input_Data`).

| Rule | Consequence |
|---|---|
| `include_avr` ⇒ `gen_order = DQ_4TH` | No "2nd order + AVR" exists. A constant-EMF machine has no field winding. |
| `allow_gfm` ⇒ `FULL_BUS` **and** `DQ_4TH` **and** not DC-OPF | No "2nd order + GFM" exists either. |
| `include_governor` ⇒ `USE_PM` **and** `FULL_BUS` | The governor is legal on a 2nd-order machine (07), but not on Kron. |
| `gen_order = DQ_4TH` ⇒ `FULL_BUS`, `USE_PM`, `bound_style = :coi_box` | The 4th-order machine has one legal network/mech/bound shape. |
| `FULL_BUS` ⇒ `USE_PM`, hence `:coi_box` | Checked before any solve; the builder repeats it as a backstop. |
| `ode_first_step = :backward_euler` ⇒ `FULL_BUS` | The Kron swing rows are trapezoidal at every step, so Kron rejects it. |
| TSC-DCOPF forbids `DQ_4TH`, `FULL_BUS` and `allow_gfm` | Kron + 2nd order is the only linearised shape. |
| ACOPF and TSC-ACOPF forbid HiGHS and Gurobi | Nonconvex; needs Ipopt or MadNLP. |
| `trans_stab` ⇒ `type_model ∈ ("ACOPF", "DCOPF")` | ED and UC have no transient path. |
| `trans_stab = false` ⇒ `transient = nothing` | And `true` ⇒ `transient` set. |
| `solve_explicit_dual` ⇒ ED or DC-OPF **and** `cost_type = "linear"` (DC-OPF also `use_matrix = true`) | Quadratic primals use `save_duals` instead. |
| `save_warmstart_dispatch` ⇒ a TSC run with a steady-state pre-solve | FULL_BUS TSC-ACOPF (06–13) or TSC-DCOPF (04). Rejected on Kron TSC-ACOPF. |
| `save_ts_plots = true` ⇒ Plots loaded via `load_plots_extension!()` | Left `false` in every example. |

## Conventions used throughout

- **Optional boxes are off.** Every `bound_*` toggle for an optional box is `false`
  with its limit values written out above it. Switching one on adds a ≤-row per
  unit and can make the problem infeasible if the optimum sits outside — do it
  deliberately and watch the objective. The exception is the GFM family in
  example 12, which defaults to `true` because the nonconvex current limiter
  needs those guard-rails to converge.
- **`OPEN = TsBoundLimitPair(min = -Inf, max = Inf)`** marks an inactive pair. A box
  is built only when its toggle is on *and* the limit is finite.
- **`TSCOPF.CONSTRAINT`** needs the module prefix: `BoundEncoding` is exported but
  its values are not. `CONSTRAINT` builds explicit ≤-rows so `JuMP.dual()` is clean;
  `VARIABLE` uses JuMP bounds and normalises the duals through the bound manifest.
  Same optimum either way.
- **`zip_load_p` / `zip_load_q`** are `(Z, I, P)` and must each sum to 1. The
  reference implementation orders the vector `(P, I, Z)`, so its `[0,0,1]` is our
  `(1.0, 0.0, 0.0)`.
- **`result.obj_MVA` is a plain number**, not a JuMP expression. After `run_case!`
  returns, every `VariableRef` in `result.dyn_model_dict` is dead — read
  trajectories and duals from the exported CSVs under `result.path_names`, or from
  `result.RGEN` / `RBUS` / `RCIR`.
- Each script omits the knob families its avenue cannot read, and its header names
  the example that does show them.

Field-by-field reference: `docs/parameter_reference.md`, `docs/configuration_map.md`.
Model equations: Part I of the docs site (`docs/src/model/`).
