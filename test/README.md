# Test suite

Four tiers, selected by environment variable. All paths assume the project root as
the working directory. Full value ranking and audit:
[`OUTPUTS/reference/test_suite_ranking.md`](../OUTPUTS/reference/test_suite_ranking.md).

```julia
julia --project=. -e 'using Pkg; Pkg.test()'                       # fast gate
TSCOPF_RUN_HEAVY=true julia --project=. -e 'using Pkg; Pkg.test()' # + heavy tier
```

Most individual files also run standalone:

```bash
julia --project=. test/runtests_dq_fullbus.jl
```

Four do not — `runtests_fast_unit.jl`, `runtests_smoke.jl`, `runtests_tsc_builder_kron.jl`
and `runtests_tsc_builder_fullbus.jl` include no helper and define no `PROJECT_ROOT`, so
they only work inside `runtests.jl`.

## Test input data

The suite reads case data **only** from `test/INPUT_FILES/`, never from the repository's
`INPUT_FILES/`. Those fixtures are frozen: numeric pins are asserted against them, so
editing one is a deliberate re-pin with a protocol, documented in
[`test/INPUT_FILES/README.md`](INPUT_FILES/README.md). The demo tree at `INPUT_FILES/` is
the user's to edit and no longer affects whether the suite passes.

Fixtures are reached through `test_paths.jl` — `fixture_case("9bus")`,
`fixture_matpower("case39.m")`, `load_fixture_system(cfg)`, `run_fixture_case!(cfg, sys)`.
Run output still goes to `<repo>/RESULTS`. `runtests_fixture_isolation.jl` fails the fast
gate if a suite reaches back into the demo tree, including via the shape that carries no
`INPUT_FILES` string at all, `load_system(cfg, PROJECT_ROOT)`.

The two suites that read the external `_TSCOPF_simulations` repo
(`runtests_simulation_pins.jl`, `runtests_bound_encoding_tsc.jl`) are exempt.

> **Reading results after `run_case!`.** `release_solver_backend!` calls
> `empty!(model)` on Ipopt paths, so every `VariableRef` and `ConstraintRef` in the
> returned `dyn_model_dict` is **dead** once `run_case!` returns. `result.obj_MVA`
> is a plain number, not an expression — never wrap it in `JuMP.value`. For
> trajectory or dual numbers, read the exported CSVs under `result.path_names`, or
> use `result.RGEN` / `RBUS` / `RCIR` (DataFrames, unaffected). Ignoring this is
> what broke CI: `runtests_a6_fullbus_benchmark.jl` and `runtests_governor.jl` both
> queried released references.

---

## Tier 1 — fast gate (`Pkg.test()`, target ≤ 12 min)

Runs on every push and PR.

| File | What it checks | Solves |
|------|----------------|--------|
| `runtests_fast_unit.jl` | CSV parsers, `DynModelConfig`, factory, limits config, Pacc_COI inertia, all four solver-config structs, HSL/PARDISO wiring, backend release | none |
| `runtests_gfm_g0.jl` | GFM data parsing, id partition, warm-start map, `Attach_GFM_init!`, ACOPF nameplate/`Emax` limits, `resolve_gfm_bound_limits` defaults and knobs, `gfm_integrator` three-scheme structural probe | none |
| `runtests_bound_encoding.jl` | CONSTRAINT vs VARIABLE bound duals agree on a 2-variable LP; `model_details` export | tiny LP |
| `runtests_admittance_matrices.jl` | Ybus/Bbus/Kron vs independent dense references; fault/post-fault augmentation | none |
| `runtests_matpower_parser.jl` | `.m` round-trip, case39 values, gencost → quadratic mapping | none |
| `runtests_results_folders.jl` | `RESULTS/` tree shape per simulation type | 1 ED |
| `runtests_optim_matrices.jl` | Sparse COO export + save policy | 1 ACOPF |
| `runtests_dispatch_duals_registry.jl` | Steady-state dual export families and headers | ED + ACOPF |
| `runtests_explicit_dual.jl` | ED and DC-OPF explicit dual LPs vs primal duals | LPs |
| `runtests_susceptance_model.jl` | SIMPLE ≡ POWERMODELS at r=0; differ at r≠0 | DCOPFs |
| `runtests_matpower_input.jl` | `.m` ingestion == CSV ingestion (schema, bus remap, dispatch) | 2 ACOPF |
| `runtests_smoke.jl` | ED / ACOPF / DCOPF / TSC-ACOPF / TSC-DCOPF end-to-end; Kron `Qe_tf`; AC-OPF reference parity | 6 |
| `runtests_tsc_builder_kron.jl` | Kron builder: mandatory equalities, δ-COI toggles, GL trip, OB open branch | Kron |
| `runtests_tsc_builder_fullbus.jl` | ZIP factory checks + **one** FULL_BUS solve (structure, CSV exports, dual registry) | 1 FULL_BUS |
| `runtests_controls.jl` | AVR/governor validation, factory, valve limiters, field clamp | none in this tier |
| `runtests_plots_ext.jl` | `TSCOPFPlotsExt` load + SVG export, machine-referenced δ figure, corridor dual SVGs per family (needs `GKSwstype=100` on headless runners) | none |
| `runtests_uc.jl` | UC MILP + restricted-pricing duals — self-skips without a Gurobi licence | MILP |

## Tier 2 — heavy (`TSCOPF_RUN_HEAVY=true`)

Nightly on CI (`schedule`) and on `workflow_dispatch`. Adds one FULL_BUS/DQ Ipopt
solve per testset.

| File | What it adds |
|------|--------------|
| `runtests_delta_reference.jl` | Every δ / Δω corridor style on Kron 9-bus: `:highest_H` and `:ref_gen` reference resolution, n−1 row count, δ_COI as an expression, `:abs` speed box, `validate_δ_reference!`, and the reference-relative exports |
| `runtests_dq_fullbus.jl` | DQ_4TH FULL_BUS: SC, GL gen trip (COI over survivors), OB open branch, optional Ed bounds |
| `runtests_gfm_transient.jl` | Mixed SG + GFM solve on `9bus_gfm` (contingency 2, 150 ms clearing, limiter active): named GFM equality families, GFM boxes in both encodings, dual-registry entries, CONSTRAINT ≡ VARIABLE objective |
| `runtests_controls.jl` | 5 end-to-end solves: classical+TGOV1, DQ+TGOV1, DQ+AVR, DQ+AVR+TGOV1, and the flat-start coupling path |
| `runtests_tsc_builder_fullbus.jl` | warm-start dispatch save, δ-COI / δ / Qe bound toggles, asymmetric P/Q ZIP split |
| `runtests_fullbus_gld.jl` | GL gen and load trips, OB open branch, and the no-mutation guard on the input DataFrames |
| `runtests_a6_fullbus_benchmark.jl` | Numeric acceptance gate vs pinned `main.jl` scalars (objective, P_g, δ–COI trajectory) |

## Tier 3 — external inputs

Need data that is not in this repo.

**`TSCOPF_RUN_SIMULATION_PINS=true`** — `runtests_simulation_pins.jl` re-solves the
`bound_compare_*_constraint` suites and compares against reference-validated scalars in
`test/simulation_pins/pins_*.jl`. Ten suites: 4th AVR, 4th core, 2nd FULL_BUS, 2nd
Kron, GL gen3 (4th FULL_BUS + Kron), AVR+TG, TG-only, ACOPF dispatch, DCOPF dispatch.

Requires the variant case CSVs from the simulations repo — sibling
`../_TSCOPF_simulations`, or `ENV["TSCOPF_SIMULATIONS_ROOT"]`. Re-extract pins after
a new baseline run:

```bash
julia --project=. scripts/extract_simulation_pins.jl \
  /path/to/_TSCOPF_simulations/RESULTS/bound_compare_4th_fullbus_avr_constraint
```

**Known-failing locally (2026-08-03, unrelated to source changes).**
`2nd-order FULL_BUS` returns `NUMERICAL_ERROR` and `TG-only` hits its 1200 s
`time_limit_sec` under the `Manifest.toml` resolve current at that date (JuMP 1.30.1,
MathOptInterface 1.51.1). Verified against `main`'s own source in a clean worktree: with an updated
resolve (JuMP 1.31.1 / MOI 1.52.0) the 2nd-order case solves and matches its pin to
`5.5e-10`, while TG-only still times out. These solves sit close enough to the edge that
the nonlinear-expression handling in the JuMP/MOI stack decides the outcome, so a pin
failure here is a *solver-stack* signal, not necessarily a modelling regression — confirm
by re-running the same case on `main` before blaming a diff.

**Bound-encoding parity, same case (2026-08-04).** With the suite finally executable (it
had a Julia-1.11 `@testset` incompatibility that aborted it before any comparison ran),
`bound_compare_2nd_fullbus` reports three mismatched TS dual files between the committed
`_constraint` and `_variable` trees — at row 2, `Gen_7`: `dual_Pbalance` +2.82e-1 vs
−3.21e-1 (a sign flip), `dual_V_lower` −2.51e-1 vs −2.24e-1, `dual_Qbalance` 1.148e-2 vs
1.124e-2. The other three suites are clean (289 assertions pass). This compares
pre-generated CSVs in `_TSCOPF_simulations`, so it says nothing about the package source;
given the note above, the likely reading is a degenerate dual at a near-edge solve rather
than an encoding asymmetry. Worth confirming before relying on a dual comparison across
encodings.

**`TSCOPF_RUN_BOUND_ENCODING_TSC_PARITY=true`** — `runtests_bound_encoding_tsc.jl`,
a port of `compare_bound_encoding_tsc.jl`: compares CONSTRAINT vs VARIABLE CSV/dual
outputs by reading the simulations repo results.

## Tier 4 — PowerModels cross-check (`TSCOPF_RUN_PM_CROSSCHECK=true`)

`runtests_powermodels_crosscheck.jl` — case39 primal agreement against
`ACPPowerModel` (branch form and Ybus matrix form) and `DCPPowerModel`. Needs
`PowerModels` in the test environment (`scripts/setup_test_env.jl` installs it).
Also runs in the nightly heavy job.

---

## I changed X → run Y

| You touched | Run |
|-------------|-----|
| `DispatchConfig`, limits, solver configs | `runtests_fast_unit.jl` |
| ACOPF / DCOPF / ED builders | `runtests_smoke.jl`, `runtests_susceptance_model.jl` |
| Dual registries, dual export | `runtests_dispatch_duals_registry.jl`, `runtests_explicit_dual.jl`, `runtests_bound_encoding.jl` |
| Ybus / Kron reduction | `runtests_admittance_matrices.jl` |
| Kron TS builder | `runtests_tsc_builder_kron.jl` |
| δ / Δω corridor styles (`bound_style_δ`, `bound_style_Δω`, `constrain_*`) | `runtests_fast_unit.jl` (validation), then `runtests_delta_reference.jl` (heavy) |
| FULL_BUS TS builder | `runtests_tsc_builder_fullbus.jl` (+ `TSCOPF_RUN_HEAVY`) |
| DQ_4TH machine | `runtests_dq_fullbus.jl` |
| AVR or governor | `runtests_controls.jl` |
| GFM inverters | `runtests_gfm_g0.jl` (parsing, partition, box resolution), then `runtests_gfm_transient.jl` (heavy) |
| GFM filter / Q–V PI discretisation (`gfm_integrator`) | `runtests_gfm_g0.jl` (structural: three schemes, same model size), then `scripts/run_gfm_integrator_ab.jl` for the solve-level A/B |
| GL / OB disturbances | `runtests_fullbus_gld.jl`, `runtests_tsc_builder_kron.jl` |
| MATPOWER parser or `.m` mode | `runtests_matpower_parser.jl`, `runtests_matpower_input.jl` |
| `RESULTS/` layout, save functions | `runtests_results_folders.jl`, `runtests_optim_matrices.jl` |
| Plotting / `ext/` | `runtests_plots_ext.jl` |
| UC | `runtests_uc.jl` (needs Gurobi) |
| Anything numerics-critical before a reference run | `TSCOPF_RUN_HEAVY=true` + `TSCOPF_RUN_SIMULATION_PINS=true` |

## Shared helpers

`test_paths.jl` (fixture roots and helpers; included from both files below),
`test_common.jl` (run-config factories, timestamped-results assert, reference
constants), `test_env.jl` (imports for standalone scripts), `tsc_main_style_config.jl`
(main.jl-style TSC fragments), `a6_fullbus_benchmark_{config,pins}.jl`,
`simulation_pins/`.

## Reference generators (`scripts/`)

Not package tests. Generate independent PowerModels baselines under
`RESULTS/_benchmarks_PM*`:

```bash
julia --project=. scripts/generate_pm_ac_reference.jl
julia --project=. scripts/generate_pm_dc_reference.jl
```

## Environment

After a fresh clone:

```julia
julia --project=. scripts/setup_test_env.jl
```

Installs Ipopt, HiGHS, Plots, Measures, PowerModels, and — outside CI — Gurobi.
UC tests need Gurobi plus a licence; everything else runs on Ipopt + HiGHS.

`Gurobi` is deliberately **not** committed in `test/Project.toml`: the CI runner has no
licence, so `runtests_uc.jl` skips itself there and the only effect would be downloading
`Gurobi_jll` on every job. Running `setup_test_env.jl` locally adds it back, which shows
up as a diff on `test/Project.toml` — leave that change unstaged.
