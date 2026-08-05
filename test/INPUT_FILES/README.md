# Frozen test fixtures

Input data the test suite asserts numbers against. **Byte copies** of what the tests used to
read from the repository's `INPUT_FILES/`, moved here so that tree can serve its real purpose —
demo and case-study material a user is free to edit — without a routine edit silently re-pinning
the suite.

Tests reach these through `test/test_paths.jl` (`fixture_case`, `fixture_matpower`,
`load_fixture_system`, `run_fixture_case!`). Nothing under `test/` should read
`<repo>/INPUT_FILES`; `test/runtests_fixture_isolation.jl` enforces that.

## Manifest — what pins what

| Case | Files | Pinned by |
|---|---|---|
| `9bus/` | `bus_data`, `generators_data`, `line_data`, `contingencies`, `gen_dynamic_data`, `gen_dynamic_data_full` | `runtests_a6_fullbus_benchmark.jl:50` (objective at `rtol = 1e-6`), `:55` (per-gen MW), `:61` (δ−COI trajectory); `runtests_fast_unit.jl:17-28` (`Xd_tr ≈ [0.0608, 0.1198, 0.1813]`, `Xd`, `K_exc`); `runtests_smoke.jl:81` (linear-cost ACOPF objective `23 625.0 €`); `runtests_susceptance_model.jl:57` (**every branch must keep r = 0** — the SIMPLE and POWERMODELS susceptance kernels agree only on a lossless network); `runtests_fullbus_gld.jl:83-87` (**branch row order**: branch 1 is the radial 1→4 whose opening islands the slack; branches 6 and 8 are 5→7 and 7→8, which together isolate the {2,7} pocket) |
| `39bus/` | `bus_data`, `generators_data`, `line_data`, `gen_dynamic_data` | `runtests_susceptance_model.jl:83` (**must keep r ≠ 0** — this is the case that makes the two susceptance conventions differ, so the test proves the knob does something); `runtests_admittance_matrices.jl` (needs taps and the dynamic file for the Kron reference) |
| `9bus_gfm/` | steady CSVs, `contingencies`, `gen_dynamic_data_full`, `gfm_dynamic_data` | `runtests_gfm_g0.jl:16-20` (`Imax ≈ 1.2`, `Xl ≈ 0.15`, id 4 at bus 5), `:50-56` (fleet partition 3 SG + 1 GFM), `:158-176` (every `resolve_gfm_bound_limits` box is arithmetic on this row); `runtests_gfm_transient.jl:78-80`, `:159-160` |
| `9bus_test_mfile/` | `case9.m`, `gen_dynamic_data`, `contingencies` | `runtests_matpower_input.jl:143-159` (9/3/9 counts, bus-5 `p_d ≈ 187.5` after `load_factor = 1.5`, `Xd_tr`) |
| `PowerModels/case9.m` | — | `runtests_matpower_parser.jl:38-53` |
| `PowerModels/case39.m` | — | `runtests_matpower_parser.jl:59-81` (39/10/46 counts, gen costs `0.01 / 0.30 / 0.20`, bus-39 load `1104 / 250`, tap `1.025` on 2→30, ±360° angle limits); `runtests_powermodels_crosscheck.jl` |

Excluded on purpose: the variant subfolders (`Linear/`, `NonLinear/`, `original_TSC_integrated/`,
`tests_june/`, `tests_sama_data_forall_gen/`), `gen_dynamic_data_full_UC3M.csv`, the PDFs, and all
`pglib_case*.m`. No test reads them; they stay in the demo tree.

## Coupled pairs — these cannot be edited alone

1. **`PowerModels/case9.m` ↔ `9bus/`.** `runtests_matpower_parser.jl:45-52` asserts that parsing
   the `.m` reproduces the CSVs at *exactly* ×1.5 load (`imp.DBUS.p_d ≈ 1.5 .* ref_DBUS.p_d`).
   The `.m` carries the pre-scaled values (bus 5 `Pd = 187.5`); the CSVs carry the unscaled ones.
2. **`9bus_test_mfile/case9.m` ↔ `9bus/`.** `runtests_matpower_input.jl:179-204` solves both at
   `load_factor = 1.5` and asserts they agree (objective `rtol = 1e-4`, per-gen P within
   0.15 MW, V within 5e-4). Note this is a *different* file from the one above: it carries
   bus-5 `Pd = 125`, i.e. unscaled, and the load factor is applied at run time.
3. **`9bus_gfm/` fleet ↔ the saturation pin.** `runtests_gfm_transient.jl:159-160` asserts
   `maximum(loading.GFM4) > 0.99` — the converter must actually reach its current limit. That
   depends jointly on `Imax = 1.2`, the loads, `load_factor = 1.5`, and contingency 2 (three-phase
   fault at bus 7, cleared after 150 ms by opening branch 5–7). Change any one and the case may
   stop exercising the limiter, which the test would report as a failure rather than a config drift.

## Re-pin protocol

Changing a number here is a deliberate act, not a data refresh:

1. Say why in the commit message — a physics correction, a new operating point, a case swap.
2. Run the heavy tier (`TSCOPF_RUN_HEAVY=true`) and the PM cross-check
   (`TSCOPF_RUN_PM_CROSSCHECK=true`), not just the fast gate.
3. Update `test/a6_fullbus_benchmark_pins.jl` and any suite whose assertion moved, each in its own
   commit with the before/after values recorded.
4. Never re-pin to whatever the new run prints without understanding why it moved. That habit is
   how a silent physics change gets laundered through a data edit.
5. Respect the coupled pairs above — a half-landed re-pin fails in a way that looks like a parser
   bug.

These files are **copies, not moves**: `INPUT_FILES/` still carries its own versions, and the two
trees are expected to drift. A correction applied to the demo data does not reach a fixture unless
someone copies it deliberately, which is the point.
