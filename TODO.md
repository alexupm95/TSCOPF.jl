# TODO — deferred work

Low-impact / future items parked here so they are not lost. None block current runs.

## _manage_outputs (output layer)

- **Fold `Export_UC_Model` into `Export_OPF_Model`.** `_uc/functions_make_uc_model.jl`
  still carries its own copy of the model-dump print logic (objective / variables /
  constraint banners). The generic `Export_OPF_Model`
  (`_manage_outputs/functions_2_save_dispatch_model.jl`) now covers AC/DC/ED via
  `MODEL_VAR_ORDER` + `MODEL_EXPORT_SPECS`. UC differs only by the binary `u_commit`
  variable and a couple of UC-specific banner notes. Fold-in: add `:u_commit` to the
  variable order (or pass an extra var list) and route UC through `Export_OPF_Model`.
  Low impact — UC export already works; this is dedup only.

- **R3 — UC double-write.** `Save_Solution_UC_Model` calls `Save_Solution_ED_Model`
  (writes `generators_report.csv` + `OPF_Dispatch_Results.xlsx`) then overwrites both
  in `_save_uc_generator_exports!`. Redundant I/O; have UC skip the ED file writes it
  immediately overwrites.

- **R4 — archival coupling.** `Copy_Input_CSVs_To_Results!`
  (`functions_2_save_input_parameters.jl`) hardcodes `bus_data.csv` /
  `generators_data.csv` / `line_data.csv` and `throw`s if absent. Any case with
  different input names aborts at save time. Drive from the actual input filenames.

## Won't fix (decided)

- **I2 — DC line loading sign.** `Save_Solution_DCOPF_Model` reports loading without
  `abs()`, so reverse-flow lines show negative loading. Intentional for DC — the sign
  is meaningful and the interpretation is straightforward. (AC path keeps `abs`.)

## UC model (enabled by the var/eq/ineq file split)

- **Multi-period.** `opf_dict[:meta][:n_periods] = 1` is a hook. Extend to time-indexed
  `u[g,t]`, `P[g,t]`.
- **Inter-temporal constraints.** Ramp up/down, startup/shutdown, minimum up/down time —
  add as new builders in `_uc/uc_eq_constraints.jl` / `_uc/uc_ineq_constraints.jl`.
- **Reserves / piecewise cost.** Reserve requirements; piecewise-linear or MIQP cost if
  moving beyond the linear MILP requirement.

## Done

- **Asymmetric `Δω_tol_pu`.** `Δω_tol_pu_lower` / `Δω_tol_pu_upper` on `DynModelConfig`
  give independent below/above limits on `Δω − Δω_COI`, mirroring `δ_tol_deg_lower` /
  `δ_tol_deg_upper`. See `docs/parameter_reference.md` §4.2.