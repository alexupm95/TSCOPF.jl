# Quick start

## Script entry (`main.jl`)

Edit `RunConfig(...)` in `main.jl`, then:

```bash
julia --project=. main.jl
```

Results appear under `RESULTS/Results - <timestamp>/` (gitignored). Case CSVs live in `INPUT_FILES/<case>/`.

For a **δ_tol sweep**, use `main_loop.jl` instead — it builds one `RunConfig` and
one `SystemData` up front, then loops with [`reconfigure_transient`](api.md) so the
network is only read once. See section 7.6 of [Running a case](running_a_case.md)
for the pattern.

## Programmatic use

```julia
using TSCOPF

cfg = RunConfig(
    case = "9bus",
    dispatch = DispatchConfig(type_model = "ACOPF"),
    silent_solver = true,
    save_optim_matrices = false,
)
sys = load_system(cfg, @__DIR__)
result = run_case!(cfg, sys, @__DIR__, joinpath(@__DIR__, "RESULTS"))

result.status          # MOI termination status
JuMP.value(result.obj_MVA)  # objective (EUR for quadratic cost × base_MVA)
result.RGEN.p_g        # generator dispatch [MW]
```

!!! note "Check `result.status` before trusting duals"
    `run_case!` only writes output files on `OPTIMAL`, `LOCALLY_SOLVED`, or
    `ITERATION_LIMIT`. For nonconvex ACOPF/TSC-ACOPF, `LOCALLY_SOLVED` is a valid
    KKT point but not a certified global optimum — near-flat cost curvature or a
    poor warm start can land Ipopt in a different local solution, which changes
    the dual values even when the primal dispatch looks reasonable.

## TSC example (minimal)

```julia
cfg = RunConfig(
    trans_stab = true,
    case = "9bus",
    dispatch = DispatchConfig(type_model = "ACOPF"),
    silent_solver = true,
    save_optim_matrices = false,
    save_ts_plots = false,
    transient = TransientConfig(
        simulation = TsSimulationConfig(δ_tol_deg = 100.0),
        dyn_model = DynModelConfig(fault = FaultConfig(contingency_id = 2)),
    ),
)
sys = load_system(cfg, @__DIR__)
result = run_case!(cfg, sys, @__DIR__, joinpath(@__DIR__, "RESULTS"))
```

See [Parameter reference](parameter_reference.md) for `DynModelConfig`, fault types (`SC`, `GL`), and builder toggles.

!!! note "`dispatch_run_config` / `tsc_run_config` in the test suite"
    `test/test_common.jl` defines thin `dispatch_run_config(...)` / `tsc_run_config(...)`
    wrappers around `RunConfig` for test brevity. They are test-only helpers, not part
    of the `TSCOPF` module — `using TSCOPF` alone will not bring them into scope. Use
    `RunConfig` directly (as above) outside the test suite.
