#=
CODE FOR SOLVING THE OPTIMAL DISPATCH OF ENERGY — AUTOMATED δ_tol (δ_max) SWEEP
Runs the TSC-OPF for a range of δ_tol values (maximum rotor angle tolerance),
creating one timestamped RESULTS folder per scenario.

Author:      Alex Junior da Cunha Coelho
Affiliation: Technical University of Madrid
February 2026

--------------------------------------------------------------------------------
 SWEEP entry point (Phase 1 refactor).
 Thin wrapper over run_case! (engine.jl): builds ONE base RunConfig, then loops
 over δ_tol values via `reconfigure_transient`. No globals.
--------------------------------------------------------------------------------
=#

using Pkg
Pkg.activate(@__DIR__)
using TSCOPF
Clean_Terminal()

# =======================================================
#   BASE CONFIGURATION — shared across every scenario
#   (δ_tol_deg is overridden per iteration in the loop)
# =======================================================
base_cfg = RunConfig(
    trans_stab          = true,
    case                = "9bus",
    base_MVA            = 100.0,
    load_factor         = 1.5,
    solver_name         = "Ipopt",
    silent_solver       = false,
    save_duals          = true,
    save_optim_matrices = false,   # steady-state only; auto-forced false for TSC runs
    overwrite_results   = false,

    dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),

    transient = TransientConfig(
        dyn_model = DynModelConfig(fault = FaultConfig(contingency_id = 2)),
    ),
)

# =======================================================
#   SWEEP CONFIGURATION  ←  edit the range here
# =======================================================
δ_tol_range = 90:1:100   # degrees

"""
    with_δ_tol(sim, δ_tol_deg) -> TsSimulationConfig

Copy `sim`, overriding `δ_tol_deg` alone.

Needed because `TsSimulationConfig(δ_tol_deg = x)` builds a *fresh* struct in
which every other field falls back to its package default. Passing that to
`reconfigure_transient` replaces the whole simulation block, so a base config
carrying e.g. `t_end_sim = 1.0` or `clearing_time = 0.15` would silently revert
to 5.0 s and 0.3 s — and the sweep would vary the horizon and the fault duration
alongside δ_tol, making the resulting curve uninterpretable.

Today `base_cfg` above leaves `simulation` at its defaults, so the old one-field
form happened to be harmless. This keeps it harmless after someone customises it.
"""
function with_δ_tol(sim::TsSimulationConfig, δ_tol_deg::Float64)
    preserved = (
        f => getfield(sim, f)
        for f in fieldnames(TsSimulationConfig) if f !== :δ_tol_deg
    )
    return TsSimulationConfig(; preserved..., δ_tol_deg = δ_tol_deg)
end

path_main           = project_root()
path_folder_results = default_results_dir()

sys = load_system(base_cfg, path_main)

# =======================================================
#   MAIN LOOP
# =======================================================
for (i, δ_val) in enumerate(δ_tol_range)

    println("=" ^ 70)
    println(">>> δ_tol = $(δ_val)°  ($i / $(length(δ_tol_range)))")
    println("=" ^ 70)

    # δ_tol is the ONLY quantity that changes between scenarios: every other
    # timing field is carried over from base_cfg (see `with_δ_tol` above).
    tc = reconfigure_transient(base_cfg.transient;
        simulation = with_δ_tol(base_cfg.transient.simulation, Float64(δ_val)))
    cfg = reconfigure(base_cfg; transient = tc)

    try
        run_case!(cfg, sys, path_main, path_folder_results)
    catch e
        @warn "δ_tol=$(δ_val)° — iteration failed with error: $e"
        continue
    end
end

println("=" ^ 70)
println("Sweep complete.  δ_tol range: $(first(δ_tol_range))° – $(last(δ_tol_range))°  ($(length(δ_tol_range)) scenarios)")
println("=" ^ 70)

# ── Create named summary folder in RESULTS (mirrors the previous behaviour) ──
clearing_time = base_cfg.transient.simulation.clearing_time
contingency_id = base_cfg.transient.dyn_model.fault.contingency_id
active_gens  = findall(x -> x == 1, sys.DGEN.g_status)
costs_str    = join(Int.(sys.DGEN.g_cost_1[active_gens]), "_")
named_folder = "AC_cont$(contingency_id)_ct$(clearing_time)_costs_$(costs_str)"
mkpath(joinpath(path_folder_results, named_folder))
println("Created folder: $named_folder")
