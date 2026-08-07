#=
================================================================================
 engine.jl  —  single orchestration core for the TSC-OPF pipeline
================================================================================
 Phase 1 refactor (MF-1 + MF-3 of the audit blueprint).

 Purpose
 -------
 Replace the ~140-line orchestration body that was copy-pasted across main.jl,
 main_loop.jl, main_mcp.jl and main_loop_mcp.jl with ONE function, `run_case!`,
 driven by ONE configuration object, `RunConfig`.

 This file defines:
   • RunConfig    — every run knob in one immutable struct (was scattered across
                    Define_Initial_Parameters(), the swept δ_tol global, and the
                    _contingency_id global).
   • reconfigure  — copy a RunConfig overriding a few fields (used by sweeps).
   • SystemData   — the loaded network data, built once and reused across a sweep.
   • load_system  — read input files + organise the network (was inlined per run).
   • run_case!    — build → solve → (δ_ref) → dynamics → solve → save, for ONE
                    configuration. Returns a NamedTuple with the outcome.

 What it deliberately does NOT change (behaviour preservation)
 ------------------------------------------------------------
   • The model builders, constraint code, and save functions are called exactly
     as before. Only the *plumbing* around them is consolidated.
   • The pre-solve that produces δ_ref runs only for trans_stab + "DCOPF", which
     is what main.jl did. (Old main_loop.jl also pre-solved for ACOPF, a
     redundant extra solve that, for the nonconvex ACOPF, could warm-start the
     final solve into a different local optimum. Dropping it here is a fix, and
     it is flagged in the Phase 1 notes. It does not affect the DCOPF δ_tol
     sweep, where the pre-solve is required and happens identically.)

 Author: Alex Junior da Cunha Coelho  ·  refactor scaffolding, June 2026
================================================================================
=#

# ==============================================================================
#  RunConfig — the single source of truth for one run
# ==============================================================================
# Defaults reproduce the values returned by Define_Initial_Parameters() in
# config_file.jl, so `run_case!(default RunConfig)` == the old default run.
# Override only what a given scenario needs (see main.jl / main_loop.jl wrappers).
"""
    RunConfig

Top-level configuration for one TSC-OPF run: every knob shared by plain
dispatch and transient-stability runs, plus the two nested avenues
`dispatch::DispatchConfig` (steady-state OPF) and
`transient::Union{Nothing,TransientConfig}` (required when `trans_stab = true`).

Build one with keyword arguments (defaults reproduce the legacy integrated
9-bus TSC-ACOPF run), pass it to [`load_system`](@ref) and [`run_case!`](@ref),
and use [`reconfigure`](@ref) to vary a handful of fields (for example a
δ_tol sweep) without mutating the original or touching global state.

See the user guide, section 1, and the configuration map, section 1, for the
full field reference.
"""
Base.@kwdef struct RunConfig
    # --- avenue selector -----------------------------------------------------
    trans_stab::Bool = false

    # --- orchestration (shared by dispatch and TSC) --------------------------
    case::String = "9bus"
    base_MVA::Float64 = 100.0
    load_factor::Float64 = 1.5
    solver_name::String = "Ipopt"
    silent_solver::Bool = false
    time_limit_sec::Float64 = 600.0
    ipopt::IpoptSolverConfig = IpoptSolverConfig()
    highs::HiGHSSolverConfig = HiGHSSolverConfig()
    gurobi::GurobiSolverConfig = GurobiSolverConfig()
    madnlp::MadNLPSolverConfig = MadNLPSolverConfig()

    # --- results / I/O -------------------------------------------------------
    overwrite_results::Bool = false
    save_duals::Bool = true
    save_optim_matrices::Bool = false   # steady-state only; resolve_save_optim_matrices forces false for TSC
    save_matrices::Bool = true
    save_ts_plots::Bool = false   # TSC trajectory figures (Plots.jl); opt-in for sweeps/CI
    # Per-(window, gen, step) diagnostic dumps under Transient_Stability/CSV/Debug/:
    # value, distance to the limit and dual on one row. Off by default — the filter file
    # alone is three rows per generator per step, which adds up fast over a sweep.
    save_ts_debug_csv::Bool = false
    # TSC runs that pre-solve a steady-state OPF before TS assembly (FULL_BUS ACOPF
    # warm start, TSC-DCOPF δ_ref anchor): archive that solution to Dispatch_WarmStart/.
    save_warmstart_dispatch::Bool = false

    # --- avenue 1: steady-state dispatch -------------------------------------
    dispatch::DispatchConfig = DispatchConfig()

    # --- avenue 2: transient analysis (required when trans_stab=true) --------
    transient::Union{Nothing, TransientConfig} = nothing

    # --- input-data source (optional MATPOWER override) ----------------------
    # When `nothing` (default) steady-state data is read from the in-house CSVs
    # in INPUT_FILES/<case>/ (bus_data.csv, generators_data.csv, line_data.csv).
    # When set to a filename (relative to INPUT_FILES/<case>/) or an absolute
    # path, DBUS/DGEN/DCIR are parsed from that MATPOWER .m file instead.
    # Dynamic data (gen_dynamic_data.csv) and contingency tables are always
    # read from INPUT_FILES/<case>/ regardless of this setting.
    matpower_file::Union{Nothing, String} = nothing
end

"""Deep-copy a `DynModelConfig` (avoids @kwdef shared-default mutation)."""
function copy_dyn_model_config(dm::DynModelConfig)::DynModelConfig
    return DynModelConfig(;
        (f => f == :fault ? copy_fault_config(dm.fault) : getfield(dm, f)
         for f in fieldnames(DynModelConfig))...)
end

"""
    reconfigure(cfg::RunConfig; kwargs...) -> RunConfig

Copy `cfg`, overriding the fields given as keyword arguments. `dispatch` and
`transient` are deep-copied by default (via `copy_dispatch_config` /
`copy_transient_config`) unless explicitly passed in `kwargs`, so sweeps can
vary one knob (for example `load_factor` or `δ_tol_deg` through
[`reconfigure_transient`](@ref)) without mutating the original `cfg` or
touching global state.
"""
function reconfigure(cfg::RunConfig; kwargs...)
    base = (; (f => getfield(cfg, f) for f in fieldnames(RunConfig))...)
    merged = pairs((; base..., kwargs...))
    args = Dict{Symbol, Any}(merged)
    if !haskey(args, :dispatch)
        args[:dispatch] = copy_dispatch_config(cfg.dispatch)
    end
    if !haskey(args, :transient) && cfg.transient !== nothing
        args[:transient] = copy_transient_config(cfg.transient)
    end
    return RunConfig(; args...)
end

"""
    validate_run_config!(cfg::RunConfig)

Fail fast on incoherent run configuration (avenue split, dispatch, transient).
"""
function validate_run_config!(cfg::RunConfig)
    validate_dispatch_config!(cfg.dispatch)
    if cfg.trans_stab
        cfg.transient === nothing &&
            throw(ArgumentError("trans_stab=true requires transient::TransientConfig."))
        validate_transient_config!(cfg.transient)
        validate_dyn_config!(cfg)
    elseif cfg.transient !== nothing
        throw(ArgumentError("trans_stab=false requires transient=nothing."))
    end
    if cfg.trans_stab && cfg.save_ts_plots && !load_plots_extension!()
        throw(ArgumentError(
            "save_ts_plots=true requires Plots.jl in the active environment. " *
            "Add it with `Pkg.add(\"Plots\")` and call `load_plots_extension!()` " *
            "before `run_case!` (loads Plots + Measures for the extension)."))
    end
    if cfg.save_warmstart_dispatch && !warmstart_dispatch_applicable(cfg)
        throw(ArgumentError(
            "save_warmstart_dispatch=true requires a TSC run with a steady-state pre-solve: " *
            "either dispatch.type_model=\"ACOPF\" with network_form=FULL_BUS (ACOPF warm start) " *
            "or dispatch.type_model=\"DCOPF\" with network_form=KRON_REDUCED (δ_ref anchor)."))
    end
    if is_ipopt_backend_solver(cfg.solver_name)
        validate_ipopt_solver_config!(cfg.ipopt, cfg.solver_name)
    elseif cfg.solver_name == "HiGHS"
        validate_highs_solver_config!(cfg.highs)
    elseif cfg.solver_name == "Gurobi"
        validate_gurobi_solver_config!(cfg.gurobi)
    elseif cfg.solver_name == "MadNLP"
        validate_madnlp_solver_config!(cfg.madnlp)
    end
    return nothing
end

"""
    validate_dyn_config!(cfg::RunConfig)

Fail fast on incoherent model-selection combinations. Default `DynModelConfig()`
always passes so existing runs are unchanged.
"""
function validate_dyn_config!(cfg::RunConfig)
    cfg.transient === nothing && return nothing
    dyn = cfg.transient.dyn_model
    type_model = cfg.dispatch.type_model

    # Active and reactive demand carry independent (Z, I, P) splits; each must sum to 1
    # on its own — a shared check would let a mismatched pair through.
    for (name, split) in ((:zip_load_p, dyn.zip_load_p), (:zip_load_q, dyn.zip_load_q))
        z, i, p = split   # (Z, I, P): impedance, current, power
        if !isapprox(z + i + p, 1.0; atol=1e-9)
            throw(ArgumentError(
                "$name (Z, I, P) coefficients must sum to 1 (got Z=$z, I=$i, P=$p)."))
        end
    end

    # Kron folds loads into Y_red as constant admittance, so the splits are dead knobs
    # there. Warn rather than throw — a non-default split is a misconfiguration, not a
    # reason to block an otherwise valid Kron run.
    if dyn.network_form == KRON_REDUCED &&
       (dyn.zip_load_p != (1.0, 0.0, 0.0) || dyn.zip_load_q != (1.0, 0.0, 0.0))
        @warn "zip_load_p / zip_load_q are ignored on KRON_REDUCED (loads are folded " *
              "into Y_red as constant admittance). Use network_form=FULL_BUS to apply them." *
              " Got zip_load_p=$(dyn.zip_load_p), zip_load_q=$(dyn.zip_load_q)."
    end

    if dyn.gen_order == DQ_4TH && dyn.network_form != FULL_BUS
        throw(ArgumentError(
            "DQ_4TH requires FULL_BUS network_form (Kron reduction is inconsistent with dq dynamics)."))
    end

    if dyn.include_avr && dyn.gen_order != DQ_4TH
        throw(ArgumentError("include_avr=true requires gen_order=DQ_4TH."))
    end

    # The FULL_BUS builders need mechanical power as its own state (the swing is driven
    # by an explicit nodal Pe, so pinning P_mech ≡ P_g would over-constrain the coupling).
    # Both builders already throw, but only after the warm-start ACOPF has been solved —
    # check here so the run dies before paying for that solve.
    if dyn.network_form == FULL_BUS && dyn.mech_power_mode != USE_PM
        throw(ArgumentError(
            "network_form=FULL_BUS requires mech_power_mode=USE_PM " *
            "(P_m must be an independent variable on the full-network paths)."))
    end

    if dyn.gen_order == DQ_4TH
        # Milestone 1/2/3: machine core; optional AVR and governor.
        dyn.mech_power_mode != USE_PM && throw(ArgumentError(
            "DQ_4TH requires mech_power_mode=USE_PM."))
        dyn.bound_style != :coi_box && throw(ArgumentError(
            "DQ_4TH requires bound_style=:coi_box."))
    end

    if dyn.include_governor && dyn.mech_power_mode != USE_PM
        throw(ArgumentError(
            "include_governor=true requires mech_power_mode=USE_PM (governor acts on P_m / P_mech)."))
    end

    if dyn.include_governor && dyn.network_form != FULL_BUS
        throw(ArgumentError(
            "include_governor=true is currently supported only on network_form=FULL_BUS " *
            "(the Kron path does not run the FULL_BUS coupling initialisation pipeline)."))
    end

    if cfg.trans_stab && type_model == "DCOPF" && dyn.gen_order == DQ_4TH
        throw(ArgumentError(
            "TSC-DCOPF with DQ_4TH is not implemented (no linearised dq formulation yet)."))
    end

    if dyn.bound_style ∉ (:swing_propagated, :coi_box)
        throw(ArgumentError("bound_style must be :swing_propagated or :coi_box."))
    end

    if dyn.ode_first_step ∉ (:trapezoidal, :backward_euler)
        throw(ArgumentError(
            "ode_first_step must be :trapezoidal or :backward_euler (got $(dyn.ode_first_step))."))
    end

    # The Kron swing rows hard-code the trapezoidal average at every step, including
    # t=1, so backward Euler is unimplemented there rather than merely unwired. Throw
    # instead of silently integrating with a scheme the user did not ask for.
    if dyn.ode_first_step === :backward_euler && dyn.network_form == KRON_REDUCED
        throw(ArgumentError(
            "ode_first_step=:backward_euler is not implemented on network_form=KRON_REDUCED " *
            "(the Kron swing equations are trapezoidal at every step). " *
            "Use network_form=FULL_BUS for the backward-Euler first step."))
    end

    if dyn.gfm_integrator ∉ (:follow_ode_first_step, :backward_euler, :trapezoidal)
        throw(ArgumentError(
            "gfm_integrator must be :follow_ode_first_step, :backward_euler or " *
            ":trapezoidal (got $(dyn.gfm_integrator))."))
    end

    # The GFM knob only reaches the DQ FULL_BUS converter path. Warn rather than throw:
    # a stale non-default on a Kron or classical run is a misconfiguration, not a reason
    # to block an otherwise valid case.
    if dyn.gfm_integrator != :backward_euler && !dyn.allow_gfm
        @warn "gfm_integrator=$(dyn.gfm_integrator) is ignored when allow_gfm=false " *
              "(it only governs the GFM measurement filters and the Q–V PI integrator)."
    end

    if dyn.mech_power_mode == USE_PM && dyn.bound_style != :coi_box
        throw(ArgumentError(
            "USE_PM requires bound_style=:coi_box (swing_propagated is invalid with explicit P_m)."))
    end

    if cfg.trans_stab && type_model == "DCOPF" && dyn.network_form == FULL_BUS
        throw(ArgumentError(
            "TSC-DCOPF with FULL_BUS is not implemented yet (Phase 3 is TSC-ACOPF only)."))
    end

    if dyn.constrain_Δω_COI
        dyn.Δω_tol_pu > 0.0 ||
            throw(ArgumentError("constrain_Δω_COI=true requires Δω_tol_pu > 0."))
        # Overrides are positive magnitudes below/above COI (see Δω_tol_tuple).
        for (name, v) in ((:Δω_tol_pu_lower, dyn.Δω_tol_pu_lower),
                          (:Δω_tol_pu_upper, dyn.Δω_tol_pu_upper))
            v === nothing || v > 0.0 ||
                throw(ArgumentError("$name must be positive when set."))
        end
    end

    if dyn.allow_gfm
        dyn.network_form == FULL_BUS || throw(ArgumentError(
            "allow_gfm=true requires network_form=FULL_BUS."))
        dyn.gen_order == DQ_4TH || throw(ArgumentError(
            "allow_gfm=true requires gen_order=DQ_4TH."))
        type_model == "DCOPF" && throw(ArgumentError(
            "allow_gfm=true is not supported with TSC-DCOPF (linearize path)."))
    end

    return nothing
end

"""
    validate_dyn_data!(cfg::RunConfig, DGEN_DYN)

Fail fast when the machine CSV named by `TransientConfig.gen_dynamic_filename` does
not carry the columns the selected `DynModelConfig` needs — `R, T1, T2, T3` under
`include_governor`, `T_exc, K_exc` under `include_avr`, the dq set under `DQ_4TH`.

Separate from [`validate_dyn_config!`](@ref) because that one is config-only and
runs before any data is read; this needs `DGEN_DYN`, so it runs from `run_case!`
where `SystemData` is in hand — the same shape as `validate_fault_config!`.

Without this the run would not fail at all on the classical path: the parser
pre-fills absent optional columns with `NaN`, so `DGEN_DYN.R` exists whatever the
file contains, and those `NaN`s get stamped straight into governor constraint
coefficients.

Every row is checked, out-of-service generators included — the same rule
`has_required_dyn_columns` has always applied. Mixed SG + GFM fleets cannot
false-positive here: grid-forming units live in `gfm_dynamic_data.csv` and are not
rows of `DGEN_DYN`.
"""
function validate_dyn_data!(cfg::RunConfig, DGEN_DYN::Union{DataFrame, Nothing})
    cfg.trans_stab || return nothing
    dyn = cfg.transient.dyn_model
    fname = cfg.transient.gen_dynamic_filename

    DGEN_DYN === nothing && throw(ArgumentError(
        "trans_stab=true requires generator dynamic data, but none was loaded " *
        "(expected $fname). Build the SystemData with load_system on a TSC RunConfig."))

    missing_cols = missing_dyn_columns(dyn, DGEN_DYN)
    isempty(missing_cols) && return nothing

    cols = join(string.(missing_cols), ", ")
    why = join(dyn_column_reasons(missing_cols), ", ")
    throw(ArgumentError(
        "$fname is missing machine data required by the selected dynamic model: " *
        "$cols (required by $why). Columns must be present and finite for every " *
        "generator row."))
end

# ==============================================================================
#  SystemData — network data loaded once, reused across a sweep
# ==============================================================================
# DGEN_DYN is only present when trans_stab is true; hence the Union with Nothing.
"""
    SystemData

Network data loaded once by [`load_system`](@ref) and reused across a run or
a parameter sweep: bus/generator/circuit `DataFrame`s (`DBUS`, `DGEN`,
`DCIR`), optional SG dynamic machine data (`DGEN_DYN`, `nothing` unless
`trans_stab = true`), optional GFM dynamic data (`DGFM`, `nothing` unless
`allow_gfm`), the bus-index remap (`bus_mapping`,
`reverse_bus_mapping`), element counts (`nBUS`, `nGEN`, `nCIR`), adjacency
lookups (`bus_gen_circ_dict`, `bus_gen_circ_dict_ON`), and the resolved
input-file path (`path_input`).

Demand (`p_d`, `q_d`) is already scaled by `RunConfig.load_factor` at this
point — build a fresh `SystemData` if `load_factor` changes.
"""
struct SystemData
    DBUS::DataFrame
    DGEN::DataFrame
    DGEN_DYN::Union{DataFrame, Nothing}
    DGFM::Union{DataFrame, Nothing}
    DCIR::DataFrame
    bus_mapping::OrderedDict
    reverse_bus_mapping::OrderedDict
    nBUS::Int
    nGEN::Int
    nCIR::Int
    bus_gen_circ_dict::OrderedDict
    bus_gen_circ_dict_ON::OrderedDict
    path_input::String
end

"""
    load_system(cfg::RunConfig, path_main::String=project_root()) -> SystemData

Read the input files, scale demand by `cfg.load_factor`, and organise the
bus/generator/circuit adjacency into a [`SystemData`](@ref). `path_main`
defaults to the repository root ([`project_root`](@ref)).

Two input modes, governed by `cfg.matpower_file`:

- CSV mode (`matpower_file = nothing`, default): reads `bus_data.csv`,
  `generators_data.csv`, `line_data.csv` from `<path_main>/INPUT_FILES/<case>/`.
- MATPOWER mode (`matpower_file = "case9.m"` or an absolute path):
  steady-state data (`DBUS`, `DGEN`, `DCIR`) is parsed in-memory from the
  `.m` file; dynamic data (`DGEN_DYN`) and contingency tables are still read
  from `<path_main>/INPUT_FILES/<case>/`.

Every input path derives from that one join, so a case tree outside the
repository is used by passing its root — no other argument or environment
variable is involved.

Call once per case and reuse the returned `SystemData` across a sweep
(`load_factor` is only applied here, so build a fresh `SystemData` if it
changes between runs).
"""
function load_system(cfg::RunConfig, path_main::String=project_root())::SystemData
    path_input = joinpath(path_main, "INPUT_FILES", cfg.case)
    allow_gfm = cfg.trans_stab && cfg.transient !== nothing &&
                cfg.transient.dyn_model.allow_gfm
    gfm_dyn_file = cfg.trans_stab && cfg.transient !== nothing ?
        cfg.transient.gfm_dynamic_filename : "gfm_dynamic_data.csv"

    if isnothing(cfg.matpower_file)
        # ── CSV path (existing behaviour, unchanged) ───────────────────────────
        if cfg.trans_stab
            gen_dyn_file = cfg.transient.gen_dynamic_filename
            DBUS, DGEN, DGEN_DYN, DCIR, bus_mapping, reverse_bus_mapping =
                Read_Input_Data(path_input, cfg.trans_stab;
                    gen_dynamic_filename=gen_dyn_file)
            nBUS = length(DBUS.bus); nGEN = length(DGEN.id); nCIR = length(DCIR.from_bus)
            if allow_gfm
                DGFM = Read_GFM_Dynamic_Data(path_input;
                    filename=gfm_dyn_file, sys_base_MVA=cfg.base_MVA)
                # Prefer explicit `id` from SG CSV when present (partitioned fleets).
                DGEN_DYN = _ensure_gen_dyn_ids_from_raw_if_needed(path_input, gen_dyn_file, DGEN_DYN)
                validate_gen_dyn_partition!(DGEN, DGEN_DYN, DGFM)
            else
                DGFM = nothing
                Check_Gen_Sta_Dyn_Data(nGEN, nrow(DGEN_DYN))
                validate_dgen_dyn_row_ids!(DGEN, DGEN_DYN)
            end
        else
            DBUS, DGEN, DCIR, bus_mapping, reverse_bus_mapping =
                Read_Input_Data(path_input, cfg.trans_stab)
            nBUS = length(DBUS.bus); nGEN = length(DGEN.id); nCIR = length(DCIR.from_bus)
            DGEN_DYN = nothing
            DGFM = nothing
        end
    else
        # ── MATPOWER .m path ───────────────────────────────────────────────────
        path_m = isabspath(cfg.matpower_file) ?
                 cfg.matpower_file :
                 joinpath(path_input, cfg.matpower_file)
        isfile(path_m) || throw(ArgumentError(
            "matpower_file not found: \"$path_m\". " *
            "Verify RunConfig.matpower_file and RunConfig.case."))

        DBUS, DGEN, DCIR, parsed_baseMVA, bus_mapping, reverse_bus_mapping =
            matpower_to_inhouse_dataframes(path_m)

        if !isapprox(parsed_baseMVA, cfg.base_MVA; rtol=1e-6)
            @warn ("MATPOWER baseMVA=$(parsed_baseMVA) differs from " *
                   "RunConfig.base_MVA=$(cfg.base_MVA). Using RunConfig value. " *
                   "Set base_MVA=$(parsed_baseMVA) to suppress this warning.")
        end

        nBUS = length(DBUS.bus); nGEN = length(DGEN.id); nCIR = length(DCIR.from_bus)

        if cfg.trans_stab
            gen_dyn_file = cfg.transient.gen_dynamic_filename
            DGEN_DYN = Read_Gen_Dynamic_Data(path_input; filename=gen_dyn_file)
            if allow_gfm
                DGFM = Read_GFM_Dynamic_Data(path_input;
                    filename=gfm_dyn_file, sys_base_MVA=cfg.base_MVA)
                DGEN_DYN = _ensure_gen_dyn_ids_from_raw_if_needed(path_input, gen_dyn_file, DGEN_DYN)
                validate_gen_dyn_partition!(DGEN, DGEN_DYN, DGFM)
            else
                DGFM = nothing
                Check_Gen_Sta_Dyn_Data(nGEN, nrow(DGEN_DYN))
                validate_dgen_dyn_row_ids!(DGEN, DGEN_DYN)
            end
        else
            DGEN_DYN = nothing
            DGFM = nothing
        end
    end

    # Scale demand (vectorised, in place on the freshly-read DataFrame columns).
    DBUS.p_d = cfg.load_factor .* DBUS.p_d
    DBUS.q_d = cfg.load_factor .* DBUS.q_d

    bus_gen_circ_dict, bus_gen_circ_dict_ON = Organize_Bus_Gen_Circ(DBUS, DGEN, DCIR)

    return SystemData(DBUS, DGEN, DGEN_DYN, DGFM, DCIR, bus_mapping, reverse_bus_mapping,
                      nBUS, nGEN, nCIR, bus_gen_circ_dict, bus_gen_circ_dict_ON,
                      path_input)
end

"""
If the SG dynamic CSV has an `id` column, replace auto `1:n` ids with those values.
Needed when `allow_gfm` partitions the fleet and SG rows are a subset of `DGEN`.
"""
function _ensure_gen_dyn_ids_from_raw_if_needed(
    folder_path::String,
    filename::String,
    DGEN_DYN::DataFrame,
)::DataFrame
    path = joinpath(folder_path, filename)
    isfile(path) || return DGEN_DYN
    df_raw = CSV.read(path, DataFrame; delim=';')
    headers = lowercase.(strip.(string.(names(df_raw))))
    id_j = findfirst(==("id"), headers)
    id_j === nothing && return DGEN_DYN
    nrow(df_raw) == nrow(DGEN_DYN) || return DGEN_DYN
    DGEN_DYN = copy(DGEN_DYN)
    DGEN_DYN.id = Int64.(df_raw[!, id_j])
    return DGEN_DYN
end

"""
    resolve_fullbus_coupling_hints!(cfg, model, opf_dict, obj_function_MVA,
                                    path_names, sys, solver_log_warmstart)

Solve the steady-state ACOPF that seeds every FULL_BUS coupling variable and return
its operating point as [`SteadyStateHints`](@ref).

The pre-solve is **mandatory** on FULL_BUS: the dynamic variables (E, δ, Ed/Eq,
Id/Iq, bus V/θ) are initialised from it, and a flat guess leaves the joint NLP far
enough from any equilibrium that Ipopt typically stalls at iteration 0. A failed
pre-solve therefore aborts the run rather than falling back.
"""
function resolve_fullbus_coupling_hints!(
    cfg::RunConfig,
    model::Model,
    opf_dict::OrderedDict{Symbol, Any},
    obj_function_MVA,
    path_names::OrderedDict{Symbol, String},
    sys::SystemData,
    solver_log_warmstart::String,
)::SteadyStateHints
    println("\n--- FULL_BUS: solving steady-state ACOPF for warm start ---")
    set_solver_log_path!(model, cfg.solver_name, solver_log_warmstart)
    optimize!(model)
    ws_status = termination_status(model)
    if ws_status != MOI.OPTIMAL && ws_status != MOI.LOCALLY_SOLVED
        throw(ArgumentError(
            "FULL_BUS requires a successful ACOPF warm start before TS assembly " *
            "(termination status: $ws_status). Check the case data and the steady-state " *
            "limits, or relax the Ipopt tolerances; see $solver_log_warmstart."))
    end
    hints = extract_opf_solved_hints(opf_dict)
    println("ACOPF warm start succeeded — injecting solved V/θ/P_g/Q_g into TS starts.")
    println("Warm-start objective: $(JuMP.value.(obj_function_MVA))\n")
    cfg.save_warmstart_dispatch && save_warmstart_dispatch!(
        cfg, path_names, model, obj_function_MVA, opf_dict, sys;
        type_model="ACOPF",
        model_label=cfg.dispatch.use_matrix ? "AC-OPF (Ybus, warm start)" : "AC-OPF (warm start)")
    return hints
end

"""
    save_warmstart_dispatch!(cfg, path_names, model, obj_function_MVA, opf_dict, sys;
                             type_model, model_label)

Persist the steady-state pre-solve that precedes TS assembly to `Dispatch_WarmStart/`.

Used by both pre-solving paths — the FULL_BUS ACOPF warm start and the TSC-DCOPF
solve that fixes the Taylor anchor `δ_ref` — because `Export_OPF_Model` and
`Save_Solution_Optimal_Dispatch` already select their content from `opf_dict`.
"""
function save_warmstart_dispatch!(
    cfg::RunConfig,
    path_names::OrderedDict{Symbol, String},
    model::Model,
    obj_function_MVA,
    opf_dict::OrderedDict{Symbol, Any},
    sys::SystemData;
    type_model::String,
    model_label::String,
)
    ws_paths = dispatch_warmstart_path_names(path_names)
    mkpath(ws_paths[:pf_dispatch])
    mkpath(ws_paths[:pf_dispatch_CSV])
    cfg.save_duals && mkpath(ws_paths[:pf_dispatch_CSV_duals])

    Export_OPF_Model(model, ws_paths, obj_function_MVA, opf_dict; model_label=model_label)
    Save_Solution_Optimal_Dispatch(
        ws_paths, model, type_model, cfg.dispatch.use_matrix,
        obj_function_MVA, opf_dict,
        sys.bus_gen_circ_dict, sys.DBUS, sys.DGEN, sys.DCIR,
        cfg.base_MVA, sys.nBUS, sys.nGEN, sys.nCIR,
        sys.bus_mapping, sys.reverse_bus_mapping;
        _save_duals=cfg.save_duals)
    println("Warm-start dispatch ($type_model) saved in: ", ws_paths[:pf_dispatch])
    return nothing
end

"""
    save_delta_ref_csv!(path_names, δ_ref)

Write the TSC-DCOPF linearisation anchor to `Dispatch_WarmStart/CSV/delta_ref.csv`.

`δ_ref[g] = P_g·X'_d + θ_bus(g)` from the DC pre-solve is the point the electrical
power is Taylor-expanded around, so it is a property of the *run*, not of the final
solution: nothing downstream re-derives it.
"""
function save_delta_ref_csv!(
    path_names::OrderedDict{Symbol, String},
    δ_ref::OrderedDict{Int64, Float64},
)
    isempty(δ_ref) && return nothing
    ws_csv = path_names[:pf_dispatch_warmstart_CSV]
    mkpath(ws_csv)
    gens = sort(collect(keys(δ_ref)))
    table = DataFrame(
        gen = gens,
        delta_ref_rad = [δ_ref[g] for g in gens],
        delta_ref_deg = [rad2deg(δ_ref[g]) for g in gens],
    )
    CSV.write(joinpath(ws_csv, "delta_ref.csv"), table; delim=';')
    println("TSC-DCOPF linearisation anchor saved to: ", joinpath(ws_csv, "delta_ref.csv"))
    return nothing
end

"""
    run_case!(cfg::RunConfig, sys::SystemData, path_main::String, path_folder_results::String)

Build, solve, and save one TSC-OPF run: validate `cfg`, resolve the results
folder layout, build the steady-state (and, when `cfg.trans_stab = true`,
transient-stability) JuMP model, solve it, and write outputs under
`path_folder_results` (conditional subfolders — see the user guide, section 8).

Validation runs in order before any model is built: `Check_Coherence_Input_Data`
(model/solver pairing), `validate_run_config!`, and, for TSC runs,
`validate_fault_config!`.

Returns a `NamedTuple` with `status` (the `MOI` termination status),
`obj_MVA`, `RBUS`, `RGEN`, `RCIR` (result `DataFrame`s), `dyn_model_dict`,
`dyn_parameters_dict`, `t_build`, `t_solve`, and `path_names` (the resolved
results paths).

!!! warning "The JuMP model is released before this function returns"
    `release_solver_backend!` calls `empty!(model)` on Ipopt paths, so **every**
    `JuMP.VariableRef` created during the run is invalid once `run_case!` returns.
    `obj_MVA` is therefore a plain **number** (the already-evaluated objective, or
    `nothing` when the solve did not converge) — do not wrap it in `JuMP.value`.
    The same applies to `dyn_model_dict`: its `:vars` entries are dead references
    afterwards. Query variable values inside the builders, or read them back from
    the exported CSVs under `path_names`.

`path_folder_results` is typically `joinpath(path_main, "RESULTS")`.
"""
function run_case!(cfg::RunConfig, sys::SystemData,
                   path_main::String, path_folder_results::String)

    # --- coherence / sanity checks (fail fast before building anything) -------
    # Throws on an invalid (type_model, solver, trans_stab) combo, e.g. an
    # ACOPF / TSC-ACOPF run with Gurobi or HiGHS (NLP solver required).
    Check_Coherence_Input_Data(cfg.trans_stab, cfg.dispatch.type_model, cfg.solver_name)
    validate_run_config!(cfg)
    if cfg.trans_stab
        validate_fault_config!(
            cfg.transient.dyn_model.fault, sys.DGEN, sys.DBUS, sys.DCIR)
        # Machine data must satisfy the selected dyn model before anything is built:
        # a missing governor/AVR/dq column reaches the builders as NaN, not as an error.
        validate_dyn_data!(cfg, sys.DGEN_DYN)
    end

    # Resolve the optimization-matrix export request. Forced false (with a warning)
    # for TSC runs, where the Jacobian/Hessian are huge. Non-interactive — never
    # blocks a sweep/test/batch job. The resolved value is what we both act on and
    # record in the input-parameters file, so the log reflects what actually ran.
    save_optim_matrices_eff =
        resolve_save_optim_matrices(cfg.save_optim_matrices, cfg.trans_stab)

    # --- results folder for this run (simulation-dependent subfolders only) ---
    path_names = build_results_paths(path_main, path_folder_results, cfg)
    path_names[:pf_input_files] = sys.path_input
    # Toy / in-memory systems may have no INPUT_FILES folder — skip archival.
    if !isempty(sys.path_input) && isdir(sys.path_input)
        Copy_Input_CSVs_To_Results!(
            path_names;
            trans_stab = cfg.trans_stab,
            gen_dynamic_filename = cfg.transient === nothing ?
                "gen_dynamic_data.csv" : cfg.transient.gen_dynamic_filename,
            gfm_dynamic_filename = (cfg.transient !== nothing &&
                cfg.transient.dyn_model.allow_gfm) ?
                cfg.transient.gfm_dynamic_filename : nothing,
            matpower_file = cfg.matpower_file,
        )
    end

    # --- optimiser setup ------------------------------------------------------
    model = Setup_Optim_Model(
        cfg.solver_name;
        ipopt=cfg.ipopt,
        highs=cfg.highs,
        gurobi=cfg.gurobi,
        madnlp=cfg.madnlp,
        silent=cfg.silent_solver,
    )
    JuMP.set_time_limit_sec(model, cfg.time_limit_sec)
    log_dir = path_names[:pf_results_date]
    solver_log_main = joinpath(log_dir, "solver_log.txt")
    solver_log_warmstart = joinpath(log_dir, "solver_log_warmstart.txt")

    # --- build steady-state model --------------------------------------------
    t_build = time()
    model, obj_function, obj_function_MVA, opf_dict = Build_SteadyState_Model!(
        path_names, model, cfg.dispatch,
        sys.DBUS, sys.DGEN, sys.DCIR, sys.bus_gen_circ_dict_ON,
        cfg.base_MVA, sys.nBUS, sys.nGEN, sys.nCIR;
        save_matrices=cfg.save_matrices)

    type_model = cfg.dispatch.type_model

    # Phase G1: GFM steady-state ACOPF current / internal-E limits (system-base Imax).
    if type_model == "ACOPF" &&
       cfg.trans_stab &&
       cfg.transient !== nothing &&
       cfg.transient.dyn_model.allow_gfm &&
       sys.DGFM !== nothing &&
       nrow(sys.DGFM) > 0
        attach_gfm_acopf_limits!(model, opf_dict, sys.DGEN, sys.DGFM, cfg.base_MVA)
    end

    # --- DCOPF steady-state objective before TS (for explicit dual verification) ---
    primal_dcopf_f_star = nothing
    δ_ref = OrderedDict{Int64, Float64}()
    steady_state_hints = nothing
    if cfg.trans_stab && type_model == "DCOPF"
        set_solver_log_path!(model, cfg.solver_name, solver_log_warmstart)
        optimize!(model)
        status = termination_status(model)
        if status == MOI.OPTIMAL || status == MOI.LOCALLY_SOLVED
            primal_dcopf_f_star = objective_value(model)
            Pg_sol = [JuMP.value(v) for (i, v) in opf_dict[:vars][:P_g]]
            θ_sol  = [JuMP.value(v) for (i, v) in opf_dict[:vars][:θ]]
            active_gen = findall(x -> x == 1, sys.DGEN.g_status)
            for gen in active_gen
                bus = sys.DGEN.bus[gen]
                Xd_prime = sys.DGEN_DYN.Xd_tr[gen]   # transient reactance
                δ_ref[gen] = Pg_sol[gen] * Xd_prime + θ_sol[bus]
            end
            println("Steady-state objective: $(JuMP.value.(obj_function_MVA)) \n")
            # This solve is a genuine pre-solve: its dispatch is what δ_ref is built
            # from, and the TS constraints below linearise around it.
            if cfg.save_warmstart_dispatch
                save_warmstart_dispatch!(
                    cfg, path_names, model, obj_function_MVA, opf_dict, sys;
                    type_model="DCOPF",
                    model_label=cfg.dispatch.use_matrix ? "DC-OPF (Bbus, warm start)" :
                                                          "DC-OPF (warm start)")
                save_delta_ref_csv!(path_names, δ_ref)
            end
        end
    elseif cfg.trans_stab && type_model == "ACOPF" &&
           cfg.transient.dyn_model.network_form == FULL_BUS
        steady_state_hints = resolve_fullbus_coupling_hints!(
            cfg, model, opf_dict, obj_function_MVA, path_names, sys, solver_log_warmstart)
    end

    # --- add transient-stability constraints ---------------------------------
    # δ_tol and contingency_id are threaded in explicitly (no globals, no runtime
    # redefinition of Common_Parameters_4_TS).
    dyn_model_dict = nothing
    dyn_parameters_dict = nothing
    if cfg.trans_stab
        # Machine-data columns were checked in the sanity block above (validate_dyn_data!),
        # which covers every gen_order and control flag, not just DQ_4TH, and fires before
        # the warm-start ACOPF solve rather than after it.
        # Factory dispatch (Phase 1.2): DCOPF → linearized Pe; ACOPF → nonlinear Pe
        linearize = type_model == "DCOPF"
        model, dyn_model_dict, dyn_parameters_dict = Build_Dynamic_Model!(
            path_names, model, opf_dict,
            sys.DBUS, sys.DGEN, sys.DGEN_DYN, sys.DCIR, sys.bus_gen_circ_dict_ON,
            cfg.base_MVA, sys.nBUS, sys.nGEN, sys.nCIR, cfg.transient;
            linearize=linearize,
            δ_ref=linearize ? δ_ref : nothing,
            steady_state_hints=steady_state_hints,
            DGFM=sys.DGFM,
        )
        # After TS assembly, Ipopt cold-starts from `start=` (not the prior ACOPF
        # primal). Re-stamp solved V/θ/P_g/Q_g so the joint NLP begins at the reference
        # warm-start operating point (avoids obj=0 / inf_pr~1e10 at iter 0).
        if steady_state_hints !== nothing
            apply_steady_state_hints_to_opf!(opf_dict, steady_state_hints)
        end
        # Snapshot pre-fault coupling JuMP start= values under Dispatch_WarmStart/
        # (before joint optimize!). Independent of save_warmstart_dispatch, but only
        # meaningful where the starts came from a solved ACOPF: off FULL_BUS the Kron
        # builders seed δ=0 / E=1 constants, so the file would record nothing.
        if steady_state_hints !== nothing && dyn_model_dict !== nothing
            Save_Prefault_Coupling_Starts!(dyn_model_dict, path_names)
        end
    end
    t_build = time() - t_build

    # --- solve the full problem ----------------------------------------------
    t_solve = time()
    set_solver_log_path!(model, cfg.solver_name, solver_log_main)
    if cfg.solver_name in ("Ipopt", "Ipopt-ma57", "Ipopt-ma97", "Ipopt-pardiso")
        # Prefer previous primal when available; starts from apply_* cover OPF vars.
        JuMP.set_optimizer_attribute(model, "warm_start_init_point", "yes")
    end
    JuMP.optimize!(model)
    t_solve = time() - t_solve
    status_model = JuMP.termination_status(model)
    println("\nTime to build model : $(round(t_build; digits=2)) sec")
    println("Time to solve model : $(round(t_solve; digits=2)) sec")
    println("Termination status  : $status_model \n")
    println("-" ^ 70)

    # --- save results ---------------------------------------------------------
    RBUS = nothing; RGEN = nothing; RCIR = nothing
    status_solved = status_model == OPTIMAL || status_model == LOCALLY_SOLVED ||
                    status_model == ITERATION_LIMIT || status_model == ALMOST_LOCALLY_SOLVED
    if status_solved
        println("Objective value: $(JuMP.value.(obj_function_MVA)) \n")

        RBUS, RGEN, RCIR = Save_Solution_Optimal_Dispatch(
            path_names, model, type_model, cfg.dispatch.use_matrix,
            obj_function_MVA, opf_dict,
            sys.bus_gen_circ_dict, sys.DBUS, sys.DGEN, sys.DCIR,
            cfg.base_MVA, sys.nBUS, sys.nGEN, sys.nCIR,
            sys.bus_mapping, sys.reverse_bus_mapping;
            _save_duals=cfg.save_duals)

        # Export the optimization matrices (Jacobian, Hessian, Lagrangian gradient)
        # as sparse COO CSV when requested. Already resolved to false for TSC runs.
        if save_optim_matrices_eff
            Compute_and_Save_Optimal_Matrices(model, path_names)
        end

        if cfg.trans_stab
            # Model-agnostic dynamic-results saver (handles both TSC-ACOPF and
            # TSC-DCOPF). Renamed from Save_Results_Dynamic_Model_tsred_linear (SF-2).
            Save_Results_Dynamic_Model(
                model, path_names, dyn_model_dict, dyn_parameters_dict,
                cfg.base_MVA, dyn_model_dict[:refs][:P_mech], sys.DGEN_DYN;
                save_ts_plots=cfg.save_ts_plots,
                save_ts_debug_csv=cfg.save_ts_debug_csv)
            if cfg.save_duals
                Save_Duals_Dynamic_Model_tsred(model, path_names, dyn_model_dict,
                    dyn_parameters_dict; save_ts_plots=cfg.save_ts_plots)
            end
        end

        # --- explicit dual LP (in-house builder, separate Dispatch_Dual/) ---
        if cfg.dispatch.solve_explicit_dual && type_model == "DCOPF"
            f_star = primal_dcopf_f_star
            if f_star === nothing
                f_star = objective_value(model)
            end
            Run_Explicit_DC_OPF_Dual!(
                path_names, cfg.dispatch, model, opf_dict,
                sys.DBUS, sys.DGEN, sys.DCIR, sys.bus_gen_circ_dict_ON,
                cfg.base_MVA, sys.nBUS, sys.nGEN, sys.nCIR;
                primal_f_star = f_star,
                solver_name = cfg.solver_name,
                silent_solver = cfg.silent_solver,
                highs = cfg.highs,
                gurobi = cfg.gurobi,
                cross_check_primal_duals = !cfg.trans_stab,
            )
        elseif cfg.dispatch.solve_explicit_dual && type_model == "ED"
            Run_Explicit_ED_Dual!(
                path_names, cfg.dispatch, model, opf_dict,
                sys.DGEN, sys.bus_gen_circ_dict_ON,
                cfg.base_MVA, sys.nGEN;
                primal_f_star = objective_value(model),
                solver_name = cfg.solver_name,
                silent_solver = cfg.silent_solver,
                highs = cfg.highs,
                gurobi = cfg.gurobi,
            )
        end
    else
        @warn "Optimisation did not converge (status: $status_model). Results not saved."
    end

    # --- save the input parameters used in this run --------------------------
    fault = cfg.transient === nothing ? nothing : cfg.transient.dyn_model.fault
    if dyn_parameters_dict !== nothing
        Print_Input_Parameters(path_names, cfg.overwrite_results, cfg.trans_stab,
            type_model, cfg.solver_name, cfg.dispatch.use_matrix, cfg.silent_solver,
            cfg.case, cfg.base_MVA, cfg.load_factor, save_optim_matrices_eff;
            fault=fault, dyn_parameters_dict=dyn_parameters_dict)
    else
        Print_Input_Parameters(path_names, cfg.overwrite_results, cfg.trans_stab,
            type_model, cfg.solver_name, cfg.dispatch.use_matrix, cfg.silent_solver,
            cfg.case, cfg.base_MVA, cfg.load_factor, save_optim_matrices_eff;
            fault=fault)
    end

    # Snapshot the objective BEFORE the backend is released. `release_solver_backend!`
    # calls `empty!(model)` on Ipopt paths (to free the solver log handle), which
    # invalidates every `VariableRef` — including the ones inside `obj_function_MVA`
    # and inside `dyn_model_dict`. Returning the live expression would hand the caller
    # a reference that throws on `JuMP.value`. Broadcast keeps the shape for the
    # scalar and container forms alike.
    obj_MVA_value = status_solved ? JuMP.value.(obj_function_MVA) : nothing

    release_solver_backend!(model)
    model = nothing

    return (; status=status_model, obj_MVA=obj_MVA_value,
              RBUS, RGEN, RCIR, dyn_model_dict, dyn_parameters_dict,
              t_build, t_solve, path_names)
end
