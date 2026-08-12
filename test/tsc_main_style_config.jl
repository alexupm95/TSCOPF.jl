#=
 Shared RunConfig fragments mirroring `main.jl` TSC settings (9-bus, SC contingency 2).
 Used by `runtests_tsc_builder_kron.jl` and `runtests_tsc_builder_fullbus.jl`.
=#

"""`TsBuilderConfig` matching the production block in `main.jl`."""
function main_style_ts_builder(; kwargs...)
    base = (
        bound_E   = true,
        bound_δ   = true,
        bound_P_m = true,
        bound_δ_tf    = false,
        bound_Δω_tf   = false,
        bound_Pe_tf   = false,
        bound_Qe_tf   = false,
        bound_δCOI_tf = false,
        bound_δ_tpf    = false,
        bound_Δω_tpf   = false,
        bound_Pe_tpf   = false,
        bound_Qe_tpf   = false,
        bound_δCOI_tpf = false,
        ineq_δ_COI_tf_lower  = true,
        ineq_δ_COI_tf_upper  = true,
        ineq_δ_COI_tpf_lower = true,
        ineq_δ_COI_tpf_upper = true,
    )
    return TsBuilderConfig(; base..., kwargs...)
end

"""Base TSC `RunConfig` fields shared by Kron and FULL_BUS builder tests."""
function main_style_tsc_base(;
    network_form::NetworkForm,
    mech_power_mode::MechPowerMode,
    bound_style_δ::Symbol,
    builder::TsBuilderConfig=main_style_ts_builder(),
    constrain_δ::Bool=true,
    δ_ref_gen_id::Union{Nothing, Int}=nothing,
    constrain_Δω::Bool=false,
    bound_style_Δω::Symbol=:coi_box,
    Δω_tol_pu::Float64=0.5,
    fault::FaultConfig=FaultConfig(fault_type = SC, contingency_id = 2),
    kwargs...
)
    return RunConfig(;
        trans_stab = true,
        case = "9bus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = "Ipopt",
        silent_solver = true,
        # Measured on the GitHub runner: a full FULL_BUS TSC-ACOPF (build + solve)
        # takes ~33 s — the old "~11 min/solve, so allow 1800 s" note was wrong and
        # only meant a stalled solve burned half an hour before failing. 300 s is a
        # generous ceiling that still fails fast.
        time_limit_sec = 300.0,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        save_duals = false,
        save_matrices = false,
        save_ts_plots = false,
        save_optim_matrices = false,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = true,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
        ),
        transient = TransientConfig(
            # Shorter horizon keeps CI under the time budget while still covering
            # fault + post-fault windows (clearing at 0.3 s).
            simulation = TsSimulationConfig(δ_tol_deg = 100.0, t_end_sim = 1.0),
            builder = builder,
            dyn_model = DynModelConfig(
                network_form = network_form,
                mech_power_mode = mech_power_mode,
                constrain_δ = constrain_δ,
                bound_style_δ = bound_style_δ,
                δ_ref_gen_id = δ_ref_gen_id,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                constrain_Δω = constrain_Δω,
                bound_style_Δω = bound_style_Δω,
                Δω_tol_pu = Δω_tol_pu,
                fault = fault,
            ),
        ),
        kwargs...,
    )
end

function main_style_tsc_kron_config(; kwargs...)
    return main_style_tsc_base(;
        network_form = KRON_REDUCED,
        mech_power_mode = USE_PM,
        bound_style_δ = :coi_box,
        kwargs...,
    )
end

function main_style_tsc_fullbus_config(; kwargs...)
    return main_style_tsc_base(;
        network_form = FULL_BUS,
        mech_power_mode = USE_PM,
        bound_style_δ = :coi_box,
        kwargs...,
    )
end
