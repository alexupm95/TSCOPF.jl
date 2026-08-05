#=
RunConfig for the A6 numeric acceptance gate — mirrors `main.jl` (Feb 2026).

Test runs disable heavy savers (plots, Ybus XLSX, optim matrices) to keep the
regression focused on primal/dual numerics, not I/O side effects.
=#

"""FULL_BUS TSC-ACOPF 9-bus config aligned with `main.jl`."""
function a6_main_fullbus_run_config(; kwargs...)
    base = (
        trans_stab = true,
        case = "9bus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = "Ipopt",
        silent_solver = true,
        # Measured: ~33 s on the GitHub runner, ~2 min on a local Windows box.
        # 600 s is head-room, not a licence for a stalled solve to run for half an hour.
        time_limit_sec = 600.0,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        save_duals = true,
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
            simulation = TsSimulationConfig(δ_tol_deg = 100.0),
            builder = TsBuilderConfig(
                bound_E = true,
                bound_δ = true,
                bound_P_m = true,
                bound_δ_tf = false,
                bound_Δω_tf = false,
                bound_Pe_tf = false,
                bound_δCOI_tf = false,
                bound_δ_tpf = false,
                bound_Δω_tpf = false,
                bound_Pe_tpf = false,
                bound_δCOI_tpf = false,
                ineq_δ_COI_tf_lower = true,
                ineq_δ_COI_tf_upper = true,
                ineq_δ_COI_tpf_lower = true,
                ineq_δ_COI_tpf_upper = true,
            ),
            dyn_model = DynModelConfig(
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                constrain_Δω_COI = false,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
    return RunConfig(; base..., kwargs...)
end
