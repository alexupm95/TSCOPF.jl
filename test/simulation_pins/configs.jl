#=
 RunConfig builders aligned with _TSCOPF_simulations bound_compare_* drivers.

 Input CSVs resolve from `simulations_project_root()`:
   ENV["TSCOPF_SIMULATIONS_ROOT"] or sibling `../_TSCOPF_simulations`.
=#

# `simulations_project_root` lives in test/test_paths.jl, alongside the other data
# roots. The availability check below stays here because it encodes which cases
# *these* configs need.
function simulations_input_available(project_root::String)
    root = simulations_project_root(project_root)
    root == "" && return false
    for case in (
        "9bus_4th_order_fullbus_AVR",
        "9bus_4th_order_fullbus_AVR_TG",
        "9bus_2nd_order_fullbus",
        "9bus_4th_order_fullbus",
    )
        isdir(joinpath(root, "INPUT_FILES", case)) || return false
    end
    return true
end

"""Shared simulation timing (matches tsc_run_common.jl defaults)."""
function sim_fullbus_simulation_config(;
    t_end_sim::Float64 = 5.0,
    t_step::Float64 = 0.01,
)
    return TsSimulationConfig(
        δ_tol_deg = 100.0,
        t_start_sim = 0.0,
        t_end_sim = t_end_sim,
        t_step = t_step,
        t_start_fault = 0.01,
        clearing_time = 0.3,
        f_syn = 50.0,
    )
end

function sim_fullbus_builder_config(;
    E_max_pu::Float64 = 2.0,
    include_governor_limits::Bool = false,
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
)
    base_limits = (
        E_min_pu = 0.0,
        E_max_pu = E_max_pu,
        δ_min_rad = -π,
        δ_max_rad = π,
        P_m_source = :dgen_pg_limits,
        V_bus_min_pu = 0.0,
    )
    limits = if include_governor_limits
        TsBoundLimitsConfig(;
            base_limits...,
            gov_valve_min_pu = 0.0,
            gov_valve_max_source = :dgen_pg_limits,
        )
    else
        TsBoundLimitsConfig(; base_limits...)
    end
    return TsBuilderConfig(
        bound_E = true,
        bound_δ = true,
        bound_P_m = true,
        ineq_δ_COI_tf_lower = true,
        ineq_δ_COI_tf_upper = true,
        ineq_δ_COI_tpf_lower = true,
        ineq_δ_COI_tpf_upper = true,
        limits = limits,
        bound_encoding = bound_encoding,
    )
end

function _sim_run_io()
    return (
        save_duals = true,
        save_matrices = false,
        save_ts_plots = false,
        save_optim_matrices = false,
        save_warmstart_dispatch = true,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        silent_solver = true,
    )
end

"""
DQ_4TH FULL_BUS + AVR, t_end=5 s (bound_compare_4th_fullbus_avr).

The pins were extracted under `CONSTRAINT` encoding and that is what the suite asserts.
For interactive re-runs of this case, `VARIABLE` converges more reliably (it is the
encoding the production AVR driver uses) and reaches the same optimum within `obj_rtol`;
pass `bound_encoding = TSCOPF.VARIABLE` if a manual re-solve stalls.
"""
function sim_config_4th_fullbus_avr(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "Ipopt",
)
    return RunConfig(;
        trans_stab = true,
        case = "9bus_4th_order_fullbus_AVR",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        _sim_run_io()...,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = false,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data.csv",
            simulation = sim_fullbus_simulation_config(),
            builder = sim_fullbus_builder_config(E_max_pu = 4.5, bound_encoding = bound_encoding),
            dyn_model = DynModelConfig(
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                include_avr = true,
                include_governor = false,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
end

"""DQ_4TH FULL_BUS core, t_end=1 s (bound_compare_4th_fullbus)."""
function sim_config_4th_fullbus(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "Ipopt",
)
    return RunConfig(;
        trans_stab = true,
        case = "9bus_4th_order_fullbus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        _sim_run_io()...,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = false,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data.csv",
            simulation = sim_fullbus_simulation_config(t_end_sim = 1.0),
            builder = sim_fullbus_builder_config(bound_encoding = bound_encoding),
            dyn_model = DynModelConfig(
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                include_avr = false,
                include_governor = false,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
end

"""CLASSICAL_2ND FULL_BUS, t_end=1 s (bound_compare_2nd_fullbus).

Baseline in simulations used `solver_name = \"Ipopt-pardiso\"`; tests prefer Ipopt when
PARDISO is unavailable (see `sim_config_2nd_fullbus_solver`).
"""
function sim_config_2nd_fullbus(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "Ipopt",
    ipopt::IpoptSolverConfig = IpoptSolverConfig(),
)
    cfg_kwargs = (
        trans_stab = true,
        case = "9bus_2nd_order_fullbus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        _sim_run_io()...,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = false,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data.csv",
            simulation = sim_fullbus_simulation_config(t_end_sim = 1.0),
            builder = sim_fullbus_builder_config(bound_encoding = bound_encoding),
            dyn_model = DynModelConfig(
                gen_order = CLASSICAL_2ND,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                include_avr = false,
                include_governor = false,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
    if solver_name == "Ipopt-pardiso"
        return RunConfig(; cfg_kwargs..., ipopt = ipopt)
    end
    return RunConfig(; cfg_kwargs...)
end

"""Pick solver for 2nd-order pin test: PARDISO when lib available, else Ipopt."""
function sim_config_2nd_fullbus_solver()
    pardiso_path = get(ENV, "JULIA_PARDISO_LIB", "")
    if !isempty(pardiso_path) && isfile(pardiso_path) && pardiso_available()
        return sim_config_2nd_fullbus(;
            solver_name = "Ipopt-pardiso",
            ipopt = IpoptSolverConfig(;
                tol = 1e-8,
                constr_viol_tol = 1e-8,
                dual_inf_tol = 1e-8,
                compl_inf_tol = 1e-8,
                pardiso_lib_path = pardiso_path,
            ),
        )
    end
    return sim_config_2nd_fullbus(solver_name = "Ipopt")
end

const SIMULATION_PIN_SOLVED = (OPTIMAL, LOCALLY_SOLVED, ITERATION_LIMIT)

"""Kron I/O: no pre-solve on this path, so no Dispatch_WarmStart folder (matches bound_compare_2nd_kron)."""
function _sim_kron_run_io()
    return (
        save_duals = true,
        save_matrices = false,
        save_ts_plots = false,
        save_optim_matrices = false,
        save_warmstart_dispatch = false,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        silent_solver = true,
    )
end

"""CLASSICAL_2ND Kron, t_end=1 s, ACOPF matrix (bound_compare_2nd_kron)."""
function sim_config_2nd_kron(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "Ipopt",
    ipopt::IpoptSolverConfig = IpoptSolverConfig(),
)
    cfg_kwargs = (
        trans_stab = true,
        case = "9bus_2nd_order_fullbus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        _sim_kron_run_io()...,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = true,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data.csv",
            simulation = sim_fullbus_simulation_config(t_end_sim = 1.0),
            builder = sim_fullbus_builder_config(bound_encoding = bound_encoding),
            dyn_model = DynModelConfig(
                gen_order = CLASSICAL_2ND,
                network_form = KRON_REDUCED,
                # Recorded before Kron honoured `bound_style_δ`: USE_PM + :coi_box then
                # built the swing-propagated bound. USE_PG + :swing_propagated names
                # that same model, so the pins keep their meaning.
                mech_power_mode = USE_PG,
                bound_style_δ = :swing_propagated,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                include_avr = false,
                include_governor = false,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
    if solver_name == "Ipopt-pardiso"
        return RunConfig(; cfg_kwargs..., ipopt = ipopt)
    end
    return RunConfig(; cfg_kwargs...)
end

function sim_config_2nd_kron_solver()
    pardiso_path = get(ENV, "JULIA_PARDISO_LIB", "")
    if !isempty(pardiso_path) && isfile(pardiso_path) && pardiso_available()
        return sim_config_2nd_kron(;
            solver_name = "Ipopt-pardiso",
            ipopt = IpoptSolverConfig(;
                tol = 1e-8,
                constr_viol_tol = 1e-8,
                dual_inf_tol = 1e-8,
                compl_inf_tol = 1e-8,
                pardiso_lib_path = pardiso_path,
            ),
        )
    end
    return sim_config_2nd_kron(solver_name = "Ipopt")
end

const SIM_GL_GEN3_IDS = [3]

"""DQ_4TH FULL_BUS GL gen trip (gen 3), t_end=5 s (bound_compare_gl_gen3_4th_fullbus)."""
function sim_config_gl_gen3_4th_fullbus(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "Ipopt",
)
    return RunConfig(;
        trans_stab = true,
        case = "9bus_4th_order_fullbus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        _sim_run_io()...,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = false,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data.csv",
            simulation = sim_fullbus_simulation_config(t_end_sim = 5.0),
            builder = sim_fullbus_builder_config(bound_encoding = bound_encoding),
            dyn_model = DynModelConfig(
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                include_avr = false,
                include_governor = false,
                fault = FaultConfig(fault_type = GL, gl_gen_ids = SIM_GL_GEN3_IDS),
            ),
        ),
    )
end

"""CLASSICAL_2ND Kron GL gen trip (gen 3), t_end=5 s (bound_compare_gl_gen3_kron)."""
function sim_config_gl_gen3_kron(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "Ipopt",
)
    return RunConfig(;
        trans_stab = true,
        case = "9bus_2nd_order_fullbus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        _sim_kron_run_io()...,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = true,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data.csv",
            simulation = sim_fullbus_simulation_config(t_end_sim = 5.0),
            builder = sim_fullbus_builder_config(bound_encoding = bound_encoding),
            dyn_model = DynModelConfig(
                gen_order = CLASSICAL_2ND,
                network_form = KRON_REDUCED,
                # See sim_config_2nd_kron: names the model the pins were recorded on.
                mech_power_mode = USE_PG,
                bound_style_δ = :swing_propagated,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                include_avr = false,
                include_governor = false,
                fault = FaultConfig(fault_type = GL, gl_gen_ids = SIM_GL_GEN3_IDS),
            ),
        ),
    )
end

"""DQ_4TH FULL_BUS + AVR + TGOV1 (reference Gvnr case; Results - 2026-07-14 baseline).

Uses `bound_encoding = VARIABLE` and `use_matrix = true` as in
`main_4th_order_fullbus_AVR_TG.jl` (not yet in bound_compare suite).
"""
function sim_config_4th_fullbus_avr_tg(;
    bound_encoding::BoundEncoding = TSCOPF.VARIABLE,
    solver_name::String = "Ipopt",
)
    return RunConfig(;
        trans_stab = true,
        case = "9bus_4th_order_fullbus_AVR_TG",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        save_duals = true,
        save_matrices = false,
        save_ts_plots = false,
        save_optim_matrices = false,
        save_warmstart_dispatch = true,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        silent_solver = true,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = true,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data.csv",
            simulation = sim_fullbus_simulation_config(),
            builder = sim_fullbus_builder_config(;
                E_max_pu = 4.5,
                include_governor_limits = true,
                bound_encoding = bound_encoding,
            ),
            dyn_model = DynModelConfig(
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                include_avr = true,
                include_governor = true,
                governor_limiter = GOV_SMOOTH,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
end

"""DQ_4TH FULL_BUS + TGOV1 only, smooth limits, t_end=5 s (bound_compare_4th_fullbus_tg)."""
function sim_config_4th_fullbus_tg(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "Ipopt",
)
    return RunConfig(;
        trans_stab = true,
        case = "9bus_4th_order_fullbus_TG",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        save_duals = true,
        save_matrices = false,
        save_ts_plots = false,
        save_optim_matrices = false,
        save_warmstart_dispatch = true,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        time_limit_sec = 1200.0,
        silent_solver = true,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = false,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data.csv",
            simulation = sim_fullbus_simulation_config(),
            builder = sim_fullbus_builder_config(;
                E_max_pu = 4.5,
                include_governor_limits = true,
                bound_encoding = bound_encoding,
            ),
            dyn_model = DynModelConfig(
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                include_avr = false,
                include_governor = true,
                governor_limiter = GOV_SMOOTH,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
end

"""Dispatch-only run I/O (no transient stability; keep CSV outputs)."""
function _dispatch_run_io()
    return (
        save_duals = true,
        save_matrices = true,
        save_ts_plots = false,
        save_optim_matrices = false,
        # Not applicable when `trans_stab=false`: there is no pre-solve to archive.
        save_warmstart_dispatch = false,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        time_limit_sec = 1200.0,
        silent_solver = true,
    )
end

"""ACOPF dispatch-only pins (matches `main_acopf_matrix.jl`: use_matrix=true)."""
function sim_config_acopf_dispatch_constraint(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "Ipopt",
)
    return RunConfig(;
        trans_stab = false,
        case = "9bus_2nd_order_fullbus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        _dispatch_run_io()...,
        dispatch = DispatchConfig(
            type_model = "ACOPF",
            cost_type = "quadratic",
            use_matrix = true,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = nothing,
    )
end

"""DCOPF dispatch-only pins (matches `main_dcopf_matrix.jl`: use_matrix=true)."""
function sim_config_dcopf_dispatch_constraint(;
    bound_encoding::BoundEncoding = TSCOPF.CONSTRAINT,
    solver_name::String = "HiGHS",
)
    return RunConfig(;
        trans_stab = false,
        case = "9bus_2nd_order_fullbus",
        base_MVA = 100.0,
        load_factor = 1.5,
        solver_name = solver_name,
        _dispatch_run_io()...,
        dispatch = DispatchConfig(
            type_model = "DCOPF",
            cost_type = "linear",
            use_matrix = true,
            ineq_sbranch_upper = true,
            ineq_ang_diff_branch = true,
            bound_encoding = bound_encoding,
        ),
        transient = nothing,
    )
end
