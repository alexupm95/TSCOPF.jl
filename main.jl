#=
CODE FOR SOLVING THE OPTIMAL DISPATCH OF ENERGY
ADDITIONAL SECURITY CONSTRAINTS MAY BE ADDED

Author:      Alex Junior da Cunha Coelho
Affiliation: Technical University of Madrid
February 2026

--------------------------------------------------------------------------------
 SINGLE-RUN entry point (Phase 1 refactor).
 This script is now a thin wrapper: it sets up the environment, builds ONE
 RunConfig, loads the system, and calls run_case! (defined in engine.jl).
 For a δ_tol (δ_max) sweep, use main_loop.jl instead.
--------------------------------------------------------------------------------
=#

using Pkg
Pkg.activate(@__DIR__)
using TSCOPF
Clean_Terminal()

# =======================================================
#   RUN CONFIGURATION  ←  edit everything for the run here
# =======================================================
# Avenue 1 — steady-state dispatch (OPF builder toggles on DispatchConfig).
# Avenue 2 — transient analysis (required when trans_stab=true).
#
# Optional: Ipopt + PARDISO linear solver (licensed libpardiso.dll):
#   solver_name = "Ipopt-pardiso",
#   ipopt = IpoptSolverConfig(pardiso_lib_path = raw"C:\Libraries\Pardiso\lib\libpardiso.dll"),
cfg = RunConfig(
    trans_stab          = true,
    case                = "9bus",
    base_MVA            = 100.0,
    load_factor         = 1.5,
    solver_name         = "Ipopt",
    silent_solver       = false,
    overwrite_results   = false,

    # --- saver toggles -------------------------------------------------------
    save_duals          = true,
    save_matrices       = true,    # Ybus/Bbus XLSX + fault matrices
    save_ts_plots       = true,    # trajectory SVG figures (Plots.jl)
    save_optim_matrices = true,    # auto-forced false for TSC (warning printed)

    # --- avenue 1: steady-state ACOPF (matrix form) --------------------------
    dispatch = DispatchConfig(
        type_model          = "ACOPF",
        cost_type            = "quadratic",
        use_matrix          = true,
        ineq_sbranch_upper  = true,   # branch thermal limits
        ineq_ang_diff_branch = true,  # branch angle-difference limits
    ),

    # --- avenue 2: TSC — FULL_BUS, USE_PM, COI box, pure constant-Z loads ---
    transient = TransientConfig(
        simulation = TsSimulationConfig(δ_tol_deg = 100.0),
        builder = TsBuilderConfig(
            # ── pre-fault steady-state variable bounds ─────────────────────────
            bound_E   = true,   # internal voltage magnitude E (per generator)
            bound_δ   = true,   # rotor angle δ  (−π … +π when true)
            bound_P_m = true,   # mechanical power Pm (pg_min … pg_max when true)

            # ── fault-period variable bounds ───────────────────────────────────
            bound_δ_tf    = false,   # δ during fault
            bound_Δω_tf   = false,   # speed deviation during fault
            bound_Pe_tf   = false,   # electrical power during fault
            bound_Qe_tf   = false,   # electrical reactive power during fault
            bound_δCOI_tf = false,   # COI angle during fault

            # ── post-fault variable bounds ─────────────────────────────────────
            bound_δ_tpf    = false,
            bound_Δω_tpf   = false,
            bound_Pe_tpf   = false,
            bound_Qe_tpf   = false,
            bound_δCOI_tpf = false,

            # Pre-fault P/Q/Pm init equalities are always-on physics (not toggles).

            # ── δ-COI stability bounds (separate from bound_style on DynModelConfig)
            ineq_δ_COI_tf_lower  = true,   # δ_i − δ_COI ≥ −δ_tol
            ineq_δ_COI_tf_upper  = true,   # δ_i − δ_COI ≤ +δ_tol
            ineq_δ_COI_tpf_lower = true,
            ineq_δ_COI_tpf_upper = true,

            # ── bound limit values (physical defaults; optional tf/tpf default unbounded) ──
            limits = TsBoundLimitsConfig(
                E_min_pu = 0.0,
                E_max_pu = 2.0,
                δ_min_rad = -π,
                δ_max_rad = π,
                P_m_source = :dgen_pg_limits,
                V_bus_min_pu = 0.0,
                gov_valve_min_pu = 0.0,
                gov_valve_max_source = :dgen_pg_limits,
            ),
        ),
        dyn_model  = DynModelConfig(
            network_form      = FULL_BUS,
            mech_power_mode   = USE_PM,
            bound_style       = :coi_box,
            # Independent (Z, I, P) splits for active and reactive demand. Both pure
            # constant impedance here; e.g. REE style would be zip_load_p = (0.0, 1.0, 0.0)
            # (constant current) with zip_load_q = (1.0, 0.0, 0.0) (constant admittance).
            zip_load_p        = (1.0, 0.0, 0.0),  # (Z, I, P) — active demand
            zip_load_q        = (1.0, 0.0, 0.0),  # (Z, I, P) — reactive demand
            constrain_Δω_COI  = false,
            fault             = FaultConfig(fault_type = SC, contingency_id = 2),
        ),
        
    ),
)

# =======================================================
#   RUN
# =======================================================
path_main           = project_root()
path_folder_results = default_results_dir()

sys    = load_system(cfg, path_main)
result = run_case!(cfg, sys, path_main, path_folder_results)

println("\nDone. Termination status: $(result.status)")
