#=
================================================================================
 13 — TSC-ACOPF — FULL_BUS + DQ_4TH, GL disturbance (generator trip)
================================================================================
  Avenue:   `trans_stab = true` with `type_model = "ACOPF"`.
  Case:     INPUT_FILES/9bus/  (+ gen_dynamic_data_full.csv, contingencies.csv)
  Fault:    GL — generator 3 disconnects at t = 0.01 s and never returns.
  Solver:   Ipopt (NLP).
  Results:  RESULTS/Results - <timestamp>/{Dispatch,Transient_Stability}/

  Same model as example 08; different disturbance. GL (generation/load) is not a
  short circuit: nothing faults, a unit simply leaves. The surviving machines
  must absorb its power on inertia alone, so this is the disturbance that makes
  the frequency corridor interesting rather than the angle corridor.

  Timing differs from SC in a way worth internalising. An SC run has two windows
  — fault, then post-fault after `clearing_time`. A GL run has ONE window
  (`time_windows_gld`): the trip happens at `t_start_fault` and there is no
  reconnection, so `clearing_time` is simply not read on this path.

  The same FaultConfig also does load disturbances, through two parallel vectors:
      gl_load_bus_ids  = [5]
      gl_percent_power = [-1.0]     α = -1 → full trip of the load at bus 5
  with p_d_new = p_d * (1 + α). So α = -0.5 sheds half, α = 0 changes nothing,
  α = +1 doubles it. Generator and load entries may be combined in one run.

  Disturbances are applied to a `deepcopy` of the input DataFrames — the CSVs and
  the loaded `SystemData` are never mutated in place.

  Note: `contingencies.csv` must exist in the case folder even though a GL run
  never reads it. The input-archival step copies it unconditionally
  (_manage_outputs/functions_2_save_input_parameters.jl:50-56).

  Knob families omitted here, with the example that shows them: AVR set-points
  and boxes (09), governor valve limits and boxes (07, 10), GFM (12).
================================================================================
=#

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))   # activate the repo project, not examples/

using TSCOPF

const PATH_MAIN    = project_root()          # repo root — the folder that contains INPUT_FILES/
const PATH_RESULTS = default_results_dir()   # <repo root>/RESULTS
mkpath(PATH_RESULTS)                         # create the results tree on first run

# (-Inf, Inf) = inactive. An optional box is built only when its `bound_*` toggle
# is on AND the limit is finite; Inf never enters a ≤-row or a JuMP bound.
const OPEN = TsBoundLimitPair(min = -Inf, max = Inf)   # the "no limit on either side" pair

# ── Steady-state limit values (DispatchLimitsConfig) ──────────────────────────
const dispatch_limits = DispatchLimitsConfig(
    θ_min_rad             = -π,     # lower bus-angle bound [rad]; -π = one full turn, effectively unrestricted
    θ_max_rad             = π,      # upper bus-angle bound [rad]; π = one full turn, effectively unrestricted
    ang_diff_clamp_ac_deg = 60.0,   # the active clamp here: DCIR angmin/angmax wider than ±60° are tightened to it
    ang_diff_clamp_dc_deg = 30.0,   # DC-path clamp; read only when type_model = "DCOPF"
)

# ── Steady-state OPF builder (DispatchConfig) ─────────────────────────────────
const dispatch_cfg = DispatchConfig(
    type_model           = "ACOPF",       # "ACOPF" + trans_stab = true selects the nonlinear TSC path
    use_matrix           = true,          # true = one Ybus matrix balance row per bus; false = branch-by-branch summation
    cost_type            = "quadratic",   # "quadratic" = c2·P² + c1·P + c0; this case has c2 = 0, so it is linear in practice
    susceptance_model    = SIMPLE,        # SIMPLE = b_ik = 1/x; read only on the DC-OPF path, so inert here
    bound_V              = true,          # true = build V_min ≤ V ≤ V_max rows from bus_data.csv
    bound_θ              = true,          # true = build the θ box from limits above
    bound_P_g            = true,          # true = build Pmin ≤ P_g ≤ Pmax rows from generators_data.csv
    bound_Q_g            = true,          # true = build Qmin ≤ Q_g ≤ Qmax rows
    bound_P_ik           = false,         # false = no explicit box on the from-side active branch flow
    bound_Q_ik           = false,         # false = no explicit box on the from-side reactive branch flow
    bound_P_ki           = false,         # false = no explicit box on the to-side active branch flow
    bound_Q_ki           = false,         # false = no explicit box on the to-side reactive branch flow
    ineq_sg_upper        = false,         # false = no |S_g| ≤ S_max generator capability circle
    ineq_sbranch_upper   = true,          # true = |S_ik| ≤ l_cap_1 thermal limit on every rated branch
    ineq_ang_diff_branch = true,          # true = enforce per-branch angle-difference limits
    solve_explicit_dual  = false,         # false is mandatory here: the dual LP is ED / DC-OPF + linear cost only
    limits               = dispatch_limits,     # the DispatchLimitsConfig instance built above
    bound_encoding       = TSCOPF.CONSTRAINT,   # CONSTRAINT = explicit ≤-rows with clean duals; VARIABLE = JuMP bounds instead
)

# ── Transient timing (TsSimulationConfig) ─────────────────────────────────────
const ts_simulation = TsSimulationConfig(
    δ_tol_deg       = 100.0,   # half-width of the |δ_i − δ_COI| stability corridor [deg]
    δ_tol_deg_lower = nothing, # nothing = reuse δ_tol_deg below the COI; a number makes the corridor asymmetric
    δ_tol_deg_upper = nothing, # nothing = reuse δ_tol_deg above the COI
    t_start_sim     = 0.0,     # first instant of the simulated horizon [s]
    t_end_sim       = 0.6,     # last instant [s]; one continuous window on the GL path, no clearing split
    t_step          = 0.02,    # integration step [s]; 30 steps over this horizon
    t_start_fault   = 0.01,    # the instant generator 3 disconnects [s]
    clearing_time   = 0.2,     # NOT read on the GL path: there is nothing to clear and no reconnection
    f_syn           = 50.0,    # synchronous frequency [Hz]; sets ω_0 = 2π·50 in the swing equation
    Δω_0            = 0.0,     # initial speed deviation [pu]; 0.0 = start exactly at synchronous speed
)

# ── Transient limit values (TsBoundLimitsConfig) ──────────────────────────────
const ts_limits = TsBoundLimitsConfig(
    # Pre-fault physical
    E_min_pu     = 0.0,               # lower bound on the pre-fault internal EMF [pu]; 0.0 = no practical floor
    E_max_pu     = 2.0,               # upper bound on the pre-fault internal EMF [pu]
    δ_min_rad    = -π,                # lower bound on the pre-fault rotor angle [rad]
    δ_max_rad    = π,                 # upper bound on the pre-fault rotor angle [rad]
    P_m_source   = :dgen_pg_limits,   # :dgen_pg_limits = take the P_m box from DGEN Pmin/Pmax ÷ base_MVA
    V_bus_min_pu = 0.0,               # forced floor on trajectory bus voltages [pu]; 0.0 = no floor
    # Optional shared tf–tpf boxes (inactive: OPEN + toggle off below)
    δ_tf     = OPEN,   # rotor angle, disturbance window — inactive
    Δω_tf    = OPEN,   # speed deviation, disturbance window — inactive
    Pe_tf    = OPEN,   # electrical power, disturbance window — inactive
    Qe_tf    = OPEN,   # reactive power, disturbance window — inactive
    δCOI_tf  = OPEN,   # centre-of-inertia angle, disturbance window — inactive
    δ_tpf    = OPEN,   # rotor angle, second window — inactive (GL uses a single window)
    Δω_tpf   = OPEN,   # speed deviation, second window — inactive
    Pe_tpf   = OPEN,   # electrical power, second window — inactive
    Qe_tpf   = OPEN,   # reactive power, second window — inactive
    δCOI_tpf = OPEN,   # centre-of-inertia angle, second window — inactive
    # DQ pre-fault algebraic states (applied only if bound_Ed/Eq/Id/Iq are on)
    Ed_min_pu = -2.0,   # lower bound on the pre-fault d-axis EMF E'd [pu]
    Ed_max_pu = 2.0,    # upper bound on the pre-fault d-axis EMF E'd [pu]
    Eq_min_pu = -2.0,   # lower bound on the pre-fault q-axis EMF E'q [pu]
    Eq_max_pu = 2.0,    # upper bound on the pre-fault q-axis EMF E'q [pu]
    Id_min_pu = -5.0,   # lower bound on the pre-fault d-axis stator current [pu]
    Id_max_pu = 5.0,    # upper bound on the pre-fault d-axis stator current [pu]
    Iq_min_pu = -5.0,   # lower bound on the pre-fault q-axis stator current [pu]
    Iq_max_pu = 5.0,    # upper bound on the pre-fault q-axis stator current [pu]
    # DQ machine states, both windows (inactive)
    Ed_tf  = OPEN,   # d-axis EMF, disturbance window — inactive
    Eq_tf  = OPEN,   # q-axis EMF, disturbance window — inactive
    Id_tf  = OPEN,   # d-axis current, disturbance window — inactive
    Iq_tf  = OPEN,   # q-axis current, disturbance window — inactive
    Te_tf  = OPEN,   # electrical torque, disturbance window — inactive
    Ed_tpf = OPEN,   # d-axis EMF, second window — inactive
    Eq_tpf = OPEN,   # q-axis EMF, second window — inactive
    Id_tpf = OPEN,   # d-axis current, second window — inactive
    Iq_tpf = OPEN,   # q-axis current, second window — inactive
    Te_tpf = OPEN,   # electrical torque, second window — inactive
)

# ── Transient builder toggles (TsBuilderConfig) ───────────────────────────────
const ts_builder = TsBuilderConfig(
    # Pre-fault boxes
    bound_E   = true,    # true = build the E box from E_min_pu / E_max_pu above
    bound_δ   = true,    # true = build the pre-fault δ box from δ_min_rad / δ_max_rad
    bound_P_m = true,    # true = build the mechanical-power box from P_m_source
    # Optional shared tf / tpf boxes — each needs BOTH this toggle and a finite pair above
    bound_δ_tf     = false,   # false = no ≤-row on the rotor angle through the disturbance window
    bound_Δω_tf    = false,   # false = no ≤-row on the speed deviation through the disturbance window
    bound_Pe_tf    = false,   # false = no ≤-row on the electrical power through the disturbance window
    bound_Qe_tf    = false,   # false = no ≤-row on the reactive power through the disturbance window
    bound_δCOI_tf  = false,   # false = no ≤-row on the COI angle through the disturbance window
    bound_δ_tpf    = false,   # false = no ≤-row on the rotor angle in the second window
    bound_Δω_tpf   = false,   # false = no ≤-row on the speed deviation in the second window
    bound_Pe_tpf   = false,   # false = no ≤-row on the electrical power in the second window
    bound_Qe_tpf   = false,   # false = no ≤-row on the reactive power in the second window
    bound_δCOI_tpf = false,   # false = no ≤-row on the COI angle in the second window
    # DQ pre-fault algebraic states
    bound_Ed = false,   # false = E'd is free at the pre-fault point; true applies Ed_min_pu / Ed_max_pu
    bound_Eq = false,   # false = E'q is free at the pre-fault point; true applies Eq_min_pu / Eq_max_pu
    bound_Id = false,   # false = Id is free at the pre-fault point; true applies Id_min_pu / Id_max_pu
    bound_Iq = false,   # false = Iq is free at the pre-fault point; true applies Iq_min_pu / Iq_max_pu
    # DQ machine states — disturbance window
    bound_Ed_tf = false,   # false = no ≤-row on E'd through the disturbance window
    bound_Eq_tf = false,   # false = no ≤-row on E'q through the disturbance window
    bound_Id_tf = false,   # false = no ≤-row on Id through the disturbance window
    bound_Iq_tf = false,   # false = no ≤-row on Iq through the disturbance window
    bound_Te_tf = false,   # false = no ≤-row on the electrical torque through the disturbance window
    # DQ machine states — second window
    bound_Ed_tpf = false,  # false = no ≤-row on E'd in the second window
    bound_Eq_tpf = false,  # false = no ≤-row on E'q in the second window
    bound_Id_tpf = false,  # false = no ≤-row on Id in the second window
    bound_Iq_tpf = false,  # false = no ≤-row on Iq in the second window
    bound_Te_tpf = false,  # false = no ≤-row on the electrical torque in the second window
    # δ–COI stability inequalities — the transient-stability constraint proper
    ineq_δ_COI_tf_lower  = true,   # true = enforce δ_i − δ_COI ≥ −δ_tol through the disturbance window
    ineq_δ_COI_tf_upper  = true,   # true = enforce δ_i − δ_COI ≤ +δ_tol through the disturbance window
    ineq_δ_COI_tpf_lower = true,   # true = same lower corridor in the second window
    ineq_δ_COI_tpf_upper = true,   # true = same upper corridor in the second window
    limits         = ts_limits,             # the TsBoundLimitsConfig instance built above
    bound_encoding = TSCOPF.CONSTRAINT,     # CONSTRAINT = boxes become ≤-rows with their own duals
)

# ── Disturbance (FaultConfig) ─────────────────────────────────────────────────
# GL: generator 3 trips at t_start_fault and never reconnects.
const fault_cfg = FaultConfig(
    fault_type       = GL,          # GL = generation/load disturbance; no short circuit, no clearing stage
    contingency_id   = 2,           # SC-only field: not read on this path
    # Generator trip: DGEN row indices. Several units may go at once.
    gl_gen_ids       = [3],         # trip generator 3; an empty vector would mean no unit leaves
    # Load change on the same disturbance: parallel vectors, one α per bus.
    # p_d_new = p_d * (1 + α). Example: gl_load_bus_ids = [5],
    #                                   gl_percent_power = [-1.0]  → full trip at bus 5.
    gl_load_bus_ids  = Int[],       # empty = no load bus is disturbed
    gl_percent_power = Float64[],   # empty = no scaling factors; must stay the same length as gl_load_bus_ids
    ob_branch_ids    = Int[],       # empty = no branch opened; read only when fault_type = OB (see example 14)
)

# ── Dynamic physics and network form (DynModelConfig) ─────────────────────────
const dyn_model_cfg = DynModelConfig(
    gen_order               = DQ_4TH,           # 4th-order dq machine: δ, Δω plus the flux-decay states E'd and E'q
    network_form            = FULL_BUS,         # required by DQ_4TH; sparse Ybus + nodal KCL at every bus
    mech_power_mode         = USE_PM,           # required by DQ_4TH: P_m is its own variable
    dq_speed_dev_in_algebra = true,             # true = keep the (1+Δω) factor in Pe and the stator algebra (RMS convention)
    include_avr             = false,            # legal to set true here; see example 09
    include_governor        = false,            # legal to set true here; see example 10
    governor_limiter        = GOV_NO_LIMIT,     # valve saturation mode; read only when include_governor = true
    allow_gfm               = false,            # legal to set true here; see example 12
    ode_first_step          = :trapezoidal,     # scheme for the first row of each window; :backward_euler matches the reference
    gfm_integrator          = :backward_euler,  # discretisation of the GFM filters / Q–V PI; read only when allow_gfm = true
    # (Z, I, P) splits — real physics on FULL_BUS. (1,0,0) = constant impedance.
    zip_load_p              = (1.0, 0.0, 0.0),  # active demand: 100 % constant impedance (∝ V²), 0 % current, 0 % power
    zip_load_q              = (1.0, 0.0, 0.0),  # reactive demand: same split, set independently of the active one
    bound_style             = :coi_box,         # required by DQ_4TH: the stability limit is a direct corridor around the COI
    # A generator trip is a frequency event first. Switching this on puts a
    # corridor on Δω_i − Δω_COI, which is the natural constraint for GL.
    constrain_Δω_COI        = false,            # false = no frequency corridor is built
    Δω_tol_pu               = 0.5,              # half-width of that Δω corridor [pu]; read only when constrain_Δω_COI = true
    Δω_tol_pu_lower         = nothing,          # nothing = reuse Δω_tol_pu below the COI
    Δω_tol_pu_upper         = nothing,          # nothing = reuse Δω_tol_pu above the COI
    fault                   = fault_cfg,        # the FaultConfig instance built above
)

# ── Transient bundle (TransientConfig) ────────────────────────────────────────
const transient_cfg = TransientConfig(
    simulation           = ts_simulation,   # the TsSimulationConfig instance built above
    builder              = ts_builder,      # the TsBuilderConfig instance built above
    dyn_model            = dyn_model_cfg,   # the DynModelConfig instance built above
    gen_dynamic_filename = "gen_dynamic_data_full.csv",   # DQ_4TH needs the full 17-column machine header
    gfm_dynamic_filename = "gfm_dynamic_data.csv",        # never opened here, since allow_gfm = false
)

# ── Solver option blocks (all four; `solver_name` picks the active one) ───────
const ipopt_cfg = IpoptSolverConfig(
    tol                        = 1e-8,       # overall convergence tolerance
    print_level                = 5,          # 0 = silent … 12 = maximum; silent_solver = true forces 0
    max_iter                   = 5_000,      # iteration cap; hitting it returns ITERATION_LIMIT
    constr_viol_tol            = 1e-8,       # largest primal constraint violation accepted
    dual_inf_tol               = 1e-8,       # largest dual infeasibility accepted
    compl_inf_tol              = 1e-8,       # largest complementarity violation accepted
    hessian_approximation      = "exact",    # "exact" = analytic second derivatives; "limited-memory" = L-BFGS
    limited_memory_max_history = 50,         # L-BFGS history depth; read only when "limited-memory"
    acceptable_tol             = nothing,    # nothing = leave Ipopt's relaxed-acceptance tolerance untouched
    obj_scaling_factor         = nothing,    # nothing = no manual objective scaling
    nlp_scaling_method         = nothing,    # nothing = Ipopt default; e.g. "gradient-based" to rescale rows
    pardiso_lib_path           = nothing,    # nothing = fall back to ENV["JULIA_PARDISO_LIB"]
    pardiso_license_message    = true,       # true = let PARDISO print its licence banner
)

const highs_cfg = HiGHSSolverConfig(
    output_flag                  = true,     # true = print the HiGHS solve log
    primal_feasibility_tolerance = 1e-8,     # largest primal violation accepted
    dual_feasibility_tolerance   = 1e-8,     # largest dual violation accepted
    ipm_optimality_tolerance     = 1e-8,     # interior-point optimality tolerance
    simplex_iteration_limit      = 5_000,    # cap on simplex iterations
    ipm_iteration_limit          = 5_000,    # cap on interior-point iterations
    solver                       = "choose", # "choose" = let HiGHS pick; unused here since TSC-ACOPF forbids HiGHS
    raw_options                  = Dict{String, Any}(),   # empty = pass no extra HiGHS options through
)

const gurobi_cfg = GurobiSolverConfig(
    output_flag       = 1,          # 1 = print the Gurobi log, 0 = silent
    feasibility_tol   = 1e-8,       # largest primal violation accepted
    optimality_tol    = 1e-8,       # reduced-cost tolerance for optimality
    bar_iter_limit    = 5_000,      # cap on barrier iterations (LP/QP)
    mip_gap           = 1e-8,       # relative MIP gap; only meaningful on integer models
    nl_bar_iter_limit = 5_000,      # cap on nonlinear barrier iterations (Gurobi 13+)
    nl_bar_p_feas_tol = 1e-8,       # nonlinear barrier primal feasibility tolerance
    nl_bar_d_feas_tol = 1e-8,       # nonlinear barrier dual feasibility tolerance
    nl_bar_c_feas_tol = 1e-8,       # nonlinear barrier complementarity tolerance
    optimality_target = 1,          # 1 = accept a local optimum on nonconvex models
    raw_options       = Dict{String, Any}(),   # empty = pass no extra Gurobi parameters through
)

const madnlp_cfg = MadNLPSolverConfig(
    tol                   = 1e-8,              # overall convergence tolerance
    max_iter              = 3_000,             # iteration cap
    print_level           = MADNLP_LOG_INFO,   # MADNLP_LOG_INFO = normal log; MADNLP_LOG_ERROR = errors only
    hessian_approximation = "exact",           # "exact" | "bfgs" | "compact-lbfgs"
    ma97_num_threads      = nothing,           # nothing = library default; set e.g. 8 for "MadNLP-ma97"
    raw_options           = Dict{String, Any}(),   # empty = pass no extra MadNLP options through
)

# ── Top-level run configuration (RunConfig) ───────────────────────────────────
const cfg = RunConfig(
    trans_stab = true,         # true = build the transient model on top of the dispatch; requires `transient` below

    case            = "9bus",  # folder name under INPUT_FILES/
    base_MVA        = 100.0,   # system power base [MVA]
    load_factor     = 1.5,     # multiplies every bus demand at load time; 1.0 = the CSV as written
    solver_name     = "Ipopt", # NLP required on this path: HiGHS and Gurobi are rejected
    silent_solver   = false,   # false = let the solver print; true forces print_level to 0
    time_limit_sec  = 1800.0,  # wall-clock cap handed to the solver [s]

    ipopt  = ipopt_cfg,        # applied when solver_name is an Ipopt backend — the active one here
    highs  = highs_cfg,        # applied when solver_name = "HiGHS"
    gurobi = gurobi_cfg,       # applied when solver_name = "Gurobi"
    madnlp = madnlp_cfg,       # applied when solver_name is a MadNLP backend

    overwrite_results       = false,  # false = write a fresh timestamped folder per run
    save_duals              = true,   # true = export the dispatch and transient duals
    save_optim_matrices     = false,  # false; a TSC run forces it false anyway (steady-state only feature)
    save_matrices           = true,   # true = dump Ybus, pre- and post-trip
    save_ts_plots           = false,  # false = no trajectory figures; true additionally needs load_plots_extension!()
    save_ts_debug_csv       = false,  # false = no per-step diagnostic dumps; those are GFM-specific anyway
    save_warmstart_dispatch = true,   # true = archive the pre-TS ACOPF solution to Dispatch_WarmStart/

    dispatch  = dispatch_cfg,   # the DispatchConfig instance built above; also defines the warm-start problem
    transient = transient_cfg,  # the TransientConfig instance built above; REQUIRED when trans_stab = true

    matpower_file = nothing,    # nothing = read the three CSVs; a filename would parse a .m case instead
)

validate_run_config!(cfg)   # checks the RunConfig-level combination (avenue, savers, solver blocks)
validate_dyn_config!(cfg)   # checks the dynamic-model combination (order, network, controls, ZIP sums)

println("Case:   ", input_files_dir(cfg.case))
println("Solver: ", cfg.solver_name)
println("Disturbance: GL — generators ", cfg.transient.dyn_model.fault.gl_gen_ids, " trip at ",
        cfg.transient.simulation.t_start_fault, " s (no reconnection)")
println("Solving ACOPF warm start + TSC-ACOPF (FULL_BUS, dq 4th order, GL gen trip)…")

sys    = load_system(cfg, PATH_MAIN)                       # read + scale the case data into a SystemData
result = run_case!(cfg, sys, PATH_MAIN, PATH_RESULTS)      # warm start → build → solve → export

println("\nTermination : ", result.status)
println("Objective   : ", result.obj_MVA)   # plain number, or nothing when unsolved — never JuMP.value it
println("Results     : ", result.path_names[:pf_results_date])
println("TS duals    : ", joinpath(result.path_names[:pf_TS], "CSV_duals"))
