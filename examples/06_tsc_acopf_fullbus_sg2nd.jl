#=
================================================================================
 06 — TSC-ACOPF — FULL_BUS + CLASSICAL_2ND (no controls)
================================================================================
  Avenue:   `trans_stab = true` with `type_model = "ACOPF"`.
  Case:     INPUT_FILES/9bus/  (+ gen_dynamic_data.csv, contingencies.csv)
  Fault:    SC, contingency row 2 — three-phase fault at bus 7, cleared at 0.3 s
            by opening circuit 6 (branch 5–7).
  Solver:   Ipopt (NLP).
  Results:  RESULTS/Results - <timestamp>/{Dispatch,Transient_Stability}/

  Same classical 2nd-order machine as example 05, but the network is kept whole:
  a sparse Ybus with nodal KCL at every bus instead of a Kron reduction onto the
  machine internal nodes. Load buses stay in the model, so bus voltages are
  trajectory variables and the ZIP load split becomes real physics rather than a
  folded constant admittance.

  Three consequences worth knowing before comparing against 05:
    • An ACOPF pre-solve runs first and injects V/θ/P_g/Q_g as `start=` values on
      the dynamic variables. Its log is solver_log_warmstart.txt, separate from
      solver_log.txt. It is mandatory and unconditional — a flat start on FULL_BUS
      rarely converges — and `run_case!` throws if it does not reach optimality.
    • `mech_power_mode = USE_PM` is required (validated before any solve), which in
      turn forces `bound_style = :coi_box`.
    • Unlike the Kron avenue, `ode_first_step = :backward_euler` is honoured here:
      the first row of each window switches from the trapezoidal average to plain
      backward Euler, which is what the reference implementation does.

  There is deliberately no "FULL_BUS + 2nd order + AVR" example: `include_avr`
  requires DQ_4TH (engine.jl:214-216) because a constant-EMF machine has no field
  winding for an exciter to act on. The governor has no such restriction — see
  example 07.

  Knob families omitted here because this avenue cannot read them, each with the
  example that does show them: DQ machine states (08), AVR set-points and boxes
  (09), GFM boxes and limits (12).
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
# This block also configures the mandatory ACOPF warm start.
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
# First lever if the solve is slow: raise t_step or shorten t_end_sim.
const ts_simulation = TsSimulationConfig(
    δ_tol_deg       = 100.0,   # half-width of the |δ_i − δ_COI| stability corridor [deg]
    δ_tol_deg_lower = nothing, # nothing = reuse δ_tol_deg below the COI; a number makes the corridor asymmetric
    δ_tol_deg_upper = nothing, # nothing = reuse δ_tol_deg above the COI
    t_start_sim     = 0.0,     # first instant of the simulated horizon [s]
    t_end_sim       = 1.0,     # last instant [s]; the dominant cost driver — every step adds rows for every state
    t_step          = 0.01,    # integration step [s]; 100 steps over this horizon
    t_start_fault   = 0.01,    # instant the short circuit is applied [s]
    clearing_time   = 0.3,     # fault duration [s]; the post-fault window starts after it. Read on the SC path only
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
    # Active on FULL_BUS: forced lower bound on the bus voltages V_tf / V_tpf.
    # Raise it (e.g. 0.5) if the fault window drives voltages toward zero and the
    # solver wanders into a numerically hostile region.
    V_bus_min_pu = 0.0,               # 0.0 = no floor; voltages may collapse to zero at the faulted bus
    # Optional classical tf–tpf boxes (inactive: OPEN + toggle off below)
    δ_tf     = OPEN,   # rotor angle, fault window — inactive
    Δω_tf    = OPEN,   # speed deviation, fault window — inactive
    Pe_tf    = OPEN,   # electrical power, fault window — inactive
    Qe_tf    = OPEN,   # reactive power, fault window — inactive
    δCOI_tf  = OPEN,   # centre-of-inertia angle, fault window — inactive
    δ_tpf    = OPEN,   # rotor angle, post-fault window — inactive
    Δω_tpf   = OPEN,   # speed deviation, post-fault window — inactive
    Pe_tpf   = OPEN,   # electrical power, post-fault window — inactive
    Qe_tpf   = OPEN,   # reactive power, post-fault window — inactive
    δCOI_tpf = OPEN,   # centre-of-inertia angle, post-fault window — inactive
)

# ── Transient builder toggles (TsBuilderConfig) ───────────────────────────────
# The P/Q/Pm init equalities are always built by the path — they are physics,
# not toggles.
const ts_builder = TsBuilderConfig(
    # Pre-fault classical boxes
    bound_E   = true,    # true = build the E box from E_min_pu / E_max_pu above
    bound_δ   = true,    # true = build the pre-fault δ box from δ_min_rad / δ_max_rad
    bound_P_m = true,    # true = build the mechanical-power box from P_m_source
    # Optional classical tf / tpf boxes — each needs BOTH this toggle and a finite pair above
    bound_δ_tf     = false,   # false = no ≤-row on the rotor angle through the fault window
    bound_Δω_tf    = false,   # false = no ≤-row on the speed deviation through the fault window
    bound_Pe_tf    = false,   # false = no ≤-row on the electrical power through the fault window
    bound_Qe_tf    = false,   # false = no ≤-row on the reactive power through the fault window
    bound_δCOI_tf  = false,   # false = no ≤-row on the COI angle through the fault window
    bound_δ_tpf    = false,   # false = no ≤-row on the rotor angle through the post-fault window
    bound_Δω_tpf   = false,   # false = no ≤-row on the speed deviation through the post-fault window
    bound_Pe_tpf   = false,   # false = no ≤-row on the electrical power through the post-fault window
    bound_Qe_tpf   = false,   # false = no ≤-row on the reactive power through the post-fault window
    bound_δCOI_tpf = false,   # false = no ≤-row on the COI angle through the post-fault window
    # δ–COI stability inequalities — the transient-stability constraint proper
    ineq_δ_COI_tf_lower  = true,   # true = enforce δ_i − δ_COI ≥ −δ_tol through the fault window
    ineq_δ_COI_tf_upper  = true,   # true = enforce δ_i − δ_COI ≤ +δ_tol through the fault window
    ineq_δ_COI_tpf_lower = true,   # true = same lower corridor through the post-fault window
    ineq_δ_COI_tpf_upper = true,   # true = same upper corridor through the post-fault window
    limits         = ts_limits,             # the TsBoundLimitsConfig instance built above
    bound_encoding = TSCOPF.CONSTRAINT,     # CONSTRAINT = boxes become ≤-rows with their own duals
)

# ── Disturbance (FaultConfig) ─────────────────────────────────────────────────
const fault_cfg = FaultConfig(
    fault_type       = SC,          # SC = three-phase bus short circuit; GL = unit/load trip; OB = open branch
    contingency_id   = 2,           # row of contingencies.csv: faulted bus 7, trip circuit 6 (branch 5–7)
    gl_gen_ids       = Int[],       # empty = no generator trip; read only when fault_type = GL (see example 13)
    gl_load_bus_ids  = Int[],       # empty = no load disturbance; read only when fault_type = GL
    gl_percent_power = Float64[],   # empty = no load scaling factors; one α per entry of gl_load_bus_ids
    ob_branch_ids    = Int[],       # empty = no branch opened; read only when fault_type = OB (see example 14)
)

# ── Dynamic physics and network form (DynModelConfig) ─────────────────────────
const dyn_model_cfg = DynModelConfig(
    gen_order               = CLASSICAL_2ND,    # constant EMF behind X'd, two states (δ, Δω)
    network_form            = FULL_BUS,         # keep every bus: sparse Ybus + nodal KCL, load buses included
    mech_power_mode         = USE_PM,           # required by the FULL_BUS builder; also forces bound_style = :coi_box
    dq_speed_dev_in_algebra = true,             # (1+Δω) in the stator algebra; DQ_4TH only, so inert here
    include_avr             = false,            # false is MANDATORY on CLASSICAL_2ND — the AVR requires DQ_4TH
    include_governor        = false,            # legal to set true here; see example 07
    governor_limiter        = GOV_NO_LIMIT,     # valve saturation mode; read only when include_governor = true
    allow_gfm               = false,            # false is mandatory here: GFM requires DQ_4TH
    ode_first_step          = :trapezoidal,     # scheme for the first row of each window; :backward_euler matches the reference
    gfm_integrator          = :backward_euler,  # discretisation of the GFM filters / Q–V PI; read only when allow_gfm = true
    # (Z, I, P) splits — REAL physics on FULL_BUS, one per demand component.
    # (1,0,0) = constant impedance. Note the reference implementation orders the
    # vector (P, I, Z), so its [0,0,1] is our (1.0, 0.0, 0.0).
    zip_load_p              = (1.0, 0.0, 0.0),  # active demand: 100 % constant impedance (∝ V²), 0 % current, 0 % power
    zip_load_q              = (1.0, 0.0, 0.0),  # reactive demand: same split, set independently of the active one
    bound_style             = :coi_box,         # forced by USE_PM: the stability limit is a direct corridor around the COI
    constrain_Δω_COI        = false,            # false = no corridor on Δω_i − Δω_COI; true builds the frequency box
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
    gen_dynamic_filename = "gen_dynamic_data.csv",   # minimal header bus;Eg;Xd;H;D — all the classical machine needs
    gfm_dynamic_filename = "gfm_dynamic_data.csv",   # never opened here, since allow_gfm = false
)

# ── Solver option blocks (all four; `solver_name` picks the active one) ───────
# Ipopt backends:  "Ipopt" | "Ipopt-ma57" | "Ipopt-ma97" | "Ipopt-pardiso"
# MadNLP backends: "MadNLP" | "MadNLP-ma57" | "MadNLP-ma97"  (also `using MadNLP`)
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
    save_matrices           = true,   # true = dump Ybus and the fault matrices
    save_ts_plots           = false,  # false = no trajectory figures; true additionally needs load_plots_extension!()
    save_ts_debug_csv       = false,  # false = no per-step diagnostic dumps; those are GFM-specific anyway
    # Legal on the two avenues that pre-solve a steady-state OPF (FULL_BUS TSC-ACOPF
    # here; TSC-DCOPF in example 04): dumps that solution to Dispatch_WarmStart/.
    save_warmstart_dispatch = true,   # true = archive the warm-start dispatch alongside the TSC result

    dispatch  = dispatch_cfg,   # the DispatchConfig instance built above; also defines the warm-start problem
    transient = transient_cfg,  # the TransientConfig instance built above; REQUIRED when trans_stab = true

    matpower_file = nothing,    # nothing = read the three CSVs; a filename would parse a .m case instead
)

validate_run_config!(cfg)   # checks the RunConfig-level combination (avenue, savers, solver blocks)
validate_dyn_config!(cfg)   # checks the dynamic-model combination (order, network, controls, ZIP sums)

println("Case:   ", input_files_dir(cfg.case))
println("Solver: ", cfg.solver_name)
println("Solving ACOPF warm start + TSC-ACOPF (FULL_BUS, classical 2nd order)…")

sys    = load_system(cfg, PATH_MAIN)                       # read + scale the case data into a SystemData
result = run_case!(cfg, sys, PATH_MAIN, PATH_RESULTS)      # warm start → build → solve → export

println("\nTermination : ", result.status)
println("Objective   : ", result.obj_MVA)   # plain number, or nothing when unsolved — never JuMP.value it
println("Results     : ", result.path_names[:pf_results_date])
println("TS duals    : ", joinpath(result.path_names[:pf_TS], "CSV_duals"))
println("Warm start  : solver_log_warmstart.txt + Dispatch_WarmStart/")
