#=
================================================================================
 03 — AC Optimal Power Flow (ACOPF)
================================================================================
  Avenue:   steady state only, `trans_stab = false`.
  Case:     INPUT_FILES/9bus/
  Solver:   Ipopt (NLP). ACOPF **forbids** HiGHS and Gurobi
            (_common/functions_4_sanity_checks.jl:24-29) — the model is nonconvex.
  Results:  RESULTS/Results - <timestamp>/Dispatch/

  Full AC network: voltage magnitudes and angles, P and Q balance at every bus,
  branch flows in both directions. This is the dispatch that every FULL_BUS
  TSC-ACOPF run (examples 06–13) solves first as a warm start.

  `use_matrix = true` builds the balance from Ybus; `false` builds it branch by
  branch. Same optimum, different sparsity and different constraint labels in the
  dual export.

  `cost_type = "quadratic"` is the MATPOWER convention (c2·P² + c1·P + c0). Note
  the bundled 9-bus case has c2 = 0, so the objective is linear in practice — the
  quadratic path is still the one being exercised.

  `solve_explicit_dual` is NOT available here: the explicit dual LP is restricted
  to ED and DC-OPF with linear cost. Use `save_duals = true`, which calls
  JuMP.dual on the primal constraint rows.

  Not reachable from this avenue: everything transient. `trans_stab = false`
  *requires* `transient = nothing` (engine.jl:147-148).
================================================================================
=#

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))   # activate the repo project, not examples/

using TSCOPF

const PATH_MAIN    = project_root()          # repo root — the folder that contains INPUT_FILES/
const PATH_RESULTS = default_results_dir()   # <repo root>/RESULTS
mkpath(PATH_RESULTS)                         # create the results tree on first run

# ── Steady-state limit values (DispatchLimitsConfig) ──────────────────────────
const dispatch_limits = DispatchLimitsConfig(
    θ_min_rad             = -π,     # lower bus-angle bound [rad]; -π = one full turn, effectively unrestricted
    θ_max_rad             = π,      # upper bus-angle bound [rad]; π = one full turn, effectively unrestricted
    ang_diff_clamp_ac_deg = 60.0,   # the active clamp here: DCIR angmin/angmax wider than ±60° are tightened to it
    ang_diff_clamp_dc_deg = 30.0,   # DC-path clamp; read only when type_model = "DCOPF"
)

# ── Steady-state OPF builder (DispatchConfig) ─────────────────────────────────
const dispatch_cfg = DispatchConfig(
    type_model           = "ACOPF",       # "ACOPF" = full AC network: V, θ, P and Q balance at every bus
    use_matrix           = true,          # true = one Ybus matrix balance row per bus; false = branch-by-branch summation
    cost_type            = "quadratic",   # "quadratic" = c2·P² + c1·P + c0 (MATPOWER); this case has c2 = 0
    susceptance_model    = SIMPLE,        # SIMPLE = b_ik = 1/x; read only on the DC-OPF path, so inert here
    # Explicit ≤-form variable bounds — all four primal families exist on this path.
    bound_V              = true,          # true = build V_min ≤ V ≤ V_max rows from bus_data.csv
    bound_θ              = true,          # true = build the θ box from limits above
    bound_P_g            = true,          # true = build Pmin ≤ P_g ≤ Pmax rows from generators_data.csv
    bound_Q_g            = true,          # true = build Qmin ≤ Q_g ≤ Qmax rows
    # Branch-flow variable boxes, off by default; the thermal limit below is the
    # usual way to bound flows and keeps one dual per branch instead of four.
    bound_P_ik           = false,         # false = no explicit box on the from-side active branch flow
    bound_Q_ik           = false,         # false = no explicit box on the from-side reactive branch flow
    bound_P_ki           = false,         # false = no explicit box on the to-side active branch flow
    bound_Q_ki           = false,         # false = no explicit box on the to-side reactive branch flow
    # Optional inequality families
    ineq_sg_upper        = false,         # false = no |S_g| ≤ S_max generator capability circle
    ineq_sbranch_upper   = true,          # true = |S_ik| ≤ l_cap_1 thermal limit on every rated branch
    ineq_ang_diff_branch = true,          # true = enforce per-branch angle-difference limits
    solve_explicit_dual  = false,         # false is mandatory here: the dual LP is ED / DC-OPF + linear cost only
    limits               = dispatch_limits,     # the DispatchLimitsConfig instance built above
    bound_encoding       = TSCOPF.CONSTRAINT,   # CONSTRAINT = explicit ≤-rows with clean duals; VARIABLE = JuMP bounds instead
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
    solver                       = "choose", # "choose" = let HiGHS pick; unused here since ACOPF forbids HiGHS
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
    trans_stab = false,        # false = steady state only; no transient model is built

    case            = "9bus",  # folder name under INPUT_FILES/
    base_MVA        = 100.0,   # system power base [MVA]
    load_factor     = 1.5,     # multiplies every bus demand at load time; 1.0 = the CSV as written
    solver_name     = "Ipopt", # NLP required on this path: HiGHS and Gurobi are rejected
    silent_solver   = false,   # false = let the solver print; true forces print_level to 0
    time_limit_sec  = 600.0,   # wall-clock cap handed to the solver [s]

    ipopt  = ipopt_cfg,        # applied when solver_name is an Ipopt backend — the active one here
    highs  = highs_cfg,        # applied when solver_name = "HiGHS"
    gurobi = gurobi_cfg,       # applied when solver_name = "Gurobi"
    madnlp = madnlp_cfg,       # applied when solver_name is a MadNLP backend

    overwrite_results       = false,  # false = write a fresh timestamped folder per run
    save_duals              = true,   # true = export JuMP.dual on the primal rows — the only dual route on ACOPF
    save_optim_matrices     = true,   # true = dump the A/b/c matrices of the optimisation model
    save_matrices           = true,   # true = dump Ybus to XLSX
    save_ts_plots           = false,  # false = no trajectory figures; TSC-only and needs load_plots_extension!()
    save_ts_debug_csv       = false,  # false = no per-step diagnostic dumps; TSC-only anyway
    save_warmstart_dispatch = false,  # false = no warm-start dump; only legal on FULL_BUS TSC-ACOPF
    use_acopf_warmstart     = true,   # true is mandatory off the FULL_BUS TSC-ACOPF path; inert on a steady-state run

    dispatch  = dispatch_cfg,  # the DispatchConfig instance built above
    transient = nothing,       # nothing = no transient avenue; REQUIRED when trans_stab = false

    matpower_file = nothing,   # nothing = read the three CSVs; a filename would parse a .m case instead
)

validate_run_config!(cfg)   # fails fast on an illegal knob combination, before any model is built

println("Case:   ", input_files_dir(cfg.case))
println("Solver: ", cfg.solver_name)
println("Solving AC-OPF…")

sys    = load_system(cfg, PATH_MAIN)                       # read + scale the case data into a SystemData
result = run_case!(cfg, sys, PATH_MAIN, PATH_RESULTS)      # build → solve → export

println("\nTermination : ", result.status)
println("Objective   : ", result.obj_MVA)   # plain number, or nothing when unsolved — never JuMP.value it
println("Results     : ", result.path_names[:pf_results_date])
println("Duals       : Dispatch/Dispatch_Duals.xlsx — π_k = −λ_k on the balance rows")
