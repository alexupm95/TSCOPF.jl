#=
================================================================================
 02 — DC Optimal Power Flow (DC-OPF)
================================================================================
  Avenue:   steady state only, `trans_stab = false`.
  Case:     INPUT_FILES/9bus/
  Solver:   HiGHS (LP).
  Results:  RESULTS/Results - <timestamp>/Dispatch/

  Linearised network: bus angles, Bbus power balance, branch flows from angle
  differences. Voltages are fixed at 1 pu and reactive power does not exist.
  This is where the LMP decomposition first becomes readable — the balance dual
  per bus plus the congestion rent on binding branches.

  `susceptance_model` is the one knob unique to this avenue:
      SIMPLE       b_ik = 1/x            (the classroom convention)
      POWERMODELS  b_ik = imag(1/(r+jx)) (matches PowerModels.jl on lossy branches)
  The 9-bus case has r = 0 on every branch, so both agree here. On a lossy case
  they do not, and the resulting LMPs differ.

  `solve_explicit_dual` writes the in-house dual LP to Dispatch_Dual/. On DC-OPF
  that needs `cost_type = "linear"` AND `use_matrix = true`
  (_common/DispatchConfig.jl:188-197).

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
    ang_diff_clamp_ac_deg = 60.0,   # AC branch angle-difference cap [deg]; read only on ACOPF paths
    ang_diff_clamp_dc_deg = 30.0,   # the active clamp here: DCIR angmin/angmax wider than ±30° are tightened to it
)

# ── Steady-state OPF builder (DispatchConfig) ─────────────────────────────────
const dispatch_cfg = DispatchConfig(
    type_model           = "DCOPF",       # "DCOPF" = linearised network: angles and Bbus, no V, no Q
    use_matrix           = true,          # true = one Bbus matrix balance row per bus; required by solve_explicit_dual
    cost_type            = "linear",      # "linear" = use c1 only and ignore c2; required by solve_explicit_dual
    susceptance_model    = SIMPLE,        # SIMPLE = b_ik = 1/x; POWERMODELS = imag(1/(r+jx)), which differs on lossy branches
    # Explicit ≤-form variable bounds. V and Q do not exist on the DC path.
    bound_V              = true,          # true = build the V box; no V variable on DC-OPF, so inert
    bound_θ              = true,          # true = build the θ box from limits above — active here
    bound_P_g            = true,          # true = build Pmin ≤ P_g ≤ Pmax rows from generators_data.csv
    bound_Q_g            = true,          # true = build the Q_g box; no Q variable on DC-OPF, so inert
    bound_P_ik           = false,         # false = no explicit box on the from-side active branch flow
    bound_Q_ik           = false,         # false = no explicit box on the from-side reactive flow; inert on DC
    bound_P_ki           = false,         # false = no explicit box on the to-side active branch flow
    bound_Q_ki           = false,         # false = no explicit box on the to-side reactive flow; inert on DC
    # Optional inequality families
    ineq_sg_upper        = false,         # false = no |S_g| ≤ S_max capability circle; an AC concept, inert on DC
    ineq_sbranch_upper   = true,          # true = enforce the l_cap_1 thermal limit on every rated branch
    ineq_ang_diff_branch = true,          # true = enforce per-branch angle-difference limits — the usual congestion source
    solve_explicit_dual  = true,          # true = also build and solve the dual LP → Dispatch_Dual/, with a strong-duality report
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
    nlp_scaling_method         = nothing,    # nothing = Ipopt default; e.g. "gradient-based"
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
    solver                       = "choose", # "choose" = let HiGHS pick; or force "simplex" | "ipm" | "pdlp"
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
    solver_name     = "HiGHS", # LP solver; DC-OPF is free to use any registry entry
    silent_solver   = false,   # false = let the solver print; true forces its log level to 0
    time_limit_sec  = 600.0,   # wall-clock cap handed to the solver [s]

    ipopt  = ipopt_cfg,        # applied when solver_name is an Ipopt backend
    highs  = highs_cfg,        # applied when solver_name = "HiGHS" — the active one here
    gurobi = gurobi_cfg,       # applied when solver_name = "Gurobi"
    madnlp = madnlp_cfg,       # applied when solver_name is a MadNLP backend

    overwrite_results       = false,  # false = write a fresh timestamped folder per run
    save_duals              = true,   # true = export JuMP.dual on the primal rows — the LMPs live here
    save_optim_matrices     = true,   # true = dump the A/b/c matrices of the optimisation model
    save_matrices           = true,   # true = dump Ybus/Bbus to XLSX
    save_ts_plots           = false,  # false = no trajectory figures; TSC-only and needs load_plots_extension!()
    save_ts_debug_csv       = false,  # false = no per-step diagnostic dumps; TSC-only anyway
    save_warmstart_dispatch = false,  # false = no warm-start dump; needs a TSC run with a steady-state pre-solve

    dispatch  = dispatch_cfg,  # the DispatchConfig instance built above
    transient = nothing,       # nothing = no transient avenue; REQUIRED when trans_stab = false

    matpower_file = nothing,   # nothing = read the three CSVs; a filename would parse a .m case instead
)

validate_run_config!(cfg)   # fails fast on an illegal knob combination, before any model is built

println("Case:   ", input_files_dir(cfg.case))
println("Solver: ", cfg.solver_name)
println("Solving DC-OPF…")

sys    = load_system(cfg, PATH_MAIN)                       # read + scale the case data into a SystemData
result = run_case!(cfg, sys, PATH_MAIN, PATH_RESULTS)      # build → solve → export

println("\nTermination : ", result.status)
println("Objective   : ", result.obj_MVA)   # plain number, or nothing when unsolved — never JuMP.value it
println("Results     : ", result.path_names[:pf_results_date])
println("LMPs        : Dispatch/Dispatch_Duals.xlsx — π_k = −λ_k on the balance rows")
