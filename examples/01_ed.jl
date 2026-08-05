#=
================================================================================
 01 — Economic Dispatch (ED)
================================================================================
  Avenue:   steady state only, `trans_stab = false`.
  Case:     INPUT_FILES/9bus/  (bus_data.csv, generators_data.csv, line_data.csv)
  Solver:   HiGHS (LP). ED accepts any solver in the registry.
  Results:  RESULTS/Results - <timestamp>/Dispatch/

  ED is the network-free dispatch: generator cost minimisation against a single
  system-wide power balance, no angles, no flows, no voltages. It is the cheapest
  way to see the load-balance dual (the system marginal price) before any network
  or stability structure enters.

  This script also switches on `solve_explicit_dual`, which solves the in-house
  dual LP alongside the primal and writes it to Dispatch_Dual/. That path requires
  `cost_type = "linear"` (validate_dispatch_config!, _common/DispatchConfig.jl:188-192)
  — a quadratic primal has no LP dual to write, so use `save_duals` there instead.

  Not reachable from this avenue: everything transient. `trans_stab = false`
  *requires* `transient = nothing` (engine.jl:147-148).

  Field-by-field reference: docs/parameter_reference.md, docs/configuration_map.md.
================================================================================
=#

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))   # activate the repo project, not examples/

using TSCOPF

# Bundled cases live in <project_root>/INPUT_FILES/<case>/; results go to <project_root>/RESULTS/.
const PATH_MAIN    = project_root()          # repo root — the folder that contains INPUT_FILES/
const PATH_RESULTS = default_results_dir()   # <repo root>/RESULTS
mkpath(PATH_RESULTS)                         # create the results tree on first run

# ── Steady-state limit values (DispatchLimitsConfig) ──────────────────────────
# Inert on ED — there are no angles in this formulation — but written out so the
# struct is visible. The AC clamp is used by ACOPF, the DC clamp by DC-OPF.
const dispatch_limits = DispatchLimitsConfig(
    θ_min_rad             = -π,     # lower bus-angle bound [rad]; -π = one full turn, effectively unrestricted
    θ_max_rad             = π,      # upper bus-angle bound [rad]; π = one full turn, effectively unrestricted
    ang_diff_clamp_ac_deg = 60.0,   # cap on per-branch angle difference [deg] for AC paths; wider DCIR angmin/angmax are tightened to it
    ang_diff_clamp_dc_deg = 30.0,   # same cap for the DC path; read only when type_model = "DCOPF"
)

# ── Steady-state OPF builder (DispatchConfig) ─────────────────────────────────
const dispatch_cfg = DispatchConfig(
    type_model           = "ED",          # "ED" = single system-wide balance, no network at all
    use_matrix           = true,          # true = matrix-form balance; ED has no network matrix, so this changes nothing here
    cost_type            = "linear",      # "linear" = use c1 only and ignore c2; required by solve_explicit_dual below
    susceptance_model    = SIMPLE,        # SIMPLE = b_ik = 1/x; read only on the DC-OPF path, so inert here
    # Explicit ≤-form variable bounds. Only P_g exists on the ED path; the rest
    # are inert here and kept at their package defaults.
    bound_V              = true,          # true = build the V box from bus_data.csv; no V variable on ED, so inert
    bound_θ              = true,          # true = build the θ box from limits above; no θ variable on ED, so inert
    bound_P_g            = true,          # true = build Pmin ≤ P_g ≤ Pmax rows from generators_data.csv — the only active box here
    bound_Q_g            = true,          # true = build Qmin ≤ Q_g ≤ Qmax rows; no Q variable on ED, so inert
    bound_P_ik           = false,         # false = no explicit box on the from-side active branch flow
    bound_Q_ik           = false,         # false = no explicit box on the from-side reactive branch flow
    bound_P_ki           = false,         # false = no explicit box on the to-side active branch flow
    bound_Q_ki           = false,         # false = no explicit box on the to-side reactive branch flow
    # Optional inequality families — all network-side, so inert on ED.
    ineq_sg_upper        = false,         # false = no |S_g| ≤ S_max generator capability circle
    ineq_sbranch_upper   = true,          # true = |S_ik| ≤ l_cap_1 thermal limit on every rated branch
    ineq_ang_diff_branch = true,          # true = enforce per-branch angle-difference limits
    # In-house dual LP after the primal → Dispatch_Dual/. ED or DCOPF + linear cost.
    solve_explicit_dual  = true,          # true = also build and solve the dual LP, and print a strong-duality report
    limits               = dispatch_limits,     # the DispatchLimitsConfig instance built above
    # CONSTRAINT → bounds become explicit ≤-rows so JuMP.dual() is clean (default).
    # VARIABLE   → JuMP lower_bound/upper_bound; same optimum, duals normalised
    #              through the bound manifest. Not exported: needs the TSCOPF. prefix.
    bound_encoding       = TSCOPF.CONSTRAINT,   # CONSTRAINT = every box becomes its own ≤-row with its own dual
)

# ── Solver option blocks (all four; `solver_name` picks the active one) ───────
# Ipopt backends:  "Ipopt" | "Ipopt-ma57" | "Ipopt-ma97" | "Ipopt-pardiso"
# MadNLP backends: "MadNLP" | "MadNLP-ma57" | "MadNLP-ma97"  (also `using MadNLP`)
# The HSL aliases need HSL_jll; "Ipopt-pardiso" needs a licensed libpardiso.
const ipopt_cfg = IpoptSolverConfig(
    tol                        = 1e-8,       # overall convergence tolerance
    print_level                = 5,          # 0 = silent … 12 = maximum; silent_solver = true forces 0
    max_iter                   = 5_000,      # iteration cap; hitting it returns ITERATION_LIMIT rather than failing
    constr_viol_tol            = 1e-8,       # largest primal constraint violation accepted at the solution
    dual_inf_tol               = 1e-8,       # largest dual infeasibility (gradient of the Lagrangian) accepted
    compl_inf_tol              = 1e-8,       # largest complementarity violation accepted
    hessian_approximation      = "exact",    # "exact" = analytic second derivatives; "limited-memory" = L-BFGS
    limited_memory_max_history = 50,         # L-BFGS history depth; read only when hessian_approximation = "limited-memory"
    acceptable_tol             = nothing,    # nothing = leave Ipopt's own relaxed-acceptance tolerance untouched
    obj_scaling_factor         = nothing,    # nothing = no manual objective scaling
    nlp_scaling_method         = nothing,    # nothing = Ipopt default; e.g. "gradient-based" to rescale rows
    pardiso_lib_path           = nothing,    # nothing = fall back to ENV["JULIA_PARDISO_LIB"]; only read for "Ipopt-pardiso"
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
    mip_gap           = 1e-8,       # relative MIP gap; only meaningful on integer models (UC)
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
    ma97_num_threads      = nothing,           # nothing = library default; set e.g. 8 when solver_name = "MadNLP-ma97"
    raw_options           = Dict{String, Any}(),   # empty = pass no extra MadNLP options through
)

# ── Top-level run configuration (RunConfig) ───────────────────────────────────
const cfg = RunConfig(
    # Avenue selector
    trans_stab = false,        # false = steady state only; no transient model is built

    # Orchestration
    case            = "9bus",  # folder name under INPUT_FILES/
    base_MVA        = 100.0,   # system power base [MVA]; every pu quantity is referred to it
    load_factor     = 1.5,     # multiplies every bus demand at load time; 1.0 = the CSV as written
    solver_name     = "HiGHS", # picks which of the four blocks above is applied
    silent_solver   = false,   # false = let the solver print; true forces its log level to 0
    time_limit_sec  = 600.0,   # wall-clock cap handed to the solver [s]

    # Solver option blocks
    ipopt  = ipopt_cfg,        # applied when solver_name is an Ipopt backend
    highs  = highs_cfg,        # applied when solver_name = "HiGHS" — the active one here
    gurobi = gurobi_cfg,       # applied when solver_name = "Gurobi"
    madnlp = madnlp_cfg,       # applied when solver_name is a MadNLP backend

    # Results / I/O
    overwrite_results       = false,  # false = write a fresh timestamped folder per run; true reuses one
    save_duals              = true,   # true = export JuMP.dual on the primal rows to Dispatch_Duals.xlsx
    save_optim_matrices     = true,   # true = dump the A/b/c matrices of the optimisation model
    save_matrices           = true,   # true = dump Ybus/Bbus to XLSX
    save_ts_plots           = false,  # false = no trajectory figures; true is TSC-only and needs load_plots_extension!()
    save_ts_debug_csv       = false,  # false = no per-step diagnostic dumps; TSC-only anyway
    save_warmstart_dispatch = false,  # false = no warm-start dump; only legal on FULL_BUS TSC-ACOPF
    use_acopf_warmstart     = true,   # true is mandatory off the FULL_BUS TSC-ACOPF path; inert on a steady-state run

    # Nested avenues
    dispatch  = dispatch_cfg,  # the DispatchConfig instance built above
    transient = nothing,       # nothing = no transient avenue; REQUIRED when trans_stab = false

    # Optional MATPOWER override (CSV case data when nothing)
    matpower_file = nothing,   # nothing = read the three CSVs; a filename would parse a .m case instead
)

validate_run_config!(cfg)   # fails fast on an illegal knob combination, before any model is built

println("Case:   ", input_files_dir(cfg.case))
println("Solver: ", cfg.solver_name)
println("Solving ED…")

sys    = load_system(cfg, PATH_MAIN)                       # read + scale the case data into a SystemData
result = run_case!(cfg, sys, PATH_MAIN, PATH_RESULTS)      # build → solve → export

println("\nTermination : ", result.status)
println("Objective   : ", result.obj_MVA)   # plain number, or nothing when unsolved — never JuMP.value it
println("Results     : ", result.path_names[:pf_results_date])
println("Duals       : Dispatch/Dispatch_Duals.xlsx + Dispatch_Dual/ (explicit LP)")
