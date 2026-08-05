#=
================================================================================
 15 — δ_tol SWEEP — TSC-ACOPF, KRON_REDUCED + CLASSICAL_2ND, 90° … 100°
================================================================================
  Avenue:   `trans_stab = true` with `type_model = "ACOPF"`, run once per δ_tol.
  Case:     INPUT_FILES/9bus/  (+ gen_dynamic_data.csv, contingencies.csv)
  Fault:    SC, contingency row 2 — three-phase fault at bus 7, cleared at 0.3 s
            by opening circuit 6 (branch 5–7). Identical in every scenario.
  Solver:   Ipopt (NLP).
  Results:  one RESULTS/Results - <timestamp>/ per scenario, plus a single
            summary CSV at RESULTS/sweep_delta_tol_kron_<timestamp>.csv

  Everything is example 05, held fixed, except the one knob being swept: the
  half-width of the |δ_i − δ_COI| stability corridor. Narrowing that corridor is
  what eventually forces the dispatch off the merit order, so the sweep is the
  cheapest way to find where the stability constraint starts to bind and to read
  the cost of stability as a curve rather than a single number.

  Expect the objective to be flat across much of the range on the bundled 9-bus
  case and then to rise once the corridor bites. A flat sweep is information, not
  a failure: it says the corridor is slack at this load factor, and the lever to
  reach for is `LOAD_FACTOR` below or a longer fault.

  ── The trap this script exists to avoid ─────────────────────────────────────
  `reconfigure_transient(tc; simulation = TsSimulationConfig(δ_tol_deg = x))`
  REPLACES the whole simulation block, so every field you do not restate silently
  reverts to its package default — t_end_sim back to 5.0 s, clearing_time back to
  0.3 s. The sweep then varies two things at once and the curve means nothing.
  `sweep_simulation` below rebuilds the full struct each iteration with every
  field written out, so δ_tol_deg is provably the only thing that moves.

  Runtime: 11 scenarios; on the bundled 9-bus case each Kron solve is on the
  order of a minute, so budget roughly ten minutes for the whole sweep. Shorten
  `t_end_sim` or coarsen `SWEEP_STEP_DEG` to trade resolution for time.

  Field-by-field reference: docs/parameter_reference.md, docs/configuration_map.md.
================================================================================
=#

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))   # activate the repo project, not examples/

using TSCOPF
using DataFrames          # the summary table
using CSV                 # writes that table next to the per-scenario folders
using Dates               # timestamp for the summary filename
using Printf              # aligned console table

const PATH_MAIN    = project_root()          # repo root — the folder that contains INPUT_FILES/
const PATH_RESULTS = default_results_dir()   # <repo root>/RESULTS
mkpath(PATH_RESULTS)                         # create the results tree on first run

# (-Inf, Inf) = inactive. An optional box is built only when its `bound_*` toggle
# is on AND the limit is finite; Inf never enters a ≤-row or a JuMP bound.
const OPEN = TsBoundLimitPair(min = -Inf, max = Inf)   # the "no limit on either side" pair

# ── Sweep range — the only quantity that varies between scenarios ─────────────
const SWEEP_MIN_DEG  = 90.0    # tightest corridor solved [deg]; the binding end of the range
const SWEEP_MAX_DEG  = 100.0   # loosest corridor solved [deg]; the slack end of the range
const SWEEP_STEP_DEG = 1.0     # resolution [deg]; 1.0 over [90, 100] gives 11 scenarios
const δ_TOL_RANGE    = SWEEP_MIN_DEG:SWEEP_STEP_DEG:SWEEP_MAX_DEG   # inclusive of both ends

# ── Steady-state limit values (DispatchLimitsConfig) ──────────────────────────
const dispatch_limits = DispatchLimitsConfig(
    θ_min_rad             = -π,     # lower bus-angle bound [rad]; -π = one full turn, effectively unrestricted
    θ_max_rad             = π,      # upper bus-angle bound [rad]; π = one full turn, effectively unrestricted
    ang_diff_clamp_ac_deg = 60.0,   # the active clamp here: DCIR angmin/angmax wider than ±60° are tightened to it
    ang_diff_clamp_dc_deg = 30.0,   # DC-path clamp; read only when type_model = "DCOPF"
)

# ── Steady-state OPF builder (DispatchConfig) ─────────────────────────────────
# Identical in every scenario: the sweep must not move the dispatch formulation.
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

# ── Transient timing, rebuilt per scenario (TsSimulationConfig) ───────────────
"""
    sweep_simulation(δ_tol_deg) -> TsSimulationConfig

Full timing block for one scenario. Every field is restated so that `δ_tol_deg`
is demonstrably the only quantity that changes across the sweep — see the trap
note in the file header.
"""
function sweep_simulation(δ_tol_deg::Float64)
    return TsSimulationConfig(
        δ_tol_deg       = δ_tol_deg,   # ← THE SWEPT KNOB: corridor half-width [deg]
        δ_tol_deg_lower = nothing,     # nothing = reuse δ_tol_deg below the COI, keeping the corridor symmetric
        δ_tol_deg_upper = nothing,     # nothing = reuse δ_tol_deg above the COI
        t_start_sim     = 0.0,         # first instant of the simulated horizon [s]
        t_end_sim       = 1.0,         # last instant [s]; the dominant cost driver, held fixed across the sweep
        t_step          = 0.01,        # integration step [s]; 100 steps per scenario
        t_start_fault   = 0.01,        # instant the short circuit is applied [s]
        clearing_time   = 0.3,         # fault duration [s]; the post-fault window starts after it. SC path only
        f_syn           = 50.0,        # synchronous frequency [Hz]; sets ω_0 = 2π·50 in the swing equation
        Δω_0            = 0.0,         # initial speed deviation [pu]; 0.0 = start exactly at synchronous speed
    )
end

# ── Transient limit values (TsBoundLimitsConfig) ──────────────────────────────
const ts_limits = TsBoundLimitsConfig(
    # Pre-fault physical
    E_min_pu     = 0.0,               # lower bound on the pre-fault internal EMF [pu]; 0.0 = no practical floor
    E_max_pu     = 2.0,               # upper bound on the pre-fault internal EMF [pu]
    δ_min_rad    = -π,                # lower bound on the pre-fault rotor angle [rad]
    δ_max_rad    = π,                 # upper bound on the pre-fault rotor angle [rad]
    P_m_source   = :dgen_pg_limits,   # :dgen_pg_limits = take the P_m box from DGEN Pmin/Pmax ÷ base_MVA
    V_bus_min_pu = 0.0,               # forced floor on trajectory bus voltages [pu]; FULL_BUS only, so inert here
    # Optional classical tf–tpf boxes (inactive: OPEN + toggle off below).
    # Leaving these off matters for a sweep: an extra binding box would compete
    # with the δ–COI corridor and blur which constraint is pricing the result.
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
    # δ–COI stability inequalities — the rows the sweep is actually moving.
    # All four stay on: switching a side off would leave that half of the
    # corridor unpriced and the sweep would only measure the other half.
    ineq_δ_COI_tf_lower  = true,   # true = enforce δ_i − δ_COI ≥ −δ_tol through the fault window
    ineq_δ_COI_tf_upper  = true,   # true = enforce δ_i − δ_COI ≤ +δ_tol through the fault window
    ineq_δ_COI_tpf_lower = true,   # true = same lower corridor through the post-fault window
    ineq_δ_COI_tpf_upper = true,   # true = same upper corridor through the post-fault window
    limits         = ts_limits,             # the TsBoundLimitsConfig instance built above
    bound_encoding = TSCOPF.CONSTRAINT,     # CONSTRAINT = boxes become ≤-rows with their own duals
)

# ── Disturbance (FaultConfig) ─────────────────────────────────────────────────
# Held fixed across the sweep: only the corridor width may vary.
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
    gen_order               = CLASSICAL_2ND,    # constant EMF behind X'd, two states (δ, Δω); the cheap model a sweep wants
    network_form            = KRON_REDUCED,     # eliminate every non-machine bus into Y_red — the cheapest network form
    mech_power_mode         = USE_PM,           # USE_PM = P_m is its own variable, which forces bound_style = :coi_box
    dq_speed_dev_in_algebra = true,             # (1+Δω) in the stator algebra; DQ_4TH only, so inert here
    include_avr             = false,            # false is mandatory here: the AVR requires DQ_4TH
    include_governor        = false,            # false is mandatory here: the governor requires FULL_BUS
    governor_limiter        = GOV_NO_LIMIT,     # valve saturation mode; read only when include_governor = true
    allow_gfm               = false,            # false is mandatory here: GFM requires FULL_BUS + DQ_4TH
    ode_first_step          = :trapezoidal,     # :trapezoidal is mandatory here: the Kron swing rows have no backward-Euler form
    gfm_integrator          = :backward_euler,  # discretisation of the GFM filters / Q–V PI; read only when allow_gfm = true
    # (Z, I, P) splits — IGNORED on KRON_REDUCED (loads fold into Y_red).
    zip_load_p              = (1.0, 0.0, 0.0),  # active demand: 100 % constant impedance, 0 % current, 0 % power
    zip_load_q              = (1.0, 0.0, 0.0),  # reactive demand: same split, set independently of the active one
    bound_style             = :coi_box,         # required by USE_PM; also what makes δ_tol_deg a directly priced corridor
    constrain_Δω_COI        = false,            # false = no Δω corridor, so δ_tol is the only stability limit being swept
    Δω_tol_pu               = 0.5,              # half-width of that Δω corridor [pu]; read only when constrain_Δω_COI = true
    Δω_tol_pu_lower         = nothing,          # nothing = reuse Δω_tol_pu below the COI
    Δω_tol_pu_upper         = nothing,          # nothing = reuse Δω_tol_pu above the COI
    fault                   = fault_cfg,        # the FaultConfig instance built above
)

# ── Transient bundle (TransientConfig) — the base; simulation is swapped per run ──
const transient_cfg = TransientConfig(
    simulation           = sweep_simulation(SWEEP_MIN_DEG),   # placeholder: the loop overrides it every iteration
    builder              = ts_builder,      # the TsBuilderConfig instance built above
    dyn_model            = dyn_model_cfg,   # the DynModelConfig instance built above
    gen_dynamic_filename = "gen_dynamic_data.csv",   # minimal header bus;Eg;Xd;H;D — all the classical machine needs
    gfm_dynamic_filename = "gfm_dynamic_data.csv",   # never opened here, since allow_gfm = false
)

# ── Solver option blocks (all four; `solver_name` picks the active one) ───────
const ipopt_cfg = IpoptSolverConfig(
    tol                        = 1e-8,       # overall convergence tolerance
    print_level                = 5,          # 0 = silent … 12 = maximum; silent_solver = true forces 0
    max_iter                   = 5_000,      # iteration cap; hitting it returns ITERATION_LIMIT rather than failing
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

# ── Base run configuration (RunConfig) — shared by every scenario ─────────────
const base_cfg = RunConfig(
    trans_stab = true,         # true = build the transient model on top of the dispatch; requires `transient` below

    case            = "9bus",  # folder name under INPUT_FILES/
    base_MVA        = 100.0,   # system power base [MVA]
    load_factor     = 1.5,     # multiplies every bus demand; raise it to make the corridor bind sooner
    solver_name     = "Ipopt", # NLP required on this path: HiGHS and Gurobi are rejected
    silent_solver   = true,    # true here (unlike the single-run examples): 11 solver logs would drown the sweep table
    time_limit_sec  = 1800.0,  # wall-clock cap per scenario [s], not for the sweep as a whole

    ipopt  = ipopt_cfg,        # applied when solver_name is an Ipopt backend — the active one here
    highs  = highs_cfg,        # applied when solver_name = "HiGHS"
    gurobi = gurobi_cfg,       # applied when solver_name = "Gurobi"
    madnlp = madnlp_cfg,       # applied when solver_name is a MadNLP backend

    overwrite_results       = false,  # false = one fresh timestamped folder PER SCENARIO; true would collapse the sweep into one
    save_duals              = true,   # true = export the duals; the δ–COI shadow prices are the point of the sweep
    save_optim_matrices     = false,  # false; a TSC run forces it false anyway (steady-state only feature)
    save_matrices           = false,  # false = skip the Ybus/Y_red dumps; identical in all 11 scenarios, so 11 copies is waste
    save_ts_plots           = false,  # false = no trajectory figures; true additionally needs load_plots_extension!()
    save_ts_debug_csv       = false,  # false = no per-step diagnostic dumps; those are GFM-specific anyway
    save_warmstart_dispatch = false,  # false is mandatory here: Kron TSC-ACOPF has no pre-solve to archive

    dispatch  = dispatch_cfg,   # the DispatchConfig instance built above
    transient = transient_cfg,  # the TransientConfig instance built above; REQUIRED when trans_stab = true

    matpower_file = nothing,    # nothing = read the three CSVs; a filename would parse a .m case instead
)

validate_run_config!(base_cfg)   # checks the RunConfig-level combination once, before any solve
validate_dyn_config!(base_cfg)   # checks the dynamic-model combination once; the sweep never changes it

# The case data does not depend on δ_tol, so it is read once and reused. Rebuild
# it only if `load_factor` changes — demand is scaled at load time, not at solve.
const sys = load_system(base_cfg, PATH_MAIN)

println("Case:     ", input_files_dir(base_cfg.case))
println("Solver:   ", base_cfg.solver_name)
println("Sweeping: δ_tol ", SWEEP_MIN_DEG, "° … ", SWEEP_MAX_DEG, "° in steps of ",
        SWEEP_STEP_DEG, "° — ", length(δ_TOL_RANGE), " scenarios")
println("Model:    TSC-ACOPF, Kron-reduced, classical 2nd order, SC contingency ",
        base_cfg.transient.dyn_model.fault.contingency_id)

# ── Sweep ─────────────────────────────────────────────────────────────────────
# One row per scenario. `objective` is `nothing` when a scenario does not solve,
# so a failed point leaves a gap in the curve instead of aborting the sweep.
summary = DataFrame(
    δ_tol_deg    = Float64[],   # the swept corridor half-width [deg]
    status       = String[],    # MOI termination status, or "ERROR" if the scenario threw
    objective    = Union{Missing, Float64}[],   # objective [€]; missing when unsolved
    t_build_s    = Union{Missing, Float64}[],   # model assembly time [s]
    t_solve_s    = Union{Missing, Float64}[],   # solver time [s]
    results_path = String[],    # the per-scenario RESULTS folder, for reading duals back
)

for (i, δ_val) in enumerate(δ_TOL_RANGE)
    println("\n", "="^70)
    @printf("  scenario %2d / %2d    δ_tol = %.1f°\n", i, length(δ_TOL_RANGE), δ_val)
    println("="^70)

    # Rebuild the timing block in full, then swap it into a copy of the base
    # config. `reconfigure` deep-copies dispatch and transient unless overridden,
    # so scenarios never share mutable state.
    tc  = reconfigure_transient(base_cfg.transient; simulation = sweep_simulation(Float64(δ_val)))
    cfg = reconfigure(base_cfg; transient = tc)

    try
        result = run_case!(cfg, sys, PATH_MAIN, PATH_RESULTS)
        push!(summary, (
            δ_tol_deg    = Float64(δ_val),
            status       = string(result.status),
            # obj_MVA is already a plain number (or nothing) — never JuMP.value it,
            # the backend is released before run_case! returns.
            objective    = result.obj_MVA === nothing ? missing : Float64(result.obj_MVA),
            t_build_s    = result.t_build,
            t_solve_s    = result.t_solve,
            results_path = result.path_names[:pf_results_date],
        ))
        @printf("  → %s   objective %s\n", string(result.status),
                result.obj_MVA === nothing ? "unsolved" : string(result.obj_MVA))
    catch e
        @warn "δ_tol = $(δ_val)° — scenario failed, continuing the sweep" exception = e
        push!(summary, (
            δ_tol_deg    = Float64(δ_val),
            status       = "ERROR",
            objective    = missing,
            t_build_s    = missing,
            t_solve_s    = missing,
            results_path = "",
        ))
    end
end

# ── Summary ───────────────────────────────────────────────────────────────────
const SUMMARY_PATH = joinpath(
    PATH_RESULTS,
    "sweep_delta_tol_kron_" * Dates.format(now(), "yyyy-mm-dd HHMMSS") * ".csv",
)
CSV.write(SUMMARY_PATH, summary)

println("\n", "="^70)
println("  δ_tol SWEEP SUMMARY")
println("="^70)
@printf("  %-12s  %-18s  %-14s  %-9s\n", "δ_tol [deg]", "status", "objective [€]", "solve [s]")
for r in eachrow(summary)
    @printf("  %-12.1f  %-18s  %-14s  %-9s\n",
            r.δ_tol_deg,
            r.status,
            r.objective === missing ? "—" : @sprintf("%.2f", r.objective),
            r.t_solve_s === missing ? "—" : @sprintf("%.1f", r.t_solve_s))
end

# The interesting quantity is the spread: a flat column means the corridor never
# bound over this range, so nothing was priced and the sweep needs a tighter
# range, a heavier load_factor, or a longer fault before it says anything.
solved = filter(r -> r.objective !== missing, summary)
if nrow(solved) >= 2
    spread = maximum(solved.objective) - minimum(solved.objective)
    @printf("\n  objective spread across the swept range: %.4f €\n", spread)
    if spread <= 1e-6
        println("  → flat: the δ–COI corridor never binds here. Tighten the range,")
        println("    raise load_factor, or lengthen clearing_time to make it bite.")
    else
        println("  → the corridor binds somewhere in this range; the duals in each")
        println("    scenario's Transient_Stability/CSV_duals price it.")
    end
end

println("\n  scenarios : ", nrow(summary), " (", nrow(solved), " solved)")
println("  summary   : ", SUMMARY_PATH)
println("  per-run   : one RESULTS/Results - <timestamp>/ folder per scenario")
