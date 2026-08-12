#=
================================================================================
 16 — GFM Imax SWEEP — TSC-ACOPF, FULL_BUS + DQ_4TH + AVR + TGOV1 + GFM
                       converter current limit 1.1 … 1.3 pu
================================================================================
  Avenue:   `trans_stab = true` with `type_model = "ACOPF"`, run once per Imax.
  Case:     INPUT_FILES/9bus_gfm/  (3 SG + 1 GFM at bus 5)
  Fault:    SC, contingency row 2 — three-phase fault at bus 7, cleared at 150 ms
            by opening circuit 6 (branch 5–7). Identical in every scenario.
  Solver:   Ipopt (NLP).
  Results:  one RESULTS/Results - <timestamp>/ per scenario, plus a single
            summary CSV at RESULTS/sweep_gfm_imax_<timestamp>.csv

  Everything is example 12, held fixed, except the converter current rating. Imax
  is how much current the semiconductors are allowed to push during the fault, so
  this sweep asks a specific question: what is a converter's brief overcurrent
  capability worth to the system, and where does tightening it start to cost
  money?

  ── Imax is NOT a RunConfig knob ─────────────────────────────────────────────
  Unlike δ_tol (example 15), Imax is per-unit case data: it is a column of
  `gfm_dynamic_data.csv`, not a field of any config struct. Sweeping it therefore
  means editing `sys.DGFM.Imax` between runs rather than calling `reconfigure`.

  Two consequences the script is built around:

    • `load_system` is called fresh inside the loop, and only that freshly-read
      copy is written to. The house rule is never to mutate shared input
      DataFrames in place; reloading is the cheap, obviously-correct way to
      honour it, and the CSVs are small.

    • `Imax` is already on SYSTEM base by the time you see it.
      `apply_gfm_base_conversion!` divides the CSV column by
      `sys_base_MVA / mach_base_MVA` at load time. For this case
      `mach_base_MVA = InvBase = 100` and `base_MVA = 100`, so the ratio is 1 and
      the post-load number coincides with the CSV number. On a case where the
      converter has its own MVA base they differ, and the value assigned below is
      the system-base one.

  ── What this sweep does and does not move ───────────────────────────────────
  Imax feeds the TRANSIENT current limiter only. The steady-state ACOPF circle
  attached by `attach_gfm_acopf_limits!` uses `mach_base_MVA / base_MVA`, not
  Imax (functions_4_TS_gfm.jl:147), because a sustained dispatch cannot bank on
  overload. So the dispatch feasible set is identical in all five scenarios and
  any objective movement is attributable to the fault-window limiter alone —
  which is exactly what makes the sweep interpretable.

  This is a property of the code, not an assumption of this script: every reader
  of `DGFM.Imax` is either a transient builder (functions_4_TS_gfm.jl:68, 405,
  796) or a results writer (functions_2_save_TS_results.jl:2793, 2973). No
  steady-state constraint reads the column, so overwriting it below cannot reach
  the dispatch model even in principle.

  The whole range stays far below `_GFM_IMAX_NO_LIMIT` (20 pu), so the nonconvex
  limiter is genuinely built in every scenario. Had the range crossed that
  threshold the model would silently change shape to unsaturated stator algebra,
  and the two halves of the curve would not be comparable; the loop checks and
  reports this per scenario rather than trusting it.

  Runtime: 5 scenarios of the heaviest model in the package. Budget generously,
  and coarsen `SWEEP_STEP_PU` if you only need the endpoints.

  Reference: docs/src/model/09_grid_forming.md.
================================================================================
=#

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))   # activate the repo project, not examples/

using TSCOPF
using DataFrames          # the summary table, and nrow() on the fleet
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
const SWEEP_MIN_PU  = 1.1    # tightest converter current rating solved [pu, system base]
const SWEEP_MAX_PU  = 1.3    # loosest converter current rating solved [pu, system base]
const SWEEP_STEP_PU = 0.05   # resolution [pu]; 0.05 over [1.1, 1.3] gives 5 scenarios
const IMAX_RANGE    = SWEEP_MIN_PU:SWEEP_STEP_PU:SWEEP_MAX_PU   # inclusive of both ends

# ── Steady-state limit values (DispatchLimitsConfig) ──────────────────────────
const dispatch_limits = DispatchLimitsConfig(
    θ_min_rad             = -π,     # lower bus-angle bound [rad]; -π = one full turn, effectively unrestricted
    θ_max_rad             = π,      # upper bus-angle bound [rad]; π = one full turn, effectively unrestricted
    ang_diff_clamp_ac_deg = 60.0,   # the active clamp here: DCIR angmin/angmax wider than ±60° are tightened to it
    ang_diff_clamp_dc_deg = 30.0,   # DC-path clamp; read only when type_model = "DCOPF"
)

# ── Steady-state OPF builder (DispatchConfig) ─────────────────────────────────
# Identical in every scenario, and genuinely unaffected by Imax: the converter's
# steady-state circle is sized from mach_base_MVA, not from the swept value.
const dispatch_cfg = DispatchConfig(
    type_model           = "ACOPF",       # "ACOPF" is mandatory with allow_gfm: DC-OPF is rejected
    use_matrix           = true,          # true = one Ybus matrix balance row per bus; false = branch-by-branch summation
    cost_type            = "quadratic",   # "quadratic" = c2·P² + c1·P + c0 from generators_data.csv
    susceptance_model    = SIMPLE,        # SIMPLE = b_ik = 1/x; read only on the DC-OPF path, so inert here
    bound_V              = true,          # true = build V_min ≤ V ≤ V_max rows from bus_data.csv
    bound_θ              = true,          # true = build the θ box from limits above
    bound_P_g            = true,          # true = build Pmin ≤ P_g ≤ Pmax rows, converters included
    bound_Q_g            = true,          # true = build Qmin ≤ Q_g ≤ Qmax rows
    bound_P_ik           = false,         # false = no explicit box on the from-side active branch flow
    bound_Q_ik           = false,         # false = no explicit box on the from-side reactive branch flow
    bound_P_ki           = false,         # false = no explicit box on the to-side active branch flow
    bound_Q_ki           = false,         # false = no explicit box on the to-side reactive branch flow
    ineq_sg_upper        = false,         # false = no |S_g| ≤ S_max circle on the machines; the GFM circle is attached separately
    ineq_sbranch_upper   = true,          # true = |S_ik| ≤ l_cap_1 thermal limit on every rated branch
    ineq_ang_diff_branch = true,          # true = enforce per-branch angle-difference limits
    solve_explicit_dual  = false,         # false is mandatory here: the dual LP is ED / DC-OPF + linear cost only
    limits               = dispatch_limits,     # the DispatchLimitsConfig instance built above
    bound_encoding       = TSCOPF.CONSTRAINT,   # CONSTRAINT = explicit ≤-rows with clean duals; VARIABLE = JuMP bounds instead
)

# ── Transient timing (TsSimulationConfig) — fixed across the sweep ────────────
const ts_simulation = TsSimulationConfig(
    δ_tol_deg       = 100.0,   # half-width of the |δ_i − δ_COI| corridor [deg]; SG units only, held fixed here
    δ_tol_deg_lower = nothing, # nothing = reuse δ_tol_deg below the COI
    δ_tol_deg_upper = nothing, # nothing = reuse δ_tol_deg above the COI
    t_start_sim     = 0.0,     # first instant of the simulated horizon [s]
    t_end_sim       = 0.6,     # last instant [s]: fault 0.01–0.16, post-fault 0.18–0.6
    t_step          = 0.02,    # integration step [s]; 30 steps per scenario
    t_start_fault   = 0.01,    # instant the short circuit is applied [s]
    clearing_time   = 0.15,    # fault duration [s]: 150 ms — the validated point for this case
    f_syn           = 50.0,    # synchronous frequency [Hz]; the reference for both the SG swing and the converter droop
    Δω_0            = 0.0,     # initial speed deviation [pu]; 0.0 = start exactly at synchronous speed
)

# ── Transient limit values (TsBoundLimitsConfig) ──────────────────────────────
const ts_limits = TsBoundLimitsConfig(
    # Pre-fault physical (shared with the classical path)
    E_min_pu     = 0.0,               # lower bound on the pre-fault internal EMF [pu]; 0.0 = no practical floor
    E_max_pu     = 4.0,               # upper bound on the pre-fault internal EMF [pu]; roomier than the SG-only examples
    δ_min_rad    = -π,                # lower bound on the pre-fault rotor angle [rad]
    δ_max_rad    = π,                 # upper bound on the pre-fault rotor angle [rad]
    P_m_source   = :dgen_pg_limits,   # :dgen_pg_limits = take the P_m box from DGEN Pmin/Pmax ÷ base_MVA
    V_bus_min_pu = 0.0,               # forced floor on trajectory bus voltages [pu]; 0.0 = no floor
    # Governor valve travel (read when include_governor and limiter ≠ GOV_NO_LIMIT)
    gov_valve_min_pu     = 0.0,               # lower end of the valve travel [pu]; 0.0 = fully closed
    gov_valve_max_source = :dgen_pg_limits,   # :dgen_pg_limits = upper travel from DGEN.Pmax ÷ base_MVA
    # Optional shared tf–tpf boxes (inactive: OPEN + toggle off below)
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
    # DQ pre-fault algebraic states (applied only if bound_Ed/Eq/Id/Iq are on)
    Ed_min_pu = -2.0,   # lower bound on the pre-fault d-axis EMF E'd [pu]
    Ed_max_pu = 2.0,    # upper bound on the pre-fault d-axis EMF E'd [pu]
    Eq_min_pu = -2.0,   # lower bound on the pre-fault q-axis EMF E'q [pu]
    Eq_max_pu = 2.0,    # upper bound on the pre-fault q-axis EMF E'q [pu]
    Id_min_pu = -5.0,   # lower bound on the pre-fault d-axis stator current [pu]
    Id_max_pu = 5.0,    # upper bound on the pre-fault d-axis stator current [pu]
    Iq_min_pu = -5.0,   # lower bound on the pre-fault q-axis stator current [pu]
    Iq_max_pu = 5.0,    # upper bound on the pre-fault q-axis stator current [pu]
    # AVR / governor pre-fault set-points
    V_ref_min_pu = 0.8,               # lower bound on the AVR voltage set-point [pu]; read only when bound_V_ref
    V_ref_max_pu = 1.2,               # upper bound on the AVR voltage set-point [pu]
    P_ref_source = :dgen_pg_limits,   # :dgen_pg_limits = take the P_ref box from DGEN Pmin/Pmax ÷ base_MVA
    # DQ machine states, fault and post-fault windows (inactive)
    Ed_tf  = OPEN,   # d-axis EMF, fault window — inactive
    Eq_tf  = OPEN,   # q-axis EMF, fault window — inactive
    Id_tf  = OPEN,   # d-axis current, fault window — inactive
    Iq_tf  = OPEN,   # q-axis current, fault window — inactive
    Te_tf  = OPEN,   # electrical torque, fault window — inactive
    Ed_tpf = OPEN,   # d-axis EMF, post-fault window — inactive
    Eq_tpf = OPEN,   # q-axis EMF, post-fault window — inactive
    Id_tpf = OPEN,   # d-axis current, post-fault window — inactive
    Iq_tpf = OPEN,   # q-axis current, post-fault window — inactive
    Te_tpf = OPEN,   # electrical torque, post-fault window — inactive
    # AVR / governor research boxes (inactive; saturation physics unchanged)
    E_fd_unlim_tf  = OPEN,   # unlimited field voltage, fault window — inactive
    E_fd_unlim_tpf = OPEN,   # unlimited field voltage, post-fault window — inactive
    Pv_tf  = OPEN,   # valve position, fault window — inactive
    Pv_tpf = OPEN,   # valve position, post-fault window — inactive
    Pm_tf  = OPEN,   # mechanical power, fault window — inactive
    Pm_tpf = OPEN,   # mechanical power, post-fault window — inactive

    # ── GFM boxes (read only when allow_gfm) ─────────────────────────────────
    # Margins and floors, not final numbers: `resolve_gfm_bound_limits` combines
    # them per unit with the DGFM row — including the swept Imax. These are always
    # built; the nonconvex limiter needs the guard-rails.
    gfm_δ_min_rad          = -π,     # lower bound on the converter angle [rad]
    gfm_δ_max_rad          = π,      # upper bound on the converter angle [rad]
    gfm_V_meas_min_pu      = 0.0,    # lower bound on the measured (filtered) terminal voltage [pu]
    gfm_V_meas_max_pu      = 2.5,    # upper bound on the measured terminal voltage [pu]
    gfm_E_raw_extra_pu     = 0.25,   # band added around [Emin, Emax] for the raw PI states
    gfm_E_raw_slack_pu     = 0.5,    # further slack so the box never pre-empts the clip
    gfm_E_clip_slack_pu    = 0.05,   # slack outside the smooth-clip range
    gfm_PQ_bound_scale     = 1.5,    # margin factor multiplying the nameplate in the P/Q/I box formulas
    gfm_PQ_bound_offset_pu = 0.05,   # additive offset in the same formulas
    gfm_P_meas_floor_pu    = 3.0,    # floor on the measured-active-power box [pu]
    gfm_Q_meas_floor_pu    = 3.0,    # floor on the measured-reactive-power box [pu]
    # NOTE: with the swept Imax in [1.1, 1.3] this floor of 5.0 pu is far above
    # the rating, so the CURRENT BOX is governed by the floor and does not move
    # across the sweep. What moves is the limiter itself, which clamps to Imax.
    # Keeping the box fixed is deliberate: it isolates the limiter's effect.
    gfm_I_floor_pu         = 5.0,    # floor on the current box [pu]
    gfm_I_ceiling_pu       = 20.0,   # ceiling on the current box [pu]
)

# ── Transient builder toggles (TsBuilderConfig) ───────────────────────────────
const ts_builder = TsBuilderConfig(
    # Pre-fault boxes shared with the classical path
    bound_E   = true,    # true = build the E box from E_min_pu / E_max_pu above
    bound_δ   = true,    # true = build the pre-fault δ box from δ_min_rad / δ_max_rad
    bound_P_m = true,    # true = build the mechanical-power box from P_m_source
    # Optional shared tf / tpf boxes — each needs BOTH this toggle and a finite pair above
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
    # DQ pre-fault algebraic states
    bound_Ed = false,   # false = E'd is free at the pre-fault point; true applies Ed_min_pu / Ed_max_pu
    bound_Eq = false,   # false = E'q is free at the pre-fault point; true applies Eq_min_pu / Eq_max_pu
    bound_Id = false,   # false = Id is free at the pre-fault point; true applies Id_min_pu / Id_max_pu
    bound_Iq = false,   # false = Iq is free at the pre-fault point; true applies Iq_min_pu / Iq_max_pu
    # AVR / governor pre-fault set-point boxes
    bound_V_ref = false,      # false = V_ref is a free variable, unconstrained by V_ref_min_pu / V_ref_max_pu
    bound_P_ref = false,      # false = P_ref is a free variable, unconstrained by DGEN limits
    # DQ machine states — fault window
    bound_Ed_tf = false,   # false = no ≤-row on E'd through the fault window
    bound_Eq_tf = false,   # false = no ≤-row on E'q through the fault window
    bound_Id_tf = false,   # false = no ≤-row on Id through the fault window
    bound_Iq_tf = false,   # false = no ≤-row on Iq through the fault window
    bound_Te_tf = false,   # false = no ≤-row on the electrical torque through the fault window
    # DQ machine states — post-fault window
    bound_Ed_tpf = false,  # false = no ≤-row on E'd through the post-fault window
    bound_Eq_tpf = false,  # false = no ≤-row on E'q through the post-fault window
    bound_Id_tpf = false,  # false = no ≤-row on Id through the post-fault window
    bound_Iq_tpf = false,  # false = no ≤-row on Iq through the post-fault window
    bound_Te_tpf = false,  # false = no ≤-row on the electrical torque through the post-fault window
    # AVR / governor research boxes
    bound_E_fd_unlim_tf  = false,   # false = no ≤-row on the unlimited field voltage, fault window
    bound_E_fd_unlim_tpf = false,   # false = no ≤-row on the unlimited field voltage, post-fault window
    bound_Pv_tf  = false,     # false = no ≤-row on the valve position through the fault window
    bound_Pv_tpf = false,     # false = no ≤-row on the valve position through the post-fault window
    bound_Pm_tf  = false,     # false = no ≤-row on the mechanical power through the fault window
    bound_Pm_tpf = false,     # false = no ≤-row on the mechanical power through the post-fault window

    # ── GFM boxes (read only when allow_gfm) ─────────────────────────────────
    # All true, as shipped: the nonconvex current limiter needs these guard-rails
    # to converge, and holding them on across the sweep keeps the five scenarios
    # structurally identical apart from Imax itself.
    bound_gfm_δ              = true,   # true = box the pre-fault converter angle
    bound_gfm_P_meas_tf      = true,   # true = box the filtered active-power measurement, fault window
    bound_gfm_Q_meas_tf      = true,   # true = box the filtered reactive-power measurement, fault window
    bound_gfm_V_meas_tf      = true,   # true = box the filtered voltage measurement, fault window
    bound_gfm_E_int_raw_tf   = true,   # true = box the raw Q–V PI integrator state, fault window
    bound_gfm_E_int_tf       = true,   # true = box the clipped Q–V PI integrator state, fault window
    bound_gfm_E_droop_raw_tf = true,   # true = box the raw droop voltage command, fault window
    bound_gfm_E_droop_tf     = true,   # true = box the clipped droop voltage command, fault window
    bound_gfm_Id_tf          = true,   # true = box the converter d-axis current, fault window
    bound_gfm_Iq_tf          = true,   # true = box the converter q-axis current, fault window
    bound_gfm_P_meas_tpf      = true,  # true = box the filtered active-power measurement, post-fault window
    bound_gfm_Q_meas_tpf      = true,  # true = box the filtered reactive-power measurement, post-fault window
    bound_gfm_V_meas_tpf      = true,  # true = box the filtered voltage measurement, post-fault window
    bound_gfm_E_int_raw_tpf   = true,  # true = box the raw Q–V PI integrator state, post-fault window
    bound_gfm_E_int_tpf       = true,  # true = box the clipped Q–V PI integrator state, post-fault window
    bound_gfm_E_droop_raw_tpf = true,  # true = box the raw droop voltage command, post-fault window
    bound_gfm_E_droop_tpf     = true,  # true = box the clipped droop voltage command, post-fault window
    bound_gfm_Id_tpf          = true,  # true = box the converter d-axis current, post-fault window
    bound_gfm_Iq_tpf          = true,  # true = box the converter q-axis current, post-fault window

    # δ–COI stability inequalities (SG units only on a mixed fleet)
    ineq_δ_COI_tf_lower  = true,   # true = enforce δ_i − δ_COI ≥ −δ_tol through the fault window
    ineq_δ_COI_tf_upper  = true,   # true = enforce δ_i − δ_COI ≤ +δ_tol through the fault window
    ineq_δ_COI_tpf_lower = true,   # true = same lower corridor through the post-fault window
    ineq_δ_COI_tpf_upper = true,   # true = same upper corridor through the post-fault window
    limits         = ts_limits,             # the TsBoundLimitsConfig instance built above
    bound_encoding = TSCOPF.CONSTRAINT,     # CONSTRAINT = GFM boxes become ≤-rows exported as dual_LB/UB_gfm_*
)

# ── Disturbance (FaultConfig) ─────────────────────────────────────────────────
# Held fixed across the sweep: only the converter current rating may vary.
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
    gen_order               = DQ_4TH,        # required by allow_gfm; also what the AVR needs
    network_form            = FULL_BUS,      # required by allow_gfm; converters inject through nodal KCL
    mech_power_mode         = USE_PM,        # required by DQ_4TH: P_m is its own variable (machines only)
    dq_speed_dev_in_algebra = true,          # true = keep the (1+Δω) factor in Pe and the stator algebra (RMS convention)
    include_avr             = true,          # true = first-order exciter on the SG units; converters are unaffected
    include_governor        = true,          # true = TGOV1 on the SG units; converters use their own P–f droop instead
    governor_limiter        = GOV_SMOOTH,    # GOV_SMOOTH = differentiable saturation; GOV_NO_LIMIT skips it, GOV_HARD_BOUND clamps
    allow_gfm               = true,          # true = load gfm_dynamic_data.csv and build the converter fleet — required for this sweep
    ode_first_step          = :trapezoidal,  # :trapezoidal | :backward_euler; applies to SG families and the GFM δ integrator
    gfm_integrator          = :backward_euler,  # BE on the GFM filters and Q–V PI: L-stable and the affordable choice; held fixed here
    # (Z, I, P) splits — real physics on FULL_BUS. (1,0,0) = constant impedance.
    zip_load_p              = (1.0, 0.0, 0.0),  # active demand: 100 % constant impedance (∝ V²), 0 % current, 0 % power
    zip_load_q              = (1.0, 0.0, 0.0),  # reactive demand: same split, set independently of the active one
    bound_style_δ             = :coi_box,      # required by DQ_4TH: the stability limit is a direct corridor around the COI
    constrain_Δω        = false,         # false = no Δω corridor; like δ–COI it would cover SG units only
    Δω_tol_pu               = 0.5,           # half-width of that Δω corridor [pu]; read only when constrain_Δω = true
    Δω_tol_pu_lower         = nothing,       # nothing = reuse Δω_tol_pu below the COI
    Δω_tol_pu_upper         = nothing,       # nothing = reuse Δω_tol_pu above the COI
    fault                   = fault_cfg,     # the FaultConfig instance built above
)

# ── Transient bundle (TransientConfig) ────────────────────────────────────────
const transient_cfg = TransientConfig(
    simulation           = ts_simulation,   # the TsSimulationConfig instance built above; fixed across the sweep
    builder              = ts_builder,      # the TsBuilderConfig instance built above
    dyn_model            = dyn_model_cfg,   # the DynModelConfig instance built above
    gen_dynamic_filename = "gen_dynamic_data_full.csv",   # SG rows only, ids 1–3 — the machine half of the partition
    gfm_dynamic_filename = "gfm_dynamic_data.csv",        # GFM rows only, id 4 — the file whose Imax column is swept
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

# ── Run configuration (RunConfig) — identical in every scenario ───────────────
# Nothing here changes across the sweep: Imax lives in the case data, not the config.
const cfg = RunConfig(
    trans_stab = true,             # true = build the transient model on top of the dispatch; requires `transient` below

    case            = "9bus_gfm",  # the mixed-fleet case: 3 SG + 1 GFM, with its own dynamic CSVs
    base_MVA        = 100.0,       # system power base [MVA]; the base Imax is converted to at load time
    load_factor     = 1.5,         # multiplies every bus demand at load time; 1.0 = the CSV as written
    solver_name     = "Ipopt",     # NLP required on this path: HiGHS and Gurobi are rejected
    silent_solver   = true,        # true here (unlike example 12): 5 solver logs would drown the sweep table
    time_limit_sec  = 3600.0,      # wall-clock cap per scenario [s], not for the sweep as a whole

    ipopt  = ipopt_cfg,        # applied when solver_name is an Ipopt backend — the active one here
    highs  = highs_cfg,        # applied when solver_name = "HiGHS"
    gurobi = gurobi_cfg,       # applied when solver_name = "Gurobi"
    madnlp = madnlp_cfg,       # applied when solver_name is a MadNLP backend

    overwrite_results       = false,  # false = one fresh timestamped folder PER SCENARIO; true would collapse the sweep into one
    save_duals              = true,   # true = export the duals; dual_gfm_limiter_Id/_Iq is what this sweep is pricing
    save_optim_matrices     = false,  # false; a TSC run forces it false anyway (steady-state only feature)
    save_matrices           = false,  # false = skip the Ybus dumps; identical in all 5 scenarios, so 5 copies is waste
    save_ts_plots           = false,  # false = no trajectory figures; true additionally needs load_plots_extension!()
    save_ts_debug_csv       = true,   # true = per-step GFM limiter diagnostics; the point of comparison across scenarios
    save_warmstart_dispatch = false,  # false = skip the warm-start dump; the dispatch is identical in every scenario

    dispatch  = dispatch_cfg,   # the DispatchConfig instance built above; also defines the warm-start problem
    transient = transient_cfg,  # the TransientConfig instance built above; REQUIRED when trans_stab = true

    matpower_file = nothing,    # nothing = read the CSVs; a filename would parse a .m case instead
)

validate_run_config!(cfg)   # checks the RunConfig-level combination once, before any solve
validate_dyn_config!(cfg)   # checks the dynamic-model combination once, including the GFM requirements

println("Case:     ", input_files_dir(cfg.case))
println("Solver:   ", cfg.solver_name)
println("Sweeping: GFM Imax ", SWEEP_MIN_PU, " … ", SWEEP_MAX_PU, " pu in steps of ",
        SWEEP_STEP_PU, " pu — ", length(IMAX_RANGE), " scenarios")
println("Model:    TSC-ACOPF, FULL_BUS, dq 4th order + AVR + TGOV1, mixed SG/GFM fleet")

# ── Sweep ─────────────────────────────────────────────────────────────────────
# One row per scenario. `limiter` records whether the nonconvex clamp was really
# built, so a scenario that silently switched to unsaturated stator algebra is
# visible in the table rather than quietly comparable to the others.
summary = DataFrame(
    imax_pu      = Float64[],   # the swept converter current rating [pu, system base]
    status       = String[],    # MOI termination status, or "ERROR" if the scenario threw
    objective    = Union{Missing, Float64}[],   # objective [€]; missing when unsolved
    limiter      = String[],    # "active" or "bypassed" (Imax ≥ 20 pu changes the model shape)
    t_build_s    = Union{Missing, Float64}[],   # model assembly time [s]
    t_solve_s    = Union{Missing, Float64}[],   # solver time [s]
    results_path = String[],    # the per-scenario RESULTS folder, for reading duals back
)

for (i, imax_val) in enumerate(IMAX_RANGE)
    println("\n", "="^70)
    @printf("  scenario %2d / %2d    GFM Imax = %.2f pu\n", i, length(IMAX_RANGE), imax_val)
    println("="^70)

    try
        # Fresh read per scenario. Imax is case data, so the override happens on
        # the loaded DataFrame — and reloading guarantees we never write into a
        # DGFM shared with an earlier scenario.
        sys = load_system(cfg, PATH_MAIN)
        # Post-load values are already on system base (apply_gfm_base_conversion!),
        # so this assigns the system-base rating directly. `.=` writes every
        # converter row; index the vector to sweep one unit on a larger fleet.
        sys.DGFM.Imax .= Float64(imax_val)

        result = run_case!(cfg, sys, PATH_MAIN, PATH_RESULTS)

        meta = result.dyn_model_dict[:meta]   # :meta survives the backend release; :vars does not
        limiter_state = haskey(meta, :gfm_limiter_bypassed) ? "bypassed" : "active"

        push!(summary, (
            imax_pu      = Float64(imax_val),
            status       = string(result.status),
            # obj_MVA is already a plain number (or nothing) — never JuMP.value it,
            # the backend is released before run_case! returns.
            objective    = result.obj_MVA === nothing ? missing : Float64(result.obj_MVA),
            limiter      = limiter_state,
            t_build_s    = result.t_build,
            t_solve_s    = result.t_solve,
            results_path = result.path_names[:pf_results_date],
        ))
        @printf("  → %s   objective %s   limiter %s\n", string(result.status),
                result.obj_MVA === nothing ? "unsolved" : string(result.obj_MVA),
                limiter_state)
    catch e
        @warn "Imax = $(imax_val) pu — scenario failed, continuing the sweep" exception = e
        push!(summary, (
            imax_pu      = Float64(imax_val),
            status       = "ERROR",
            objective    = missing,
            limiter      = "—",
            t_build_s    = missing,
            t_solve_s    = missing,
            results_path = "",
        ))
    end
end

# ── Summary ───────────────────────────────────────────────────────────────────
const SUMMARY_PATH = joinpath(
    PATH_RESULTS,
    "sweep_gfm_imax_" * Dates.format(now(), "yyyy-mm-dd HHMMSS") * ".csv",
)
CSV.write(SUMMARY_PATH, summary)

println("\n", "="^70)
println("  GFM Imax SWEEP SUMMARY")
println("="^70)
@printf("  %-10s  %-18s  %-14s  %-9s  %-9s\n",
        "Imax [pu]", "status", "objective [€]", "limiter", "solve [s]")
for r in eachrow(summary)
    @printf("  %-10.2f  %-18s  %-14s  %-9s  %-9s\n",
            r.imax_pu,
            r.status,
            r.objective === missing ? "—" : @sprintf("%.2f", r.objective),
            r.limiter,
            r.t_solve_s === missing ? "—" : @sprintf("%.1f", r.t_solve_s))
end

# A bypassed limiter anywhere in the table breaks comparability: that scenario is
# a different model, not a different parameter value.
if any(r -> r.limiter == "bypassed", eachrow(summary))
    @warn "At least one scenario bypassed the current limiter — those rows are a " *
          "different model shape and are not comparable with the clamped ones."
end

solved = filter(r -> r.objective !== missing, summary)
if nrow(solved) >= 2
    spread = maximum(solved.objective) - minimum(solved.objective)
    @printf("\n  objective spread across the swept range: %.4f €\n", spread)
    if spread <= 1e-6
        println("  → flat: the converter current limiter never binds over [", SWEEP_MIN_PU,
                ", ", SWEEP_MAX_PU, "] pu.")
        println("    Push the lower end down, lengthen clearing_time, or raise load_factor")
        println("    so the converter is actually driven into its limit during the fault.")
    else
        println("  → the limiter binds somewhere in this range. The shadow prices live in")
        println("    each scenario's Transient_Stability/CSV_duals as dual_gfm_limiter_Id/_Iq,")
        println("    and the per-step distance-to-limit is in CSV/Debug/gfm_limiter_debug.csv.")
    end
end

println("\n  scenarios : ", nrow(summary), " (", nrow(solved), " solved)")
println("  summary   : ", SUMMARY_PATH)
println("  per-run   : one RESULTS/Results - <timestamp>/ folder per scenario")
