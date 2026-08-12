#=
================================================================================
 12 — TSC-ACOPF — FULL_BUS + DQ_4TH + AVR + TGOV1 + grid-forming converters
================================================================================
  Avenue:   `trans_stab = true` with `type_model = "ACOPF"`.
  Case:     INPUT_FILES/9bus_gfm/
              generators_data.csv       4 units: SG at buses 1, 2, 3 + GFM at bus 5
              gen_dynamic_data_full.csv SG dynamics, ids 1–3
              gfm_dynamic_data.csv      GFM dynamics, id 4 (Imax = 1.2 pu)
  Fault:    SC, contingency row 2 — three-phase fault at bus 7, cleared at 150 ms
            by opening circuit 6 (branch 5–7). The validated point for this case.
  Solver:   Ipopt (NLP).
  Results:  RESULTS/Results - <timestamp>/{Dispatch,Transient_Stability}/

  Example 11 with a mixed fleet: three synchronous machines with AVR and governor
  plus one grid-forming converter. The converter carries a P–f droop, a Q–V PI
  regulator and a current limiter, and it is *not* a machine — it has no inertia
  and no rotor angle in the mechanical sense.

  That asymmetry shows up in the model in a specific place: the centre of inertia
  and both stability corridors (δ–COI and Δω–COI) iterate **SG units only**,
  while nodal KCL injects every active unit including the converters. A
  converter cannot pull the COI, but it does hold up the bus.

  Fleet membership is an `id` partition across two files — there is no `is_gfm`
  column. Every id in `generators_data.csv` must appear in exactly one of
  `gen_dynamic_data_full.csv` (SG) or `gfm_dynamic_data.csv` (GFM), which
  `validate_gen_dyn_partition!` checks at load time.

  Requirements (engine.jl:286-293): `allow_gfm = true` needs `network_form =
  FULL_BUS`, `gen_order = DQ_4TH`, and a non-DCOPF dispatch. A 2nd-order machine
  cannot host a converter fleet, which is why this example is 4th order.

  Because Imax = 1.2 < _GFM_IMAX_NO_LIMIT (20 pu), the converter current limiter
  is really built here; a case with Imax ≥ 20 would silently get unsaturated
  stator algebra instead (the run logs it, and the check at the bottom reports it).

  Reference: docs/src/model/09_grid_forming.md.
================================================================================
=#

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))   # activate the repo project, not examples/

using TSCOPF
using DataFrames          # nrow() for the fleet summary below

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

# ── Transient timing (TsSimulationConfig) ─────────────────────────────────────
# First lever if the solve is slow: raise t_step or shorten t_end_sim. The second
# lever on this path is `gfm_integrator` below — see the note there.
const ts_simulation = TsSimulationConfig(
    δ_tol_deg       = 100.0,   # half-width of the |δ_i − δ_COI| corridor [deg]; applied to SG units only
    δ_tol_deg_lower = nothing, # nothing = reuse δ_tol_deg below the COI; a number makes the corridor asymmetric
    δ_tol_deg_upper = nothing, # nothing = reuse δ_tol_deg above the COI
    t_start_sim     = 0.0,     # first instant of the simulated horizon [s]
    t_end_sim       = 0.6,     # last instant [s]: fault 0.01–0.16, post-fault 0.18–0.6
    t_step          = 0.02,    # integration step [s]; 30 steps over this horizon
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
    # them per unit with the DGFM row (Emin/Emax, Imax, Pmin/Pmax). Unlike the
    # optional tf/tpf pairs above these are ALWAYS built — the nonconvex limiter
    # needs the guard-rails — and they follow `bound_encoding`. Values below are
    # the shipped defaults, i.e. the numbers the reference port hard-coded.
    gfm_δ_min_rad          = -π,     # lower bound on the converter angle [rad]
    gfm_δ_max_rad          = π,      # upper bound on the converter angle [rad]
    gfm_V_meas_min_pu      = 0.0,    # lower bound on the measured (filtered) terminal voltage [pu]
    gfm_V_meas_max_pu      = 2.5,    # upper bound on the measured terminal voltage [pu]
    gfm_E_raw_extra_pu     = 0.25,   # band added around [Emin, Emax] for the raw PI states
    gfm_E_raw_slack_pu     = 0.5,    # further slack so the box never pre-empts the clip
    gfm_E_clip_slack_pu    = 0.05,   # slack outside the smooth-clip range
    gfm_PQ_bound_scale     = 1.5,    # margin factor multiplying the nameplate in the P/Q/I box formulas
    gfm_PQ_bound_offset_pu = 0.05,   # additive offset in the same formulas
    gfm_P_meas_floor_pu    = 3.0,    # floor on the measured-active-power box [pu]; keeps it from collapsing on small units
    gfm_Q_meas_floor_pu    = 3.0,    # floor on the measured-reactive-power box [pu]
    gfm_I_floor_pu         = 5.0,    # floor on the current box [pu]; with Imax = 1.2 this floor is what governs
    gfm_I_ceiling_pu       = 20.0,   # ceiling on the current box [pu]
)

# ── Transient builder toggles (TsBuilderConfig) ───────────────────────────────
# The P/Q/Pm init equalities are always built by the path — they are physics,
# not toggles. GFM boxes are likewise unconditional (see above).
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
    # Same shape as the SG toggles above, but default TRUE: these are guard-rails
    # the nonconvex current limiter needs to converge. Values come from the
    # `gfm_*` fields of ts_limits. Switching one off removes that box only — no
    # JuMP bound, no ≤-row, no manifest entry. If the objective moves when you do,
    # the box was binding and is a real limit, not a guard-rail.
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
    limits         = ts_limits,   # the TsBoundLimitsConfig instance built above
    # CONSTRAINT → GFM boxes become ≤-rows and their duals land in
    # Transient_Stability/CSV_duals as dual_LB/UB_gfm_*. VARIABLE keeps JuMP
    # bounds (the reference's shape) and routes the same names through the
    # bound manifest — same optimum either way.
    bound_encoding = TSCOPF.CONSTRAINT,
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
# allow_gfm=true loads gfm_dynamic_data.csv, partitions SG/GFM by id, and keeps the
# inertia-weighted corridors (COI, :coi_box on either knob) SG-only. The machine-referenced
# δ styles and bound_style_Δω=:abs bound the converters too; KCL always injects every unit.
const dyn_model_cfg = DynModelConfig(
    gen_order               = DQ_4TH,        # required by allow_gfm; also what the AVR needs
    network_form            = FULL_BUS,      # required by allow_gfm; converters inject through nodal KCL
    mech_power_mode         = USE_PM,        # required by DQ_4TH: P_m is its own variable (machines only)
    dq_speed_dev_in_algebra = true,          # true = keep the (1+Δω) factor in Pe and the stator algebra (RMS convention)
    include_avr             = true,          # true = first-order exciter on the SG units; converters are unaffected
    include_governor        = true,          # true = TGOV1 on the SG units; converters use their own P–f droop instead
    governor_limiter        = GOV_SMOOTH,    # GOV_SMOOTH = differentiable saturation; GOV_NO_LIMIT skips it, GOV_HARD_BOUND clamps
    allow_gfm               = true,          # true = load gfm_dynamic_data.csv and build the converter fleet
    # First ODE row of each window, for BOTH fleets: SG swing/EMF/AVR/governor and
    # the GFM δ integrator. :backward_euler reproduces the reference implementation;
    # :trapezoidal keeps machines and converters at the same order.
    ode_first_step          = :trapezoidal,  # :trapezoidal | :backward_euler
    # Time discretisation of the GFM measurement filters (P/Q/V) and the Q–V PI
    # integrator ONLY — GFM δ and every SG family keep following ode_first_step.
    #   :backward_euler        BE at every step (default). L-stable, never rings,
    #                          and the only scheme that stays affordable as the
    #                          horizon grows.
    #   :follow_ode_first_step same rule the SG families use.
    #   :trapezoidal           trapezoidal at every step; second-order accurate but
    #                          the row carries u_{t-1} (Pe/Qe, nonlinear) into every
    #                          filter equation, and the Hessian fill-in compounds.
    #                          Measured on this case: ≈70× total solve time at a 3 s
    #                          horizon, for ≈2 % of swing in accuracy.
    gfm_integrator          = :backward_euler,
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
    simulation           = ts_simulation,   # the TsSimulationConfig instance built above
    builder              = ts_builder,      # the TsBuilderConfig instance built above
    dyn_model            = dyn_model_cfg,   # the DynModelConfig instance built above
    gen_dynamic_filename = "gen_dynamic_data_full.csv",   # SG rows only, ids 1–3 — the machine half of the partition
    gfm_dynamic_filename = "gfm_dynamic_data.csv",        # GFM rows only, id 4 — the converter half of the partition
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
    trans_stab = true,             # true = build the transient model on top of the dispatch; requires `transient` below

    case            = "9bus_gfm",  # the mixed-fleet case: 3 SG + 1 GFM, with its own dynamic CSVs
    base_MVA        = 100.0,       # system power base [MVA]; the GFM Imax is expressed on it
    load_factor     = 1.5,         # multiplies every bus demand at load time; 1.0 = the CSV as written
    solver_name     = "Ipopt",     # NLP required on this path: HiGHS and Gurobi are rejected
    silent_solver   = false,       # false = let the solver print; true forces print_level to 0
    time_limit_sec  = 3600.0,      # wall-clock cap [s]; roomier here, the converter model is the heaviest

    ipopt  = ipopt_cfg,        # applied when solver_name is an Ipopt backend — the active one here
    highs  = highs_cfg,        # applied when solver_name = "HiGHS"
    gurobi = gurobi_cfg,       # applied when solver_name = "Gurobi"
    madnlp = madnlp_cfg,       # applied when solver_name is a MadNLP backend

    overwrite_results       = false,  # false = write a fresh timestamped folder per run
    save_duals              = true,   # true = export the dispatch, transient and GFM-specific duals
    save_optim_matrices     = false,  # false; a TSC run forces it false anyway (steady-state only feature)
    save_matrices           = true,   # true = dump Ybus and the fault matrices
    save_ts_plots           = false,  # false = no trajectory figures; true additionally needs load_plots_extension!()
    # Per-(window, gen, step) diagnostics under Transient_Stability/CSV/Debug/:
    # gfm_{filter,limiter,voltage}_debug.csv + swing_debug.csv, each row carrying
    # the value, its distance to the limit, and the dual. Large on sweeps.
    save_ts_debug_csv       = true,   # true = write the per-step GFM diagnostic CSVs
    save_warmstart_dispatch = true,   # true = archive the pre-TS ACOPF solution to Dispatch_WarmStart/

    dispatch  = dispatch_cfg,   # the DispatchConfig instance built above; also defines the warm-start problem
    transient = transient_cfg,  # the TransientConfig instance built above; REQUIRED when trans_stab = true

    matpower_file = nothing,    # nothing = read the three CSVs; a filename would parse a .m case instead
)

validate_run_config!(cfg)   # checks the RunConfig-level combination (avenue, savers, solver blocks)
validate_dyn_config!(cfg)   # checks the dynamic-model combination, including the GFM requirements

println("Case:   ", input_files_dir(cfg.case))
sys = load_system(cfg, PATH_MAIN)   # also partitions SG vs GFM by id and validates the split
println("Fleet:  ", nrow(sys.DGEN), " units — ", nrow(sys.DGEN_DYN), " SG, ",
        sys.DGFM === nothing ? 0 : nrow(sys.DGFM), " GFM")
println("Solver: ", cfg.solver_name)
println("Solving ACOPF warm start + TSC-ACOPF (dq 4th order, AVR + TGOV1, mixed SG/GFM)…")

result = run_case!(cfg, sys, PATH_MAIN, PATH_RESULTS)   # warm start → build → solve → export

println("\nTermination : ", result.status)
println("Objective   : ", result.obj_MVA)   # plain number, or nothing when unsolved — never JuMP.value it
println("Results     : ", result.path_names[:pf_results_date])

meta = result.dyn_model_dict[:meta]   # :meta survives the backend release; :vars does not
println("SG units    : ", get(meta, :sg_gens, Int[]))
println("GFM units   : ", get(meta, :gfm_gens, Int[]))
if haskey(meta, :gfm_limiter_bypassed)
    @warn "GFM current limiter bypassed (Imax ≥ 20 pu)" ids = meta[:gfm_limiter_bypassed]
else
    println("GFM limiter : active (Imax below the bypass threshold)")
end
println("\nGFM duals   : ", joinpath(result.path_names[:pf_TS], "CSV_duals"),
        "\n              dual_gfm_droop, dual_gfm_limiter_Id/_Iq, dual_gfm_Eint_clip,",
        "\n              dual_gfm_Edroop_clip, dual_gfm_Pe/_Qe, dual_LB/UB_gfm_*")

#=
  Not reachable from RunConfig — module constants in functions_4_TS_gfm.jl:

    _GFM_EPS_E          = 1e-4    smoothing of the E-clip sqrt composition
    _GFM_EPS_NORM_V     = 1e-8    √ regularisation inside the limiter norms
    _GFM_EPS_LIM_I      = 1e-3    smooth-max scale of the current limiter
    _GFM_IMAX_NO_LIMIT  = 20.0    Imax at/above this replaces the clamp with
                                  unsaturated stator algebra (model changes shape)

  Each has one correct order of magnitude; varying them changes solver behaviour
  rather than the model. The GFM limits that ARE knobs are the `gfm_*` fields on
  TsBoundLimitsConfig above. Per-unit Xl, mp, mq, Kpv, Kiv, Emin/Emax, Pmin/Pmax,
  Tf and Imax come from gfm_dynamic_data.csv.

  Steady state and transient deliberately use different converter current
  ratings: the ACOPF circle uses the nameplate MVA base, because a sustained
  dispatch cannot bank on overload, while the transient limiter uses the CSV
  Imax — the brief overcurrent the semiconductor junctions tolerate.
=#
