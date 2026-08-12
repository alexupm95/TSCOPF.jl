#=
================================================================================
 test/runtests_controls.jl — machine controls: AVR (exciter) + TGOV1 governor
================================================================================
 Merges three files that covered overlapping ground on the same 9-bus SC case:

   runtests_governor.jl    → classical FULL_BUS + TGOV1
   runtests_dq_governor.jl → DQ_4TH FULL_BUS + TGOV1
   runtests_avr.jl         → DQ_4TH FULL_BUS + AVR, and AVR + TGOV1

 plus the "flat-start coupling (DQ + AVR + governor)" testset that used to sit in
 runtests_tsc_builder_fullbus.jl.

 Layout: every no-solve check (validation, factory, limiter/clamp builders) runs
 in the fast gate; the five end-to-end solves run only in the heavy tier
 (`RUN_HEAVY` from test/runtests.jl, or always when this file is run standalone).

 Note on reading results: `run_case!` releases the JuMP backend before returning,
 so `dyn_model_dict[:vars]` holds dead references. Numeric post-conditions are
 therefore checked against the exported CSVs and the returned `RGEN` DataFrame —
 never with `JuMP.value` on the returned dict.
================================================================================
=#

using Test
using JuMP
using CSV, DataFrames
using DataStructures: OrderedDict

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))

# Standalone runs execute everything; inside Pkg.test() the solves are heavy-tier.
const CONTROLS_RUN_SOLVES = !@isdefined(RUN_HEAVY) || RUN_HEAVY

const CTRL_SOLVED = (OPTIMAL, LOCALLY_SOLVED, ITERATION_LIMIT)
const CTRL_ZIP    = (1.0, 0.0, 0.0)   # (Z, I, P) — pure constant impedance
const CTRL_BASE_MVA = 100.0
# Short horizon: fault 0.01..0.21, post-fault 0.23..0.6 — exercises both windows fast.
const CTRL_SIM = TsSimulationConfig(δ_tol_deg = 100.0, t_end_sim = 0.6,
    t_step = 0.02, clearing_time = 0.2)

"""
    controls_run(; gen_order, include_avr, include_governor, limiter, ode_first_step,
                   gov_valve_min_pu)

One FULL_BUS TSC-ACOPF SC run with the requested control stack. `gen_order`
selects the classical or dq machine; the dq path needs `gen_dynamic_data_full.csv`.
`gov_valve_min_pu` raises the valve lower limit — the only side of the `:gov_valve`
spec a user can tighten, since the upper side is pinned to `pg_max/base_MVA` — which is
how the clamp is made to bind in the saturation test. Returns `(result, sys)` — `sys`
carries `DGEN_DYN` for the droop constants.
"""
function controls_run(; gen_order::GenOrder = DQ_4TH,
                        include_avr::Bool = false,
                        include_governor::Bool = false,
                        limiter::GovernorLimiter = GOV_NO_LIMIT,
                        ode_first_step::Symbol = :trapezoidal,
                        gov_valve_min_pu::Float64 = 0.0)
    cfg = RunConfig(;
        trans_stab = true,
        case = "9bus",
        base_MVA = CTRL_BASE_MVA,
        solver_name = "Ipopt",
        silent_solver = true,
        save_duals = true,
        save_ts_plots = false,
        save_optim_matrices = false,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        load_factor = 1.5,
        dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),
        transient = TransientConfig(
            simulation = CTRL_SIM,
            builder = TsBuilderConfig(
                limits = TsBoundLimitsConfig(gov_valve_min_pu = gov_valve_min_pu)),
            gen_dynamic_filename = "gen_dynamic_data_full.csv",
            dyn_model = DynModelConfig(
                gen_order = gen_order,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                zip_load_p = CTRL_ZIP,
                zip_load_q = CTRL_ZIP,
                include_avr = include_avr,
                include_governor = include_governor,
                governor_limiter = limiter,
                ode_first_step = ode_first_step,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
    validate_dyn_config!(cfg)
    sys = load_fixture_system(cfg)
    result = run_fixture_case!(cfg, sys)
    return (result = result, sys = sys)
end

"""Generator id from a `G<id>` column/row label in the exported CSVs."""
_gen_id(label::AbstractString) = parse(Int, replace(String(label), "G" => ""))

"""Max deviation of any generator trajectory column from its own first sample."""
function _max_excursion(df::DataFrame)
    m = 0.0
    for c in names(df)[2:end]          # column 1 is time
        col = df[!, c]
        m = max(m, maximum(abs.(col .- col[1])))
    end
    return m
end

"""
First generator column of an exported trajectory CSV, as a plain vector.

Read from disk rather than from `dyn_model_dict`: `run_case!` empties the JuMP model
before returning, so every `VariableRef` it holds is dead by the time a test runs.
"""
function _first_gen_series(result, filename::AbstractString)
    df = CSV.read(joinpath(result.path_names[:pf_TS_CSV], filename), DataFrame; delim = ';')
    return Float64.(df[!, 2])          # column 1 is time, column 2 is the first machine
end

"""Toy per-(gen,t) variable dict for the limiter / clamp unit tests (no solve)."""
function _toy_gen_time_vars(model::JuMP.Model, gens::Vector{Int}, nt::Int, prefix::String)
    d = OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}}()
    for g in gens
        d[g] = OrderedDict{Int, JuMP.VariableRef}()
        for t in 1:nt
            d[g][t] = JuMP.@variable(model, base_name = "$(prefix)[$g,$t]", start = 1.0)
        end
    end
    return d
end

@testset "TSC-ACOPF machine controls (AVR + TGOV1)" begin

    # ====================================================================== #
    #  No-solve: configuration, factory, and the limiter/clamp builders      #
    # ====================================================================== #

    @testset "config validation (fail fast)" begin
        # Governor on the Kron path is rejected (no ACOPF warm start there).
        cfg_kron = tsc_run_config(; dyn_model = DynModelConfig(
            network_form = KRON_REDUCED, mech_power_mode = USE_PM,
            bound_style_δ = :coi_box, include_governor = true,
            fault = FaultConfig(contingency_id = 2)))
        @test_throws ArgumentError validate_dyn_config!(cfg_kron)

        # Governor with USE_PG is rejected (the governor acts on P_m).
        cfg_pg = tsc_run_config(; dyn_model = DynModelConfig(
            network_form = FULL_BUS, mech_power_mode = USE_PG,
            include_governor = true, fault = FaultConfig(contingency_id = 2)))
        @test_throws ArgumentError validate_dyn_config!(cfg_pg)

        # AVR requires the 4th-order machine.
        cfg_avr_2nd = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(gen_order = CLASSICAL_2ND, include_avr = true))
        @test_throws ArgumentError validate_dyn_config!(cfg_avr_2nd)
    end

    @testset "factory wires the control flags" begin
        dq(; kwargs...) = TSCOPF.dynamic_gen_model(
            DynModelConfig(; gen_order = DQ_4TH, network_form = FULL_BUS,
                mech_power_mode = USE_PM, bound_style_δ = :coi_box, kwargs...);
            linearize = false)

        m_avr = dq(include_avr = true)
        @test m_avr isa TSCOPF.DqFullBusModel
        @test m_avr.include_avr
        @test !m_avr.include_governor

        m_gov = dq(include_governor = true)
        @test m_gov isa TSCOPF.DqFullBusModel
        @test m_gov.include_governor
        @test !m_gov.include_avr

        m_both = dq(include_avr = true, include_governor = true)
        @test m_both.include_avr && m_both.include_governor
    end

    @testset "AVR field-voltage smooth clamp (structural)" begin
        gens = [1, 2]; nt = 3
        e_min = Dict(1 => 0.0, 2 => 0.0)
        e_max = Dict(1 => 2.0, 2 => 1.5)
        m = JuMP.Model()
        unlim = _toy_gen_time_vars(m, gens, nt, "E_fd_unlim")
        sat, eq = TSCOPF.apply_avr_field_limit_smooth!(
            m, gens, unlim, collect(1.0:nt), e_min, e_max)
        @test sat !== unlim
        @test eq !== nothing
        @test all(length(eq[g]) == nt for g in gens)
    end

    @testset "AVR lead-lag row (structural)" begin
        # Trapezoidal lead-lag is normalized by n = 2·Tb + Δt so the coefficient on
        # E_LL[t] is exactly 1; that is what keeps dual_avr_leadlag comparable across
        # machines. The both-zero bypass must drop every generator from the build list.
        gens = [1, 2, 3]
        DGEN = DataFrame(bus = [1, 2, 3])
        DGEN_DYN = DataFrame(
            Ta_exc = [2.0, 0.0, 0.0],
            Tb_exc = [5.0, 0.0, 0.0],
            K_exc  = [100.0, 100.0, 100.0],
        )
        @test TSCOPF._avr_leadlag_gens(gens, DGEN_DYN) == [1]

        Δt = 0.02; nt = 3
        Ta, Tb = 2.0, 5.0
        n = 2 * Tb + Δt
        m = JuMP.Model()
        E_LL = _toy_gen_time_vars(m, [1], nt, "E_LL")
        V_ref = OrderedDict(1 => JuMP.@variable(m, base_name = "Vref[1]"))
        V_tf = OrderedDict(1 => OrderedDict(
            t => JuMP.@variable(m, base_name = "V[1,$t]") for t in 1:nt))
        y0 = OrderedDict(1 => JuMP.@variable(m, base_name = "y0[1]"))
        u0 = OrderedDict(1 => JuMP.@variable(m, base_name = "u0[1]"))
        eq = TSCOPF.eq_const_avr_leadlag!(
            m, [1], DGEN, DGEN_DYN, E_LL, V_ref, V_tf, y0, u0,
            zeros(nt), Δt; ode_first_step = :trapezoidal)
        for t in 1:nt
            con = eq[1][t]
            @test JuMP.normalized_coefficient(con, E_LL[1][t]) ≈ 1.0
            y_prev = t == 1 ? y0[1] : E_LL[1][t - 1]
            @test JuMP.normalized_coefficient(con, y_prev) ≈ -(2 * Tb - Δt) / n
            # u_curr = V_ref − V[t]; u_prev is the free u0 at t=1, else V_ref − V[t−1].
            if t == 1
                @test JuMP.normalized_coefficient(con, V_ref[1]) ≈ -(2 * Ta + Δt) / n
                @test JuMP.normalized_coefficient(con, V_tf[1][t]) ≈ (2 * Ta + Δt) / n
                @test JuMP.normalized_coefficient(con, u0[1]) ≈ (2 * Ta - Δt) / n
            else
                @test JuMP.normalized_coefficient(con, V_ref[1]) ≈
                    -((2 * Ta + Δt) / n) + ((2 * Ta - Δt) / n)
                @test JuMP.normalized_coefficient(con, V_tf[1][t]) ≈ (2 * Ta + Δt) / n
                @test JuMP.normalized_coefficient(con, V_tf[1][t - 1]) ≈ -(2 * Ta - Δt) / n
            end
        end
    end

    @testset "governor valve limiters (structural)" begin
        gens = [1, 2]; nt = 3
        pmin = Dict(1 => 0.0, 2 => 0.0)
        pmax = Dict(1 => 2.0, 2 => 1.5)

        # NO_LIMIT: output is the raw state, no extra constraints.
        m0 = JuMP.Model()
        pv0 = _toy_gen_time_vars(m0, gens, nt, "pv")
        out0, eq0, ineq0 = TSCOPF.apply_gov_valve_limit_none!(m0, gens, pv0, zeros(nt), pmin, pmax)
        @test out0 === pv0
        @test eq0 === nothing && ineq0 === nothing

        # SMOOTH: separate limited output tied to the raw state by one equality per (gen,t).
        ms = JuMP.Model()
        pvs = _toy_gen_time_vars(ms, gens, nt, "pv")
        outs, eqs, ineqs = TSCOPF.apply_gov_valve_limit_smooth!(ms, gens, pvs, zeros(nt), pmin, pmax)
        @test outs !== pvs
        @test eqs !== nothing && ineqs === nothing
        @test all(length(eqs[g]) == nt for g in gens)

        # HARD_BOUND: raw state plus explicit lower/upper bound constraints.
        # All three limiters share one contract — `(Pv_output, extra_eq, extra_ineq)` —
        # so the bound pair belongs in the THIRD slot, not spread over slots 2 and 3.
        # It used to return `(Pv_raw, lower, upper)`, which made `_store_gov_limit!`
        # file the lower bounds as an equality family and destructure the upper
        # container as if it were the pair.
        mh = JuMP.Model()
        pvh = _toy_gen_time_vars(mh, gens, nt, "pv")
        outh, eqh, ineqh = TSCOPF.apply_gov_valve_limit_hard!(mh, gens, pvh, zeros(nt), pmin, pmax)
        @test outh === pvh
        @test eqh === nothing
        @test ineqh isa Tuple && length(ineqh) == 2
        lowh, uph = ineqh
        @test all(length(lowh[g]) == nt for g in gens)
        @test all(length(uph[g]) == nt for g in gens)

        # Under the VARIABLE encoding the same limiter stamps JuMP bounds on the raw
        # state and reports it with a marker, leaving the dual export to the manifest.
        mv = JuMP.Model()
        pvv = _toy_gen_time_vars(mv, gens, nt, "pv")
        outv, eqv, ineqv = TSCOPF.apply_gov_valve_limit_hard!(
            mv, gens, pvv, zeros(nt), pmin, pmax; encoding = TSCOPF.VARIABLE)
        @test outv === pvv
        @test eqv === nothing
        @test ineqv === :variable_bounds
        @test all(JuMP.has_lower_bound(pvv[g][t]) && JuMP.has_upper_bound(pvv[g][t])
                  for g in gens, t in 1:nt)

        # Dispatcher routes each mode to the matching builder.
        md = JuMP.Model()
        pvd = _toy_gen_time_vars(md, gens, nt, "pv")
        _, eq_d, ineq_d = TSCOPF.apply_gov_valve_limit!(md, GOV_SMOOTH, gens, pvd, zeros(nt), pmin, pmax)
        @test eq_d !== nothing && ineq_d === nothing
    end

    @testset "governor turbine row sees only the limited valve (structural)" begin
        # The turbine block discretizes T3·dPm/dt + Pm = T2·dPv/dt + Pv directly, on the
        # *limited* Pv alone. It used to be stamped in the substituted state form
        #     T3·dPm/dt = (1−T2/T1)·Pv + [(T2/T1)/R]·(P_ref−Δω) − Pm,
        # which is exact only while the valve is unsaturated: under GOV_SMOOTH the
        # feedforward term carried the raw, unclamped droop signal straight past the
        # limiter, so valve saturation never capped P_m. These coefficient checks are what
        # fail if that substitution — or a sign — creeps back in.
        gens = [1, 2]; nt = 3
        Δt = 0.02
        DGEN_DYN = DataFrame(T2 = [0.4, 0.7], T3 = [8.0, 5.0])

        m  = JuMP.Model()
        Pm = _toy_gen_time_vars(m, gens, nt, "Pm")
        Pv = _toy_gen_time_vars(m, gens, nt, "Pv")
        Pm0 = OrderedDict(g => JuMP.@variable(m, base_name = "Pm0[$g]") for g in gens)
        Pv0 = OrderedDict(g => JuMP.@variable(m, base_name = "Pv0[$g]") for g in gens)

        eq = TSCOPF.eq_const_gov_mech!(m, gens, DGEN_DYN, Pm, Pv, Pm0, Pv0,
                                       zeros(nt), Δt; ode_first_step = :trapezoidal)

        for g in gens
            T2, T3 = DGEN_DYN.T2[g], DGEN_DYN.T3[g]
            c, lead = Δt / (2 * T3), T2 / T3
            for t in 1:nt
                con = eq[g][t]
                prev_m = t == 1 ? Pm0[g] : Pm[g][t - 1]
                prev_v = t == 1 ? Pv0[g] : Pv[g][t - 1]
                @test JuMP.normalized_coefficient(con, Pm[g][t]) ≈  (1 + c)
                @test JuMP.normalized_coefficient(con, prev_m)   ≈ -(1 - c)
                @test JuMP.normalized_coefficient(con, Pv[g][t]) ≈ -(lead + c)
                @test JuMP.normalized_coefficient(con, prev_v)   ≈  (lead - c)
                # Exactly four terms: no droop feedforward, no Δω, no P_ref.
                @test length(JuMP.constraint_object(con).func.terms) == 4
            end
        end

        # Physical invariant, independent of the coefficients above: at any equilibrium
        # Pm = Pv = P₀ (both derivatives zero) the residual must vanish, for either
        # first-step rule. A wrong lead ratio or a flipped sign breaks this.
        for first_step in (:trapezoidal, :backward_euler)
            me  = JuMP.Model()
            Pme = _toy_gen_time_vars(me, gens, nt, "Pm")
            Pve = _toy_gen_time_vars(me, gens, nt, "Pv")
            Pm0e = OrderedDict(g => JuMP.@variable(me, base_name = "Pm0[$g]") for g in gens)
            Pv0e = OrderedDict(g => JuMP.@variable(me, base_name = "Pv0[$g]") for g in gens)
            eqe = TSCOPF.eq_const_gov_mech!(me, gens, DGEN_DYN, Pme, Pve, Pm0e, Pv0e,
                                            zeros(nt), Δt; ode_first_step = first_step)
            for g in gens, t in 1:nt
                func = JuMP.constraint_object(eqe[g][t]).func
                @test JuMP.value(_ -> 0.83, func) ≈ 0.0 atol = 1e-12
            end
        end
    end

    @testset "governor valve row feeds back the limited valve (structural)" begin
        # The valve integrator is anti-windup: its previous-step term is the *limited* output
        # Pv_sat, not the raw state it integrates. With raw feedback the state is a free
        # integrator, so under GOV_SMOOTH it ramps past the clamp and has to unwind that excess
        # before the output can leave saturation. `Pv_sat === Pv_raw` for every other mode, so
        # only distinct containers expose the difference — which is what this builds.
        gens = [1, 2]; nt = 3
        Δt = 0.02
        DGEN_DYN = DataFrame(R = [0.05, 0.04], T1 = [0.5, 0.3])

        m      = JuMP.Model()
        Pv_raw = _toy_gen_time_vars(m, gens, nt, "Pv_raw")
        Pv_sat = _toy_gen_time_vars(m, gens, nt, "Pv_sat")
        Δω     = _toy_gen_time_vars(m, gens, nt, "dw")
        P_ref  = OrderedDict(g => JuMP.@variable(m, base_name = "P_ref[$g]") for g in gens)
        Pv0    = OrderedDict(g => JuMP.@variable(m, base_name = "Pv0[$g]") for g in gens)

        eq = TSCOPF.eq_const_gov_valve!(m, gens, DGEN_DYN, Pv_raw, Pv_sat, P_ref, Δω,
                                        Pv0, 0.0, zeros(nt), Δt; ode_first_step = :trapezoidal)

        for g in gens
            R, T1 = DGEN_DYN.R[g], DGEN_DYN.T1[g]
            c = Δt / (2 * T1)
            for t in 1:nt
                con = eq[g][t]
                @test JuMP.normalized_coefficient(con, Pv_raw[g][t]) ≈ (1 + c)
                if t == 1
                    @test JuMP.normalized_coefficient(con, Pv0[g]) ≈ -(1 - c)
                else
                    # The history term is the saturated output; the raw state of the previous
                    # step must not appear in the row at all.
                    @test JuMP.normalized_coefficient(con, Pv_sat[g][t - 1]) ≈ -(1 - c)
                    @test JuMP.normalized_coefficient(con, Pv_raw[g][t - 1]) == 0.0
                end
                @test JuMP.normalized_coefficient(con, P_ref[g]) ≈ -Δt / (R * T1)
            end
        end
    end

    # ====================================================================== #
    #  End-to-end solves (heavy tier)                                        #
    # ====================================================================== #
    if CONTROLS_RUN_SOLVES

        @testset "classical FULL_BUS + TGOV1 — SC end-to-end" begin
            out = controls_run(gen_order = CLASSICAL_2ND, include_governor = true)
            result, sys = out.result, out.sys
            @test result.status in CTRL_SOLVED
            assert_timestamped_results!(result.path_names)

            dmd = result.dyn_model_dict
            for key in (:P_ref, :Pv_tf, :Pm_tf, :Pv_tpf, :Pm_tpf)
                @test haskey(dmd[:vars], key)
            end
            @test haskey(dmd[:eq_const], :eq_const_gov_valve_tf)
            @test haskey(dmd[:eq_const], :eq_const_gov_mech_tpf)
            @test haskey(dmd[:eq_const], :eq_const_Pm_init)   # governor self-pins P_m = P_g

            # The TXT dump used to route the whole governor block through a
            # `gen_order == DQ_4TH` gate, so a classical FULL_BUS run wrote a details
            # file with no governor variables and no governor rows — including the
            # GOV_SMOOTH anti-windup equality — while duals and CSVs were complete.
            txt = read(joinpath(result.path_names[:pf_TS], "dynamic_model_details.txt"), String)
            for marker in ("Governor Init — Set-point P_ref",
                           "Governor Valve ODE (P_valve_raw)",
                           "Governor Mech Power ODE (P_mech)",
                           "P_ref[", "P_valve_raw_tf[", "P_mech_tpf[")
                @test occursin(marker, txt)
            end

            csv_dir  = result.path_names[:pf_TS_CSV]
            duals_dir = result.path_names[:pf_TS_CSV_duals]
            for f in ("governor_P_mech.csv", "governor_P_valve.csv", "governor_P_ref.csv")
                @test isfile(joinpath(csv_dir, f))
            end
            for f in ("dual_Pref_init.csv", "dual_gov_valve.csv", "dual_gov_mech.csv")
                @test isfile(joinpath(duals_dir, f))
            end

            # Set-point equilibrium P_ref = R · P_m, read back from the exported CSV.
            # `eq_const_Pm_init` pins P_m to the dispatched P_g, and RGEN survives the
            # backend release, so this is an exact cross-check of the droop wiring.
            p_ref = CSV.read(joinpath(csv_dir, "governor_P_ref.csv"), DataFrame; delim = ';')
            pg_pu = Dict(Int(r.id) => r.p_g / CTRL_BASE_MVA for r in eachrow(result.RGEN))
            for row in eachrow(p_ref)
                g = _gen_id(row.gen)
                @test row.P_ref_pu ≈ sys.DGEN_DYN.R[g] * pg_pu[g] atol = 1e-6
                @test row.P_ref_pu > 0.0    # anchored to real dispatch, not a drift to zero
            end

            # The governor actually responds: the fast valve leaves its equilibrium.
            pv = CSV.read(joinpath(csv_dir, "governor_P_valve.csv"), DataFrame; delim = ';')
            @test _max_excursion(pv) > 1e-5
        end

        @testset "GOV_SMOOTH valve clamp actually caps the mechanical power" begin
            # The turbine row must see the *clamped* valve and nothing else. The old
            # substituted form carried a feedforward `[(T2/T1)/R]·(P_ref − Δω)` around the
            # limiter — on this case that gain is (2.5/0.5)/0.05 = 100 — so a saturated
            # valve left P_m free to keep falling. Both runs below use GOV_SMOOTH; only
            # the valve lower limit differs, which isolates the clamp as the sole cause.
            gov(min_pu) = controls_run(gen_order = CLASSICAL_2ND, include_governor = true,
                                       limiter = GOV_SMOOTH, gov_valve_min_pu = min_pu)
            read_gov(res, f) = CSV.read(joinpath(res.path_names[:pf_TS_CSV], f),
                                        DataFrame; delim = ';')

            # 1. Calibration run with the clamp inactive (0 pu), to locate the valve travel.
            loose = gov(0.0).result
            @test loose.status in CTRL_SOLVED
            pv_loose = read_gov(loose, "governor_P_valve.csv")
            machines = names(pv_loose)[2:end]
            start_MW  = [Float64(pv_loose[1, c])            for c in machines]
            trough_MW = [minimum(Float64.(pv_loose[!, c]))  for c in machines]

            # The fault accelerates the machines, so (P_ref − Δω)/R drops and every valve
            # closes. Clamp 2 % under the *lowest* equilibrium: above that the pre-fault
            # anchor Pv(0) = P_m would itself be clamped on the smallest machine.
            @test all(trough_MW .< start_MW)
            clamp_MW = 0.98 * minimum(start_MW)
            # Without this the run below would prove nothing — the clamp has to bind.
            @test any(trough_MW .< clamp_MW)

            # 2. Same case, clamp binding.
            tight = gov(clamp_MW / CTRL_BASE_MVA).result
            @test tight.status in CTRL_SOLVED
            pv_tight = read_gov(tight, "governor_P_valve.csv")
            pm_tight = read_gov(tight, "governor_P_mech.csv")
            # sqrt smoothing with ρ = 1e-4 lets the clamp be soft by ≈ √ρ/2 pu; 0.02 pu of
            # slack is generous next to the excursions being measured.
            tol_MW = 0.02 * CTRL_BASE_MVA

            for c in machines
                @test minimum(Float64.(pv_tight[!, c])) > clamp_MW - tol_MW
                # The claim under test. T2 = 2.5 < T3 = 7.5 makes the turbine a
                # lag-dominant lead-lag, so P_m cannot leave the range of its input — and
                # its input is now the clamped valve, floored at clamp_MW.
                floor_MW = min(clamp_MW, Float64(pv_tight[1, c]))
                @test minimum(Float64.(pm_tight[!, c])) > floor_MW - tol_MW
            end

            # And the clamp must move P_m at all: if the two runs agreed, the mechanical
            # power would not be listening to the limiter in the first place.
            pm_loose = read_gov(loose, "governor_P_mech.csv")
            @test maximum(abs.(Float64.(pm_tight[!, 2]) .- Float64.(pm_loose[!, 2]))) > 1e-3
        end

        @testset "DQ_4TH + TGOV1 — SC end-to-end" begin
            result = controls_run(include_governor = true).result
            @test result.status in CTRL_SOLVED
            dmd = result.dyn_model_dict
            @test haskey(dmd[:vars], :P_ref)
            @test haskey(dmd[:vars], :Pm_tf)
            @test haskey(dmd[:vars], :Pv_tf)
            @test haskey(dmd[:eq_const], :eq_const_Pref_init)
            @test haskey(dmd[:eq_const], :eq_const_gov_valve_tf)
            @test haskey(dmd[:eq_const], :eq_const_gov_mech_tf)
            @test get(dmd[:meta], :include_governor, false)
            @test get(dmd[:meta], :gen_order, "") == "DQ_4TH"

            csv_dir   = result.path_names[:pf_TS_CSV]
            duals_dir = result.path_names[:pf_TS_CSV_duals]
            for f in ("governor_P_ref.csv", "governor_P_mech.csv", "governor_P_valve.csv")
                @test isfile(joinpath(csv_dir, f))
            end
            for f in ("dual_Pref_init.csv", "dual_gov_valve.csv", "dual_gov_mech.csv")
                @test isfile(joinpath(duals_dir, f))
            end
        end

        @testset "DQ_4TH + AVR — SC end-to-end" begin
            result = controls_run(include_avr = true).result
            @test result.status in CTRL_SOLVED
            dmd = result.dyn_model_dict
            @test haskey(dmd[:vars], :V_ref)
            @test haskey(dmd[:vars], :E_fd_unlim_tf)
            @test haskey(dmd[:vars], :E_fd_tf)
            @test haskey(dmd[:eq_const], :eq_const_Efd_init)
            @test haskey(dmd[:eq_const], :eq_const_E_fd_unlim_tf)
            @test haskey(dmd[:eq_const], :eq_const_E_fd_tf)
            @test get(dmd[:meta], :include_avr, false)
            # Fixture ships Ta_exc = Tb_exc = 0 → lead-lag bypassed (no vars / duals).
            @test !haskey(dmd[:vars], :E_LL_tf)
            @test !haskey(dmd[:eq_const], :eq_const_avr_leadlag_tf)

            csv_dir   = result.path_names[:pf_TS_CSV]
            duals_dir = result.path_names[:pf_TS_CSV_duals]
            for f in ("avr_V_ref.csv", "dq_E_fd.csv", "dq_E_fd_pu.csv")
                @test isfile(joinpath(csv_dir, f))
            end
            @test !isfile(joinpath(csv_dir, "dq_E_LL_pu.csv"))
            for f in ("dual_Vref_init.csv", "dual_avr_E_fd.csv", "dual_avr_E_fd_sat.csv")
                @test isfile(joinpath(duals_dir, f))
            end
            @test !isfile(joinpath(duals_dir, "dual_avr_leadlag.csv"))
        end

        @testset "DQ_4TH + AVR lead-lag — SC end-to-end (ANDES SEXS defaults)" begin
            cfg = RunConfig(;
                trans_stab = true, case = "9bus", base_MVA = CTRL_BASE_MVA,
                solver_name = "Ipopt", silent_solver = true, save_duals = true,
                save_ts_plots = false, save_optim_matrices = false,
                overwrite_results = TEST_OVERWRITE_RESULTS, load_factor = 1.5,
                dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),
                transient = TransientConfig(
                    simulation = CTRL_SIM,
                    gen_dynamic_filename = "gen_dynamic_data_full.csv",
                    dyn_model = DynModelConfig(
                        gen_order = DQ_4TH, network_form = FULL_BUS,
                        mech_power_mode = USE_PM, bound_style_δ = :coi_box,
                        zip_load_p = CTRL_ZIP, zip_load_q = CTRL_ZIP,
                        include_avr = true,
                        fault = FaultConfig(fault_type = SC, contingency_id = 2),
                    ),
                ),
            )
            validate_dyn_config!(cfg)
            sys = load_fixture_system(cfg)
            # ANDES SEXS defaults: TB = 5, TA/TB = 0.4 → TA = 2.
            sys.DGEN_DYN.Ta_exc .= 2.0
            sys.DGEN_DYN.Tb_exc .= 5.0
            # The mutation happens after load, so assert the values the builder is about to
            # consume are values the validator accepts. Without this the suite validates
            # lead-lag data it never builds and builds lead-lag data it never validates —
            # a rule change on either side could keep both halves green while making a
            # legitimate configuration unbuildable.
            @test TSCOPF.validate_dyn_data!(cfg, sys.DGEN_DYN) === nothing
            result = run_fixture_case!(cfg, sys)
            @test result.status in CTRL_SOLVED
            dmd = result.dyn_model_dict
            @test haskey(dmd[:vars], :E_LL_tf)
            @test haskey(dmd[:vars], :E_LL_tpf)
            @test haskey(dmd[:eq_const], :eq_const_avr_leadlag_tf)
            @test haskey(dmd[:eq_const], :eq_const_avr_leadlag_tpf)
            @test isfile(joinpath(result.path_names[:pf_TS_CSV], "dq_E_LL_pu.csv"))
            @test isfile(joinpath(result.path_names[:pf_TS_CSV_duals], "dual_avr_leadlag.csv"))
        end

        @testset "DQ_4TH + AVR lead-lag — Ta=Tb pass-through matches bypass" begin
            # Building the identity block (Ta = Tb = 5) must reproduce the 0;0 bypass
            # objective and dispatch to solver tolerance — the row is correct independently
            # of the bypass optimization.
            function _avr_run_with_ll!(Ta, Tb)
                cfg = RunConfig(;
                    trans_stab = true, case = "9bus", base_MVA = CTRL_BASE_MVA,
                    solver_name = "Ipopt", silent_solver = true, save_duals = false,
                    save_ts_plots = false, save_optim_matrices = false,
                    overwrite_results = TEST_OVERWRITE_RESULTS, load_factor = 1.5,
                    dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),
                    transient = TransientConfig(
                        simulation = CTRL_SIM,
                        gen_dynamic_filename = "gen_dynamic_data_full.csv",
                        dyn_model = DynModelConfig(
                            gen_order = DQ_4TH, network_form = FULL_BUS,
                            mech_power_mode = USE_PM, bound_style_δ = :coi_box,
                            zip_load_p = CTRL_ZIP, zip_load_q = CTRL_ZIP,
                            include_avr = true,
                            fault = FaultConfig(fault_type = SC, contingency_id = 2),
                        ),
                    ),
                )
                validate_dyn_config!(cfg)
                sys = load_fixture_system(cfg)
                sys.DGEN_DYN.Ta_exc .= Ta
                sys.DGEN_DYN.Tb_exc .= Tb
                return run_fixture_case!(cfg, sys)
            end
            res_bypass = _avr_run_with_ll!(0.0, 0.0)
            res_pass   = _avr_run_with_ll!(5.0, 5.0)
            @test res_bypass.status in CTRL_SOLVED
            @test res_pass.status in CTRL_SOLVED
            @test !haskey(res_bypass.dyn_model_dict[:vars], :E_LL_tf)
            @test haskey(res_pass.dyn_model_dict[:vars], :E_LL_tf)
            @test isapprox(res_pass.obj_MVA, res_bypass.obj_MVA; rtol = 1e-5)
            @test isapprox(res_pass.RGEN.p_g, res_bypass.RGEN.p_g; rtol = 1e-5)
        end

        @testset "DQ_4TH + AVR + TGOV1 — SC end-to-end (reference control stack)" begin
            result = controls_run(include_avr = true, include_governor = true).result
            @test result.status in CTRL_SOLVED
            dmd = result.dyn_model_dict
            @test haskey(dmd[:vars], :V_ref)
            @test haskey(dmd[:vars], :E_fd_tf)
            @test haskey(dmd[:vars], :Pm_tf)
            @test haskey(dmd[:vars], :P_ref)
            @test get(dmd[:meta], :include_avr, false)
            @test get(dmd[:meta], :include_governor, false)
        end

        # `ode_first_step` reaches the classical FULL_BUS swing rows (it used to be
        # threaded only to the dq builder, so a CLASSICAL_2ND run silently integrated
        # trapezoidally whatever was asked for). Backward Euler drops the Δω_0 term
        # from the t=1 angle row, so that row has one term fewer than the trapezoidal
        # one; every later row is unchanged.
        @testset "CLASSICAL_2ND FULL_BUS — ode_first_step reaches the swing rows" begin
            res_be = controls_run(gen_order = CLASSICAL_2ND,
                                  ode_first_step = :backward_euler).result
            res_tr = controls_run(gen_order = CLASSICAL_2ND,
                                  ode_first_step = :trapezoidal).result
            @test res_be.dyn_model_dict[:meta][:ode_first_step] === :backward_euler
            @test res_tr.dyn_model_dict[:meta][:ode_first_step] === :trapezoidal
            # Trajectories must differ: the first integration step is a different rule.
            δ_be = _first_gen_series(res_be, "angle_abs.csv")
            δ_tr = _first_gen_series(res_tr, "angle_abs.csv")
            @test length(δ_be) == length(δ_tr)
            @test !isapprox(δ_be, δ_tr; atol = 1e-10)
        end

    end   # CONTROLS_RUN_SOLVES
end
