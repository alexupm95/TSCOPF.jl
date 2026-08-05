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
    controls_run(; gen_order, include_avr, include_governor, limiter, ode_first_step)

One FULL_BUS TSC-ACOPF SC run with the requested control stack. `gen_order`
selects the classical or dq machine; the dq path needs `gen_dynamic_data_full.csv`.
Returns `(result, sys)` — `sys` carries `DGEN_DYN` for the droop constants.
"""
function controls_run(; gen_order::GenOrder = DQ_4TH,
                        include_avr::Bool = false,
                        include_governor::Bool = false,
                        limiter::GovernorLimiter = GOV_NO_LIMIT,
                        ode_first_step::Symbol = :trapezoidal)
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
            builder = TsBuilderConfig(),
            gen_dynamic_filename = "gen_dynamic_data_full.csv",
            dyn_model = DynModelConfig(
                gen_order = gen_order,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style = :coi_box,
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
            bound_style = :coi_box, include_governor = true,
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
                mech_power_mode = USE_PM, bound_style = :coi_box, kwargs...);
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

            csv_dir   = result.path_names[:pf_TS_CSV]
            duals_dir = result.path_names[:pf_TS_CSV_duals]
            for f in ("avr_V_ref.csv", "dq_E_fd.csv", "dq_E_fd_pu.csv")
                @test isfile(joinpath(csv_dir, f))
            end
            for f in ("dual_Vref_init.csv", "dual_avr_E_fd.csv", "dual_avr_E_fd_sat.csv")
                @test isfile(joinpath(duals_dir, f))
            end
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
