#=
 Fast unit tests (no full pipeline solves). Included by `Pkg.test()`.

 Covers CSV parsers, `DynModelConfig` validation, the dynamic model factory,
 default `TransientConfig`, and the Pacc_COI inertia regression guard (C2).
=#

using DataFrames

const INPUT_9BUS = fixture_case("9bus")

@testset "Fast unit tests" begin
    assert_quadratic_opf_objective!()

    @testset "gen_dynamic_data CSV parser" begin
        df_min = TSCOPF.Read_Gen_Dynamic_Data(INPUT_9BUS; filename = "gen_dynamic_data.csv")
        @test nrow(df_min) == 3
        @test names(df_min) ⊇ ["id", "bus", "Xd_tr", "H", "D"]
        @test !("E_int" in names(df_min))
        @test df_min.Xd_tr ≈ [0.0608, 0.1198, 0.1813]
        @test !TSCOPF.has_full_machine_data(df_min)

        df_full = TSCOPF.Read_Gen_Dynamic_Data(INPUT_9BUS; filename = "gen_dynamic_data_full.csv")
        @test nrow(df_full) == 3
        @test df_full.Xd_tr ≈ [0.0608, 0.1198, 0.1813]
        @test df_full.Xd ≈ [0.2432, 0.4792, 0.7252]
        @test df_full.K_exc ≈ fill(100.0, 3)
        @test TSCOPF.has_full_machine_data(df_full)

        dyn_dq = DynModelConfig(gen_order = DQ_4TH, include_avr = false, include_governor = false)
        @test TSCOPF.has_required_dyn_columns(dyn_dq, df_full)
        df_dq_only = copy(df_full)
        for col in (:T_exc, :K_exc, :R, :T1, :T2, :T3)
            df_dq_only[!, col] .= NaN
        end
        @test TSCOPF.has_full_machine_data(df_dq_only)
        @test TSCOPF.has_required_dyn_columns(dyn_dq, df_dq_only)
        dyn_avr = DynModelConfig(gen_order = DQ_4TH, include_avr = true, include_governor = false)
        @test !TSCOPF.has_required_dyn_columns(dyn_avr, df_dq_only)
        dyn_gov = DynModelConfig(gen_order = DQ_4TH, include_avr = false, include_governor = true)
        @test !TSCOPF.has_required_dyn_columns(dyn_gov, df_dq_only)

        for col in (:bus, :Xd_tr, :H, :D)
            @test df_min[!, col] ≈ df_full[!, col]
        end
    end

    @testset "DynModelConfig validation" begin
        cfg_default = RunConfig()
        @test validate_dyn_config!(cfg_default) === nothing

        # Each (Z, I, P) vector is checked on its own — a shared check would let a
        # mismatched pair through.
        bad_zip_p = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(zip_load_p = (0.5, 0.5, 0.5)))
        @test_throws ArgumentError validate_dyn_config!(bad_zip_p)

        bad_zip_q = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(zip_load_q = (0.5, 0.5, 0.5)))
        @test_throws ArgumentError validate_dyn_config!(bad_zip_q)

        # REE style: constant-current P, constant-admittance Q. Different vectors, each
        # summing to 1 — valid on FULL_BUS.
        ree_zip = reconfigure_dyn(tsc_run_config(); dyn_model = DynModelConfig(
            network_form = FULL_BUS, mech_power_mode = USE_PM, bound_style = :coi_box,
            zip_load_p = (0.0, 1.0, 0.0), zip_load_q = (1.0, 0.0, 0.0)))
        @test validate_dyn_config!(ree_zip) === nothing

        bad_dq_kron = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(gen_order = DQ_4TH, network_form = KRON_REDUCED))
        @test_throws ArgumentError validate_dyn_config!(bad_dq_kron)

        bad_gov_pg = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(include_governor = true, mech_power_mode = USE_PG))
        @test_throws ArgumentError validate_dyn_config!(bad_gov_pg)

        bad_gov_kron = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(include_governor = true, mech_power_mode = USE_PM,
                network_form = KRON_REDUCED, bound_style = :coi_box))
        @test_throws ArgumentError validate_dyn_config!(bad_gov_kron)

        bad_dcopf_dq = reconfigure_dyn(tsc_run_config(type_model = "DCOPF");
            dyn_model = DynModelConfig(gen_order = DQ_4TH, network_form = FULL_BUS))
        @test_throws ArgumentError validate_dyn_config!(bad_dcopf_dq)

        ok_future = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(
                network_form = FULL_BUS, mech_power_mode = USE_PM,
                include_governor = true, bound_style = :coi_box))
        @test validate_dyn_config!(ok_future) === nothing

        bad_Δω_tol = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(constrain_Δω_COI = true, Δω_tol_pu = 0.0))
        @test_throws ArgumentError validate_dyn_config!(bad_Δω_tol)

        ok_Δω_COI = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(constrain_Δω_COI = true, Δω_tol_pu = 0.5))
        @test validate_dyn_config!(ok_Δω_COI) === nothing

        ok_Δω_asym = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(constrain_Δω_COI = true,
                Δω_tol_pu_lower = 0.05, Δω_tol_pu_upper = 0.03))
        @test validate_dyn_config!(ok_Δω_asym) === nothing

        for bad in (-0.1, 0.0)
            bad_lo = reconfigure_dyn(tsc_run_config();
                dyn_model = DynModelConfig(constrain_Δω_COI = true, Δω_tol_pu_lower = bad))
            @test_throws ArgumentError validate_dyn_config!(bad_lo)
            bad_hi = reconfigure_dyn(tsc_run_config();
                dyn_model = DynModelConfig(constrain_Δω_COI = true, Δω_tol_pu_upper = bad))
            @test_throws ArgumentError validate_dyn_config!(bad_hi)
        end

        bad_pm_swing = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(mech_power_mode = USE_PM, bound_style = :swing_propagated))
        @test_throws ArgumentError validate_dyn_config!(bad_pm_swing)

        # FULL_BUS needs P_m as its own state. The builder also throws, but only after
        # the warm-start ACOPF has been solved — this rule fires before any solve.
        bad_fullbus_pg = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(network_form = FULL_BUS, mech_power_mode = USE_PG))
        @test_throws ArgumentError validate_dyn_config!(bad_fullbus_pg)

        # Backward Euler has no Kron implementation (those swing rows are trapezoidal
        # at every step), so asking for it must fail rather than be silently ignored.
        bad_kron_be = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(network_form = KRON_REDUCED,
                ode_first_step = :backward_euler))
        @test_throws ArgumentError validate_dyn_config!(bad_kron_be)

        ok_fullbus_be = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(network_form = FULL_BUS, mech_power_mode = USE_PM,
                bound_style = :coi_box, ode_first_step = :backward_euler))
        @test validate_dyn_config!(ok_fullbus_be) === nothing
    end

    @testset "dynamic_gen_model factory" begin
        m_ac = TSCOPF.dynamic_gen_model(DynModelConfig(); linearize = false)
        @test m_ac isa TSCOPF.ClassicalKronModel
        @test !m_ac.linearize
        @test m_ac.mech_power_mode == USE_PG
        @test TSCOPF.network_form(m_ac) == KRON_REDUCED
        @test TSCOPF.gen_order(m_ac) == CLASSICAL_2ND

        m_dc = TSCOPF.dynamic_gen_model(DynModelConfig(); linearize = true)
        @test m_dc.linearize
        @test m_dc.mech_power_mode == USE_PG

        m_pm = TSCOPF.dynamic_gen_model(
            DynModelConfig(mech_power_mode = USE_PM, constrain_Δω_COI = true, bound_style = :coi_box);
            linearize = true,
        )
        @test m_pm.mech_power_mode == USE_PM
        @test m_pm.constrain_Δω_COI
        # Unset overrides → symmetric pair from Δω_tol_pu.
        @test m_pm.Δω_tol == (-0.5, 0.5)

        # Asymmetric overrides are positive magnitudes below/above COI.
        m_asym = TSCOPF.dynamic_gen_model(
            DynModelConfig(mech_power_mode = USE_PM, bound_style = :coi_box,
                constrain_Δω_COI = true, Δω_tol_pu_lower = 0.05, Δω_tol_pu_upper = 0.03);
            linearize = true,
        )
        @test m_asym.Δω_tol == (-0.05, 0.03)
        # One override set → the other side falls back to Δω_tol_pu.
        @test TSCOPF.Δω_tol_tuple(
            DynModelConfig(Δω_tol_pu = 0.4, Δω_tol_pu_lower = 0.1)) == (-0.1, 0.4)
        @test_throws ArgumentError TSCOPF.Δω_tol_tuple(
            DynModelConfig(Δω_tol_pu_upper = -0.2))

        m_fb = TSCOPF.dynamic_gen_model(
            DynModelConfig(network_form = FULL_BUS, mech_power_mode = USE_PM, bound_style = :coi_box);
            linearize = false,
        )
        @test m_fb isa TSCOPF.ClassicalFullBusModel
        @test TSCOPF.network_form(m_fb) == FULL_BUS
        @test m_fb.mech_power_mode == USE_PM
        @test m_fb.bound_style == :coi_box

        @test_throws ArgumentError TSCOPF.dynamic_gen_model(
            DynModelConfig(network_form = FULL_BUS); linearize = true)
        m_dq = TSCOPF.dynamic_gen_model(
            DynModelConfig(gen_order = DQ_4TH, network_form = FULL_BUS,
                mech_power_mode = USE_PM, bound_style = :coi_box);
            linearize = false)
        @test m_dq isa TSCOPF.DqFullBusModel
        @test TSCOPF.gen_order(m_dq) == DQ_4TH
    end

    @testset "TransientConfig defaults" begin
        tc = TSCOPF.default_transient_config()
        @test tc.dyn_model.gen_order == CLASSICAL_2ND
        @test tc.dyn_model.network_form == KRON_REDUCED
        @test tc.dyn_model.mech_power_mode == USE_PG
        @test !tc.dyn_model.constrain_Δω_COI
        @test tc.gen_dynamic_filename == "gen_dynamic_data.csv"
        @test tc.gfm_dynamic_filename == "gfm_dynamic_data.csv"
        @test !tc.dyn_model.include_avr
        @test !tc.dyn_model.include_governor
        @test tc.simulation.δ_tol_deg == 90.0
        @test tc.simulation.δ_tol_deg_lower === nothing
        @test tc.simulation.δ_tol_deg_upper === nothing
        δ_tol, = TSCOPF.common_ts_parameters(tc.simulation)
        @test δ_tol[1] ≈ deg2rad(-90.0)
        @test δ_tol[2] ≈ deg2rad(90.0)
        @test tc.dyn_model.fault.contingency_id == 2
        @test tc.builder.limits.E_max_pu == 2.0
        @test tc.builder.limits.P_m_source == :dgen_pg_limits
    end

    @testset "asymmetric δ_tol vs COI" begin
        sim = TsSimulationConfig(δ_tol_deg = 90.0, δ_tol_deg_lower = 30.0, δ_tol_deg_upper = 120.0)
        δ_tol, = TSCOPF.common_ts_parameters(sim)
        @test δ_tol[1] ≈ deg2rad(-30.0)
        @test δ_tol[2] ≈ deg2rad(120.0)
        @test_throws ArgumentError TSCOPF.common_ts_parameters(
            TsSimulationConfig(δ_tol_deg = 90.0, δ_tol_deg_lower = 0.0))
    end

    @testset "validate_dgen_dyn_row_ids!" begin
        cfg = RunConfig(case = "9bus", trans_stab = true,
            transient = TransientConfig(dyn_model = DynModelConfig(network_form = FULL_BUS)))
        sys = load_fixture_system(cfg)
        @test TSCOPF.validate_dgen_dyn_row_ids!(sys.DGEN, sys.DGEN_DYN) === nothing

        bad_dyn = copy(sys.DGEN_DYN)
        bad_dyn.bus[1] = sys.DGEN.bus[2]
        bad_dyn.bus[2] = sys.DGEN.bus[1]
        @test_throws ArgumentError TSCOPF.validate_dgen_dyn_row_ids!(sys.DGEN, bad_dyn)

        bad_id = copy(sys.DGEN_DYN)
        bad_id.id = [2, 1, 3]
        @test_throws ArgumentError TSCOPF.validate_dgen_dyn_row_ids!(sys.DGEN, bad_id)
    end

    @testset "save_warmstart_dispatch validation" begin
        # Kron TSC-ACOPF solves once, jointly: there is no pre-solve to archive.
        cfg_kron = RunConfig(
            trans_stab = true,
            dispatch = DispatchConfig(type_model = "ACOPF"),
            transient = TSCOPF.default_transient_config(),
            save_warmstart_dispatch = true,
        )
        @test_throws ArgumentError validate_run_config!(cfg_kron)

        # Steady-state runs have no TS assembly, hence no warm start either.
        cfg_dispatch_only = RunConfig(
            dispatch = DispatchConfig(type_model = "ACOPF"),
            save_warmstart_dispatch = true,
        )
        @test_throws ArgumentError validate_run_config!(cfg_dispatch_only)

        # The two paths that do pre-solve: FULL_BUS TSC-ACOPF and TSC-DCOPF (δ_ref).
        cfg_fullbus = RunConfig(
            trans_stab = true,
            dispatch = DispatchConfig(type_model = "ACOPF"),
            transient = TransientConfig(dyn_model = DynModelConfig(
                network_form = FULL_BUS, mech_power_mode = USE_PM, bound_style = :coi_box)),
            save_warmstart_dispatch = true,
        )
        @test validate_run_config!(cfg_fullbus) === nothing

        cfg_dcopf = RunConfig(
            trans_stab = true,
            solver_name = "HiGHS",
            dispatch = DispatchConfig(type_model = "DCOPF"),
            transient = TSCOPF.default_transient_config(),
            save_warmstart_dispatch = true,
        )
        @test validate_run_config!(cfg_dcopf) === nothing
    end

    @testset "DispatchLimitsConfig" begin
        lims = DispatchLimitsConfig()
        @test lims.θ_min_rad ≈ -π
        @test lims.θ_max_rad ≈ π
        @test lims.ang_diff_clamp_ac_deg == 60.0
        @test lims.ang_diff_clamp_dc_deg == 30.0

        oip = build_opf_input_param(DispatchConfig(
            limits = DispatchLimitsConfig(θ_max_rad = π / 2)))
        @test oip[:limits].θ_max_rad ≈ π / 2

        @test_throws ArgumentError validate_dispatch_config!(DispatchConfig(
            limits = DispatchLimitsConfig(θ_min_rad = 1.0, θ_max_rad = -1.0)))
        @test_throws ArgumentError validate_dispatch_config!(DispatchConfig(
            limits = DispatchLimitsConfig(ang_diff_clamp_dc_deg = 0.0)))

        # Safety clamp tightens out-of-range case data, passes in-range through.
        @test TSCOPF.clamp_angle_diff_limits(-45.0, 45.0, 30.0, 1, 2) == (-30.0, 30.0)
        @test TSCOPF.clamp_angle_diff_limits(-20.0, 25.0, 30.0, 1, 2) == (-20.0, 25.0)

        # Unrated branches (l_cap_1 == 0) → ±Inf; rated → ±cap/base_MVA
        sys = load_fixture_system(RunConfig(case = "9bus"))
        lo, hi = TSCOPF.branch_flow_limit_vectors(sys.DCIR, 100.0)
        for i in sys.DCIR.id
            if sys.DCIR.l_cap_1[i] == 0.0
                @test lo[i] == -Inf && hi[i] == Inf
            else
                cap = sys.DCIR.l_cap_1[i] / 100.0
                @test lo[i] ≈ -cap
                @test hi[i] ≈ cap
            end
        end
    end

    @testset "TsBoundLimitsConfig resolve" begin
        sys = load_fixture_system(RunConfig(case = "9bus"))
        active_gen = findall(x -> x == 1, sys.DGEN.g_status)
        base_MVA = 100.0
        limits = TsBoundLimitsConfig(E_max_pu = 1.8)
        specs = TSCOPF.resolve_ts_bound_limits(limits, active_gen, sys.DGEN, base_MVA)
        @test specs[:E][2] ≈ fill(1.8, length(active_gen))
        δ_min, δ_max = specs[:δ]
        @test δ_min ≈ -π
        @test δ_max ≈ π
        @test specs[:P_m][1] ≈ [sys.DGEN.pg_min[g] / base_MVA for g in active_gen]
        @test specs[:P_m][2] ≈ [sys.DGEN.pg_max[g] / base_MVA for g in active_gen]
        @test specs[:δ_tf] == (-Inf, Inf)
        @test specs[:Qe_tf] == (-Inf, Inf)
        @test specs[:Ed_tf] == (-Inf, Inf)
        @test specs[:Pv_tf] == (-Inf, Inf)
        @test specs[:V_bus_min] == (0.0, nothing)
        @test specs[:gov_valve][1] ≈ fill(0.0, length(active_gen))
        @test specs[:gov_valve][2] ≈ [sys.DGEN.pg_max[g] / base_MVA for g in active_gen]

        @test_throws ArgumentError TSCOPF.validate_ts_bound_limits!(
            TsBoundLimitsConfig(gov_valve_max_source = :other))
        @test_throws ArgumentError TSCOPF.validate_ts_bound_limits!(
            TsBoundLimitsConfig(E_min_pu = 2.0, E_max_pu = 1.0))
        pair = TSCOPF.TsBoundLimitPair()
        @test pair.min == -Inf && pair.max == Inf
    end

    @testset "optional tf bound toggle with Inf limits is a no-op" begin
        model = Model()
        dyn_model_dict = OrderedDict{Symbol, Any}()
        dyn_model_dict[:vars] = OrderedDict{Symbol, Any}()
        dyn_model_dict[:ineq_const] = OrderedDict{Symbol, Any}()
        dyn_model_dict[:vars][:δ_tf] = OrderedDict(
            1 => OrderedDict(1 => @variable(model, base_name = "d1")),
            2 => OrderedDict(1 => @variable(model, base_name = "d2")),
        )
        dyn_model_dict[:meta] = OrderedDict(
            :var_bounds => OrderedDict(:δ_tf => true),
            :var_limit_specs => OrderedDict(:δ_tf => (-Inf, Inf)),
            :bound_encoding => TSCOPF.CONSTRAINT,
        )
        TSCOPF.attach_kron_gen_time_var_bounds!(
            model, dyn_model_dict, dyn_model_dict[:vars][:δ_tf],
            :δ_tf, :δ_tf, :ineq_const_δ_tf_lower, :ineq_const_δ_tf_upper)
        @test !haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_tf_lower)
        @test !haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_tf_upper)
    end

    @testset "CONSTRAINT vector Inf limits are skipped; finite vectors attach" begin
        model = Model()
        dyn_model_dict = OrderedDict{Symbol, Any}()
        dyn_model_dict[:vars] = OrderedDict{Symbol, Any}()
        dyn_model_dict[:ineq_const] = OrderedDict{Symbol, Any}()
        dyn_model_dict[:vars][:P_ref] = OrderedDict(
            1 => @variable(model, base_name = "Pref1"),
            2 => @variable(model, base_name = "Pref2"),
        )
        dyn_model_dict[:meta] = OrderedDict(
            :var_bounds => OrderedDict(:P_ref => true),
            :var_limit_specs => OrderedDict(:P_ref => (fill(-Inf, 2), fill(Inf, 2))),
            :bound_encoding => TSCOPF.CONSTRAINT,
        )
        TSCOPF.attach_kron_gen_scalar_var_bounds!(
            model, dyn_model_dict, dyn_model_dict[:vars][:P_ref],
            :P_ref, :P_ref, :ineq_const_P_ref_lower, :ineq_const_P_ref_upper)
        @test !haskey(dyn_model_dict[:ineq_const], :ineq_const_P_ref_lower)
        @test !haskey(dyn_model_dict[:ineq_const], :ineq_const_P_ref_upper)

        dyn_model_dict[:meta][:var_limit_specs][:P_ref] = ([0.0, 0.1], [1.0, 1.2])
        TSCOPF.attach_kron_gen_scalar_var_bounds!(
            model, dyn_model_dict, dyn_model_dict[:vars][:P_ref],
            :P_ref, :P_ref, :ineq_const_P_ref_lower, :ineq_const_P_ref_upper)
        @test haskey(dyn_model_dict[:ineq_const], :ineq_const_P_ref_lower)
        @test haskey(dyn_model_dict[:ineq_const], :ineq_const_P_ref_upper)
        @test length(dyn_model_dict[:ineq_const][:ineq_const_P_ref_lower]) == 2
    end

    @testset "COI inertia denominator (active generators)" begin
        DGEN_DYN = DataFrame(H = [4.0, 5.0, 6.0])
        active_gen = [1, 2]

        model = Model()
        tw = [1.0]
        δ = OrderedDict(
            g => OrderedDict(1 => @variable(model, base_name = "d$g")) for g in active_gen)
        δCOI = OrderedDict(1 => @variable(model))
        eq = TSCOPF.eq_const_kron_COI_generic!(model, δ, δCOI, active_gen, DGEN_DYN, tw)

        JuMP.fix(δ[1][1], 0.1; force = true)
        JuMP.fix(δ[2][1], 0.2; force = true)
        H_active = sum(DGEN_DYN.H[active_gen])
        coi_val = (DGEN_DYN.H[1] * 0.1 + DGEN_DYN.H[2] * 0.2) / H_active
        JuMP.fix(δCOI[1], coi_val; force = true)
        set_optimizer(model, Ipopt.Optimizer)
        set_silent(model)
        @objective(model, Min, 0.0)
        optimize!(model)

        residual = JuMP.value(δCOI[1]) -
            (DGEN_DYN.H[1] * JuMP.value(δ[1][1]) + DGEN_DYN.H[2] * JuMP.value(δ[2][1])) / H_active
        @test isapprox(residual, 0.0; atol = 1e-12)
        @test length(eq) == 1

        wrong_residual = JuMP.value(δCOI[1]) -
            (DGEN_DYN.H[1] * JuMP.value(δ[1][1]) + DGEN_DYN.H[2] * JuMP.value(δ[2][1])) / sum(DGEN_DYN.H)
        @test !isapprox(wrong_residual, 0.0; atol = 1e-12)
    end

    @testset "Pacc_COI uses generator-keyed inertia (C2)" begin
        DGEN_DYN = DataFrame(H = [3.0, 5.0, 7.0])
        base_MVA = 100.0

        Pacc = OrderedDict{Int, Vector{Float64}}(
            1 => [1.0, 2.0],
            3 => [4.0, 6.0],
        )
        PaccCOI_total = Pacc[1] .+ Pacc[3]
        H_total = DGEN_DYN.H[1] + DGEN_DYN.H[3]

        out = TSCOPF._pacc_relative_to_coi(Pacc, PaccCOI_total, DGEN_DYN, H_total, base_MVA)

        expected_1 = (Pacc[1] .- ((DGEN_DYN.H[1] .* PaccCOI_total) ./ H_total)) ./ base_MVA
        expected_3 = (Pacc[3] .- ((DGEN_DYN.H[3] .* PaccCOI_total) ./ H_total)) ./ base_MVA
        @test out[1] ≈ expected_1
        @test out[3] ≈ expected_3

        buggy_3 = (Pacc[3] .- ((DGEN_DYN.H[2] .* PaccCOI_total) ./ H_total)) ./ base_MVA
        @test !(out[3] ≈ buggy_3)
    end

    @testset "All-GFM fleet: zero inertia degrades instead of throwing (A6)" begin
        # A 100 % converter fleet has no SG inertia, so the H-weighted COI is undefined.
        # Post-processing must still complete: `_H_total` reports `nothing` and the
        # COI-relative accelerating power collapses to the absolute value in pu.
        DGEN_DYN = DataFrame(H = [0.0, 0.0])
        base_MVA = 100.0
        @test TSCOPF._H_total(DGEN_DYN, [1, 2]) === nothing
        @test TSCOPF._H_total(DataFrame(H = [0.0, 4.0]), [1, 2]) == 4.0

        Pacc = OrderedDict{Int, Vector{Float64}}(1 => [1.0, 2.0], 2 => [3.0, 4.0])
        out = TSCOPF._pacc_relative_to_coi(
            Pacc, Pacc[1] .+ Pacc[2], DGEN_DYN, nothing, base_MVA)
        @test out[1] ≈ Pacc[1] ./ base_MVA
        @test out[2] ≈ Pacc[2] ./ base_MVA
    end

    @testset "Pacc uses time-varying mechanical power when governor on" begin
        DGEN_DYN = DataFrame(H = [5.0, 5.0])
        base_MVA = 100.0
        active_gen = [1, 2]
        Pet = OrderedDict(
            1 => [80.0, 70.0, 75.0],
            2 => [60.0, 55.0, 58.0],
        )
        Pmt = OrderedDict(
            1 => [100.0, 90.0, 95.0],
            2 => [60.0, 60.0, 60.0],
        )
        δ_COIt = OrderedDict(
            1 => [0.0, 0.1, 0.2],
            2 => [0.0, 0.05, 0.1],
        )
        Pacc, _, _, Vpe = TSCOPF._compute_pacc_trajectories(
            Pet, Pmt, δ_COIt, DGEN_DYN, active_gen, base_MVA)
        @test Pacc[1] ≈ [20.0, 20.0, 20.0]
        @test Pacc[2] ≈ [0.0, 5.0, 2.0]
        @test Vpe[1][1] == 0.0
        @test Vpe[1][end] != 0.0
    end

    @testset "HiGHSSolverConfig defaults and raw_options" begin
        cfg = HiGHSSolverConfig()
        validate_highs_solver_config!(cfg)
        @test cfg.output_flag == true
        @test cfg.ipm_optimality_tolerance == 1e-8
        @test cfg.simplex_iteration_limit == 5_000
        @test cfg.solver == "choose"
        model = Setup_Optim_Model("HiGHS"; highs=cfg, silent=true)
        @test JuMP.get_optimizer_attribute(model, "primal_feasibility_tolerance") == 1e-8
        @test JuMP.get_optimizer_attribute(model, "dual_feasibility_tolerance") == 1e-8
        @test JuMP.get_optimizer_attribute(model, "ipm_optimality_tolerance") == 1e-8
        @test JuMP.get_optimizer_attribute(model, "simplex_iteration_limit") == 5_000
        @test JuMP.get_optimizer_attribute(model, "ipm_iteration_limit") == 5_000
        @test JuMP.get_optimizer_attribute(model, "solver") == "choose"

        raw_cfg = HiGHSSolverConfig(raw_options = Dict("time_limit" => 99.0))
        model2 = Model(HiGHS.Optimizer)
        apply_highs_options!(model2, raw_cfg; silent=true)
        @test JuMP.get_optimizer_attribute(model2, "time_limit") == 99.0

        @test_throws ArgumentError validate_highs_solver_config!(
            HiGHSSolverConfig(ipm_iteration_limit = 0))
        @test_throws ArgumentError validate_highs_solver_config!(
            HiGHSSolverConfig(solver = "bad"))
    end

    @testset "GurobiSolverConfig" begin
        defaults = GurobiSolverConfig()
        validate_gurobi_solver_config!(defaults)
        @test defaults.output_flag == 1
        @test defaults.feasibility_tol == 1e-8
        @test defaults.optimality_tol == 1e-8
        @test defaults.bar_iter_limit == 5_000
        @test defaults.mip_gap == 1e-8
        @test defaults.nl_bar_iter_limit == 5_000
        @test defaults.nl_bar_p_feas_tol == 1e-8
        @test defaults.nl_bar_d_feas_tol == 1e-8
        @test defaults.nl_bar_c_feas_tol == 1e-8
        # Must stay -1 ("leave it to Gurobi"). Pinning a number here is what broke
        # UC: Gurobi 13 renumbered OptimalityTarget so that 1 means LOCAL, and
        # local optimization rejects every discrete model with Error 10016.
        @test defaults.optimality_target == -1
        @test_throws ArgumentError validate_gurobi_solver_config!(
            GurobiSolverConfig(nl_bar_iter_limit = 0))
        @test_throws ArgumentError validate_gurobi_solver_config!(
            GurobiSolverConfig(optimality_target = 4))
        if gurobi_available()
            cfg = GurobiSolverConfig(mip_gap = 0.02, nl_bar_iter_limit = 100)
            validate_gurobi_solver_config!(cfg)
            model = Setup_Optim_Model("Gurobi"; gurobi=cfg, silent=true)
            @test JuMP.get_optimizer_attribute(model, "MIPGap") == 0.02
            try
                @test JuMP.get_optimizer_attribute(model, "NLBarIterLimit") == 100
                @test JuMP.get_optimizer_attribute(model, "OptimalityTarget") == 1
                @test JuMP.get_optimizer_attribute(model, "NLBarPFeasTol") == 1e-8
            catch err
                @test err isa MOI.UnsupportedAttribute
            end
        end
    end

    @testset "MadNLPSolverConfig validation" begin
        cfg = MadNLPSolverConfig()
        validate_madnlp_solver_config!(cfg)
        @test cfg.ma97_num_threads === nothing
        @test madnlp_hessian_type_name("bfgs") == "BFGS"
        @test madnlp_hessian_type_name("compact-lbfgs") == "CompactLBFGS"
        @test_throws ArgumentError validate_madnlp_solver_config!(
            MadNLPSolverConfig(hessian_approximation = "bad"))
        @test_throws ArgumentError validate_madnlp_solver_config!(
            MadNLPSolverConfig(ma97_num_threads = 0))
        validate_madnlp_solver_config!(MadNLPSolverConfig(ma97_num_threads = 4))
        @test is_madnlp_backend_solver("MadNLP")
        @test is_madnlp_backend_solver("MadNLP-ma57")
        @test is_madnlp_backend_solver("MadNLP-ma97")
        @test !is_madnlp_backend_solver("MadNLP-pardiso")
        @test !is_madnlp_backend_solver("Ipopt-ma57")
        @test !is_madnlp_backend_solver("Ipopt-pardiso")
        @test is_ipopt_backend_solver("Ipopt-ma57")
        @test is_ipopt_backend_solver("Ipopt-pardiso")
        @test !is_ipopt_backend_solver("ma57")
        @test !is_ipopt_backend_solver("pardiso")
    end

    @testset "MadNLP HSL coherence (optional packages)" begin
        if madnlp_hsl_ext_available()
            @test Check_Coherence_Input_Data(false, "ACOPF", "MadNLP-ma57") ==
                (false, "MadNLP-ma57")
            @test Check_Coherence_Input_Data(false, "ACOPF", "MadNLP-ma97") ==
                (false, "MadNLP-ma97")
        else
            @test_throws ArgumentError Check_Coherence_Input_Data(false, "ACOPF", "MadNLP-ma57")
            @test_throws ArgumentError Check_Coherence_Input_Data(false, "ACOPF", "MadNLP-ma97")
        end
        @test_throws ArgumentError Check_Coherence_Input_Data(false, "ACOPF", "MadNLP-pardiso")
    end

    @testset "IpoptSolverConfig defaults and L-BFGS options" begin
        cfg = IpoptSolverConfig()
        @test cfg.tol == 1e-8
        @test cfg.print_level == 5
        @test cfg.max_iter == 5_000
        @test cfg.constr_viol_tol == 1e-8
        @test cfg.dual_inf_tol == 1e-8
        @test cfg.compl_inf_tol == 1e-8
        @test cfg.hessian_approximation == "exact"
        validate_ipopt_solver_config!(cfg)

        lbfgs = IpoptSolverConfig(hessian_approximation = "limited-memory", limited_memory_max_history = 50)
        validate_ipopt_solver_config!(lbfgs)

        model = Model(Ipopt.Optimizer)
        apply_ipopt_options!(model, lbfgs)
        @test JuMP.get_optimizer_attribute(model, "hessian_approximation") == "limited-memory"
        @test JuMP.get_optimizer_attribute(model, "limited_memory_max_history") == 50

        @test_throws ArgumentError validate_ipopt_solver_config!(
            IpoptSolverConfig(hessian_approximation = "bfgs"))
    end

    # Absorbed from the retired runtests_acopf.jl: the HSL-backed Ipopt linear
    # solvers must either build a real model (HSL_jll present) or fail coherence
    # fast (HSL_jll absent). No NLP solve involved, so it belongs here.
    @testset "Ipopt + HSL linear solvers (optional)" begin
        if hsl_jll_available()
            model = Setup_Optim_Model("Ipopt-ma57"; silent = true)
            @test model isa JuMP.Model
            @test JuMP.unsafe_backend(model) isa Ipopt.Optimizer
        else
            @test_throws ArgumentError Check_Coherence_Input_Data(false, "ACOPF", "Ipopt-ma57")
            @test_throws ArgumentError Check_Coherence_Input_Data(false, "ACOPF", "Ipopt-ma97")
        end
    end

    @testset "PARDISO Ipopt linear solver config" begin
        old_pardiso = get(ENV, "JULIA_PARDISO_LIB", nothing)
        if old_pardiso === nothing
            delete!(ENV, "JULIA_PARDISO_LIB")
        else
            ENV["JULIA_PARDISO_LIB"] = ""
        end
        try
            @test_throws ArgumentError validate_ipopt_solver_config!(
                IpoptSolverConfig(), "Ipopt-pardiso")
            @test_throws ArgumentError Check_Coherence_Input_Data(false, "ACOPF", "Ipopt-pardiso")
        finally
            if old_pardiso === nothing
                delete!(ENV, "JULIA_PARDISO_LIB")
            else
                ENV["JULIA_PARDISO_LIB"] = old_pardiso
            end
        end

        fake_lib = joinpath(mktempdir(), "libpardiso.dll")
        open(fake_lib, "w") do io
            write(io, "stub")
        end
        cfg = IpoptSolverConfig(pardiso_lib_path = fake_lib)
        @test pardiso_available(cfg)
        validate_ipopt_solver_config!(cfg, "Ipopt-pardiso")
        model = Setup_Optim_Model("Ipopt-pardiso"; ipopt=cfg, silent=true)
        @test JuMP.get_optimizer_attribute(model, "linear_solver") == "pardiso"
        @test JuMP.get_optimizer_attribute(model, "pardisolib") == fake_lib
        @test ENV["JULIA_PARDISO_LIB"] == fake_lib
        @test ENV["PARDISOLICMESSAGE"] == "1"
    end

    @testset "release_solver_backend! unlocks Ipopt output_file" begin
        log = joinpath(mktempdir(), "ipopt_release_test.txt")
        model = Model(Ipopt.Optimizer)
        set_optimizer_attribute(model, "output_file", log)
        set_silent(model)
        @variable(model, x)
        @constraint(model, x >= 1)
        @objective(model, Min, x)
        optimize!(model)
        @test termination_status(model) == LOCALLY_SOLVED
        TSCOPF.release_solver_backend!(model)
        open(log, "a") do io
            println(io, "# released")
        end
        @test occursin("# released", read(log, String))
    end
end
