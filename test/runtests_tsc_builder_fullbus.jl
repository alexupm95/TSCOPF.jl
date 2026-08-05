#=
 FULL_BUS TSC-ACOPF builder regression (main.jl-style settings).

 Merges the former runtests_fullbus_acopf.jl into this file: both re-solved the
 same 9-bus FULL_BUS SC case, so the baseline testset below now carries the union
 of their assertions (structure + CSV/dual exports) for the price of one solve.

 Tiering: the baseline solve and the no-solve ZIP factory checks run in the fast
 gate; the toggle/variant testsets need one extra FULL_BUS solve each and run
 only in the heavy tier (`RUN_HEAVY`, set from test/runtests.jl).
=#

const FULLBUS_TSC_SOLVED_STATUSES = (
    MOI.OPTIMAL,
    MOI.LOCALLY_SOLVED,
    MOI.ITERATION_LIMIT,
    MOI.ALMOST_LOCALLY_SOLVED,
)

const ZIP_IMPEDANCE = (1.0, 0.0, 0.0)   # (Z, I, P) — pure constant impedance
const ZIP_CURRENT   = (0.0, 1.0, 0.0)   # (Z, I, P) — pure constant current

function run_tsc_builder_fullbus_case(cfg::RunConfig)
    sys = load_fixture_system(cfg)
    return run_fixture_case!(cfg, sys)
end

"""Classical FULL_BUS TSC-ACOPF with an explicit ZIP split (from runtests_fullbus_acopf.jl)."""
function run_fullbus_acopf(; zip_load_p = ZIP_IMPEDANCE, zip_load_q = ZIP_IMPEDANCE,
                             contingency_id::Int = 2, δ_tol_deg::Float64 = 100.0)
    cfg = reconfigure_dyn(tsc_run_config(
        type_model = "ACOPF",
        solver_name = "Ipopt",
        δ_tol_deg = δ_tol_deg,
        contingency_id = contingency_id,
        silent_solver = true,
        save_optim_matrices = false,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        load_factor = 1.5,
    ); dyn_model = DynModelConfig(
        network_form = FULL_BUS,
        mech_power_mode = USE_PM,
        bound_style = :coi_box,
        zip_load_p = zip_load_p,
        zip_load_q = zip_load_q,
        fault = FaultConfig(contingency_id = contingency_id),
    ))
    validate_dyn_config!(cfg)
    sys = load_fixture_system(cfg)
    return run_fixture_case!(cfg, sys)
end

@testset "TSC builder — FULL_BUS (main.jl style)" begin

    # --- no-solve config/factory checks (fast) -------------------------------
    @testset "DynModelConfig + quadratic OPF objective" begin
        assert_quadratic_opf_objective!()
        dyn = DynModelConfig(
            network_form = FULL_BUS,
            mech_power_mode = USE_PM,
            bound_style = :coi_box,
            zip_load_p = ZIP_IMPEDANCE,
            zip_load_q = ZIP_IMPEDANCE,
        )
        @test dyn.zip_load_p == ZIP_IMPEDANCE
        @test dyn.zip_load_q == ZIP_IMPEDANCE
        @test validate_dyn_config!(reconfigure_dyn(tsc_run_config(type_model = "ACOPF");
            dyn_model = dyn)) === nothing
        m = TSCOPF.dynamic_gen_model(dyn; linearize = false)
        @test m isa TSCOPF.ClassicalFullBusModel
        @test m.zip_load_p == ZIP_IMPEDANCE
        @test m.zip_load_q == ZIP_IMPEDANCE
    end

    # Asymmetric split (REE style: constant-current P, constant-admittance Q) must survive
    # the factory as two distinct vectors — a shared-vector regression would collapse them.
    @testset "independent P/Q ZIP splits reach the model struct" begin
        dyn = DynModelConfig(
            network_form = FULL_BUS,
            mech_power_mode = USE_PM,
            bound_style = :coi_box,
            zip_load_p = ZIP_CURRENT,
            zip_load_q = ZIP_IMPEDANCE,
        )
        m = TSCOPF.dynamic_gen_model(dyn; linearize = false)
        @test m.zip_load_p == ZIP_CURRENT
        @test m.zip_load_q == ZIP_IMPEDANCE
        @test m.zip_load_p != m.zip_load_q
    end

    # --- the one FULL_BUS solve that runs on every push ----------------------
    @testset "baseline solve — structure, exports, dual registry" begin
        # `main_style_tsc_base` sets save_duals = false for speed; this testset
        # absorbed the dual-export assertions from runtests_fullbus_acopf.jl, so it
        # needs them written. Still one solve.
        result = run_tsc_builder_fullbus_case(
            reconfigure(main_style_tsc_fullbus_config(); save_duals = true))
        @test result.status in FULLBUS_TSC_SOLVED_STATUSES
        assert_timestamped_results!(result.path_names)
        @test isfinite(result.obj_MVA)

        dmd = result.dyn_model_dict
        @test dmd[:meta][:network_form] == "FULL_BUS"
        @test dmd[:meta][:gen_model_type] == "ClassicalFullBusModel"
        @test dmd[:meta][:coupling_init_source] == "acopf_warmstart"
        @test dmd[:meta][:zip_load_p] ≈ collect(ZIP_IMPEDANCE)
        @test dmd[:meta][:zip_load_q] ≈ collect(ZIP_IMPEDANCE)
        @test dmd[:mech_power_mode] == USE_PM
        @test dmd[:meta][:constrain_Δω_COI] == false

        @test haskey(dmd[:vars], :E)
        @test haskey(dmd[:vars], :P_m)
        @test haskey(dmd[:vars], :V_tf)
        @test haskey(dmd[:vars], :θ_tf)
        @test haskey(dmd[:vars], :Pe_tf)
        @test haskey(dmd[:vars], :Qe_tf)

        @test haskey(dmd[:eq_const], :eq_const_P_init)
        @test haskey(dmd[:eq_const], :eq_const_Q_init)
        @test haskey(dmd[:eq_const], :eq_const_Pm_init)
        @test haskey(dmd[:eq_const], :eq_const_δCOI_tf)
        @test haskey(dmd[:eq_const], :eq_const_Pe_tf)
        @test haskey(dmd[:eq_const], :eq_const_Pbalance_tf)
        @test haskey(dmd[:eq_const], :eq_const_Qbalance_tf)
        @test haskey(dmd[:eq_const], :eq_const_Pbalance_tpf)
        @test haskey(dmd[:eq_const], :eq_const_Qbalance_tpf)

        @test haskey(dmd[:ineq_const], :ineq_const_δ_COI_tf_lower)
        @test haskey(dmd[:ineq_const], :ineq_const_δ_COI_tf_upper)
        @test haskey(dmd[:ineq_const], :ineq_const_V_tf_lower)
        @test !haskey(dmd[:ineq_const], :ineq_const_Δω_COI_tf_lower)

        @test isfile(joinpath(result.path_names[:pf_TS], "dynamic_model_details.txt"))
        @test isfile(joinpath(result.path_names[:pf_TS], "model_summary.txt"))

        pf_csv = result.path_names[:pf_TS_CSV]
        for f in ("electrical_reactive_power.csv", "generator_reactive_power.csv",
                  "generator_active_power.csv", "generator_internal_voltage.csv",
                  "bus_reactive_power.csv")
            @test isfile(joinpath(pf_csv, f))
        end

        export_names = [e.export_name for e in dmd[:dual_registry]]
        @test :dual_Pbalance in export_names
        @test :dual_Qbalance in export_names
        @test :dual_Qe in export_names
        @test :dual_Qe_init in export_names

        pf_duals = result.path_names[:pf_TS_CSV_duals]
        for f in ("dual_Qe.csv", "dual_Pbalance.csv", "dual_Qbalance.csv",
                  "dual_delta_COI.csv")
            @test isfile(joinpath(pf_duals, f))
        end
    end

    # --- one extra FULL_BUS solve each: heavy tier only ----------------------
    if RUN_HEAVY

        @testset "warm-start dispatch save" begin
            cfg = reconfigure(main_style_tsc_fullbus_config();
                save_warmstart_dispatch = true, save_duals = false)
            result = run_tsc_builder_fullbus_case(cfg)
            @test result.status in FULLBUS_TSC_SOLVED_STATUSES

            ws_dir = joinpath(result.path_names[:pf_dispatch_warmstart])
            @test isfile(joinpath(ws_dir, "generators_report.txt"))
            @test isfile(joinpath(ws_dir, "CSV", "generators_report.csv"))
            @test isfile(joinpath(ws_dir, "OPF_Dispatch_Results.xlsx"))
            @test isfile(joinpath(ws_dir, "model_summary.txt"))

            @test isfile(joinpath(result.path_names[:pf_dispatch], "generators_report.txt"))

            gen_df = CSV.read(joinpath(ws_dir, "CSV", "generators_report.csv"), DataFrame; delim = ';')
            @test nrow(gen_df) == length(result.dyn_model_dict[:active_gen])
            @test all(isfinite, gen_df.P_MW)
        end

        @testset "δ-COI ineq toggles (post-fault upper off)" begin
            cfg = main_style_tsc_fullbus_config(;
                builder = main_style_ts_builder(ineq_δ_COI_tpf_upper = false))
            result = run_tsc_builder_fullbus_case(cfg)
            @test result.status in FULLBUS_TSC_SOLVED_STATUSES
            dmd = result.dyn_model_dict
            @test haskey(dmd[:ineq_const], :ineq_const_δ_COI_tpf_lower)
            @test !haskey(dmd[:ineq_const], :ineq_const_δ_COI_tpf_upper)
        end

        @testset "fault δ explicit bounds toggle" begin
            cfg = main_style_tsc_fullbus_config(;
                builder = main_style_ts_builder(
                    bound_δ_tf = true,
                    limits = TsBoundLimitsConfig(
                        δ_tf = TsBoundLimitPair(min = -9999.0, max = 9999.0))))
            result = run_tsc_builder_fullbus_case(cfg)
            @test result.status in FULLBUS_TSC_SOLVED_STATUSES
            dmd = result.dyn_model_dict
            @test haskey(dmd[:ineq_const], :ineq_const_δ_tf_lower)
            @test haskey(dmd[:ineq_const], :ineq_const_δ_tf_upper)
            @test !haskey(dmd[:ineq_const], :ineq_const_Pe_tf_lower)
            @test !haskey(dmd[:ineq_const], :ineq_const_Qe_tf_lower)
        end

        @testset "fault Qe explicit bounds toggle" begin
            cfg = main_style_tsc_fullbus_config(;
                builder = main_style_ts_builder(
                    bound_Qe_tf = true,
                    limits = TsBoundLimitsConfig(
                        Qe_tf = TsBoundLimitPair(min = -9999.0, max = 9999.0))))
            result = run_tsc_builder_fullbus_case(cfg)
            @test result.status in FULLBUS_TSC_SOLVED_STATUSES
            dmd = result.dyn_model_dict
            @test haskey(dmd[:ineq_const], :ineq_const_Qe_tf_lower)
            @test haskey(dmd[:ineq_const], :ineq_const_Qe_tf_upper)
            @test !haskey(dmd[:ineq_const], :ineq_const_Pe_tf_lower)
        end

        # End-to-end with the two ZIP channels genuinely different: active demand
        # constant-current, reactive demand constant-admittance (Spanish TSO
        # convention). Both splits still sum to 1, so the model stays consistent
        # with the constant-power ACOPF point at t = 0.
        @testset "build + solve with asymmetric P/Q ZIP split" begin
            result = run_fullbus_acopf(zip_load_p = ZIP_CURRENT, zip_load_q = ZIP_IMPEDANCE)
            @test result.status in FULLBUS_TSC_SOLVED_STATUSES

            dmd = result.dyn_model_dict
            @test dmd[:meta][:zip_load_p] ≈ collect(ZIP_CURRENT)
            @test dmd[:meta][:zip_load_q] ≈ collect(ZIP_IMPEDANCE)

            pf_duals = result.path_names[:pf_TS_CSV_duals]
            @test isfile(joinpath(pf_duals, "dual_Pbalance.csv"))
            @test isfile(joinpath(pf_duals, "dual_Qbalance.csv"))

            println("[asymmetric ZIP] status=$(result.status)  " *
                    "zip_p=$(dmd[:meta][:zip_load_p])  zip_q=$(dmd[:meta][:zip_load_q])")
        end

    end   # RUN_HEAVY
end
