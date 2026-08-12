#=
================================================================================
 test/runtests_dq_fullbus.jl — DQ_4TH FULL_BUS feasibility smoke
================================================================================
 One end-to-end SC solve with gen_dynamic_data_full.csv. Not a reference parity gate.
=#

using Test
using JuMP
using DataStructures: OrderedDict
using DataFrames

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))

const DQ_SOLVED = (OPTIMAL, LOCALLY_SOLVED, ITERATION_LIMIT)
const DQ_ZIP = (1.0, 0.0, 0.0)
const DQ_SIM = TsSimulationConfig(δ_tol_deg = 100.0, t_end_sim = 0.6,
    t_step = 0.02, clearing_time = 0.2)

function dq_run(; speed_in_algebra::Bool = true, fault::FaultConfig = FaultConfig(fault_type = SC, contingency_id = 2))
    cfg = RunConfig(;
        trans_stab = true,
        case = "9bus",
        solver_name = "Ipopt",
        silent_solver = true,
        save_duals = true,
        save_ts_plots = false,
        save_optim_matrices = false,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        load_factor = 1.5,
        dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),
        transient = TransientConfig(
            simulation = DQ_SIM,
            builder = TsBuilderConfig(),
            gen_dynamic_filename = "gen_dynamic_data_full.csv",
            dyn_model = DynModelConfig(
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                zip_load_p = DQ_ZIP,
                zip_load_q = DQ_ZIP,
                dq_speed_dev_in_algebra = speed_in_algebra,
                fault = fault,
            ),
        ),
    )
    validate_dyn_config!(cfg)
    sys = load_fixture_system(cfg)
    result = run_fixture_case!(cfg, sys)
    return result
end

@testset "TSC-ACOPF DQ_4TH FULL_BUS" begin

    @testset "config validation" begin
        bad_avr = reconfigure_dyn(tsc_run_config();
            dyn_model = DynModelConfig(
                gen_order = CLASSICAL_2ND, include_avr = true))
        @test_throws ArgumentError validate_dyn_config!(bad_avr)
    end

    @testset "factory" begin
        m = TSCOPF.dynamic_gen_model(
            DynModelConfig(
                gen_order = DQ_4TH, network_form = FULL_BUS,
                mech_power_mode = USE_PM, bound_style_δ = :coi_box);
            linearize = false)
        @test m isa TSCOPF.DqFullBusModel
        @test m.dq_speed_dev_in_algebra
        @test !m.include_avr
        @test !m.include_governor
    end

    @testset "SC bus fault — speed in algebra (default)" begin
        result = dq_run(speed_in_algebra = true)
        @test result.status in DQ_SOLVED
        dmd = result.dyn_model_dict
        for key in (:E_fd, :Ed_tf, :Eq_tf, :Id_tf, :Iq_tf, :Te_tf)
            @test haskey(dmd[:vars], key)
        end
        @test get(dmd[:meta], :dq_speed_dev_in_algebra, false)
        @test get(dmd[:meta], :gen_order, "") == "DQ_4TH"
        if result.status in DQ_SOLVED
            pf_csv = result.path_names[:pf_TS_CSV]
            pf_duals = result.path_names[:pf_TS_CSV_duals]
            for f in ("dq_Ed_pu.csv", "dq_Eq_pu.csv", "dq_Id_pu.csv", "dq_Iq_pu.csv", "dq_Te_MW.csv", "dq_E_fd.csv")
                @test isfile(joinpath(pf_csv, f))
            end
            for f in ("dual_Te.csv", "dual_Ed.csv", "dual_Eq.csv", "dual_Vd.csv", "dual_Vq.csv",
                      "dual_Ed_init.csv")
                @test isfile(joinpath(pf_duals, f))
            end
        end
    end

    @testset "GL gen trip — COI inertia over surviving set" begin
        cfg = RunConfig(;
            trans_stab = true,
            case = "9bus",
            solver_name = "Ipopt",
            silent_solver = true,
            save_duals = false,
            save_ts_plots = false,
            save_optim_matrices = false,
            overwrite_results = TEST_OVERWRITE_RESULTS,
            load_factor = 1.5,
            dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),
            transient = TransientConfig(
                simulation = DQ_SIM,
                builder = TsBuilderConfig(),
                gen_dynamic_filename = "gen_dynamic_data_full.csv",
                dyn_model = DynModelConfig(
                    gen_order = DQ_4TH,
                    network_form = FULL_BUS,
                    mech_power_mode = USE_PM,
                    bound_style_δ = :coi_box,
                    zip_load_p = DQ_ZIP,
                    zip_load_q = DQ_ZIP,
                    fault = FaultConfig(fault_type = GL, gl_gen_ids = [3]),
                ),
            ),
        )
        validate_dyn_config!(cfg)
        sys = load_fixture_system(cfg)
        result = run_fixture_case!(cfg, sys)
        @test result.status in DQ_SOLVED
        dmd = result.dyn_model_dict
        @test 3 ∉ dmd[:active_gen]
    end

    @testset "OB open branch — circuit 6" begin
        result = dq_run(
            speed_in_algebra = true,
            fault = FaultConfig(fault_type = OB, ob_branch_ids = [6]),
        )
        @test result.status in DQ_SOLVED
        dmd = result.dyn_model_dict
        @test dmd[:meta][:fault_type] == "OB"
        @test dmd[:meta][:ob_branch_ids] == [6]

        ip_txt = read(joinpath(result.path_names[:pf_results_date], "input_parameters.txt"), String)
        @test occursin("Fault Category:                OB", ip_txt)
        @test occursin("Open Branch (no short-circuit): [6]", ip_txt)
    end

    @testset "optional DQ pre-fault Ed bounds (toggle on)" begin
        cfg = RunConfig(;
            trans_stab = true,
            case = "9bus",
            solver_name = "Ipopt",
            silent_solver = true,
            save_duals = false,
            save_ts_plots = false,
            save_optim_matrices = false,
            overwrite_results = TEST_OVERWRITE_RESULTS,
            load_factor = 1.5,
            dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),
            transient = TransientConfig(
                simulation = DQ_SIM,
                builder = TsBuilderConfig(bound_Ed = true),
                gen_dynamic_filename = "gen_dynamic_data_full.csv",
                dyn_model = DynModelConfig(
                    gen_order = DQ_4TH,
                    network_form = FULL_BUS,
                    mech_power_mode = USE_PM,
                    bound_style_δ = :coi_box,
                    zip_load_p = DQ_ZIP,
                    zip_load_q = DQ_ZIP,
                    fault = FaultConfig(fault_type = SC, contingency_id = 2),
                ),
            ),
        )
        result = run_fixture_case!(cfg, load_fixture_system(cfg))
        @test result.status in DQ_SOLVED
        dmd = result.dyn_model_dict
        @test haskey(dmd[:ineq_const], :ineq_const_Ed_lower)
        @test haskey(dmd[:ineq_const], :ineq_const_Ed_upper)
        @test !haskey(dmd[:ineq_const], :ineq_const_Ed_tf_lower)
    end
end
