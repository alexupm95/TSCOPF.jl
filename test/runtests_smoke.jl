#=
 9-bus end-to-end smoke tests (included by `Pkg.test()` and `runtests_all.jl`).

 Runs ED, ACOPF, DCOPF, TSC-ACOPF, and TSC-DCOPF on the 9-bus case. TSC-ACOPF
 additionally checks Kron-reduced reactive-power expressions (`Qe_tf`).
=#

const SOLVED_STATUSES = (MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ITERATION_LIMIT)

function run_smoke(cfg::RunConfig)
    cfg_ts = reconfigure(cfg; overwrite_results = TEST_OVERWRITE_RESULTS)
    sys = load_fixture_system(cfg_ts)
    return run_fixture_case!(cfg_ts, sys)
end

smoke_cases = [
    "ED" => dispatch_run_config(type_model = "ED", solver_name = "Ipopt",
        silent_solver = true, save_optim_matrices = false),

    "ACOPF" => dispatch_run_config(type_model = "ACOPF", solver_name = "Ipopt",
        silent_solver = true, save_optim_matrices = false),

    "DCOPF" => dispatch_run_config(type_model = "DCOPF", solver_name = "HiGHS",
        silent_solver = true, save_optim_matrices = false),

    "TSC-ACOPF" => reconfigure_dyn(tsc_run_config(type_model = "ACOPF", solver_name = "Ipopt",
        silent_solver = true, save_optim_matrices = false);
        dyn_model = DynModelConfig(constrain_Δω_COI = false)),

    "TSC-DCOPF" => reconfigure_dyn(tsc_run_config(type_model = "DCOPF", solver_name = "HiGHS",
        silent_solver = true, save_optim_matrices = false);
        dyn_model = DynModelConfig(constrain_Δω_COI = false)),
]

@testset "TSC-OPF smoke tests (9-bus)" begin
    assert_quadratic_opf_objective!()
    for (name, cfg) in smoke_cases
        @testset "$name" begin
            result = run_smoke(cfg)
            @test result.status in SOLVED_STATUSES
            assert_timestamped_results!(result.path_names)

            if name == "TSC-ACOPF"
                dmd = result.dyn_model_dict
                @test dmd[:meta][:network_form] == "KRON_REDUCED"
                @test haskey(dmd[:eq_const], :eq_const_P_init)
                @test haskey(dmd[:eq_const], :eq_const_Q_init)
                @test haskey(dmd[:expressions], :Qe_tf)
                @test !haskey(dmd[:vars], :Qe_tf)
                @test !haskey(dmd[:eq_const], :eq_const_Qe_tf)
                @test isfile(joinpath(result.path_names[:pf_TS_CSV], "electrical_reactive_power.csv"))
            elseif name == "TSC-DCOPF"
                dmd = result.dyn_model_dict
                @test haskey(dmd[:eq_const], :eq_const_P_init)
                @test !haskey(dmd[:eq_const], :eq_const_Q_init)
            end
        end
    end

    # Absorbed from the retired runtests_acopf.jl. The frozen
    # `9bus_reference_cost` inputs (flat linear cost, lossless and uncongested)
    # have an analytically known optimum, so this pins the AC-OPF objective
    # independently of any edit to the working `9bus/` case.
    @testset "AC-OPF reference parity (9bus_reference_cost)" begin
        @test Check_Coherence_Input_Data(false, "ACOPF", "Ipopt") == (false, "Ipopt")

        result = run_smoke(dispatch_run_config(
            case                = REFERENCE_LINEAR_ACOPF_CASE,
            type_model          = "ACOPF",
            use_matrix          = false,
            solver_name         = "Ipopt",
            load_factor         = 1.5,
            silent_solver       = true,
            save_optim_matrices = false,
        ))

        @test result.status in SOLVED_STATUSES
        @test result.dyn_model_dict === nothing
        @test result.dyn_parameters_dict === nothing
        @test isfinite(result.obj_MVA)
        @test isapprox(result.obj_MVA, REFERENCE_LINEAR_ACOPF_OBJ_EUR;
                       rtol = REFERENCE_LINEAR_ACOPF_OBJ_RTOL)
        assert_timestamped_results!(result.path_names)
        @test result.RGEN !== nothing
        @test result.RBUS !== nothing
        @test result.RCIR !== nothing
        # `>= 0`, not `> 0`: on a warm process this ACOPF builds and solves inside
        # one `time()` tick, so a strict test measures the clock, not the code.
        @test isfinite(result.t_build) && result.t_build >= 0.0
        @test isfinite(result.t_solve) && result.t_solve >= 0.0
        @test !isempty(readdir(result.path_names[:pf_dispatch_CSV]))
    end
end
