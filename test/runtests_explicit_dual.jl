#=
 Explicit ED and DC-OPF dual LPs: strong duality vs linear-cost primals (9-bus).

 Uses HiGHS when Gurobi is unavailable. Run:
   julia --project=. test/runtests_explicit_dual.jl
=#

using Test

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))

const _EXPLICIT_DUAL_SOLVER = gurobi_available() ? "Gurobi" : "HiGHS"
const _EXPLICIT_DUAL_SOLVED = (MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ITERATION_LIMIT)

function _duality_gap_rel(path_names)
    gap_line = readlines(joinpath(path_names[:pf_dispatch_dual], "duality_verification.txt"))[4]
    return parse(Float64, split(gap_line, "=")[2])
end

function _run_explicit_dual(cfg::RunConfig)
    cfg_ts = reconfigure(cfg; overwrite_results = TEST_OVERWRITE_RESULTS)
    sys = load_fixture_system(cfg_ts)
    return run_fixture_case!(cfg_ts, sys)
end

@testset "Explicit dual LPs (9-bus)" begin

    @testset "ED explicit dual" begin
        cfg = dispatch_run_config(
            type_model = "ED",
            solver_name = _EXPLICIT_DUAL_SOLVER,
            silent_solver = true,
            save_optim_matrices = false,
            dispatch = DispatchConfig(
                type_model = "ED",
                cost_type = "linear",
                solve_explicit_dual = true,
            ),
        )
        result = _run_explicit_dual(cfg)
        @test result.status in _EXPLICIT_DUAL_SOLVED
        pn = result.path_names
        @test isdir(pn[:pf_dispatch_dual])
        @test isfile(joinpath(pn[:pf_dispatch_dual], "model_summary.txt"))
        @test isfile(joinpath(pn[:pf_dispatch_dual], "duality_verification.txt"))
        @test isfile(joinpath(pn[:pf_dispatch_dual_CSV_duals], "dual_ed_SMP.csv"))
        @test isfile(joinpath(pn[:pf_dispatch_dual_CSV_duals], "dual_ed_alpha.csv"))
        @test _duality_gap_rel(pn) < 1e-5
    end

    @testset "DC-OPF explicit dual" begin
        cfg = dispatch_run_config(
            type_model = "DCOPF",
            use_matrix = true,
            solver_name = _EXPLICIT_DUAL_SOLVER,
            silent_solver = true,
            save_optim_matrices = false,
            dispatch = DispatchConfig(
                type_model = "DCOPF",
                use_matrix = true,
                cost_type = "linear",
                solve_explicit_dual = true,
            ),
        )
        result = _run_explicit_dual(cfg)
        @test result.status in _EXPLICIT_DUAL_SOLVED
        pn = result.path_names
        @test isdir(pn[:pf_dispatch_dual])
        @test isfile(joinpath(pn[:pf_dispatch_dual], "model_summary.txt"))
        @test isfile(joinpath(pn[:pf_dispatch_dual], "duality_verification.txt"))
        @test isfile(joinpath(pn[:pf_dispatch_dual_CSV_duals], "dual_dcopf_LMP.csv"))
        @test _duality_gap_rel(pn) < 1e-5
    end

    @testset "DC-OPF explicit dual without angle-diff limits" begin
        cfg = dispatch_run_config(
            type_model = "DCOPF",
            use_matrix = true,
            solver_name = _EXPLICIT_DUAL_SOLVER,
            silent_solver = true,
            save_optim_matrices = false,
            dispatch = DispatchConfig(
                type_model = "DCOPF",
                use_matrix = true,
                cost_type = "linear",
                ineq_ang_diff_branch = false,
                solve_explicit_dual = true,
            ),
        )
        result = _run_explicit_dual(cfg)
        @test result.status in _EXPLICIT_DUAL_SOLVED
        @test _duality_gap_rel(result.path_names) < 1e-5
    end

    # Regression for the primal/dual coefficient duplication (S5 in
    # OUTPUTS/archive/2026-07-02_dispatch_tsc_limits_wiring.md): with a non-default θ box the
    # dual objective must pick up the same values or strong duality breaks.
    @testset "DC-OPF explicit dual with non-default θ limits" begin
        cfg = dispatch_run_config(
            type_model = "DCOPF",
            use_matrix = true,
            solver_name = _EXPLICIT_DUAL_SOLVER,
            silent_solver = true,
            save_optim_matrices = false,
            dispatch = DispatchConfig(
                type_model = "DCOPF",
                use_matrix = true,
                cost_type = "linear",
                solve_explicit_dual = true,
                limits = DispatchLimitsConfig(θ_min_rad = -π / 2, θ_max_rad = π / 2),
            ),
        )
        result = _run_explicit_dual(cfg)
        @test result.status in _EXPLICIT_DUAL_SOLVED
        @test _duality_gap_rel(result.path_names) < 1e-5
    end

    @testset "solve_explicit_dual validation" begin
        @test_throws ArgumentError validate_dispatch_config!(DispatchConfig(
            type_model = "ED", cost_type = "quadratic", solve_explicit_dual = true))
        @test validate_dispatch_config!(DispatchConfig(
            type_model = "ED", cost_type = "linear", solve_explicit_dual = true)) === nothing
        @test_throws ArgumentError validate_dispatch_config!(DispatchConfig(
            type_model = "ACOPF", cost_type = "linear", solve_explicit_dual = true))

        @test_throws ArgumentError validate_dispatch_config!(DispatchConfig(
            type_model = "DCOPF", cost_type = "quadratic", solve_explicit_dual = true))
        @test_throws ArgumentError validate_dispatch_config!(DispatchConfig(
            type_model = "DCOPF", cost_type = "linear", use_matrix = false, solve_explicit_dual = true))
        @test validate_dispatch_config!(DispatchConfig(
            type_model = "DCOPF", cost_type = "linear", use_matrix = true, solve_explicit_dual = true)) === nothing
    end
end
