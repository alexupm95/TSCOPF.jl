#=
 DC susceptance model (`SIMPLE` vs `POWERMODELS`) regression tests.

 - 9bus  (r = 0): SIMPLE and POWERMODELS must agree (objective + Bbus matrices).
 - 39bus (r ≠ 0): SIMPLE and POWERMODELS must differ on branch flows.

 Uses HiGHS (LP) — no Gurobi license required.

 Run: `julia --project=. test/runtests_susceptance_model.jl`
=#

using Test
using Printf

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))

const SOLVED = (OPTIMAL, LOCALLY_SOLVED, ITERATION_LIMIT)

function run_dc(case::String, m::SusceptanceModel, use_matrix::Bool)
    load_factor = case == "39bus" ? 1.0 : 1.5
    cfg = RunConfig(;
        trans_stab = false,
        case = case,
        solver_name = "HiGHS",
        silent_solver = true,
        load_factor = load_factor,
        save_optim_matrices = false,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        dispatch = DispatchConfig(type_model = "DCOPF", use_matrix = use_matrix,
                                  susceptance_model = m),
    )
    sys = load_fixture_system(cfg)
    res = run_fixture_case!(cfg, sys)
    return res
end

flows(res) = sort(res.RCIR.p_ik)

@testset "Selectable DC susceptance model" begin

    @testset "dc_branch_susceptance helper math" begin
        @test TSCOPF.dc_branch_susceptance(0.0, 0.1, SIMPLE) ≈ 10.0
        @test TSCOPF.dc_branch_susceptance(0.0, 0.1, POWERMODELS) ≈ 10.0
        @test TSCOPF.dc_branch_susceptance(0.05, 0.1, SIMPLE) ≈ 10.0
        @test TSCOPF.dc_branch_susceptance(0.05, 0.1, POWERMODELS) ≈ 0.1 / (0.05^2 + 0.1^2)
    end

    @testset "9bus (r=0): SIMPLE ≡ POWERMODELS Bbus + objective" begin
        sys = load_fixture_system(RunConfig(; trans_stab = false, case = "9bus",
                  dispatch = DispatchConfig(type_model = "DCOPF")))
        B_simple = TSCOPF.Calculate_Matrix_B(sys.DBUS, sys.DCIR, sys.nBUS, sys.nCIR)
        B_pm = TSCOPF.Calculate_Matrix_B_PowerModels(sys.DBUS, sys.DCIR, sys.nBUS, sys.nCIR)
        @test isapprox(Matrix(B_simple), Matrix(B_pm); atol = 1e-9)

        r_s_m = run_dc("9bus", SIMPLE, true)
        r_s_nm = run_dc("9bus", SIMPLE, false)
        r_p_m = run_dc("9bus", POWERMODELS, true)
        r_p_nm = run_dc("9bus", POWERMODELS, false)
        for r in (r_s_m, r_s_nm, r_p_m, r_p_nm)
            @test r.status in SOLVED
        end
        obj = r_s_m.obj_MVA   # already evaluated by run_case! (model is released)
        @test isapprox(r_s_nm.obj_MVA, obj; rtol = 1e-6)
        @test isapprox(r_p_m.obj_MVA, obj; rtol = 1e-6)
        @test isapprox(r_p_nm.obj_MVA, obj; rtol = 1e-6)
        @printf("9bus obj = %.3f EUR  (SIMPLE==POWERMODELS, r=0)\n", obj)
    end

    @testset "39bus (r≠0): SIMPLE differs from POWERMODELS" begin
        r_s = run_dc("39bus", SIMPLE, true)
        r_p = run_dc("39bus", POWERMODELS, true)
        @test r_s.status in SOLVED
        @test r_p.status in SOLVED
        r_s_nm = run_dc("39bus", SIMPLE, false)
        r_p_nm = run_dc("39bus", POWERMODELS, false)
        @test isapprox(flows(r_s), flows(r_s_nm); atol = 1e-6)
        @test isapprox(flows(r_p), flows(r_p_nm); atol = 1e-6)
        maxdiff = maximum(abs.(flows(r_s) .- flows(r_p)))
        @test maxdiff > 1e-4
        @printf("39bus max|Δp_ik| between SIMPLE and POWERMODELS = %.4f p.u.\n", maxdiff)
    end
end
