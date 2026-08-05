#=
================================================================================
 test/runtests_dispatch_duals_registry.jl
================================================================================
 Guards the registry-driven steady-state dual exporter (DispatchDualRegistry.jl
 + Save_Duals_OPF_Model). Verifies:
   1. haskey filtering — ED emits only its families; ACOPF emits the AC-only ones.
   2. The header fixes are baked into the single spec table (Gen_ID, Dual_Ski_Upper,
      Dual_diff_ang_Upper).
   3. Decision 2 — branch flow-bound duals are promoted to CSV when those bounds
      are enabled.
================================================================================
=#

using Test
using CSV

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))

const SOLVED_STATUSES = (OPTIMAL, LOCALLY_SOLVED, ITERATION_LIMIT)

"""Header tokens of a `;`-delimited CSV file."""
csv_header(path) = split(strip(readline(path)), ';')

function run_dispatch(cfg::RunConfig)
    sys = load_fixture_system(cfg)
    return run_fixture_case!(cfg, sys)
end

@testset "Steady-state dual registry export" begin

    @testset "ED exports only P-balance + Pg bounds (haskey filter)" begin
        cfg = dispatch_run_config(type_model="ED", solver_name="Ipopt",
            silent_solver=true, save_optim_matrices=false,
            overwrite_results=TEST_OVERWRITE_RESULTS, save_duals=true)
        res = run_dispatch(cfg)
        @test res.status in SOLVED_STATUSES
        d = res.path_names[:pf_dispatch_CSV_duals]

        @test isfile(joinpath(d, "dual_P_balance.csv"))
        @test isfile(joinpath(d, "dual_Pg_lower.csv"))
        @test isfile(joinpath(d, "dual_Pg_upper.csv"))
        # AC-only / network families must be absent for ED.
        @test !isfile(joinpath(d, "dual_Q_balance.csv"))
        @test !isfile(joinpath(d, "dual_V_lower.csv"))
        @test !isfile(joinpath(d, "dual_Sik_Upper.csv"))

        # Generator constraint → Gen_ID (was mislabeled Bus_ID).
        @test "Gen_ID" in csv_header(joinpath(d, "dual_Pg_lower.csv"))
    end

    @testset "ACOPF exports AC families with correct headers" begin
        cfg = dispatch_run_config(type_model="ACOPF", solver_name="Ipopt",
            silent_solver=true, save_optim_matrices=false,
            overwrite_results=TEST_OVERWRITE_RESULTS, save_duals=true)
        res = run_dispatch(cfg)
        @test res.status in SOLVED_STATUSES
        d = res.path_names[:pf_dispatch_CSV_duals]

        @test isfile(joinpath(d, "dual_Q_balance.csv"))
        @test isfile(joinpath(d, "dual_V_lower.csv"))
        @test isfile(joinpath(d, "dual_Sik_Upper.csv"))
        @test isfile(joinpath(d, "dual_Ski_Upper.csv"))
        @test isfile(joinpath(d, "dual_diff_ang_Upper.csv"))

        # Ski file gets its OWN value column (was mislabeled Dual_Sik_Upper).
        ski = csv_header(joinpath(d, "dual_Ski_Upper.csv"))
        @test "Branch_ID" in ski
        @test "Dual_Ski_Upper" in ski

        # ang-diff upper labelled Upper (was mislabeled Dual_diff_ang_Lower).
        @test "Dual_diff_ang_Upper" in csv_header(joinpath(d, "dual_diff_ang_Upper.csv"))

        # XLSX workbook written.
        @test isfile(joinpath(res.path_names[:pf_dispatch], "Dispatch_Duals.xlsx"))
    end

    @testset "Branch flow-bound duals promoted to CSV (decision 2)" begin
        # P_ik bound off by default; enable it (needs explicit flow variables → use_matrix=false).
        cfg = RunConfig(; trans_stab=false, case="9bus",
            dispatch=DispatchConfig(type_model="ACOPF", use_matrix=false, bound_P_ik=true),
            solver_name="Ipopt", silent_solver=true, save_optim_matrices=false,
            overwrite_results=TEST_OVERWRITE_RESULTS, save_duals=true)
        res = run_dispatch(cfg)
        @test res.status in SOLVED_STATUSES
        d = res.path_names[:pf_dispatch_CSV_duals]

        @test isfile(joinpath(d, "dual_Pik_lower.csv"))
        @test isfile(joinpath(d, "dual_Pik_upper.csv"))
        lo = csv_header(joinpath(d, "dual_Pik_lower.csv"))
        @test "Branch_ID" in lo
        @test "Dual_Pik_Lower" in lo
    end

end
