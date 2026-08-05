#=
================================================================================
 test/runtests_a6_fullbus_benchmark.jl  —  A6 numeric acceptance gate
================================================================================
 Re-solves the FULL_BUS TSC-ACOPF case from `main.jl` and compares against
 pinned values from RESULTS/_benchmark_TSCOPF_full/Results - 2026-06-19 153727.

 Slow (~20 s Ipopt). Included from `runtests_all.jl`, not `Pkg.test()`.
================================================================================
=#

using Test
using CSV
using DataFrames

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(@__DIR__, "test_common.jl"))
include(joinpath(@__DIR__, "a6_fullbus_benchmark_pins.jl"))
include(joinpath(@__DIR__, "a6_fullbus_benchmark_config.jl"))

const A6_SOLVED = (OPTIMAL, LOCALLY_SOLVED)

function _run_a6_fullbus()
    cfg = a6_main_fullbus_run_config()
    sys = load_fixture_system(cfg)
    return run_fixture_case!(cfg, sys)
end

"""First trajectory row at `t` in a `;`-delimited TS CSV (column 1 = time)."""
function _trajectory_row_at(df::DataFrame, t::Float64)
    col_t = df[!, 1]
    idx = findfirst(x -> isapprox(x, t; atol = 1e-9), col_t)
    idx === nothing && error("time $t not found in trajectory CSV")
    return collect(df[idx, 2:end])
end

@testset "A6 FULL_BUS TSC-ACOPF benchmark (main.jl)" begin

    result = _run_a6_fullbus()
    baseline_dir = a6_baseline_results_dir(PROJECT_ROOT)

    @testset "re-solve vs pinned scalars" begin
        @test result.status in A6_SOLVED
        assert_timestamped_results!(result.path_names)

        obj = result.obj_MVA   # already evaluated by run_case! (model is released)
        @test isapprox(obj, A6_FULLBUS_OBJ_EUR; rtol = A6_FULLBUS_OBJ_RTOL)

        @test result.RGEN !== nothing
        pg_by_id = [result.RGEN.p_g[i] for i in sort(collect(result.RGEN.id))]
        @test length(pg_by_id) == length(A6_FULLBUS_PG_MW)
        @test pg_by_id ≈ collect(A6_FULLBUS_PG_MW) rtol = A6_FULLBUS_PG_RTOL

        traj_path = joinpath(result.path_names[:pf_TS_CSV], "angle_rel_COI.csv")
        @test isfile(traj_path)
        df = CSV.read(traj_path, DataFrame; delim = ';')
        row = _trajectory_row_at(df, A6_ANGLE_REL_COI_T0)
        @test row ≈ collect(A6_ANGLE_REL_COI_DEG) rtol = A6_TRAJ_RTOL
    end

    if isdir(baseline_dir)
        @testset "optional diff vs frozen RESULTS tree" begin
            ref_gen = joinpath(baseline_dir, "Dispatch", "CSV", "generators_report.csv")
            new_gen = joinpath(result.path_names[:pf_dispatch_CSV], "generators_report.csv")
            @test isfile(ref_gen) && isfile(new_gen)
            df_ref = CSV.read(ref_gen, DataFrame; delim = ';')
            df_new = CSV.read(new_gen, DataFrame; delim = ';')
            @test df_ref.P_MW ≈ df_new.P_MW rtol = A6_FULLBUS_PG_RTOL

            ref_ang = joinpath(baseline_dir, "Transient_Stability", "CSV", "angle_rel_COI.csv")
            new_ang = joinpath(result.path_names[:pf_TS_CSV], "angle_rel_COI.csv")
            @test isfile(ref_ang) && isfile(new_ang)
            df_ref_a = CSV.read(ref_ang, DataFrame; delim = ';')
            df_new_a = CSV.read(new_ang, DataFrame; delim = ';')
            @test _trajectory_row_at(df_ref_a, A6_ANGLE_REL_COI_T0) ≈
                  _trajectory_row_at(df_new_a, A6_ANGLE_REL_COI_T0) rtol = A6_TRAJ_RTOL
        end
    end
end
