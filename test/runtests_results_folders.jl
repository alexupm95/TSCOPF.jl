#=
================================================================================
 test/runtests_results_folders.jl — simulation-dependent RESULTS/ subfolders
================================================================================
=#

const PROJECT_ROOT = dirname(@__DIR__)

using Test

include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(@__DIR__, "test_common.jl"))

@testset "Results folder layout" begin
    path_main = FIXTURE_ROOT               # inputs come from test/INPUT_FILES/
    path_results = RESULTS_ROOT            # results still land in <repo>/RESULTS

    @testset "project paths (A3)" begin
        @test normpath(project_root()) == normpath(PROJECT_ROOT)
        # Path shape only. Asserting `isdir` here would make the suite depend on
        # the demo tree still existing, which is exactly what the fixture split
        # removed — a user may rename or delete INPUT_FILES/9bus.
        @test input_files_dir("9bus") == joinpath(project_root(), "INPUT_FILES", "9bus")
        @test default_results_dir() == joinpath(PROJECT_ROOT, "RESULTS")
    end

    # Regression: the run directory name has second resolution, so back-to-back runs
    # used to share one folder and interleave their outputs. `_timestamped_results_dir`
    # now claims each directory with `mkdir` and appends " (n)" on collision.
    @testset "run directories are unique within the same second" begin
        tmp = mktempdir()
        dirs = [TSCOPF._timestamped_results_dir(tmp) for _ in 1:3]
        # The invariant that matters: N calls → N distinct directories, all created.
        # (Asserting *how many* carry a " (n)" suffix would be flaky — it depends on
        # whether the clock ticks over mid-loop.)
        @test length(unique(dirs)) == 3
        @test all(isdir, dirs)
        @test all(d -> startswith(basename(d), "Results - "), dirs)
    end

    @testset "ED steady-state (minimal tree)" begin
        cfg = dispatch_run_config(
            type_model="ED",
            save_matrices=false,
            save_duals=false,
            overwrite_results=TEST_OVERWRITE_RESULTS,
        )
        sys = load_system(cfg, path_main)
        result = run_case!(cfg, sys, path_main, path_results)
        pn = result.path_names
        assert_timestamped_results!(pn)
        @test isdir(pn[:pf_inputs])
        @test isdir(pn[:pf_dispatch_CSV])
        @test !isdir(pn[:pf_bus_matrices])
        @test !isdir(pn[:pf_TS])
        @test !isdir(pn[:pf_dispatch_dual])
    end

    @testset "MATPOWER import — Inputs/ only" begin
        case_m = fixture_matpower("case9.m")
        isfile(case_m) || return  # skip if MATPOWER case not bundled
        imp = Import_Matpower_Case(case_m; path_main=path_main, path_results=path_results)
        pn = imp.path_names
        assert_timestamped_results!(pn)
        @test isdir(pn[:pf_inputs])
        @test isfile(joinpath(pn[:pf_inputs], "bus_data.csv"))
        @test !isdir(pn[:pf_dispatch_CSV])
        @test !isdir(pn[:pf_TS])
    end
end
