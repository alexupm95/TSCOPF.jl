#=
================================================================================
 test/runtests_matpower_parser.jl
   Regression tests for the standalone MATPOWER (.m) parser
   (_manage_inputs/functions_2_parse_matpower.jl)
================================================================================
 Coverage
 --------
   A. case9.m is the documented column-for-column mirror of the 9bus fixture, so
      importing it must reproduce Read_Input_Data(fixture_case("9bus")) exactly.
      Both live in test/INPUT_FILES/ and are a coupled pair: the .m carries the
      loads pre-scaled by 1.5, the CSVs carry them raw.
   B. case39.m spot-checks read straight from the .m (polynomial gencost,
      21-column gen rows, transformer taps, ±360 angle limits).
   C. The three CSVs land under RESULTS/Results - <ts>/Inputs/ and re-read cleanly.
   D. Matpower_Gencost_To_Quadratic unit cases (model/ncost handling).
================================================================================
=#
using Test

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))

const CASE9_M  = fixture_matpower("case9.m")
const CASE39_M = fixture_matpower("case39.m")

"""Assert two DataFrames have the same schema and numerically-equal columns.
`exclude` skips named columns (e.g. load columns that differ by a scale factor)."""
function assert_df_equal(a::DataFrame, b::DataFrame; atol=1e-6, exclude=String[])
    @test names(a) == names(b)
    @test nrow(a) == nrow(b)
    for col in names(a)
        col in exclude && continue
        @test isapprox(Float64.(a[!, col]), Float64.(b[!, col]); atol=atol)
    end
end

@testset "MATPOWER parser" begin

    @testset "A. case9.m round-trips to the 9bus fixture" begin
        # path_main here only decides where Import_Matpower_Case writes its
        # RESULTS/ tree; the case file itself is the frozen fixture.
        imp = Import_Matpower_Case(CASE9_M; path_main=PROJECT_ROOT)
        ref_DBUS, ref_DGEN, ref_DCIR, _, _ =
            TSCOPF.Read_Input_Data(fixture_case("9bus"), false)

        @test imp.baseMVA == 100.0
        # Generators and branches mirror the 9bus fixture exactly.
        assert_df_equal(imp.DGEN, ref_DGEN)
        assert_df_equal(imp.DCIR, ref_DCIR)
        # Bus data matches on every column EXCEPT the loads: the case9.m mirror was
        # generated at the benchmark's load_factor = 1.5, so its Pd/Qd are the
        # in-house base loads scaled by 1.5 (187.5 = 125×1.5, 75 = 50×1.5, …).
        assert_df_equal(imp.DBUS, ref_DBUS; exclude=["p_d", "q_d"])
        @test isapprox(imp.DBUS.p_d, 1.5 .* ref_DBUS.p_d; atol=1e-6)
        @test isapprox(imp.DBUS.q_d, 1.5 .* ref_DBUS.q_d; atol=1e-6)
    end

    @testset "B. case39.m parsed values" begin
        imp = Import_Matpower_Case(CASE39_M; path_main=PROJECT_ROOT)

        @test imp.baseMVA == 100.0
        @test nrow(imp.DBUS) == 39
        @test nrow(imp.DGEN) == 10
        @test nrow(imp.DCIR) == 46

        # Identical quadratic gen costs:  2 0 0 3 0.01 0.3 0.2
        @test all(isapprox.(imp.DGEN.g_cost_2, 0.01; atol=1e-9))
        @test all(isapprox.(imp.DGEN.g_cost_1, 0.30; atol=1e-9))
        @test all(isapprox.(imp.DGEN.g_cost_0, 0.20; atol=1e-9))

        # Bus 39 load (buses 1..39 → identity remap)
        @test isapprox(imp.DBUS.p_d[39], 1104.0; atol=1e-6)
        @test isapprox(imp.DBUS.q_d[39],  250.0; atol=1e-6)

        # Transformer branch 2→30 has ratio 1.025; plain lines have ratio 0 → t_tap 1.0
        row_xf  = findfirst((imp.DCIR.from_bus .== 2) .& (imp.DCIR.to_bus .== 30))
        @test row_xf !== nothing
        @test isapprox(imp.DCIR.t_tap[row_xf], 1.025; atol=1e-9)
        row_ln  = findfirst((imp.DCIR.from_bus .== 1) .& (imp.DCIR.to_bus .== 2))
        @test isapprox(imp.DCIR.t_tap[row_ln], 1.0; atol=1e-9)   # reader maps 0 → 1

        # ±360 angle limits across all branches
        @test all(isapprox.(imp.DCIR.ang_min, -360.0; atol=1e-6))
        @test all(isapprox.(imp.DCIR.ang_max,  360.0; atol=1e-6))
    end

    @testset "C. CSVs written under RESULTS/.../Inputs and re-readable" begin
        imp = Import_Matpower_Case(CASE9_M; path_main=PROJECT_ROOT)
        inputs_dir = imp.path_names[:pf_inputs]
        @test occursin("Results -", inputs_dir)
        for f in ("bus_data.csv", "generators_data.csv", "line_data.csv")
            @test isfile(joinpath(inputs_dir, f))
        end
        # Re-reading the written folder reproduces the same frames (idempotent).
        DBUS2, DGEN2, DCIR2, _, _ = TSCOPF.Read_Input_Data(inputs_dir, false)
        assert_df_equal(imp.DBUS, DBUS2)
        assert_df_equal(imp.DGEN, DGEN2)
        assert_df_equal(imp.DCIR, DCIR2)
    end

    @testset "D. gencost → quadratic mapping" begin
        # model 2, ncost 3 → [c2, c1, c0]
        gc3 = [2.0 0 0 3 0.01 0.3 0.2]
        c2, c1, c0 = TSCOPF.Matpower_Gencost_To_Quadratic(gc3, 1)
        @test (c2[1], c1[1], c0[1]) == (0.01, 0.3, 0.2)
        # model 2, ncost 2 (linear) → [0, c1, c0]
        gc2 = [2.0 0 0 2 50.0 0.0]
        c2, c1, c0 = TSCOPF.Matpower_Gencost_To_Quadratic(gc2, 1)
        @test (c2[1], c1[1], c0[1]) == (0.0, 50.0, 0.0)
        # model 2, ncost 1 (constant) → [0, 0, c0]
        gc1 = [2.0 0 0 1 7.0]
        c2, c1, c0 = TSCOPF.Matpower_Gencost_To_Quadratic(gc1, 1)
        @test (c2[1], c1[1], c0[1]) == (0.0, 0.0, 7.0)
        # 2*ng rows → first ng used
        gc_2ng = [2.0 0 0 3 1.0 2.0 3.0; 2.0 0 0 3 9.0 9.0 9.0]
        c2, c1, c0 = TSCOPF.Matpower_Gencost_To_Quadratic(gc_2ng, 1)
        @test (c2[1], c1[1], c0[1]) == (1.0, 2.0, 3.0)
        # missing gencost → zeros
        @test TSCOPF.Matpower_Gencost_To_Quadratic(nothing, 2) == (zeros(2), zeros(2), zeros(2))
        # unsupported: piecewise (model 1) and cubic (ncost 4)
        @test_throws ArgumentError TSCOPF.Matpower_Gencost_To_Quadratic([1.0 0 0 2 0 0 1 1], 1)
        @test_throws ArgumentError TSCOPF.Matpower_Gencost_To_Quadratic([2.0 0 0 4 1 2 3 4], 1)
    end
end
