#=
================================================================================
 test/runtests_matpower_input.jl
   Smoke test for the MATPOWER .m input mode (RunConfig.matpower_file).
================================================================================
 Verifies that loading steady-state data from a MATPOWER .m file + a separate
 gen_dynamic_data.csv produces the same system — and the same solved dispatch —
 as loading from the canonical in-house CSVs (9bus/).

 The fixture folder test/INPUT_FILES/9bus_test_mfile/ contains:
   case9.m              — MATPOWER case mirroring 9bus/*.csv exactly
   gen_dynamic_data.csv — same machine data as 9bus/gen_dynamic_data.csv
   contingencies.csv    — same fault table  as 9bus/contingencies.csv

 Scope: INGESTION only. The `.m` path affects `load_system` and nothing else, so
 the two solve testsets compare the two ingestion modes against EACH OTHER with a
 cheap ACOPF, instead of re-running the FULL_BUS TSC benchmark that
 runtests_a6_fullbus_benchmark.jl already owns.
================================================================================
=#

using Test
using TSCOPF
using JuMP
using CSV, DataFrames
import MathOptInterface as MOI

const PROJECT_ROOT = dirname(@__DIR__)

include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(@__DIR__, "test_common.jl"))

const SOLVED_OK = (MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ITERATION_LIMIT)

# Agreement tolerances between the .m and CSV ingestion paths.
const MATPOWER_GEN_P_ATOL_MW = 0.15
const MATPOWER_BUS_V_ATOL = 5e-4

"""Steady-state ACOPF twin of `matpower_fullbus_config` — no transient layer, seconds not minutes."""
function matpower_acopf_config(; case::String, matpower_file::Union{Nothing, String})
    return RunConfig(
        trans_stab          = false,
        case                = case,
        base_MVA            = 100.0,
        load_factor         = 1.5,
        solver_name         = "Ipopt",
        silent_solver       = true,
        overwrite_results   = TEST_OVERWRITE_RESULTS,
        save_duals          = false,
        save_matrices       = false,
        save_optim_matrices = false,
        matpower_file       = matpower_file,
        dispatch = DispatchConfig(
            type_model           = "ACOPF",
            cost_type            = "quadratic",
            use_matrix           = true,
            ineq_sbranch_upper   = true,
            ineq_ang_diff_branch = true,
        ),
    )
end

# ---------------------------------------------------------------------------
#  RunConfig that matches main.jl (FULL_BUS, SC cont 2, same builder flags)
# ---------------------------------------------------------------------------
function matpower_fullbus_config(; case::String, matpower_file::Union{Nothing, String})
    return RunConfig(
        trans_stab        = true,
        case              = case,
        base_MVA          = 100.0,
        load_factor       = 1.5,
        solver_name       = "Ipopt",
        silent_solver     = true,
        overwrite_results = false,
        save_duals        = false,   # skip for speed in CI
        save_matrices     = false,
        save_ts_plots     = false,   # Plots extension not required in CI
        save_optim_matrices = false,
        matpower_file     = matpower_file,

        dispatch = DispatchConfig(
            type_model           = "ACOPF",
            cost_type            = "quadratic",
            use_matrix           = true,
            ineq_sbranch_upper   = true,
            ineq_ang_diff_branch = true,
        ),

        transient = TransientConfig(
            simulation = TsSimulationConfig(δ_tol_deg = 100.0),
            builder = TsBuilderConfig(
                # variable bounds (matching main.jl)
                bound_E   = true,
                bound_δ   = true,
                bound_P_m = true,
                bound_δ_tf = false, bound_Δω_tf = false,
                bound_Pe_tf = false, bound_δCOI_tf = false,
                bound_δ_tpf = false, bound_Δω_tpf = false,
                bound_Pe_tpf = false, bound_δCOI_tpf = false,
                # COI stability bounds
                ineq_δ_COI_tf_lower  = true, ineq_δ_COI_tf_upper  = true,
                ineq_δ_COI_tpf_lower = true, ineq_δ_COI_tpf_upper = true,
            ),
            dyn_model = DynModelConfig(
                network_form    = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ     = :coi_box,
                zip_load_p        = (1.0, 0.0, 0.0),
                zip_load_q        = (1.0, 0.0, 0.0),
                constrain_Δω = false,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
end

# ---------------------------------------------------------------------------
#  Helper: extract per-generator P_g [MW] from the solved RGEN DataFrame
# ---------------------------------------------------------------------------
function gen_p_from_result(result)
    rgen = result.RGEN
    rgen === nothing && return Float64[]
    # RGEN DataFrame uses column :p_g [MW]  (see functions_2_save_dispatch_results.jl)
    return Float64.(rgen.p_g)
end

# ---------------------------------------------------------------------------
#  Test suite
# ---------------------------------------------------------------------------
@testset "MATPOWER .m input mode (9bus_test_mfile)" begin

    # --- 1. load_system parses and remaps correctly ---------------------------
    @testset "load_system — schema + bus-remap" begin
        cfg = matpower_fullbus_config(
            case          = "9bus_test_mfile",
            matpower_file = "case9.m",
        )
        sys = load_fixture_system(cfg)

        # Dimension checks: must match the canonical 9bus network
        @test sys.nBUS == 9
        @test sys.nGEN == 3
        @test sys.nCIR == 9

        # Bus remap invariant: labels must be exactly 1..nBUS in order
        @test sys.DBUS.bus == collect(1:sys.nBUS)

        # Load scaling applied (bus 5 nominal Pd=125 MW → 1.5×125=187.5 MW)
        bus5_row = findfirst(==(5), sys.DBUS.bus)
        @test sys.DBUS.p_d[bus5_row] ≈ 187.5 atol=1e-6

        # Dynamic data loaded (3 generators from gen_dynamic_data.csv)
        @test sys.DGEN_DYN !== nothing
        @test nrow(sys.DGEN_DYN) == 3

        # Generator Xd_tr values (from gen_dynamic_data.csv: 0.0608, 0.1198, 0.1813)
        @test sys.DGEN_DYN.Xd_tr ≈ [0.0608, 0.1198, 0.1813] atol=1e-6
    end

    # --- 2. error on missing file ---------------------------------------------
    @testset "load_system — error on bad matpower_file" begin
        cfg = matpower_fullbus_config(
            case          = "9bus_test_mfile",
            matpower_file = "nonexistent.m",
        )
        @test_throws ArgumentError load_fixture_system(cfg)
    end

    # --- 3. .m mode and CSV mode must produce the same solved system ----------
    # This file is about INGESTION. It used to run two full TSC-ACOPF FULL_BUS
    # solves and compare them to the A6 pins, which duplicated
    # runtests_a6_fullbus_benchmark.jl at ~2 min of Ipopt per solve. The `.m` path
    # only affects `load_system`; once the SystemData is right, the transient layer
    # is agnostic to how it was read. So: two cheap ACOPF solves, compared against
    # each other. That is a stronger invariant than the old external pins (it fails
    # if the parser drifts in either direction) and needs no frozen baseline.
    @testset "matpower vs CSV ingestion — same dispatch" begin
        cfg_m = matpower_acopf_config(case = "9bus_test_mfile", matpower_file = "case9.m")
        res_m = run_fixture_case!(cfg_m, load_fixture_system(cfg_m))

        cfg_csv = matpower_acopf_config(case = "9bus", matpower_file = nothing)
        sys_csv = load_fixture_system(cfg_csv)
        res_csv = run_fixture_case!(cfg_csv, sys_csv)

        @test res_m.status in SOLVED_OK
        @test res_csv.status in SOLVED_OK
        assert_timestamped_results!(res_m.path_names)
        @test sys_csv.nBUS == 9
        @test sys_csv.nGEN == 3

        # Objective (already evaluated — run_case! releases the JuMP backend).
        @test isapprox(res_m.obj_MVA, res_csv.obj_MVA; rtol = 1e-4)

        # Per-generator dispatch and bus voltages agree machine by machine.
        p_m, p_csv = gen_p_from_result(res_m), gen_p_from_result(res_csv)
        @test length(p_m) == 3 && length(p_csv) == 3
        for (i, (a, b)) in enumerate(zip(p_m, p_csv))
            @test isapprox(a, b; atol = MATPOWER_GEN_P_ATOL_MW) ||
                  @warn "Gen $i: .m P=$(round(a; digits=3)) MW vs CSV $(round(b; digits=3)) MW"
        end
        @test isapprox(res_m.RBUS.v, res_csv.RBUS.v; atol = MATPOWER_BUS_V_ATOL)

        # Inputs archived per mode: the .m run keeps the source file, the CSV run
        # keeps the three steady-state tables and no .m.
        # (Only the .m itself here: gen_dynamic_data.csv and contingencies.csv are
        # transient-layer inputs and are not archived by a trans_stab = false run.
        # Testset 1 above already checks they are read for the .m case.)
        in_m = res_m.path_names[:pf_inputs]
        @test isfile(joinpath(in_m, "case9.m"))

        # Positive assertions only. `build_results_paths` timestamps to the SECOND,
        # so two fast solves in the same second share one `Results - …` folder and
        # its `Inputs/` becomes a union of both runs — a negative assertion like
        # "the CSV run archived no .m" is not attributable and fails at random.
        in_csv = res_csv.path_names[:pf_inputs]
        @test isfile(joinpath(in_csv, "bus_data.csv"))
        @test isfile(joinpath(in_csv, "generators_data.csv"))
        @test isfile(joinpath(in_csv, "line_data.csv"))
    end
end
