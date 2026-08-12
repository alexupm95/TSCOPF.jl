#=
================================================================================
 test/runtests_fullbus_gld.jl  —  FULL_BUS GL / OB disturbances
================================================================================
=#

using Test
using DataFrames

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))

const SOLVED_STATUSES = (OPTIMAL, LOCALLY_SOLVED, ITERATION_LIMIT)
const ZIP_IMPEDANCE = (1.0, 0.0, 0.0)   # (Z, I, P) — pure constant impedance

function run_fullbus_gld(;
    fault::FaultConfig,
    δ_tol_deg::Float64 = 100.0,
    overwrite_results::Bool = TEST_OVERWRITE_RESULTS,
)
    cfg = reconfigure_dyn(tsc_run_config(
        type_model = "ACOPF",
        solver_name = "Ipopt",
        δ_tol_deg = δ_tol_deg,
        silent_solver = true,
        save_optim_matrices = false,
        overwrite_results = overwrite_results,
        load_factor = 1.5,
    ); dyn_model = DynModelConfig(
        network_form = FULL_BUS,
        mech_power_mode = USE_PM,
        bound_style_δ = :coi_box,
        zip_load_p = ZIP_IMPEDANCE,
        zip_load_q = ZIP_IMPEDANCE,
        fault = fault,
    ))
    validate_dyn_config!(cfg)
    sys = load_fixture_system(cfg)
    validate_fault_config!(cfg.transient.dyn_model.fault, sys.DGEN, sys.DBUS, sys.DCIR)
    return run_fixture_case!(cfg, sys)
end

@testset "TSC-ACOPF FULL_BUS GL disturbances" begin

    @testset "FaultConfig validation (9-bus)" begin
        sys = load_fixture_system(dispatch_run_config(case="9bus"))
        @test validate_fault_config!(
            FaultConfig(fault_type=GL, gl_gen_ids=[3]), sys.DGEN, sys.DBUS, sys.DCIR) === nothing
        @test validate_fault_config!(
            FaultConfig(fault_type=GL, gl_load_bus_ids=[5], gl_percent_power=[-1.0]),
            sys.DGEN, sys.DBUS, sys.DCIR) === nothing
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=GL, gl_gen_ids=[1]), sys.DGEN, sys.DBUS, sys.DCIR)
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=GL), sys.DGEN, sys.DBUS, sys.DCIR)
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=GL, gl_load_bus_ids=[5], gl_percent_power=[-1.5]),
            sys.DGEN, sys.DBUS, sys.DCIR)

        # OB (Open Branch) — validation
        @test validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[6]), sys.DGEN, sys.DBUS, sys.DCIR) === nothing
        details_ob = build_fault_details(
            FaultConfig(fault_type=OB, ob_branch_ids=[6]), sys.path_input)
        @test details_ob[:fault_type] == "OB"
        @test details_ob[:ob][:branch_id] == [6]
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=OB), sys.DGEN, sys.DBUS, sys.DCIR)
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[6], gl_gen_ids=[3]),
            sys.DGEN, sys.DBUS, sys.DCIR)
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=GL, gl_gen_ids=[3], ob_branch_ids=[6]),
            sys.DGEN, sys.DBUS, sys.DCIR)
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[99]), sys.DGEN, sys.DBUS, sys.DCIR)
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[6, 6]), sys.DGEN, sys.DBUS, sys.DCIR)
        # Circuit 1 is the radial 1–4 branch; opening it isolates bus 1 (slack)
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[1]), sys.DGEN, sys.DBUS, sys.DCIR)
        # Circuits 6+8 (5–7 and 7–8) isolate the {2,7} pocket; 9-bus has no connected N-2
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[6, 8]), sys.DGEN, sys.DBUS, sys.DCIR)
        DCIR_off = copy(sys.DCIR)
        DCIR_off.l_status[6] = 0
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[6]), sys.DGEN, sys.DBUS, DCIR_off)

        # Multi-branch: toy 2-bus with three parallel circuits (9-bus has no connected N-2)
        DBUS_toy = DataFrame(bus=[1, 2], type=[3, 1])
        DGEN_toy = DataFrame(bus=[1], g_status=[1])
        DCIR_toy2 = DataFrame(from_bus=[1, 1], to_bus=[2, 2], l_status=[1, 1])
        @test validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[1]),
            DGEN_toy, DBUS_toy, DCIR_toy2) === nothing
        @test_throws ArgumentError validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[1, 2]),
            DGEN_toy, DBUS_toy, DCIR_toy2)
        DCIR_toy3 = DataFrame(
            from_bus=[1, 1, 1], to_bus=[2, 2, 2], l_status=[1, 1, 1])
        @test validate_fault_config!(
            FaultConfig(fault_type=OB, ob_branch_ids=[1, 2]),
            DGEN_toy, DBUS_toy, DCIR_toy3) === nothing

        DBUS_copy = copy(sys.DBUS)
        row5 = findfirst(==(5), DBUS_copy.bus)
        p_before = DBUS_copy.p_d[row5]
        apply_gl_load_scaling!(DBUS_copy, [5], [1.0])
        @test DBUS_copy.p_d[row5] ≈ 2 * p_before
        apply_gl_load_scaling!(DBUS_copy, [5], [-1.0])
        @test DBUS_copy.p_d[row5] == 0.0
        @test DBUS_copy.q_d[row5] == 0.0
        @test_throws ArgumentError apply_gl_load_scaling!(DBUS_copy, [5], [-1.5])
    end

    @testset "build + solve (gen 3 trip)" begin
        fault = FaultConfig(fault_type=GL, gl_gen_ids=[3])
        result = run_fullbus_gld(fault=fault)
        @test result.status in SOLVED_STATUSES
        assert_timestamped_results!(result.path_names)

        dmd = result.dyn_model_dict
        @test dmd[:meta][:fault_type] == "GL"
        @test dmd[:meta][:gl_element] == "gen"
        @test 3 ∉ dmd[:active_gen]
        @test !haskey(dmd[:eq_const], :eq_const_Pbalance_tpf)
        @test isfile(joinpath(result.path_names[:pf_results_date], "input_parameters.txt"))
        ip_txt = read(joinpath(result.path_names[:pf_results_date], "input_parameters.txt"), String)
        @test occursin("Fault Category:                GL", ip_txt)
        @test occursin("Generator ID(s):               [3]", ip_txt)
    end

    @testset "build + solve (load bus 5 trip)" begin
        fault = FaultConfig(
            fault_type=GL,
            gl_load_bus_ids=[5],
            gl_percent_power=[-1.0],
        )
        result = run_fullbus_gld(fault=fault)
        @test result.status in SOLVED_STATUSES
        assert_timestamped_results!(result.path_names)

        dmd = result.dyn_model_dict
        @test dmd[:meta][:fault_type] == "GL"
        @test dmd[:meta][:gl_element] == "load"
        @test !haskey(dmd[:eq_const], :eq_const_Pbalance_tpf)
    end

    @testset "build + solve (OB open circuit 6)" begin
        fault = FaultConfig(fault_type=OB, ob_branch_ids=[6])
        result = run_fullbus_gld(fault=fault)
        @test result.status in SOLVED_STATUSES
        assert_timestamped_results!(result.path_names)

        dmd = result.dyn_model_dict
        @test dmd[:meta][:fault_type] == "OB"
        @test dmd[:meta][:ob_branch_ids] == [6]
        @test !haskey(dmd[:eq_const], :eq_const_Pbalance_tpf)
        ip_txt = read(joinpath(result.path_names[:pf_results_date], "input_parameters.txt"), String)
        @test occursin("Fault Category:                OB", ip_txt)
        @test occursin("Open Branch (no short-circuit): [6]", ip_txt)
        @test occursin("Circuit 6:", ip_txt)
    end

    @testset "input DataFrames unchanged after OB open" begin
        cfg = reconfigure_dyn(tsc_run_config(type_model="ACOPF");
            dyn_model=DynModelConfig(
                network_form=FULL_BUS,
                mech_power_mode=USE_PM,
                bound_style_δ=:coi_box,
                zip_load_p=ZIP_IMPEDANCE,
                zip_load_q=ZIP_IMPEDANCE,
                fault=FaultConfig(fault_type=OB, ob_branch_ids=[6]),
            ))
        sys = load_fixture_system(cfg)
        l_status_before = copy(sys.DCIR.l_status)
        run_fixture_case!(cfg, sys)
        sys2 = load_fixture_system(cfg)
        @test sys2.DCIR.l_status == l_status_before
        @test l_status_before[6] == 1
    end

    @testset "input DataFrames unchanged after GL gen trip" begin
        cfg = reconfigure_dyn(tsc_run_config(type_model="ACOPF");
            dyn_model=DynModelConfig(
                network_form=FULL_BUS,
                mech_power_mode=USE_PM,
                bound_style_δ=:coi_box,
                zip_load_p=ZIP_IMPEDANCE,
                zip_load_q=ZIP_IMPEDANCE,
                fault=FaultConfig(fault_type=GL, gl_gen_ids=[3]),
            ))
        sys = load_fixture_system(cfg)
        g_status_before = copy(sys.DGEN.g_status)
        run_fixture_case!(cfg, sys)
        sys2 = load_fixture_system(cfg)
        @test sys2.DGEN.g_status == g_status_before
    end
end
