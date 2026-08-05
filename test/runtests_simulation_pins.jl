#=
================================================================================
 test/runtests_simulation_pins.jl — reference-validated simulation scalar pins
================================================================================
 Re-solves cases from _TSCOPF_simulations `bound_compare_*_constraint` suites
 and compares objective, dispatch P_g, and δ-COI trajectories against committed
 scalars in test/simulation_pins/pins_*.jl.

 Requires variant INPUT_FILES from the simulations repo:
   ENV["TSCOPF_SIMULATIONS_ROOT"] or sibling ../_TSCOPF_simulations

 Run manually:
   julia --project=. test/runtests_simulation_pins.jl

 Enable in aggregate suite:
   TSCOPF_RUN_SIMULATION_PINS=true julia --project=. test/runtests_all.jl
================================================================================
=#

using Test
using CSV
using DataFrames
using JuMP

const PROJECT_ROOT = dirname(@__DIR__)
const SIM_PINS_DIR = joinpath(@__DIR__, "simulation_pins")

include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(@__DIR__, "test_common.jl"))
include(joinpath(SIM_PINS_DIR, "configs.jl"))

"""First trajectory row at `t` in a `;`-delimited TS CSV (column 1 = time)."""
function _trajectory_row_at(df::DataFrame, t::Float64)
    col_t = df[!, 1]
    idx = findfirst(x -> isapprox(Float64(x), t; atol = 1e-9), col_t)
    idx === nothing && error("time $t not found in trajectory CSV")
    return collect(df[idx, 2:end])
end

function _run_simulation_pin_case(cfg::RunConfig)
    sim_root = simulations_project_root(PROJECT_ROOT)
    sim_root == "" && error("simulations repo not found; set TSCOPF_SIMULATIONS_ROOT")
    sys = load_system(cfg, sim_root)
    return run_case!(cfg, sys, sim_root, joinpath(PROJECT_ROOT, "RESULTS"))
end

function _dispatch_obj_eur(path_names)
    p = joinpath(path_names[:pf_dispatch_CSV], "optimization_report.csv")
    @test isfile(p)
    df = CSV.read(p, DataFrame; delim = ';')
    idx = findfirst(==("Total Cost (Euros)"), df.Metric)
    idx === nothing && error("Total Cost row missing in optimization_report.csv")
    return Float64(df.Value[idx])
end

function _assert_pin_scalars!(result, pins)
    @test result.status in SIMULATION_PIN_SOLVED
    assert_timestamped_results!(result.path_names)

    # Bail on a failed solve. Without this, a NUMERICAL_ERROR run produces one real
    # failure followed by two derived ones (no optimization_report.csv was written,
    # so the objective read throws), which buries the actual cause.
    if !(result.status in SIMULATION_PIN_SOLVED)
        @warn "Solve did not converge — skipping the scalar comparisons." result.status
        return nothing
    end

    obj = _dispatch_obj_eur(result.path_names)
    @test isapprox(obj, pins.obj_eur; rtol = pins.obj_rtol)

    @test result.RGEN !== nothing
    pg_by_id = [result.RGEN.p_g[i] for i in sort(collect(result.RGEN.id))]
    @test length(pg_by_id) == length(pins.pg_mw)
    @test pg_by_id ≈ collect(pins.pg_mw) rtol = pins.pg_rtol

    if pins.angle_rel_coi_deg !== nothing
        traj_path = joinpath(result.path_names[:pf_TS_CSV], "angle_rel_COI.csv")
        @test isfile(traj_path)
        df = CSV.read(traj_path, DataFrame; delim = ';')
        for (t, expected) in pins.angle_rel_coi_deg
            row = _trajectory_row_at(df, Float64(t))
            @test length(row) == length(expected)
            @test row ≈ collect(expected) rtol = pins.traj_rtol
        end
    end
end

@testset "Simulation pins (reference-validated baselines)" begin

    if !simulations_input_available(PROJECT_ROOT)
        @warn """
        Skipping all simulation pin tests: set TSCOPF_SIMULATIONS_ROOT to
        _TSCOPF_simulations or place it as a sibling of the integrated repo.
        """
    else

        @testset "ACOPF dispatch (bound_compare_constraint)" begin
            include(joinpath(SIM_PINS_DIR, "pins_acopf_dispatch_constraint.jl"))
            cfg = sim_config_acopf_dispatch_constraint()
            result = _run_simulation_pin_case(cfg)
            _assert_pin_scalars!(result, ACOPF_DISPATCH_CONSTRAINT_PINS)
        end

        @testset "DCOPF dispatch (bound_compare_dcopf_constraint)" begin
            include(joinpath(SIM_PINS_DIR, "pins_dcopf_dispatch_constraint.jl"))
            cfg = sim_config_dcopf_dispatch_constraint()
            result = _run_simulation_pin_case(cfg)
            _assert_pin_scalars!(result, DCOPF_DISPATCH_CONSTRAINT_PINS)
        end

        @testset "4th-order FULL_BUS + AVR (bound_compare_4th_fullbus_avr)" begin
            include(joinpath(SIM_PINS_DIR, "pins_4th_fullbus_avr.jl"))
            result = _run_simulation_pin_case(sim_config_4th_fullbus_avr())
            _assert_pin_scalars!(result, BOUND_COMPARE_4TH_FULLBUS_AVR_PINS)
        end

        @testset "4th-order FULL_BUS core (bound_compare_4th_fullbus)" begin
            include(joinpath(SIM_PINS_DIR, "pins_4th_fullbus.jl"))
            result = _run_simulation_pin_case(sim_config_4th_fullbus())
            _assert_pin_scalars!(result, BOUND_COMPARE_4TH_FULLBUS_PINS)
        end

        @testset "2nd-order FULL_BUS (bound_compare_2nd_fullbus)" begin
            include(joinpath(SIM_PINS_DIR, "pins_2nd_fullbus.jl"))
            cfg = sim_config_2nd_fullbus_solver()
            result = _run_simulation_pin_case(cfg)
            _assert_pin_scalars!(result, BOUND_COMPARE_2ND_FULLBUS_PINS)
        end

        @testset "2nd-order Kron (bound_compare_2nd_kron)" begin
            include(joinpath(SIM_PINS_DIR, "pins_2nd_kron.jl"))
            cfg = sim_config_2nd_kron_solver()
            result = _run_simulation_pin_case(cfg)
            _assert_pin_scalars!(result, BOUND_COMPARE_2ND_KRON_PINS)
        end

        @testset "GL gen3 DQ FULL_BUS (bound_compare_gl_gen3_4th_fullbus)" begin
            include(joinpath(SIM_PINS_DIR, "pins_gl_gen3_4th_fullbus.jl"))
            result = _run_simulation_pin_case(sim_config_gl_gen3_4th_fullbus())
            _assert_pin_scalars!(result, BOUND_COMPARE_GL_GEN3_4TH_FULLBUS_PINS)
        end

        @testset "GL gen3 Kron (bound_compare_gl_gen3_kron)" begin
            include(joinpath(SIM_PINS_DIR, "pins_gl_gen3_kron.jl"))
            result = _run_simulation_pin_case(sim_config_gl_gen3_kron())
            _assert_pin_scalars!(result, BOUND_COMPARE_GL_GEN3_KRON_PINS)
        end

        @testset "4th-order FULL_BUS AVR+TG (reference Gvnr case)" begin
            include(joinpath(SIM_PINS_DIR, "pins_4th_fullbus_avr_tg.jl"))
            result = _run_simulation_pin_case(sim_config_4th_fullbus_avr_tg())
            _assert_pin_scalars!(result, BOUND_COMPARE_4TH_FULLBUS_AVR_TG_PINS)
        end

        @testset "4th-order FULL_BUS TG only (bound_compare_4th_fullbus_tg)" begin
            include(joinpath(SIM_PINS_DIR, "pins_4th_fullbus_tg.jl"))
            result = _run_simulation_pin_case(sim_config_4th_fullbus_tg())
            _assert_pin_scalars!(result, BOUND_COMPARE_4TH_FULLBUS_TG_PINS)
        end

    end
end
