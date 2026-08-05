#=
================================================================================
 test/test_common.jl  —  shared conventions for all test entry points
================================================================================
 Every test run writes to a timestamped folder under RESULTS/ (never overwrites
 the flat RESULTS tree).  Include from each test/*.jl after `using TSCOPF`.
================================================================================
=#

using DataStructures: OrderedDict

# Fixture roots and helpers. Guarded because test_env.jl includes the same file,
# and many suites load both.
@isdefined(FIXTURE_ROOT) || include(joinpath(@__DIR__, "test_paths.jl"))

"""Always create `RESULTS/Results - <timestamp>/…`; never reuse the flat tree."""
const TEST_OVERWRITE_RESULTS = false

# Reference AC-OPF benchmark: the frozen 9bus fixture (load_factor=1.5, flat
# linear cost c1=50, lossless/uncongested network). It used to point at a
# separate `9bus_reference_cost` case, a byte-identical clone kept as insurance
# against edits to the working 9bus data; `test/INPUT_FILES/` now provides that
# guarantee structurally, so the clone was dropped and the pinned objective is
# unchanged (same data, same solve).
const REFERENCE_LINEAR_ACOPF_CASE = "9bus"
const REFERENCE_LINEAR_ACOPF_OBJ_EUR = 23_625.0
const REFERENCE_LINEAR_ACOPF_OBJ_RTOL = 1e-4

"""Assert that `run_case!` used a timestamped run directory."""
function assert_timestamped_results!(path_names::OrderedDict{Symbol, String})
    @test occursin("Results -", path_names[:pf_results_date])
    @test isdir(path_names[:pf_results_date])
    return nothing
end

"""Steady-state OPF uses the full quadratic fuel cost (MATPOWER convention)."""
function assert_quadratic_opf_objective!()
    @test build_opf_input_param(DispatchConfig())[:obj_function][:type] == "quadratic"
    return nothing
end

"""Plain dispatch run (no transient layer)."""
function dispatch_run_config(;
    type_model::String="ACOPF",
    use_matrix::Bool=true,
    case::String="9bus",
    kwargs...
)
    return RunConfig(;
        trans_stab=false,
        case=case,
        dispatch=DispatchConfig(type_model=type_model, use_matrix=use_matrix),
        kwargs...)
end

"""UC dispatch defaults (MILP + restricted-pricing LP duals)."""
function uc_run_config(; kwargs...)
    base = (
        trans_stab = false,
        solver_name = "Gurobi",
        dispatch = DispatchConfig(type_model = "UC", cost_type = "linear", bound_P_g = true),
    )
    return RunConfig(; base..., kwargs...)
end

"""TSC run with 9-bus smoke-test defaults (δ_tol, contingency row 2)."""
function tsc_run_config(;
    type_model::String="ACOPF",
    use_matrix::Bool=true,
    δ_tol_deg::Float64=100.0,
    contingency_id::Int=2,
    simulation=TsSimulationConfig(δ_tol_deg=δ_tol_deg),
    builder::TsBuilderConfig=TsBuilderConfig(),
    dyn_model::DynModelConfig=DynModelConfig(
        fault=FaultConfig(contingency_id=contingency_id)),
    case::String="9bus",
    kwargs...
)
    return RunConfig(;
        trans_stab=true,
        case=case,
        dispatch=DispatchConfig(type_model=type_model, use_matrix=use_matrix),
        transient=TransientConfig(simulation=simulation, builder=builder, dyn_model=dyn_model),
        kwargs...)
end

# NOTE: a `verify_coi_active_inertia!(dmd, DGEN_DYN)` helper used to live here. It
# was never called, and it could not work: `run_case!` releases the JuMP backend
# before returning, so `dmd[:vars]` holds dead `VariableRef`s. The same identity
# (δ_COI = Σ H_i δ_i / Σ H_i over active generators only) is verified against a
# directly-built model in runtests_fast_unit.jl, "COI inertia denominator".

"""Override only `transient.dyn_model` on a TSC `RunConfig`."""
function reconfigure_dyn(cfg::RunConfig; dyn_model::DynModelConfig)
    cfg.transient === nothing &&
        throw(ArgumentError("reconfigure_dyn requires trans_stab=true with transient set."))
    return reconfigure(cfg;
        transient=reconfigure_transient(cfg.transient; dyn_model=dyn_model))
end
