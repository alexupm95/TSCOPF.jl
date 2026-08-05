#=
 Bound encoding tests: CONSTRAINT vs VARIABLE box limits.
 Run: julia --project=. test/runtests_bound_encoding.jl
=#

using Test
using CSV, DataFrames
using HiGHS

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(@__DIR__, "test_common.jl"))

function _mini_pg_model(encoding::BoundEncoding)
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    opf_dict = TSCOPF.init_opf_dict!()
    result = TSCOPF.var_gen_power_active!(
        model, [1, 2], "P_g";
        bounded = true,
        min_lim = [0.0, 0.0],
        max_lim = [1.0, 2.0],
        encoding = encoding,
        meta = opf_dict[:meta],
        export_key_lower = :ineq_const_pg_lower,
        export_key_upper = :ineq_const_pg_upper,
    )
    P_g = TSCOPF.store_scalar_var_bounds!(
        opf_dict, :P_g, result,
        :ineq_const_pg_lower, :ineq_const_pg_upper,
    )
    @constraint(model, P_g[1] + P_g[2] == 1.5)
    @objective(model, Min, P_g[1] + 2 * P_g[2])
    optimize!(model)
    return model, opf_dict
end

function _pg_bound_duals(opf_dict)
    if haskey(opf_dict[:ineq_const], :ineq_const_pg_lower)
        lo = [JuMP.dual(c) for (_, c) in opf_dict[:ineq_const][:ineq_const_pg_lower]]
        up = [JuMP.dual(c) for (_, c) in opf_dict[:ineq_const][:ineq_const_pg_upper]]
        return lo, up
    end
    _, lo = TSCOPF.extract_bound_manifest_duals(opf_dict, :ineq_const_pg_lower)
    _, up = TSCOPF.extract_bound_manifest_duals(opf_dict, :ineq_const_pg_upper)
    return lo, up
end

@testset "Bound encoding mini ED" begin
    model_c, opf_c = _mini_pg_model(TSCOPF.CONSTRAINT)
    model_v, opf_v = _mini_pg_model(TSCOPF.VARIABLE)
    @test termination_status(model_c) == MOI.OPTIMAL
    @test termination_status(model_v) == MOI.OPTIMAL

    P_c = [JuMP.value(opf_c[:vars][:P_g][i]) for i in 1:2]
    P_v = [JuMP.value(opf_v[:vars][:P_g][i]) for i in 1:2]
    @test P_c ≈ P_v atol = 1e-6

    lo_c, up_c = _pg_bound_duals(opf_c)
    lo_v, up_v = _pg_bound_duals(opf_v)
    @test lo_c ≈ lo_v atol = 1e-6
    @test up_c ≈ up_v atol = 1e-6
end

@testset "VARIABLE bounds in model_details export" begin
    model, opf_dict = _mini_pg_model(TSCOPF.VARIABLE)
    P_g = opf_dict[:vars][:P_g]
    tmp = mktempdir()
    path_names = OrderedDict{Symbol, String}(:pf_dispatch => tmp)
    TSCOPF.Export_OPF_Model(model, path_names, P_g[1] + 2 * P_g[2], opf_dict; model_label = "test")
    text = read(joinpath(tmp, "model_details.txt"), String)
    @test occursin("Active Power Generated - Lower Bound", text)
    @test occursin("Active Power Generated - Upper Bound", text)
    @test occursin("-P_g[1] <= -0.0", text)
    @test occursin("P_g[1] <= 1.0", text)
    @test occursin("P_g[2] <= 2.0", text)
end

@testset "build_indexed_scalar_vars NamedTuple shape" begin
    model = Model(HiGHS.Optimizer)
    set_silent(model)
    result = TSCOPF.build_indexed_scalar_vars!(model, [1, 2], "x"; bounded = false)
    @test haskey(result, :vars)
    @test haskey(result, :lower)
    @test haskey(result, :upper)
    @test result.lower === nothing
    @test result.upper === nothing
end
