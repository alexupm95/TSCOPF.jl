#= UC MILP + restricted-pricing duals (3-generator toy).
   Results go to RESULTS/Results - <timestamp>/… (same layout as run_case!).
   Standalone: `julia --project=. test/runtests_uc.jl`
   Aggregated: included from `test/runtests_all.jl`. =#

const _UC_TESTS_STANDALONE = abspath(PROGRAM_FILE) == abspath(@__FILE__)

if _UC_TESTS_STANDALONE
    using Test
    using TSCOPF
    using JuMP
    using CSV, DataFrames
    import MathOptInterface as MOI
    const PROJECT_ROOT = dirname(@__DIR__)
    include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))
end

const PATH_RESULTS = joinpath(dirname(@__DIR__), "RESULTS")
const SOLVED_STATUSES = (MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ITERATION_LIMIT)

"""Timestamped RESULTS tree for UC toy runs (mirrors build_results_paths in engine.jl)."""
function _uc_test_path_names(; save_duals::Bool = true)
    cfg = uc_run_config(
        save_duals = save_duals,
        silent_solver = true,
        overwrite_results = TEST_OVERWRITE_RESULTS,
    )
    return build_results_paths(
        dirname(@__DIR__), PATH_RESULTS, cfg;
        overwrite_results = TEST_OVERWRITE_RESULTS,
    )
end

function _run_uc_smoke()
    cfg = uc_run_config(
        silent_solver = true,
        save_optim_matrices = false,
        save_duals = true,
        overwrite_results = TEST_OVERWRITE_RESULTS,
    )
    sys = uc_toy_system_data()
    return run_case!(cfg, sys, dirname(@__DIR__), PATH_RESULTS)
end

"""Load Gurobi.jl and activate TSCOPFGurobiExt when the test env has a license."""
function _uc_tests_enabled()::Bool
    pkgid = Base.identify_package("Gurobi")
    pkgid === nothing && return false
    try
        Base.require(pkgid)
    catch
        return false
    end
    return gurobi_available()
end

if _uc_tests_enabled()
@testset "UC" begin

    @testset "run_case! smoke (toy SystemData)" begin
        result = _run_uc_smoke()
        @test result.status == MOI.OPTIMAL
        assert_timestamped_results!(result.path_names)
        @test !isdir(result.path_names[:pf_bus_matrices])
        d = result.path_names[:pf_dispatch]
        @test isfile(joinpath(d, "uc_primal_solution.txt"))
        @test isfile(joinpath(d, "duals.txt"))
        @test isfile(joinpath(d, "UC_Dispatch_Duals.xlsx"))
        @test isfile(joinpath(result.path_names[:pf_dispatch_CSV], "uc_commitment.csv"))
        @test isfile(joinpath(result.path_names[:pf_dispatch_CSV_duals], "dual_uc_commitment.csv"))
    end

    @testset "toy MILP" begin
        toy = build_uc_toy_system()
        path_names = _uc_test_path_names(save_duals = false)
        assert_timestamped_results!(path_names)
        @test !isdir(path_names[:pf_bus_matrices])
        mkpath(path_names[:pf_dispatch])

        model = Setup_Optim_Model("Gurobi"; silent = true)
        opf_input_param = build_opf_input_param(DispatchConfig(
            type_model = "UC", cost_type = "linear", bound_P_g = true))
        model, obj, obj_MVA, opf_dict = Make_UC_Model!(
            model, path_names, toy.DBUS, toy.DGEN, toy.bus_gen_circ_dict_ON,
            toy.base_MVA, toy.nBUS, toy.nGEN, opf_input_param)

        JuMP.optimize!(model)
        @test termination_status(model) == MOI.OPTIMAL

        u = opf_dict[:vars][:u_commit]
        P_g = opf_dict[:vars][:P_g]
        @test JuMP.value(u[1]) > 0.5
        @test JuMP.value(u[2]) < 0.5   # expensive + Pmin forces min load if on
        @test JuMP.value(u[3]) > 0.5
        @test sum(JuMP.value(P_g[g]) for g in keys(P_g)) ≈ 1.0 atol = 1e-6

        u_star = uc_commitment_values(u)
        lp_model, lp_dict = Solve_UC_Restricted_Pricing!(
            toy.DGEN, toy.bus_gen_circ_dict_ON, toy.base_MVA, u_star; silent_solver = true)
        @test termination_status(lp_model) == MOI.OPTIMAL
        @test JuMP.dual(lp_dict[:eq_const][:eq_const_p_balance][1]) > 0.0
    end

    @testset "save results, model details, and duals" begin
        toy = build_uc_toy_system()
        path_names = _uc_test_path_names(save_duals = true)
        assert_timestamped_results!(path_names)
        @test !isdir(path_names[:pf_bus_matrices])
        mkpath(path_names[:pf_dispatch])

        model = Setup_Optim_Model("Gurobi"; silent = true)
        opf_input_param = build_opf_input_param(DispatchConfig(
            type_model = "UC", cost_type = "linear", bound_P_g = true))
        model, obj, obj_MVA, opf_dict = Make_UC_Model!(
            model, path_names, toy.DBUS, toy.DGEN, toy.bus_gen_circ_dict_ON,
            toy.base_MVA, toy.nBUS, toy.nGEN, opf_input_param)
        JuMP.optimize!(model)
        @test termination_status(model) == MOI.OPTIMAL

        bus_mapping = OrderedDict(toy.DBUS.bus[i] => i for i in 1:toy.nBUS)
        reverse_bus_mapping = OrderedDict(i => toy.DBUS.bus[i] for i in 1:toy.nBUS)

        Save_Solution_UC_Model(
            path_names, model, obj, opf_dict, toy.bus_gen_circ_dict_ON,
            toy.DBUS, toy.DGEN, toy.DCIR, toy.base_MVA, toy.nBUS, toy.nGEN, toy.nCIR,
            bus_mapping, reverse_bus_mapping; _save_duals = true)

        d = path_names[:pf_dispatch]
        @test startswith(path_names[:pf_results_date], PATH_RESULTS)
        @test occursin("Results -", path_names[:pf_results_date])
        @test isfile(joinpath(d, "model_summary.txt"))
        @test isfile(joinpath(d, "model_details.txt"))
        @test isfile(joinpath(d, "uc_primal_solution.txt"))
        @test isfile(joinpath(d, "restricted_lp_model_summary.txt"))
        @test isfile(joinpath(d, "restricted_lp_model_details.txt"))
        @test isfile(joinpath(d, "duals.txt"))
        @test isfile(joinpath(d, "UC_Dispatch_Duals.xlsx"))
        @test isfile(joinpath(d, "OPF_Dispatch_Results.xlsx"))
        @test isfile(joinpath(path_names[:pf_dispatch_CSV], "uc_commitment.csv"))
        @test isfile(joinpath(path_names[:pf_dispatch_CSV_duals], "dual_P_balance.csv"))
        @test isfile(joinpath(path_names[:pf_dispatch_CSV_duals], "dual_uc_commitment.csv"))

        dual_txt = read(joinpath(d, "duals.txt"), String)
        @test occursin("UC restricted pricing", dual_txt)
        @test occursin("dual_eq_const_P_balance", dual_txt)

        gen_csv = CSV.read(joinpath(path_names[:pf_dispatch_CSV], "generators_report.csv"), DataFrame; delim = ';')
        @test hasproperty(gen_csv, :u_commit)
    end

    @testset "dispatch config validation" begin
        @test validate_dispatch_config!(DispatchConfig(type_model = "UC", cost_type = "linear")) === nothing
        @test_throws ArgumentError validate_dispatch_config!(DispatchConfig(type_model = "UC", cost_type = "quadratic"))
        @test_throws ArgumentError Check_Coherence_Input_Data(false, "UC", "Ipopt")
    end

end
end # gurobi_available()
