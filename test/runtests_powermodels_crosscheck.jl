#=
================================================================================
 test/runtests_powermodels_crosscheck.jl
   Cross-validation of the in-house dispatch models against PowerModels.jl
================================================================================
 Merges three former files that shared ~120 lines of identical harness:
   runtests_acopf_vs_powermodels.jl        → "AC-OPF (branch form)"
   runtests_acopf_matrix_vs_powermodels.jl → "AC-OPF (matrix / Ybus form)"
   runtests_dcopf_vs_powermodels.jl        → "DCOPF"

 Method (identical for all three): parse a MATPOWER `.m` with the in-house
 parser, stage the CSVs into a scratch `<tmp>/INPUT_FILES/_cmp_*/` case, solve with
 `run_case!`, solve the SAME `.m` with PowerModels, then assert the primal
 results agree quantity by quantity.

 Why they should agree
 ---------------------
   * The in-house Ybus follows the MATPOWER / PowerModels branch convention
     exactly — series y = 1/(r+jx), line charging b/2 at each end, transformer
     tap applied "from→to" (divide by tap), bus shunts (g+jb)/baseMVA
     (`_common/functions_4_admittance_matrices.jl`).
   * DC (`susceptance_model = POWERMODELS`): P_ik = (x/(r²+x²))·(θ_i−θ_k), taps
     and shunts ignored — the same as `DCPPowerModel`, whose
     p_fr = −b·(va_fr−va_to) with b = imag(1/(r+jx)) = −x/(r²+x²).
   * Both fix the type-3 slack-bus angle to 0, so angles are directly comparable.
   * `res.obj_MVA` is currency-on-MW, comparable to the PowerModels objective.

 Driver case: case39.m (r≠0, transformers, line charging, strictly-convex
 quadratic costs ⇒ a unique DC optimum and a well-behaved AC local optimum, so a
 per-quantity match is meaningful). case9.m is degenerate (lossless + flat cost).

 Run standalone:
     julia --project=. test/runtests_powermodels_crosscheck.jl

 Enable inside the aggregate suite (needs PowerModels in the test env):
     TSCOPF_RUN_PM_CROSSCHECK=true julia --project=. -e 'using Pkg; Pkg.test()'
================================================================================
=#
using Test
using Printf
using PowerModels

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))

PowerModels.silence()

const CASE39_M   = fixture_matpower("case39.m")
const PM_SOLVED  = (OPTIMAL, LOCALLY_SOLVED)

# Bound/inequality block shared by all three configurations. Isolated here so a
# change to the comparison basis cannot silently drift between the AC and DC runs.
const _CMP_BOUNDS = (
    bound_V = true, bound_θ = true, bound_P_g = true, bound_Q_g = true,
    bound_P_ik = false, bound_Q_ik = false, bound_P_ki = false, bound_Q_ki = false,
    ineq_sg_upper = false, ineq_sbranch_upper = true, ineq_ang_diff_branch = true,
)

# =============================================================================
#  In-house solve on a staged copy of the MATPOWER case
# =============================================================================
"""
    run_inhouse(case_file, tag, dispatch, solver_name) -> (res, baseMVA)

Parse `case_file` with the in-house parser, stage the three CSVs into a scratch
case under a temporary directory, and solve it with `run_case!` at
`load_factor = 1.0` (raw `.m` loads, to match PowerModels).

The scratch case lives at `<tmp>/INPUT_FILES/<tag>_<name>/` so that
`load_system(cfg, scratch_root)` resolves it exactly like a real case tree
(`src/engine.jl:340`). It used to be staged inside the repository's
`INPUT_FILES/`, where an exception between `mkpath` and the `try` block left a
half-populated directory behind in tracked data. `mktempdir(f)` removes the tree
on normal and exceptional exit, and returns `f`'s value.
"""
function run_inhouse(case_file::String, tag::String,
                     dispatch::DispatchConfig, solver_name::String)
    imp = Import_Matpower_Case(case_file; path_main = PROJECT_ROOT)
    case = tag * "_" * splitext(basename(case_file))[1]

    return mktempdir() do scratch_root
        case_dir = joinpath(scratch_root, "INPUT_FILES", case)
        mkpath(case_dir)
        for f in ("bus_data.csv", "generators_data.csv", "line_data.csv")
            cp(joinpath(imp.path_names[:pf_inputs], f), joinpath(case_dir, f); force = true)
        end

        cfg = RunConfig(
            trans_stab          = false,
            case                = case,
            base_MVA            = 100.0,
            load_factor         = 1.0,
            solver_name         = solver_name,
            silent_solver       = true,
            time_limit_sec      = 600.0,
            overwrite_results   = false,
            save_duals          = true,
            save_optim_matrices = true,
            save_matrices       = true,
            dispatch            = dispatch,
        )
        sys = load_system(cfg, scratch_root)
        # `Copy_Input_CSVs_To_Results!` runs inside `run_case!`, before the
        # tempdir is torn down, so the archived Inputs/ folder is unaffected.
        res = run_case!(cfg, sys, scratch_root, RESULTS_ROOT)
        return res, imp.baseMVA
    end
end

acopf_dispatch(; use_matrix::Bool) = DispatchConfig(;
    type_model = "ACOPF", use_matrix = use_matrix, cost_type = "quadratic",
    _CMP_BOUNDS...)

dcopf_dispatch() = DispatchConfig(;
    type_model = "DCOPF", use_matrix = false, cost_type = "quadratic",
    susceptance_model = POWERMODELS, _CMP_BOUNDS...)

# =============================================================================
#  PowerModels reference solves on the same MATPOWER case
# =============================================================================
"""Solve `case_file` with PowerModels `ACPPowerModel` + Ipopt (branch flows on)."""
function run_powermodels_ac(case_file::String)
    nd = PowerModels.parse_file(case_file)
    nlp = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-8, "print_level" => 0)
    result = PowerModels.solve_opf(nd, PowerModels.ACPPowerModel, nlp;
        setting = Dict("output" => Dict("branch_flows" => true)))
    return nd, result
end

"""Solve `case_file` with PowerModels `DCPPowerModel` + HiGHS (branch flows on)."""
function run_powermodels_dc(case_file::String)
    nd = PowerModels.parse_file(case_file)
    lp = JuMP.optimizer_with_attributes(HiGHS.Optimizer, "output_flag" => false)
    result = PowerModels.solve_opf(nd, PowerModels.DCPPowerModel, lp;
        setting = Dict("output" => Dict("branch_flows" => true)))
    return nd, result
end

"""Objective agreement + solved-status check, shared by the AC and DC drivers."""
function _assert_objective!(label, res, pm, rtol)
    @test res.status in PM_SOLVED
    @test pm["termination_status"] in PM_SOLVED

    # `res.obj_MVA` is already evaluated — run_case! releases the JuMP backend.
    obj_inhouse = res.obj_MVA
    obj_pm      = pm["objective"]
    @printf("\n[%s] objective: in-house = %.4f | PowerModels = %.4f | Δ = %.3e\n",
            label, obj_inhouse, obj_pm, abs(obj_inhouse - obj_pm))
    @test isapprox(obj_inhouse, obj_pm; rtol = rtol)
    return obj_inhouse
end

# =============================================================================
#  Comparison drivers
# =============================================================================
"""
    compare_ac(case_file; use_matrix, ...)

Solve `case_file` as an AC-OPF both ways and assert full primal agreement:
objective, per-generator P_g/Q_g, per-bus V/θ, per-branch p_ik/q_ik/p_ki/q_ki.
`use_matrix` selects the branch form (`false`) or the Ybus nodal form (`true`) —
the only difference between the two AC testsets.
"""
function compare_ac(case_file::String; use_matrix::Bool,
        rtol::Float64 = 1e-4, atol_mw::Float64 = 5e-2,
        atol_v::Float64 = 1e-4, atol_deg::Float64 = 1e-2)

    tag   = use_matrix ? "_cmp_acm" : "_cmp_ac"
    label = @sprintf("%s %s", basename(case_file), use_matrix ? "matrix" : "branch")

    res, base = run_inhouse(case_file, tag, acopf_dispatch(use_matrix = use_matrix), "Ipopt")
    nd, pm    = run_powermodels_ac(case_file)
    sol       = pm["solution"]

    _assert_objective!(label, res, pm, rtol)

    # --- generators: align by id (file order, shared by both) ----------------
    dpg = 0.0; dqg = 0.0
    for r in eachrow(res.RGEN)
        g = sol["gen"][string(r.id)]
        dpg = max(dpg, abs(r.p_g - g["pg"] * base))
        dqg = max(dqg, abs(r.q_g - g["qg"] * base))
        @test isapprox(r.p_g, g["pg"] * base; atol = atol_mw)
        @test isapprox(r.q_g, g["qg"] * base; atol = atol_mw)
    end

    # --- bus voltage magnitude + angle: align by original bus number ---------
    dvm = 0.0; dva = 0.0
    for r in eachrow(res.RBUS)
        b = sol["bus"][string(r.bus)]
        dvm = max(dvm, abs(r.v - b["vm"]))
        dva = max(dva, abs(r.θ - rad2deg(b["va"])))
        @test isapprox(r.v, b["vm"];          atol = atol_v)
        @test isapprox(r.θ, rad2deg(b["va"]); atol = atol_deg)
    end

    # --- branch flows: align by id (file order); check both endpoints --------
    dpf = 0.0; dqf = 0.0
    for r in eachrow(res.RCIR)
        br = nd["branch"][string(r.id)]
        @test br["f_bus"] == r.from_bus && br["t_bus"] == r.to_bus   # alignment sanity
        b = sol["branch"][string(r.id)]
        dpf = max(dpf, abs(r.p_ik - b["pf"] * base), abs(r.p_ki - b["pt"] * base))
        dqf = max(dqf, abs(r.q_ik - b["qf"] * base), abs(r.q_ki - b["qt"] * base))
        @test isapprox(r.p_ik, b["pf"] * base; atol = atol_mw)
        @test isapprox(r.q_ik, b["qf"] * base; atol = atol_mw)
        @test isapprox(r.p_ki, b["pt"] * base; atol = atol_mw)
        @test isapprox(r.q_ki, b["qt"] * base; atol = atol_mw)
    end

    @printf("[%s] max|Δ|: P_g = %.3e MW | Q_g = %.3e MVAr | V = %.3e pu | θ = %.3e deg | Pflow = %.3e | Qflow = %.3e MW\n",
            label, dpg, dqg, dvm, dva, dpf, dqf)
    return res
end

"""
    compare_dc(case_file; full)

Solve `case_file` as a DCOPF both ways. With `full = true` (unique optimum) every
quantity is asserted; with `full = false` (degenerate case such as case9) only the
objective and the lossless gen = load invariant are asserted.
"""
function compare_dc(case_file::String;
        rtol::Float64 = 1e-5, atol_mw::Float64 = 1e-3, atol_deg::Float64 = 1e-3,
        full::Bool = true)

    label = basename(case_file) * " dc"

    res, base = run_inhouse(case_file, "_cmp_dc", dcopf_dispatch(), "HiGHS")
    nd, pm    = run_powermodels_dc(case_file)
    sol       = pm["solution"]

    _assert_objective!(label, res, pm, rtol)

    # gen = load (lossless DC)
    @test isapprox(sum(res.RGEN.p_g), sum(res.RBUS.p_d); atol = 1e-3)

    full || return res

    dpg = 0.0
    for r in eachrow(res.RGEN)
        pm_pg = sol["gen"][string(r.id)]["pg"] * base
        dpg = max(dpg, abs(r.p_g - pm_pg))
        @test isapprox(r.p_g, pm_pg; atol = atol_mw)
    end

    dva = 0.0
    for r in eachrow(res.RBUS)
        pm_va = rad2deg(sol["bus"][string(r.bus)]["va"])
        dva = max(dva, abs(r.θ - pm_va))
        @test isapprox(r.θ, pm_va; atol = atol_deg)
    end

    dpf = 0.0
    for r in eachrow(res.RCIR)
        br = nd["branch"][string(r.id)]
        @test br["f_bus"] == r.from_bus && br["t_bus"] == r.to_bus   # alignment sanity
        pm_pf = sol["branch"][string(r.id)]["pf"] * base
        pm_pt = sol["branch"][string(r.id)]["pt"] * base
        dpf = max(dpf, abs(r.p_ik - pm_pf), abs(r.p_ki - pm_pt))
        @test isapprox(r.p_ik, pm_pf; atol = atol_mw)
        @test isapprox(r.p_ki, pm_pt; atol = atol_mw)
    end

    @printf("[%s] max|Δ|: P_g = %.3e MW | θ = %.3e deg | flow = %.3e MW\n",
            label, dpg, dva, dpf)
    return res
end

# =============================================================================
#  Test sets
# =============================================================================
@testset "PowerModels cross-check (case39.m)" begin
    @test isfile(CASE39_M)

    @testset "AC-OPF branch form vs ACPPowerModel" begin
        compare_ac(CASE39_M; use_matrix = false)
    end

    @testset "AC-OPF matrix (Ybus) form vs ACPPowerModel" begin
        compare_ac(CASE39_M; use_matrix = true)
    end

    @testset "DCOPF vs DCPPowerModel — full primal agreement" begin
        compare_dc(CASE39_M; full = true)
    end
end
