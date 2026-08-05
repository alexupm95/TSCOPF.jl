#=
================================================================================
 scripts/generate_pm_dc_reference.jl
   PowerModels.jl DC-OPF reference generator (NOT the in-house TSC-OPF code)
================================================================================
 Purpose
 -------
 Solve the 9-bus DC-OPF entirely through the PowerModels.jl benchmark package
 and export an independent reference solution. This is the DC counterpart of
 generate_pm_ac_reference.jl (which solves the AC polar OPF). It is the
 clean external reference you can compare the in-house DC-OPF (`_dcopf/`) and
 TSC-DCOPF results against, the same way the AC script backstops ACOPF.

 Like its AC sibling it deliberately does NOT include setup_includes.jl /
 engine.jl / any _dcopf code: PowerModels alone parses the network and solves.

 What it does
 ------------
   1. Parse INPUT_FILES/PowerModels/case9.m  (a MATPOWER mirror of the repo's
      INPUT_FILES/9bus CSVs: linear cost c1=50, r=0, b=0).
   2. Build + solve the linearised DC-OPF (DCPPowerModel) with HiGHS — an LP,
      matching the repo convention that DCOPF/TSC-DCOPF use Gurobi or HiGHS.
   3. Save the objective and bus / generator / branch reports to
      RESULTS/_benchmarks_PM_DC/.
   4. Assert the run solved and the objective matches the analytical value.

 Run from the project root:

     julia --project=. scripts/generate_pm_dc_reference.jl

 Requires PowerModels and HiGHS in the active Julia environment.

 Numerical notes
 ---------------
   * The DC formulation fixes every voltage magnitude at 1.0 p.u. and neglects
     reactive power and active losses (the network here already has r = 0, so
     even the AC solve is lossless). The DC active power flow on branch (k,m) is
     the textbook p_km = (θ_k − θ_m) / x_km, and p_mk = −p_km exactly. The bus,
     generator and branch reports below therefore carry no Q columns, vm ≡ 1.0,
     and p_loss ≡ 0 by construction (kept only for layout parity with the AC
     report).
   * Reference / slack bus is bus 1 (type 3); its angle is fixed (θ_ref = 0),
     which fixes the angle gauge of the DC-OPF LP.
   * DCPPowerModel is a linear program: HiGHS returns OPTIMAL (contrast the AC
     file, where the nonconvex ACPPowerModel only reaches LOCALLY_SOLVED).
   * Lossless network + flat marginal cost (50 EUR/MW for every unit) makes the
     optimal objective invariant to the dispatch split: it equals
     50 * sum(Pd) = 50 * 472.5 = 23625 EUR — the same analytical optimum the AC
     benchmark asserts, which is exactly why DC and AC objectives must agree
     here.
================================================================================
=#

using Test
using PowerModels
using HiGHS
using JuMP                         # for the MOI termination-status enums (OPTIMAL, …)
using DataFrames, CSV, Printf, DataStructures

# Silence PowerModels/InfrastructureModels (Memento) info+warn chatter.
PowerModels.silence()

# ------------------------------------------------------------------ paths
const PROJECT_ROOT = dirname(@__DIR__)
const CASE_FILE    = joinpath(PROJECT_ROOT, "INPUT_FILES", "PowerModels", "case9.m")
const RESULTS_DIR  = joinpath(PROJECT_ROOT, "RESULTS", "_benchmarks_PM_DC")

# ------------------------------------------------------------------ expectations
# DC-OPF is an LP, so HiGHS returns a genuine OPTIMAL (no local-solution caveat).
const SOLVED_STATUSES = (OPTIMAL, LOCALLY_SOLVED)
# Analytical objective: lossless network + flat 50 EUR/MW => 50 * total demand.
const EXPECTED_OBJ_EUR  = 23_625.0
const EXPECTED_OBJ_RTOL = 1e-4

# =============================================================================
#  Solve
# =============================================================================
"""
    solve_pm_dc_benchmark() -> (network_data, result)

Parse `case9.m` and solve the linearised DC-OPF with HiGHS. `branch_flows = true`
makes PowerModels populate per-branch pf/pt in the solution dict so the branch
report below has flows to write (the DC model carries active power only).
"""
function solve_pm_dc_benchmark()
    network_data = PowerModels.parse_file(CASE_FILE)

    lp_solver = JuMP.optimizer_with_attributes(
        HiGHS.Optimizer,
        "output_flag" => false,      # silence the HiGHS log (LP solver banner + iters)
    )

    t_solve = @elapsed begin
        result = PowerModels.solve_opf(
            network_data, PowerModels.DCPPowerModel, lp_solver;
            setting = Dict("output" => Dict("branch_flows" => true)),
        )
    end
    @printf("PowerModels DC-OPF solved in %.3f s — status: %s\n",
            t_solve, result["termination_status"])

    return network_data, result
end

# =============================================================================
#  Reports
# =============================================================================
"""
    save_reports(network_data, result, out_dir) -> NamedTuple

Build bus / generator / branch report DataFrames (active powers in MW, angles in
degrees) and write them as ';'-delimited CSVs plus a plain-text objective summary
into `out_dir`. Returns the three DataFrames and the objective for the test
asserts.

The DC solution dict carries no reactive power and no voltage magnitude, so this
defensively `get`s `vm` (defaulting to the DC assumption of 1.0 p.u.) and `pt`
(defaulting to the exact DC identity −pf). No Q columns are emitted.
"""
function save_reports(network_data::Dict, result::Dict, out_dir::String)
    mkpath(out_dir)

    base   = network_data["baseMVA"]
    sol    = result["solution"]
    objval = result["objective"]

    # --- bus ordering: by physical bus number (bus_i), ascending ----------
    bus_keys = sort(collect(keys(network_data["bus"])),
                    by = k -> network_data["bus"][k]["bus_i"])
    bus_num  = [network_data["bus"][k]["bus_i"]  for k in bus_keys]
    bus_type = [network_data["bus"][k]["bus_type"] for k in bus_keys]

    # Voltage angle (deg) from the solution; magnitude is the DC assumption 1.0
    # p.u. (DCPPowerModel does not expose vm, so default it rather than index it).
    vm     = [get(sol["bus"][k], "vm", 1.0)     for k in bus_keys]
    va_deg = [rad2deg(sol["bus"][k]["va"])      for k in bus_keys]

    # Per-bus generation and demand aggregation. PowerModels stores generators
    # and loads as their own components keyed by id, each tagged with its bus;
    # accumulating into bus-keyed dicts is the clean way to map them back. DC
    # carries active power only — no qg / qd aggregation.
    pg_at = Dict(b => 0.0 for b in bus_num)
    pd_at = Dict(b => 0.0 for b in bus_num)

    for (gid, g) in network_data["gen"]
        b = g["gen_bus"]
        pg_at[b] += sol["gen"][gid]["pg"] * base
    end
    for (_, l) in network_data["load"]
        b = l["load_bus"]
        pd_at[b] += l["pd"] * base
    end

    RBUS = DataFrame(
        bus  = bus_num,
        type = bus_type,
        v_pu = round.(vm,     digits = 4),
        θ_deg = round.(va_deg, digits = 3),
        p_g  = round.([pg_at[b] for b in bus_num], digits = 3),
        p_d  = round.([pd_at[b] for b in bus_num], digits = 3),
    )

    # --- generators -------------------------------------------------------
    gen_keys = sort(collect(keys(network_data["gen"])), by = k -> parse(Int, k))
    RGEN = DataFrame(
        id_gen = [parse(Int, k)                     for k in gen_keys],
        id_bus = [network_data["gen"][k]["gen_bus"] for k in gen_keys],
        p_g    = round.([sol["gen"][k]["pg"] * base for k in gen_keys], digits = 3),
        p_max  = round.([network_data["gen"][k]["pmax"] * base for k in gen_keys], digits = 3),
        p_min  = round.([network_data["gen"][k]["pmin"] * base for k in gen_keys], digits = 3),
    )

    # --- branches ---------------------------------------------------------
    # DC carries the from-end active flow; the to-end is the exact identity
    # pt = -pf (no losses), defaulted via `get` in case the model omits it.
    br_keys = sort(collect(keys(network_data["branch"])), by = k -> parse(Int, k))
    pf = [sol["branch"][k]["pf"]                 * base for k in br_keys]
    pt = [get(sol["branch"][k], "pt", -sol["branch"][k]["pf"]) * base for k in br_keys]
    RCIR = DataFrame(
        circ     = [parse(Int, k)                      for k in br_keys],
        from_bus = [network_data["branch"][k]["f_bus"] for k in br_keys],
        to_bus   = [network_data["branch"][k]["t_bus"] for k in br_keys],
        p_ik     = round.(pf, digits = 3),
        p_ki     = round.(pt, digits = 3),
        p_loss   = round.(pf .+ pt, digits = 3),   # = 0 here (DC, lossless), kept for parity
    )

    # --- write outputs ----------------------------------------------------
    CSV.write(joinpath(out_dir, "buses_report.csv"),      RBUS; delim = ';')
    CSV.write(joinpath(out_dir, "generators_report.csv"), RGEN; delim = ';')
    CSV.write(joinpath(out_dir, "branches_report.csv"),   RCIR; delim = ';')

    open(joinpath(out_dir, "objective_report.txt"), "w") do io
        @printf(io, "PowerModels.jl DC-OPF benchmark (DCPPowerModel + HiGHS)\n")
        @printf(io, "Case file : %s\n", CASE_FILE)
        @printf(io, "Status    : %s\n", result["termination_status"])
        @printf(io, "============================================\n")
        @printf(io, "Objective : %.2f EUR\n", objval)
        @printf(io, "Total Pg  : %.3f MW\n",  sum(RGEN.p_g))
        @printf(io, "Total Pd  : %.3f MW\n",  sum(RBUS.p_d))
        @printf(io, "Total Ploss (MW) : %.6f\n", sum(RCIR.p_loss))
        @printf(io, "============================================\n")
    end

    println("DC benchmark reports written to: ", out_dir)
    return (; RBUS, RGEN, RCIR, objective = objval)
end

# =============================================================================
#  Test set
# =============================================================================
@testset "PowerModels DC-OPF benchmark (9-bus)" begin

    @test isfile(CASE_FILE)

    network_data, result = solve_pm_dc_benchmark()

    @testset "solve" begin
        @test result["termination_status"] in SOLVED_STATUSES
        @test isfinite(result["objective"])
        # Analytical optimum: lossless + flat 50 EUR/MW => 50 * 472.5 MW.
        # Must match the AC benchmark objective (same lossless, flat-cost network).
        @test isapprox(result["objective"], EXPECTED_OBJ_EUR; rtol = EXPECTED_OBJ_RTOL)
    end

    @testset "reports" begin
        rep = save_reports(network_data, result, RESULTS_DIR)
        @test isfile(joinpath(RESULTS_DIR, "buses_report.csv"))
        @test isfile(joinpath(RESULTS_DIR, "generators_report.csv"))
        @test isfile(joinpath(RESULTS_DIR, "branches_report.csv"))
        @test isfile(joinpath(RESULTS_DIR, "objective_report.txt"))
        # Active generation must cover demand exactly on a lossless network.
        @test isapprox(sum(rep.RGEN.p_g), sum(rep.RBUS.p_d); atol = 1e-3)
        @test isapprox(sum(rep.RCIR.p_loss), 0.0; atol = 1e-3)
        # DC fixes every voltage magnitude at 1.0 p.u.
        @test all(isapprox.(rep.RBUS.v_pu, 1.0; atol = 1e-9))
    end
end
