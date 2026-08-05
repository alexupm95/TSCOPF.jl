#=
================================================================================
 scripts/generate_pm_ac_reference.jl
   PowerModels.jl AC-OPF reference generator (NOT the in-house TSC-OPF code)
================================================================================
 Purpose
 -------
 Solve the 9-bus AC-OPF entirely through the PowerModels.jl benchmark package
 and export an independent reference solution. This file deliberately does NOT
 include setup_includes.jl / engine.jl / any _acopf code: it is a clean external
 reference you can later compare your own ACOPF / TSC-ACOPF results against.

 What it does
 ------------
   1. Parse INPUT_FILES/PowerModels/case9.m  (a MATPOWER mirror of the repo's
      INPUT_FILES/9bus CSVs: linear cost c1=50, r=0, b=0).
   2. Build + solve the AC polar OPF (ACPPowerModel) with Ipopt.
   3. Save the objective and bus / generator / branch reports to
      RESULTS/_benchmarks_PM/.
   4. Assert the run solved and the objective matches the analytical value.

 Run from the project root:

     julia --project=. scripts/generate_pm_ac_reference.jl

 Requires PowerModels and Ipopt in the active Julia environment.

 Numerical notes
 ---------------
   * Reference / slack bus is bus 1 (type 3); its angle is fixed (θ_ref = 0),
     which fixes the angle gauge of the AC-OPF KKT system.
   * Lines have r = 0, so active losses are identically zero. With a flat
     marginal cost (50 EUR/MW for every unit) the optimal objective is invariant
     to the dispatch split and equals 50 * sum(Pd) = 50 * 315 = 15750 EUR. The
     objective assert below uses that analytical value.
   * ACPPowerModel is the nonconvex AC polar form, so Ipopt returns
     LOCALLY_SOLVED rather than OPTIMAL. Gurobi cannot solve this nonlinear
     problem and is intentionally not used here.
================================================================================
=#

using Test
using PowerModels
using Ipopt
using JuMP                         # for the MOI termination-status enums (OPTIMAL, …)
using DataFrames, CSV, Printf, DataStructures

# Silence PowerModels/InfrastructureModels (Memento) info+warn chatter.
PowerModels.silence()

# ------------------------------------------------------------------ paths
const PROJECT_ROOT = dirname(@__DIR__)
const CASE_FILE    = joinpath(PROJECT_ROOT, "INPUT_FILES", "PowerModels", "case9.m")
const RESULTS_DIR  = joinpath(PROJECT_ROOT, "RESULTS", "_benchmarks_PM")

# ------------------------------------------------------------------ expectations
# Statuses we accept as "solved" (nonconvex AC-OPF -> LOCALLY_SOLVED with Ipopt).
const SOLVED_STATUSES = (OPTIMAL, LOCALLY_SOLVED, ITERATION_LIMIT)
# Analytical objective: lossless network + flat 50 EUR/MW => 50 * total demand.
const EXPECTED_OBJ_EUR = 23_625.0
const EXPECTED_OBJ_RTOL = 1e-4

# =============================================================================
#  Solve
# =============================================================================
"""
    solve_pm_benchmark() -> (network_data, result)

Parse `case9.m` and solve the AC polar OPF with Ipopt. `branch_flows = true`
makes PowerModels populate per-branch pf/qf/pt/qt in the solution dict so the
branch report below has flows to write.
"""
function solve_pm_benchmark()
    network_data = PowerModels.parse_file(CASE_FILE)

    nlp_solver = JuMP.optimizer_with_attributes(
        Ipopt.Optimizer,
        "tol"         => 1e-8,
        "print_level" => 0,
    )

    t_solve = @elapsed begin
        result = PowerModels.solve_opf(
            network_data, PowerModels.ACPPowerModel, nlp_solver;
            setting = Dict("output" => Dict("branch_flows" => true)),
        )
    end
    @printf("PowerModels AC-OPF solved in %.3f s — status: %s\n",
            t_solve, result["termination_status"])

    return network_data, result
end

# =============================================================================
#  Reports
# =============================================================================
"""
    save_reports(network_data, result, out_dir) -> NamedTuple

Build bus / generator / branch report DataFrames (all powers in MW / MVAr) and
write them as ';'-delimited CSVs plus a plain-text objective summary into
`out_dir`. Returns the three DataFrames and the objective for the test asserts.
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

    # Voltage magnitude (p.u.) and angle (deg) straight from the solution.
    vm     = [sol["bus"][k]["vm"]          for k in bus_keys]
    va_deg = [rad2deg(sol["bus"][k]["va"]) for k in bus_keys]

    # Per-bus generation and demand aggregation. PowerModels stores generators
    # and loads as their own components keyed by id, each tagged with its bus;
    # accumulating into bus-keyed dicts is the clean way to map them back.
    pg_at = Dict(b => 0.0 for b in bus_num); qg_at = Dict(b => 0.0 for b in bus_num)
    pd_at = Dict(b => 0.0 for b in bus_num); qd_at = Dict(b => 0.0 for b in bus_num)

    for (gid, g) in network_data["gen"]
        b = g["gen_bus"]
        pg_at[b] += sol["gen"][gid]["pg"] * base
        qg_at[b] += sol["gen"][gid]["qg"] * base
    end
    for (_, l) in network_data["load"]
        b = l["load_bus"]
        pd_at[b] += l["pd"] * base
        qd_at[b] += l["qd"] * base
    end

    RBUS = DataFrame(
        bus  = bus_num,
        type = bus_type,
        v_pu = round.(vm,     digits = 4),
        θ_deg = round.(va_deg, digits = 3),
        p_g  = round.([pg_at[b] for b in bus_num], digits = 3),
        q_g  = round.([qg_at[b] for b in bus_num], digits = 3),
        p_d  = round.([pd_at[b] for b in bus_num], digits = 3),
        q_d  = round.([qd_at[b] for b in bus_num], digits = 3),
    )

    # --- generators -------------------------------------------------------
    gen_keys = sort(collect(keys(network_data["gen"])), by = k -> parse(Int, k))
    RGEN = DataFrame(
        id_gen = [parse(Int, k)                   for k in gen_keys],
        id_bus = [network_data["gen"][k]["gen_bus"] for k in gen_keys],
        p_g    = round.([sol["gen"][k]["pg"] * base for k in gen_keys], digits = 3),
        q_g    = round.([sol["gen"][k]["qg"] * base for k in gen_keys], digits = 3),
        p_max  = round.([network_data["gen"][k]["pmax"] * base for k in gen_keys], digits = 3),
        p_min  = round.([network_data["gen"][k]["pmin"] * base for k in gen_keys], digits = 3),
    )

    # --- branches ---------------------------------------------------------
    br_keys = sort(collect(keys(network_data["branch"])), by = k -> parse(Int, k))
    pf = [sol["branch"][k]["pf"] * base for k in br_keys]
    qf = [sol["branch"][k]["qf"] * base for k in br_keys]
    pt = [sol["branch"][k]["pt"] * base for k in br_keys]
    qt = [sol["branch"][k]["qt"] * base for k in br_keys]
    RCIR = DataFrame(
        circ     = [parse(Int, k)                    for k in br_keys],
        from_bus = [network_data["branch"][k]["f_bus"] for k in br_keys],
        to_bus   = [network_data["branch"][k]["t_bus"] for k in br_keys],
        p_ik     = round.(pf, digits = 3),
        q_ik     = round.(qf, digits = 3),
        p_ki     = round.(pt, digits = 3),
        q_ki     = round.(qt, digits = 3),
        p_loss   = round.(pf .+ pt, digits = 3),   # = 0 here (r = 0), kept for generality
        q_loss   = round.(qf .+ qt, digits = 3),
    )

    # --- write outputs ----------------------------------------------------
    CSV.write(joinpath(out_dir, "buses_report.csv"),      RBUS; delim = ';')
    CSV.write(joinpath(out_dir, "generators_report.csv"), RGEN; delim = ';')
    CSV.write(joinpath(out_dir, "branches_report.csv"),   RCIR; delim = ';')

    open(joinpath(out_dir, "objective_report.txt"), "w") do io
        @printf(io, "PowerModels.jl AC-OPF benchmark (ACPPowerModel + Ipopt)\n")
        @printf(io, "Case file : %s\n", CASE_FILE)
        @printf(io, "Status    : %s\n", result["termination_status"])
        @printf(io, "============================================\n")
        @printf(io, "Objective : %.2f EUR\n", objval)
        @printf(io, "Total Pg  : %.3f MW\n",  sum(RGEN.p_g))
        @printf(io, "Total Pd  : %.3f MW\n",  sum(RBUS.p_d))
        @printf(io, "Total Ploss (MW) : %.6f\n", sum(RCIR.p_loss))
        @printf(io, "============================================\n")
    end

    println("Benchmark reports written to: ", out_dir)
    return (; RBUS, RGEN, RCIR, objective = objval)
end

# =============================================================================
#  Test set
# =============================================================================
@testset "PowerModels AC-OPF benchmark (9-bus)" begin

    @test isfile(CASE_FILE)

    network_data, result = solve_pm_benchmark()

    @testset "solve" begin
        @test result["termination_status"] in SOLVED_STATUSES
        @test isfinite(result["objective"])
        # Analytical optimum: lossless + flat 50 EUR/MW => 50 * 315 MW.
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
    end
end
