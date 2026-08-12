#=
  GFM integrator A/B — `DynModelConfig.gfm_integrator`
  ----------------------------------------------------
  The GFM measurement filters (P/Q/V) and the Q–V PI integrator used to be backward
  Euler at every step, unreachable from any knob. `gfm_integrator` now selects:

    :backward_euler         BE at every step (what the reference implementation does)
    :trapezoidal            trapezoidal at every step
    :follow_ode_first_step  BE at t=1 of each window iff ode_first_step=:backward_euler,
                            trapezoidal after — the SG rule, and the package default

  This script is the acceptance gate for that change.

    Tier 1 (gating)     every scheme converges: status LOCALLY_SOLVED/OPTIMAL *and*
                        Ipopt iterations < max_iter. An ITERATION_LIMIT run is refused,
                        not compared — diffing a mid-iteration point is how
                        `docs/gfm_parity_status.md` ended up with phantom 195 MW gaps.
    Tier 2 (gating)     objectives agree within TOL_OBJ_REL.
    Tier 3 (reported)   max-abs trajectory diff per state, and a Δt-halving refinement
                        check: the BE-all ↔ trap-all gap must shrink roughly like Δt.
                        A wrong t=1 anchor moves trajectories while barely moving the
                        objective, so Tier 2 alone would not catch it.

  Run:
    julia --project=. scripts/run_gfm_integrator_ab.jl
=#

using TSCOPF
using JuMP
using CSV
using DataFrames
using Dates
using Printf
import MathOptInterface as MOI

const PKG_ROOT = dirname(@__DIR__)
const OUT_DIR = joinpath(PKG_ROOT, "OUTPUTS")
const RESULTS_ROOT = joinpath(PKG_ROOT, "RESULTS", "gfm_integrator_ab")

const SCHEMES = (:backward_euler, :follow_ode_first_step, :trapezoidal)

# The operating point is the one `test/runtests_gfm_transient.jl` solves — load 1.5,
# 0.6 s horizon, 150 ms clearing, no AVR/governor, stock Ipopt. That config converges in
# ~30 s; the `scripts/run_gfm_reference_parity.jl` point (load 1.3, AVR + governor,
# limited-memory Hessian, max_iter 350) does NOT — `docs/gfm_parity_status.md` open item 3
# records it exiting on the iteration limit, and an A/B built on it fails all three arms
# including `:backward_euler`, which is the pre-change behaviour. Comparing discretisation
# schemes needs a point where the baseline actually converges.
#
# AVR and governor are off deliberately: they are SG-side families that `gfm_integrator`
# does not touch, so they add solve time without adding signal.
const T_END = 0.6
const T_STEP = 0.01
const T_STEP_FINE = 0.005          # Δt-halving refinement
const CLEARING = 0.15
const LOAD_FACTOR = 1.5
const MAX_ITER = 1000

# Gates
const TOL_OBJ_REL = 1e-3

# Trajectory files compared in Tier 3, relative to Transient_Stability/CSV/.
const TRAJ_FILES = (
    "angle_abs.csv", "speed_dev.csv",
    "electrical_power.csv", "electrical_reactive_power.csv",
    "bus_voltage_magnitude.csv",
    "gfm_P_meas.csv", "gfm_Q_meas.csv", "gfm_V_meas.csv", "gfm_E_int.csv",
)

# ---------------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------------

"""
Matched config for one scheme. Everything except `gfm_integrator` is held fixed.

`ode_first_step = :backward_euler` is pinned so `:follow_ode_first_step` is exercised in
its BE-at-t=1 form; with the `:trapezoidal` default it would collapse onto the
`:trapezoidal` arm and the three-way comparison would only be two-way.
"""
function ab_cfg(scheme::Symbol; case::String, t_step::Float64)
    return RunConfig(
        case = case,
        load_factor = LOAD_FACTOR,
        trans_stab = true,
        solver_name = "Ipopt",
        # Not silent: the Ipopt summary is the only place the iteration count is
        # reported, and Tier 1 gates on it.
        silent_solver = false,
        overwrite_results = true,
        save_duals = false,
        save_matrices = false,
        save_ts_plots = false,
        save_optim_matrices = false,
        save_ts_debug_csv = true,
        ipopt = IpoptSolverConfig(max_iter = MAX_ITER),
        dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data_full.csv",
            gfm_dynamic_filename = "gfm_dynamic_data.csv",
            simulation = TsSimulationConfig(
                δ_tol_deg = 100.0,
                t_end_sim = T_END,
                t_step = t_step,
                t_start_fault = 0.01,
                clearing_time = CLEARING,
            ),
            dyn_model = DynModelConfig(
                allow_gfm = true,
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = :coi_box,
                # Package (Z, I, P); the reference [0,0,1] is (P, I, Z) → same physics.
                # Getting this backwards silently gives constant-power loads and an
                # iteration-limit exit (CLAUDE.md §2).
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                dq_speed_dev_in_algebra = true,
                ode_first_step = :backward_euler,
                gfm_integrator = scheme,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
end

# ---------------------------------------------------------------------------------
# Run + harvest
# ---------------------------------------------------------------------------------

"""Ipopt iteration count from a solver log; `missing` if the line is absent."""
function log_iterations(log_path::String)
    isfile(log_path) || return missing
    for line in eachline(log_path)
        m = match(r"Number of Iterations\.*:\s*(\d+)", line)
        m === nothing || return parse(Int, m.captures[1])
    end
    return missing
end

"""Objective from the dispatch report CSV (a number on disk, never a live JuMP ref)."""
function report_objective(results_dir::String)
    p = joinpath(results_dir, "Dispatch", "CSV", "optimization_report.csv")
    isfile(p) || return missing
    return Float64(CSV.read(p, DataFrame; delim = ';').Value[1])
end

"""
Solve one (scheme, case, Δt) point.

Every number is read back from the exported CSVs: `release_solver_backend!` empties the
model on the Ipopt path, so the returned `dyn_model_dict` holds dead refs (CLAUDE.md §7).
"""
function run_point(scheme::Symbol; case::String, t_step::Float64)
    tag = "$(case)_$(scheme)_dt$(replace(string(t_step), "." => "p"))"
    results_dir = joinpath(RESULTS_ROOT, tag)
    mkpath(results_dir)
    cfg = ab_cfg(scheme; case = case, t_step = t_step)
    sys = load_system(cfg, PKG_ROOT)

    t0 = time()
    out = run_case!(cfg, sys, PKG_ROOT, results_dir)
    elapsed = time() - t0

    iters = log_iterations(joinpath(results_dir, "solver_log.txt"))
    converged = out.status in (MOI.OPTIMAL, MOI.LOCALLY_SOLVED)
    return (
        scheme = scheme, case = case, t_step = t_step, tag = tag,
        status = out.status, converged = converged,
        iters = iters, obj = report_objective(results_dir),
        elapsed = elapsed,
        ts_csv = joinpath(results_dir, "Transient_Stability", "CSV"),
    )
end

# ---------------------------------------------------------------------------------
# Trajectory comparison
# ---------------------------------------------------------------------------------

"""
Max-abs difference between two runs of one trajectory CSV, over the shared columns.

Both sides are on the same time grid by construction (same `t_step`, same windows), so
this is a straight row-wise diff — no interpolation, nothing to misalign.
"""
function max_abs_diff(csv_a::String, csv_b::String)
    (isfile(csv_a) && isfile(csv_b)) || return missing
    a = CSV.read(csv_a, DataFrame; delim = ';')
    b = CSV.read(csv_b, DataFrame; delim = ';')
    cols = intersect(names(a), names(b))
    filter!(c -> c != "t", cols)
    isempty(cols) && return missing
    n = min(nrow(a), nrow(b))
    n == 0 && return missing
    worst = 0.0
    for c in cols
        va = Float64.(coalesce.(a[1:n, c], NaN))
        vb = Float64.(coalesce.(b[1:n, c], NaN))
        d = abs.(va .- vb)
        finite = filter(isfinite, d)
        isempty(finite) || (worst = max(worst, maximum(finite)))
    end
    return worst
end

"""Per-file max-abs diff between two runs, as an ordered `file => gap` vector."""
function trajectory_gaps(run_a, run_b)
    return [(f, max_abs_diff(joinpath(run_a.ts_csv, f), joinpath(run_b.ts_csv, f)))
            for f in TRAJ_FILES]
end

"""Largest gap across all compared trajectory files; `missing` if none were readable."""
function worst_gap(gaps)
    vals = [g for (_, g) in gaps if g !== missing]
    return isempty(vals) ? missing : maximum(vals)
end

# ---------------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------------

fmt(x::Missing) = "n/a"
fmt(x::Real) = @sprintf("%.6g", x)

function main()
    case = get(ENV, "TSCOPF_AB_CASE", "9bus_gfm")
    mkpath(OUT_DIR)
    lines = String[]
    say(s = "") = (println(s); push!(lines, s))

    say("="^78)
    say("GFM integrator A/B — case $case — $(Dates.now())")
    say("load=$LOAD_FACTOR, t_end=$T_END s, t_step=$T_STEP s, clearing=$CLEARING s, " *
        "max_iter=$MAX_ITER, ode_first_step=:backward_euler")
    say("="^78)

    # --- Tier 1 + 2: the three schemes at the nominal Δt --------------------------
    runs = Dict{Symbol, Any}()
    for s in SCHEMES
        say("\n--- running $s ---")
        runs[s] = run_point(s; case = case, t_step = T_STEP)
        r = runs[s]
        say(@sprintf("  status=%s  iters=%s  obj=%s  %.1f s",
            r.status, fmt(r.iters), fmt(r.obj), r.elapsed))
    end

    say("\n" * "-"^78)
    say("TIER 1 — convergence (gating)")
    say("-"^78)
    tier1 = true
    for s in SCHEMES
        r = runs[s]
        hit_limit = r.iters !== missing && r.iters >= MAX_ITER
        ok = r.converged && !hit_limit
        tier1 &= ok
        say(@sprintf("  %-24s %-18s iters=%-6s %s",
            s, string(r.status), fmt(r.iters), ok ? "PASS" : "FAIL"))
    end

    say("\n" * "-"^78)
    say("TIER 2 — objective agreement (gating, rel tol $(TOL_OBJ_REL))")
    say("-"^78)
    tier2 = true
    if !tier1
        say("  SKIPPED — an unconverged run must not be compared.")
        tier2 = false
    else
        ref_obj = runs[:backward_euler].obj
        for s in SCHEMES
            o = runs[s].obj
            rel = (o === missing || ref_obj === missing) ? missing :
                  abs(o - ref_obj) / max(abs(ref_obj), 1e-9)
            ok = rel !== missing && rel <= TOL_OBJ_REL
            tier2 &= ok
            say(@sprintf("  %-24s obj=%-14s rel_diff=%-12s %s",
                s, fmt(o), fmt(rel), ok ? "PASS" : "FAIL"))
        end
    end

    # --- Tier 3: trajectories + Δt refinement (reported, non-gating) --------------
    say("\n" * "-"^78)
    say("TIER 3 — trajectory gaps and Δt refinement (reported, not gating)")
    say("-"^78)
    gap_coarse = missing
    if tier1
        for (a, b) in ((:backward_euler, :trapezoidal),
                       (:backward_euler, :follow_ode_first_step),
                       (:follow_ode_first_step, :trapezoidal))
            say("\n  max|Δ| $(a) vs $(b):")
            for (f, g) in trajectory_gaps(runs[a], runs[b])
                say(@sprintf("    %-34s %s", f, fmt(g)))
            end
        end
        gap_coarse = worst_gap(trajectory_gaps(runs[:backward_euler], runs[:trapezoidal]))

        say("\n  Δt-halving refinement (t_step $(T_STEP) → $(T_STEP_FINE)):")
        say("  BE-all and trap-all differ at O(Δt), so halving Δt should roughly halve")
        say("  the gap between them. A flat or growing gap points at an anchor/sign error,")
        say("  not at discretisation error.")
        fine = Dict(s => run_point(s; case = case, t_step = T_STEP_FINE)
                    for s in (:backward_euler, :trapezoidal))
        for s in (:backward_euler, :trapezoidal)
            r = fine[s]
            say(@sprintf("    %-16s (Δt=%s) status=%s iters=%s",
                s, fmt(r.t_step), r.status, fmt(r.iters)))
        end
        if all(fine[s].converged for s in (:backward_euler, :trapezoidal))
            gap_fine = worst_gap(trajectory_gaps(fine[:backward_euler], fine[:trapezoidal]))
            ratio = (gap_coarse === missing || gap_fine === missing || gap_fine == 0) ?
                    missing : gap_coarse / gap_fine
            say(@sprintf("    worst gap  Δt=%s : %s", fmt(T_STEP), fmt(gap_coarse)))
            say(@sprintf("    worst gap  Δt=%s : %s", fmt(T_STEP_FINE), fmt(gap_fine)))
            say(@sprintf("    ratio (expect ≈ 2): %s", fmt(ratio)))
        else
            say("    SKIPPED — a fine-Δt run did not converge.")
        end
    else
        say("  SKIPPED — Tier 1 failed.")
    end

    say("\n" * "="^78)
    say("VERDICT: Tier 1 $(tier1 ? "PASS" : "FAIL") | Tier 2 $(tier2 ? "PASS" : "FAIL")")
    say("="^78)

    report = joinpath(OUT_DIR, "gfm_integrator_ab_$(case).txt")
    open(report, "w") do io
        for l in lines
            println(io, l)
        end
    end
    println("\nReport written to $report")
    return tier1 && tier2
end

main()
