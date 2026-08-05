#=
  Test bound-encoding parity for TSC runs (CONSTRAINT vs VARIABLE).

  Ported from `_TSCOPF_simulations/compare_bound_encoding_tsc.jl`.

  Requires simulation artifacts already present in `_TSCOPF_simulations/RESULTS/`:
    RESULTS/<suite>_constraint/...
    RESULTS/<suite>_variable/...

  Runs only when:
    TSCOPF_SIMULATIONS_ROOT is set (or `_TSCOPF_simulations` is a sibling of this repo)
  and the target suite folders exist.
  It is intended for manual validation against externally generated results,
  not for CI by default.
=#

using Test
using CSV
using DataFrames
using Printf

const PROJECT_ROOT = dirname(@__DIR__)

# `simulations_project_root`. This suite loads no other helper (it compares
# committed CSVs and never solves), so it includes the paths file directly.
@isdefined(FIXTURE_ROOT) || include(joinpath(@__DIR__, "test_paths.jl"))

const PRIMAL_ATOL = 1e-6
const PRIMAL_RTOL = 1e-6
const DUAL_ATOL = 1e-4
const DUAL_RTOL = 1e-4

const COMPARE_SUBDIRS = (
    ("Dispatch/CSV", PRIMAL_ATOL, PRIMAL_RTOL),
    ("Dispatch/CSV_duals", DUAL_ATOL, DUAL_RTOL),
    ("Dispatch_WarmStart/CSV", PRIMAL_ATOL, PRIMAL_RTOL),
    ("Dispatch_WarmStart/CSV_duals", DUAL_ATOL, DUAL_RTOL),
    ("Transient_Stability/CSV", PRIMAL_ATOL, PRIMAL_RTOL),
    ("Transient_Stability/CSV_duals", DUAL_ATOL, DUAL_RTOL),
)

_read_table(path::String) = isfile(path) ? CSV.read(path, DataFrame; delim = ';') : nothing
_numeric_cols(df::DataFrame) = [c for c in names(df) if eltype(df[!, c]) <: Real]

_close_enough(a, b; atol, rtol) = isapprox(a, b; atol = atol, rtol = rtol, nans = true)

deg2rad(x) = x * (π / 180)

function _compare_bus_voltage_angle(constraint_root::String, variable_root::String; atol, rtol)
    rel = "Transient_Stability/CSV/bus_voltage_angle.csv"
    mag_rel = "Transient_Stability/CSV/bus_voltage_magnitude.csv"
    ang_c = _read_table(joinpath(constraint_root, rel))
    ang_v = _read_table(joinpath(variable_root, rel))
    mag_c = _read_table(joinpath(constraint_root, mag_rel))
    mag_v = _read_table(joinpath(variable_root, mag_rel))

    if ang_c === nothing || ang_v === nothing
        return true, "angle file missing (skipped)"
    end
    if mag_c === nothing || mag_v === nothing
        return true, "magnitude file missing (skipped)"
    end

    max_err = 0.0
    for col in names(ang_c)
        col == :t && continue
        col in names(mag_c) || continue
        for i in 1:nrow(ang_c)
            th_c = deg2rad(ang_c[i, col])
            th_v = deg2rad(ang_v[i, col])
            Vc = mag_c[i, col] * cis(th_c)
            Vv = mag_v[i, col] * cis(th_v)
            err = abs(Vc - Vv)
            max_err = max(max_err, err)
            if !_close_enough(Vc, Vv; atol = atol, rtol = rtol)
                return false,
                    @sprintf("row %d col %s complex |Δ|=%.6e (V∠θ phasor mismatch)", i, col, err)
            end
        end
    end
    return true, @sprintf("complex phasor max |Δ| = %.3e", max_err)
end

function compare_csv(constraint_root::String, variable_root::String, rel_path::String; atol, rtol)
    rel_path = replace(rel_path, '\\' => '/')
    if rel_path == "Transient_Stability/CSV/bus_voltage_angle.csv"
        return _compare_bus_voltage_angle(constraint_root, variable_root; atol = atol, rtol = rtol)
    end

    path_c = joinpath(constraint_root, rel_path)
    path_v = joinpath(variable_root, rel_path)
    df_c = _read_table(path_c)
    df_v = _read_table(path_v)

    if df_c === nothing && df_v === nothing
        return true, "both missing (skipped)"
    end
    if df_c === nothing || df_v === nothing
        return false, "only one side present ($path_c vs $path_v)"
    end
    if names(df_c) != names(df_v) || nrow(df_c) != nrow(df_v)
        return false, "shape mismatch ($(nrow(df_c))×$(ncol(df_c)) vs $(nrow(df_v))×$(ncol(df_v)))"
    end

    max_err = 0.0
    for col in _numeric_cols(df_c)
        for i in 1:nrow(df_c)
            a = df_c[i, col]
            b = df_v[i, col]
            ok = _close_enough(a, b; atol = atol, rtol = rtol)
            err = abs(a - b)
            max_err = max(max_err, err)
            ok || return false, @sprintf("row %d col %s: %.6e vs %.6e (Δ=%.6e)", i, col, a, b, err)
        end
    end
    return true, @sprintf("max |Δ| = %.3e", max_err)
end

function _collect_csv_files(root::String, sub::String)
    dir = joinpath(root, sub)
    isdir(dir) || return String[]
    files = String[]
    for f in readdir(dir)
        endswith(f, ".csv") || continue
        push!(files, joinpath(sub, f))
    end
    return sort(files)
end

function _suite_exists(sim_root::String, suite::String)
    c = joinpath(sim_root, "RESULTS", "$(suite)_constraint")
    v = joinpath(sim_root, "RESULTS", "$(suite)_variable")
    return isdir(c) && isdir(v)
end

const DEFAULT_SUITES = (
    "bound_compare_2nd_fullbus",
    "bound_compare_4th_fullbus",
    "bound_compare_4th_fullbus_avr",
)

@testset "Bound encoding parity (CONSTRAINT vs VARIABLE) — port of compare_bound_encoding_tsc.jl" begin
    sim_root = simulations_project_root(PROJECT_ROOT)
    sim_root == "" && @info "Skipping parity test: simulations repo not found." && return

    for suite in DEFAULT_SUITES
        # Julia's Test requires a literal or interpolated string here; passing the
        # `String` variable bare errors out before the body runs ("Expected `suite`
        # to be an AbstractTestSet"). This suite is a manual gate, so nothing caught it.
        @testset "$suite" begin
            if !_suite_exists(sim_root, suite)
                @info "Skipping suite parity: missing $(suite)_constraint or $(suite)_variable" suite= suite
                return
            end

            constraint_root = joinpath(sim_root, "RESULTS", "$(suite)_constraint")
            variable_root = joinpath(sim_root, "RESULTS", "$(suite)_variable")

            all_ok = true
            seen = Set{String}()
            for (sub, atol, rtol) in COMPARE_SUBDIRS
                files = _collect_csv_files(constraint_root, sub)
                isempty(files) && (files = _collect_csv_files(variable_root, sub))
                isempty(files) && continue

                for f in files
                    f in seen && continue
                    push!(seen, f)
                    ok, msg = compare_csv(constraint_root, variable_root, f; atol = atol, rtol = rtol)
                    all_ok &= ok
                    # `@test ok || @error(...)` errored instead of failing: `@error`
                    # returns `nothing`, so the tested expression was non-Boolean and
                    # the mismatch message never reached the report. Log first, then
                    # assert on the Boolean alone.
                    ok || @error "Mismatch" suite = suite file = f message = msg
                    @test ok
                end
            end
            @test all_ok
        end
    end
end

