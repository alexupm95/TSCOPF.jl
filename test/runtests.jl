#=
================================================================================
 test/runtests.jl  —  package test entry point (`Pkg.test("TSCOPF")`)
================================================================================
 Four tiers, selected by environment variable. The default (no variables set) is
 the FAST GATE: everything that runs on every push, target ≤ 12 min.

   (default)                        fast gate — unit tests, cheap solves, one
                                    FULL_BUS structural solve
   TSCOPF_RUN_HEAVY=true            + DQ_4TH, AVR/governor, GL/OB, A6 numeric pin
   TSCOPF_RUN_SIMULATION_PINS=true  + reference-validated pins (needs ../_TSCOPF_simulations)
   TSCOPF_RUN_BOUND_ENCODING_TSC_PARITY=true
                                    + CONSTRAINT-vs-VARIABLE parity (same repo)
   TSCOPF_RUN_PM_CROSSCHECK=true    + PowerModels case39 cross-check (needs PowerModels)

 See test/README.md for the "I changed X → run Y" table.
================================================================================
=#

using Test
using TSCOPF
using JuMP
using CSV, DataFrames
using Ipopt
using HiGHS
import MathOptInterface as MOI

const PROJECT_ROOT = dirname(@__DIR__)

_gate(name::String) = get(ENV, name, "false") == "true"
const RUN_HEAVY = _gate("TSCOPF_RUN_HEAVY")

include(joinpath(@__DIR__, "test_common.jl"))
include(joinpath(@__DIR__, "tsc_main_style_config.jl"))

# --- fast gate: no solve, or cheap LP/Kron/ACOPF solves ----------------------
# First: the suite reads test/INPUT_FILES/, never the editable demo tree.
include(joinpath(@__DIR__, "runtests_fixture_isolation.jl"))
include(joinpath(@__DIR__, "runtests_fast_unit.jl"))
include(joinpath(@__DIR__, "runtests_gfm_g0.jl"))
include(joinpath(@__DIR__, "runtests_bound_encoding.jl"))
include(joinpath(@__DIR__, "runtests_admittance_matrices.jl"))
include(joinpath(@__DIR__, "runtests_matpower_parser.jl"))
include(joinpath(@__DIR__, "runtests_results_folders.jl"))
include(joinpath(@__DIR__, "runtests_optim_matrices.jl"))
include(joinpath(@__DIR__, "runtests_dispatch_duals_registry.jl"))
include(joinpath(@__DIR__, "runtests_explicit_dual.jl"))
include(joinpath(@__DIR__, "runtests_susceptance_model.jl"))
include(joinpath(@__DIR__, "runtests_matpower_input.jl"))
include(joinpath(@__DIR__, "runtests_smoke.jl"))
include(joinpath(@__DIR__, "runtests_tsc_builder_kron.jl"))
# One FULL_BUS structural solve runs in the fast gate; the remaining toggle
# testsets inside this file are gated on RUN_HEAVY (see the file itself).
include(joinpath(@__DIR__, "runtests_tsc_builder_fullbus.jl"))
# Same self-gating pattern: control validation/factory/limiter checks are free,
# the five end-to-end control solves inside are heavy-tier only.
include(joinpath(@__DIR__, "runtests_controls.jl"))
include(joinpath(@__DIR__, "runtests_plots_ext.jl"))

# UC self-skips unless Gurobi is installed AND licensed (see runtests_uc.jl).
try
    using Gurobi
catch
end
include(joinpath(@__DIR__, "runtests_uc.jl"))

# --- heavy tier: DQ_4TH, controls, disturbances, numeric pins ----------------
if RUN_HEAVY
    # Eight Kron TSC solves: every δ/Δω corridor style, plus the reference-relative
    # exports. The config-level corridor validation is free and lives in the fast gate
    # (runtests_fast_unit.jl); only the structural and export checks need a solve.
    include(joinpath(@__DIR__, "runtests_delta_reference.jl"))
    include(joinpath(@__DIR__, "runtests_dq_fullbus.jl"))
    include(joinpath(@__DIR__, "runtests_gfm_transient.jl"))
    include(joinpath(@__DIR__, "runtests_fullbus_gld.jl"))
    include(joinpath(@__DIR__, "runtests_a6_fullbus_benchmark.jl"))
end

# --- external-input tiers ----------------------------------------------------
if _gate("TSCOPF_RUN_SIMULATION_PINS")
    include(joinpath(@__DIR__, "runtests_simulation_pins.jl"))
end

if _gate("TSCOPF_RUN_BOUND_ENCODING_TSC_PARITY")
    include(joinpath(@__DIR__, "runtests_bound_encoding_tsc.jl"))
end

if _gate("TSCOPF_RUN_PM_CROSSCHECK")
    include(joinpath(@__DIR__, "runtests_powermodels_crosscheck.jl"))
end
