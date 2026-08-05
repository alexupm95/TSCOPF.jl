#=
================================================================================
 test/test_paths.jl  —  where the test suite reads its input data
================================================================================
 The suite reads case data ONLY from `test/INPUT_FILES/`, never from the
 repository's `INPUT_FILES/`.  The two trees have different jobs:

   INPUT_FILES/         demo and case-study material — the user's to edit
   test/INPUT_FILES/    frozen fixtures — editing one is a deliberate re-pin

 Before this split the suite asserted numeric pins (the A6 objective at
 rtol = 1e-6, parsed gen costs, GFM `Imax`) against the same files a user is
 invited to modify, so a routine edit to the demo data turned CI red for
 reasons that had nothing to do with the code.

 The mechanism is `load_system`'s second argument: it composes
 `<path_main>/INPUT_FILES/<case>` (`src/engine.jl:340`), so pointing `path_main`
 at `test/` is all that is needed — no package change, and the same trick
 `runtests_simulation_pins.jl` already uses for the external simulations repo.

 This file is dependency-free on purpose: it is included from both
 `test_common.jl` and `test_env.jl`, so every suite reaches it by one path or
 the other regardless of which of the two it loads.

 Fixture manifest and re-pin protocol: `test/INPUT_FILES/README.md`.
================================================================================
=#

"""Frozen input root. `@__DIR__` resolves per included file, so this is `test/`."""
const FIXTURE_ROOT = @__DIR__

"""The fixture case tree: `test/INPUT_FILES/`."""
const FIXTURE_INPUT_FILES = joinpath(FIXTURE_ROOT, "INPUT_FILES")

"""Results still land in the repository's `RESULTS/`, not under `test/`."""
const RESULTS_ROOT = joinpath(dirname(@__DIR__), "RESULTS")

"""Directory of a frozen case, e.g. `fixture_case("9bus")`."""
fixture_case(case::AbstractString) = joinpath(FIXTURE_INPUT_FILES, case)

"""Path of a frozen MATPOWER file, e.g. `fixture_matpower("case39.m")`."""
fixture_matpower(name::AbstractString) = joinpath(FIXTURE_INPUT_FILES, "PowerModels", name)

"""`load_system` against the frozen fixtures instead of the demo tree."""
load_fixture_system(cfg) = load_system(cfg, FIXTURE_ROOT)

"""`run_case!` reading frozen fixtures, writing to the repository `RESULTS/`."""
run_fixture_case!(cfg, sys) = run_case!(cfg, sys, FIXTURE_ROOT, RESULTS_ROOT)

"""
    simulations_project_root([project_root]) -> String

Root of the external `_TSCOPF_simulations` driver repo, which holds the variant
`INPUT_FILES/` and frozen `RESULTS/` trees behind the pinned-simulation and
bound-encoding-parity suites. Resolution order: `ENV["TSCOPF_SIMULATIONS_ROOT"]`
if set and a directory, else a `_TSCOPF_simulations` sibling of this repository.

Returns `""` when neither exists — callers treat that as "skip", since these
suites are external-validation gates that must not fail on a machine without the
sibling checkout.

This is the one data root that is deliberately *not* a fixture: the runs it gates
are validated against externally generated results, which is why they are also the
only suites exempt from `runtests_fixture_isolation.jl`.
"""
function simulations_project_root(project_root::AbstractString = dirname(@__DIR__))
    env = get(ENV, "TSCOPF_SIMULATIONS_ROOT", "")
    if !isempty(env) && isdir(env)
        return abspath(env)
    end
    sibling = abspath(project_root, "..", "_TSCOPF_simulations")
    isdir(sibling) && return sibling
    return ""
end
