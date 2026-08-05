#=
================================================================================
 test/test_env.jl  —  shared imports for standalone test scripts
================================================================================
 Replaces `include("setup_includes.jl")`.  Loads the package API into the test
 script scope (MOI termination statuses as bare names for legacy test files).
================================================================================
=#

using TSCOPF
using JuMP
import MathOptInterface as MOI
using DataStructures: OrderedDict

# Fixture roots and helpers. Guarded because test_common.jl includes the same
# file, and many suites load both.
@isdefined(FIXTURE_ROOT) || include(joinpath(@__DIR__, "test_paths.jl"))
