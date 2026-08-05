# ==============================================================================
#  TSC-OPF — global numerical / physical constants
# ==============================================================================
# Single place to tune sentinel values and other shared literals used across
# dispatch, transient-stability, and I/O layers.  Included early from
# `src/TSCOPF.jl` so every entry point (main, tests, engine) sees the same
# definitions in `Main`.  Phase 6 will wrap this file in the package module and
# export the public constants for `using TSCOPF` workflows.
#
# Registry tables (dual export specs, variable order, …) stay in their own
# files — they are schema metadata, not user-tunable physical constants.
# ==============================================================================

# Large shunt admittance stamped on the faulted bus to emulate a solid three-phase
# short circuit (bus voltage ≈ 0) during the fault window. Per-unit on base_MVA.
const FAULT_BUS_SHUNT = 1e10
