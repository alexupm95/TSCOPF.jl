#=
Pinned scalars from the reference TSC-ACOPF FULL_BUS run (June 2026).

Baseline folder (local, gitignored under RESULTS/):
  RESULTS/_benchmark_TSCOPF_full/Results - 2026-06-19 153727

Source config: `main.jl` at commit time of the benchmark.
Ipopt log reports 236.25 in internal MVA-scaled units; `run_case!` exposes
`JuMP.value(result.obj_MVA)` as **23625 EUR** (quadratic cost × base_MVA).
=#

const A6_BENCHMARK_FOLDER = "_benchmark_TSCOPF_full"
const A6_BENCHMARK_RUN_DIR = "Results - 2026-06-19 153727"

"""Absolute path to the frozen reference run (may be absent on CI clones)."""
a6_baseline_results_dir(root::String=project_root()) =
    joinpath(root, "RESULTS", A6_BENCHMARK_FOLDER, A6_BENCHMARK_RUN_DIR)

const A6_FULLBUS_OBJ_EUR = 23_625.0
const A6_FULLBUS_OBJ_RTOL = 1e-6

# Dispatch generators_report.csv (MW), generator IDs 1–3.
const A6_FULLBUS_PG_MW = (
    249.68646697820182,
    113.85693934147395,
    108.95659368032426,
)
const A6_FULLBUS_PG_RTOL = 1e-5

# angle_rel_COI.csv at t = 0.01 s (deg) for generators 1–3.
const A6_ANGLE_REL_COI_T0 = 0.01
const A6_ANGLE_REL_COI_DEG = (
    1.9252398102277306,
    -5.901103117031117,
    -2.5732920813237223,
)
const A6_TRAJ_RTOL = 1e-5
