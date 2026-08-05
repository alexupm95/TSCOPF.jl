using Pkg
const REPO = dirname(@__DIR__)
Pkg.activate(joinpath(REPO, "test"))
Pkg.develop(PackageSpec(path = REPO))
for pkg in (
    "Test", "JuMP", "CSV", "DataFrames", "MathOptInterface", "DataStructures",
    "Ipopt", "HiGHS", "Plots", "Measures",
    # stdlibs still need declaring in a package test environment
    "LinearAlgebra", "Printf", "SparseArrays",
    # PowerModels backs the opt-in cross-check suite (TSCOPF_RUN_PM_CROSSCHECK).
    "PowerModels",
)
    Pkg.add(pkg)
end
# Optional: UC / extension smoke when a Gurobi license is present. Skipped on CI —
# the runner has no license, so `runtests_uc.jl` would skip itself anyway and we
# would only be paying to download Gurobi_jll on every job.
if get(ENV, "CI", "false") == "true"
    @info "CI detected — skipping Gurobi; UC tests will be skipped."
else
    try
        Pkg.add("Gurobi")
    catch err
        @warn "Gurobi not added to test environment; UC tests will be skipped." err
    end
end
Pkg.resolve()
println("test/Project.toml ready.")
