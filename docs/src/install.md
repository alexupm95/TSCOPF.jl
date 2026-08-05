# Install

Julia **≥ 1.10** is required (`Project.toml` `[compat]`).

!!! note "Research code, not a registered package"
    TSCOPF.jl is not on the General registry. Install it from a local clone or
    directly from GitHub (below) — `Pkg.add("TSCOPF")` will not work.

## Local clone

```julia
using Pkg
Pkg.activate("path/to/TSCOPF.jl")
Pkg.instantiate()
using TSCOPF
```

`Manifest.toml` is not committed; `Pkg.instantiate()` resolves versions from `[compat]`.

## Install from GitHub

```julia
using Pkg
Pkg.add(url = "https://github.com/alexupm95/TSCOPF.jl")
using TSCOPF
```

## Solvers

| Problem class | Typical solver | Notes |
|---------------|----------------|-------|
| ACOPF, TSC-ACOPF | Ipopt, MadNLP | NLP |
| DCOPF, TSC-DCOPF, ED | HiGHS, Gurobi | LP |
| UC toy | Gurobi | MILP; optional |

Smoke tests and CI use **Ipopt** + **HiGHS** (no Gurobi license required). `Pkg.test()` also runs the A6 FULL_BUS numeric gate (~1–2 min on CI).

!!! warning "Solver ↔ formulation pairing is enforced, not a suggestion"
    `run_case!` calls `Check_Coherence_Input_Data` before building anything: an
    ACOPF or TSC-ACOPF run with `solver_name = "Gurobi"` or `"HiGHS"` (LP/MILP
    solvers) throws immediately, and likewise a DCOPF/ED run with Ipopt/MadNLP.
    See the solver-compatibility table in the [Parameter reference](parameter_reference.md).

## Tests

```julia
julia --project=. -e 'using Pkg; Pkg.test()'   # smoke + fast unit + A6 + UC (when Gurobi present)
```

Full tier map and manual regression scripts: [`test/README.md`](https://github.com/alexupm95/TSCOPF.jl/blob/main/test/README.md).

Bootstrap the test environment after a fresh clone if needed:

```julia
julia --project=. scripts/setup_test_env.jl
```

## Build these docs locally

```julia
julia docs/setup_docs_env.jl          # first time only
julia --project=docs --color=yes docs/make.jl
```

Open `docs/build/index.html` in a browser.

!!! tip
    `docs/parameter_reference.md`, `docs/running_a_case.md`, and `docs/configuration_map.md` are the
    single source of truth for those two pages — `make.jl` copies them into
    `docs/src/` at build time. Edit the top-level `docs/*.md` files, not the
    copies under `docs/src/` (those are gitignored and get overwritten).
