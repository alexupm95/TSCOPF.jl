# TSCOPF.jl

[![CI](https://github.com/alexupm95/TSCOPF.jl/actions/workflows/CI.yml/badge.svg)](https://github.com/alexupm95/TSCOPF.jl/actions/workflows/CI.yml)
[![Docs](https://img.shields.io/badge/docs-dev-blue.svg)](https://alexupm95.github.io/TSCOPF.jl/dev/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Transient-Stability-Constrained Optimal Power Flow** in Julia/JuMP.

> **Status:** research code. Not registered in the Julia General registry, and the API is not stable across versions.

TSCOPF builds and solves steady-state OPF subproblems (ED, DCOPF, ACOPF, UC) and transient-stability-constrained formulations, with structured dual export for economic analysis.

Transient paths: Kron-reduced and full-bus nodal networks; classical second-order or fourth-order dq machines; optional AVR and TGOV1 governor; mixed synchronous + grid-forming (GFM) fleets. Disturbances cover bus short circuits with branch tripping, generator/load disconnection, and open-branch events.

## Install

Julia ≥ 1.10. From the REPL:

```julia
using Pkg
Pkg.activate("path/to/TSCOPF.jl")   # local clone
# or
Pkg.add(url = "https://github.com/alexupm95/TSCOPF.jl")
Pkg.instantiate()
using TSCOPF
```

`Manifest.toml` is not committed; `Pkg.instantiate()` resolves deps from `[compat]` in `Project.toml`.

**Solvers:** core dependency is **Ipopt** (NLP) + **HiGHS** (LP). **Gurobi**, **MadNLP**, and **Plots** are optional [package extensions](https://pkgdocs.julialang.org/v1/creating-packages/#Conditional-loading-of-code-in-packages-(extensions)): add them to your environment (`Pkg.add("Gurobi")`) and `using` them after `using TSCOPF` to enable MILP/UC, MadNLP NLP, and trajectory plots. Smoke tests and CI use Ipopt + HiGHS only; UC tests run when Gurobi is installed and licensed.

## Quick start

Edit `RunConfig(...)` in `main.jl` (single run) or `main_loop.jl` (δ_tol sweep), then:

```bash
julia --project=. main.jl
```

Results are written under `RESULTS/Results - <timestamp>/` (gitignored). Case data live in `INPUT_FILES/<case>/` and are yours to edit — the test suite reads its own frozen copies from `test/INPUT_FILES/`.

Minimal programmatic use:

```julia
using TSCOPF
cfg = RunConfig(case = "9bus", dispatch = DispatchConfig(type_model = "ACOPF"), silent_solver = true)
sys = load_system(cfg, @__DIR__)
result = run_case!(cfg, sys, @__DIR__, joinpath(@__DIR__, "RESULTS"))
```

## Tests

```julia
julia --project=. -e 'using Pkg; Pkg.test()'                        # fast gate (~2 min)
TSCOPF_RUN_HEAVY=true julia --project=. -e 'using Pkg; Pkg.test()'  # + DQ_4TH, AVR/governor, GL-OB, A6 pin
```

See [test/README.md](test/README.md) for the four tiers and the "I changed X → run Y" table.

Bootstrap the test environment after a fresh clone if needed:

```julia
julia --project=. scripts/setup_test_env.jl
```

## Documentation

| Resource                                                                           | Description                                     |
| ---------------------------------------------------------------------------------- | ----------------------------------------------- |
| [Documentation (GitHub Pages)](https://alexupm95.github.io/TSCOPF.jl/dev/) | Install, quick start, manuals, API              |
| [docs/parameter_reference.md](docs/parameter_reference.md)                          | Every `RunConfig` field and valid combinations |
| [docs/running_a_case.md](docs/running_a_case.md)                                    | Workflow, presets, outputs, validation          |
| [docs/dynamic_controls_avr_governor.md](docs/dynamic_controls_avr_governor.md)      | AVR and TGOV1 governor blocks                   |
| [docs/configuration_map.md](docs/configuration_map.md)                              | `RunConfig` → builder structs                |
| [docs/README.md](docs/README.md)                                                    | Doc index                                       |

Configuration is through `RunConfig`, `DispatchConfig`, `TransientConfig`, and related structs in `src/` (see `engine.jl`).

## Project layout

```
src/TSCOPF.jl          # package module
main.jl                # single-run entry
main_loop.jl           # parameter sweep entry
examples/              # one standalone script per avenue, every knob written out
INPUT_FILES/           # case CSVs — demo and case-study data, yours to edit
test/                  # Pkg.test + benchmarks (frozen fixtures in test/INPUT_FILES/)
docs/                  # user-facing guides
```

## License

MIT — see [LICENSE](LICENSE). Bundled third-party case data and the funder logos carry their own terms; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Citation

If you use this code, cite the repository. Machine-readable metadata is in [`CITATION.cff`](CITATION.cff), which GitHub renders as a *Cite this repository* button.

| Authors | Affiliation | Contact |
|---|---|---|
| Alex Junior da Cunha Coelho | Technical University of Madrid | alexjunior.dacunhacoelho@upm.es |
| Jorge Navarro Fidalgo | Technical University of Madrid | jorge.navarro.fidalgo@upm.es |

## Acknowledgements

This work was supported by **MICIU/AEI/10.13039/501100011033** and **ERDF/EU** under grants **PID2023-150401OA-C22** and **PID2022-141609OB-I00**, as well as by the Madrid Government (Comunidad de Madrid-Spain) under the Multiannual Agreement 2023-2026 with Universidad Politécnica de Madrid, `Line A - Emerging PIs' (grant number: **24-DWGG5L-33-SMHGZ1**).

<div align="center" style="display:flex; flex-wrap:wrap; align-items:center; justify-content:center; gap:24px;">
  <img src="./utils/logo_MICIU_AEI.png" alt="MICIU/AEI" height="200" style="object-fit:contain;">
  <img src="./utils/logo_CM.png" alt="Comunidad de Madrid" height="200" style="object-fit:contain;">
</div>
