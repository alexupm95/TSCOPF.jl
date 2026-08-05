# TSC-OPF documentation

**Install and quick start:** see [README.md](../README.md) at the repo root.

**Published docs:** [alexupm95.github.io/TSCOPF.jl](https://alexupm95.github.io/TSCOPF.jl/dev/) (built with Documenter.jl from `docs/make.jl`).

| Document | Audience | Purpose |
|---|---|---|
| [Parameter reference](parameter_reference.md) | End users / researchers | Every `RunConfig` field and valid combinations |
| [Running a case](running_a_case.md) | End users / researchers | Workflow, examples, outputs, validation |
| [Dynamic controls](dynamic_controls_avr_governor.md) | End users / researchers | AVR and TGOV1 governor blocks, equations, outputs |
| [2. Steady-state OPF](src/model/02_steady_state_opf.md) | Researchers | AC / DC / ED formulation (Part I) |
| [3. Generator models](src/model/03_generator_models.md) | Researchers | Classical, dq fourth-order, and GFM inverters |
| [Configuration map](configuration_map.md) | Developers / maintainers | Field map: `RunConfig` → `DispatchConfig` / `TransientConfig` |
| [Developer architecture](src/developer_architecture.md) | Developers / maintainers | Source layout for TS builders and optional extensions |
| [Test suite](https://github.com/alexupm95/TSCOPF.jl/blob/main/test/README.md) | Developers / maintainers | `Pkg.test()` tiers and manual regression scripts |

`user_guide_input_parameters.md` is kept only as a redirect stub — it was split into the parameter reference and the running-a-case guide.

**Quick start:** edit `RunConfig(...)` in `main.jl` (single run) or `main_loop.jl` (δ_tol sweep), then run the script.

**Source files for runtime configuration:**

- `engine.jl` — `RunConfig`, `reconfigure`, `load_system`, `run_case!`
- `_common/DispatchConfig.jl` — steady-state OPF builder (`type_model`, `susceptance_model`, constraint toggles)
- `_transient_stability/TsConfig.jl` — `TransientConfig`, `TsSimulationConfig`, `TsBuilderConfig`
- `_transient_stability/DynModelConfig.jl` — physics / network-form selection
- `_transient_stability/FaultConfig.jl` — disturbance specification (SC / GL / OB)
- `_transient_stability/functions_4_TS_*` — transient-stability variable / equality / inequality builder families
- `_manage_inputs/functions_2_parse_matpower.jl` — MATPOWER `.m` → in-house CSVs/DataFrames (`Import_Matpower_Case`)

> The legacy `config_file.jl` shim layer has been removed — configure runs through the structs above.

## Maintainer checklist (feature / solver / API changes)

Update in the **same commit** as the code change, then run `julia --project=docs docs/make.jl` before pushing to `main` (triggers [Documentation.yml](../.github/workflows/Documentation.yml) → GitHub Pages).

1. `CHANGELOG.md` — `[Unreleased]`
2. `docs/parameter_reference.md` — fields, solver table, examples
3. `docs/configuration_map.md` — struct map
4. `docs/running_a_case.md` — workflow / examples when behaviour changes
5. `docs/src/api.md` — `@docs` entries only for symbols with docstrings
6. `docs/src/developer_architecture.md` — factory / module layout when wiring changes

Narrative files under `docs/*.md` are copied into `docs/src/` at build time (`docs/make.jl`); edit the `docs/` originals.
