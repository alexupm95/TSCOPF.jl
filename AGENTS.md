# TSCOPF.jl — orientation

Transient-Stability-Constrained Optimal Power Flow in Julia/JuMP. This file is a
short map of the codebase. Full documentation is at
[alexupm95.github.io/TSCOPF.jl](https://alexupm95.github.io/TSCOPF.jl/dev/).

## What it does

Builds and solves steady-state OPF subproblems (ED, DC-OPF, AC-OPF, a toy UC) and
transient-stability-constrained formulations that couple the dispatch to a
time-discretised swing model. Every constraint family exports its dual through a
registry, so shadow prices on stability limits come out alongside nodal prices.

Julia ≥ 1.10. Core solvers are **Ipopt** (NLP) and **HiGHS** (LP), both hard
dependencies. **Gurobi**, **MadNLP**, and **Plots** are optional package
extensions in `ext/` — `Pkg.add` them and `using` them after `using TSCOPF`.

## Entry points

| Path | Purpose |
|---|---|
| `main.jl` | Single run. Edit the `RunConfig(...)` at the top, then `julia --project=. main.jl` |
| `main_loop.jl` | Parameter sweep over `δ_tol` |
| `examples/` | One standalone, runnable script per modelling avenue, every knob written out |

Programmatic use is three calls:

```julia
using TSCOPF
cfg    = RunConfig(case = "9bus", dispatch = DispatchConfig(type_model = "ACOPF"))
sys    = load_system(cfg, @__DIR__)
result = run_case!(cfg, sys, @__DIR__, joinpath(@__DIR__, "RESULTS"))
```

## Configuration

Everything is nested structs, no global config file:

```
RunConfig                          case, load_factor, solver, trans_stab, save_duals, …
├── dispatch::DispatchConfig       type_model, cost_type, bound_encoding, limits
└── transient::TransientConfig     (required when trans_stab = true)
    ├── simulation::TsSimulationConfig   time windows, clearing_time, step size
    ├── builder::TsBuilderConfig         constraint toggles, optional state boxes
    └── dyn_model::DynModelConfig        gen_order, network_form, AVR/governor, GFM
        └── fault::FaultConfig           SC (short circuit) | GL (gen/load trip) | OB (open branch)
```

`run_case!` validates the combination before building anything — invalid
solver/formulation pairings and incoherent dynamic configs throw immediately
rather than producing a silently wrong model.

## Source layout

| Path | Contents |
|---|---|
| `src/TSCOPF.jl` | Module root, exports, extension hooks |
| `src/engine.jl` | `RunConfig`, `load_system`, `run_case!`, `reconfigure` |
| `src/_common/` | `DispatchConfig`, limit configs, admittance matrices |
| `src/_acopf/`, `_dcopf/`, `_ed/`, `_uc/` | Steady-state formulations |
| `src/_transient_stability/` | TS builders (Kron-reduced, full-bus, dq fourth-order), AVR, governor, grid-forming inverters |
| `src/_manage_inputs/` | CSV and MATPOWER `.m` ingestion, solver setup |
| `src/_manage_outputs/` | Result export: CSV, XLSX, figures, dual dumps |
| `ext/` | Gurobi, MadNLP, and Plots extensions |
| `INPUT_FILES/<case>/` | Case data as CSVs — yours to edit |
| `test/INPUT_FILES/` | Frozen test fixtures — do **not** edit these to change a run |

## Running the tests

```bash
julia --project=. -e 'using Pkg; Pkg.test()'                        # fast gate, ~2 min
TSCOPF_RUN_HEAVY=true julia --project=. -e 'using Pkg; Pkg.test()'  # + heavy tier
```

`test/README.md` has the four tiers and an "I changed X → run Y" table. Tiers 3
and 4 need data or packages that are not in this repository and skip themselves
when absent.

## Three things that will cost you time otherwise

1. **`zip_load` is ordered `(Z, I, P)`** — constant impedance, constant current,
   constant power. Other codes order it `(P, I, Z)`. Getting this backwards
   silently gives constant-power loads and a transient solve that runs to the
   iteration limit instead of failing loudly.

2. **Dual signs.** Inequalities are built in `(LHS - RHS) ≤ 0` form, so JuMP
   returns **non-positive** duals for active constraints. Nodal prices are
   `π_k = +λ_k` on the power-balance equalities: the balance is coded
   `P_g - P_d - Σflows == 0` and JuMP's Lagrangian is `f - Σλ(LHS - RHS)`, so
   the two minus signs cancel. The **explicit dual LP** (`Dispatch_Dual/`) uses
   the textbook convention instead and its own λ satisfies `π_k = −λ_k`; the two
   λ differ by a sign. Check the convention on a small case before trusting
   exported numbers.

3. **The JuMP model is dead after `run_case!` returns.** The solver backend is
   released with `empty!(model)`, so every `VariableRef` and `ConstraintRef` in
   the returned dict is invalid. Read results from the CSVs under
   `result.path_names`, or from the `result.RGEN` / `RBUS` / `RCIR` DataFrames.
   `result.obj_MVA` is already a plain number — do not call `JuMP.value` on it.

## Conventions

- Mutating builders are `PascalCase_with_underscores!`; low-level creators are
  `lowercase_snake_case!`; variables are `snake_case`, Greek letters welcome.
- Model containers are `OrderedDict{Symbol, Any}` with `:vars`, `:eq_const`,
  `:ineq_const`, `:meta`, `:dual_registry`.
- Bounds are explicit `@constraint` rows in `≤` form rather than variable
  bounds, so `JuMP.dual()` is clean. That is `bound_encoding = CONSTRAINT`, the
  default; `VARIABLE` drops the rows and normalises variable-bound duals to the
  same exported column names.
- Never mutate the input DataFrames when applying a disturbance — `deepcopy`
  first. The builders rely on this.
