# Case data

Demo and case-study material: the networks bundled with the package, plus whatever you add for
your own runs. **This tree is yours to edit.** Change loads, add a case, retune machine data,
delete what you do not use — the test suite does not read any of it.

That was not always true. Until 2026-08, the same files carried the numeric pins the suite
asserts against (an objective at `rtol = 1e-6`, parsed generator costs, GFM current limits), so
adjusting a load to try something out could turn CI red for reasons unrelated to the code. The
frozen copies now live under **`test/INPUT_FILES/`** and are documented there.

## What is here

| Case | Contents |
|---|---|
| `9bus/` | IEEE 9-bus (WSCC). The package default (`RunConfig.case`), used by `main.jl`, `main_loop.jl`, and most of the documentation. Steady-state CSVs, `contingencies.csv`, classical and 4th-order dynamic data. |
| `39bus/` | IEEE 39-bus (New England). Steady-state plus classical dynamic data; no contingency file. |
| `9bus_gfm/` | Mixed fleet: 3 synchronous machines + 1 grid-forming converter at bus 5 (`Imax = 1.2` pu on system base). |
| `9bus_gfm_imax12/` | The three-converter fleet used by the GFM parity scripts under `scripts/`. |
| `9bus_reference_cost/` | Flat linear-cost 9-bus variant. |
| `9bus_test_mfile/` | A 9-bus supplied as a MATPOWER `.m` file rather than CSVs, for the `matpower_file` ingestion path. |
| `PowerModels/` | Raw MATPOWER and PGLib `.m` files for `Import_Matpower_Case` — see the README in that folder. |

Nested folders inside a case (`Linear/`, `NonLinear/`, `tests_june/`, …) are historical variants.
`Read_Input_Data` only reads the flat `<case>/*.csv`, so they are inert.

## Adding a case

Create `INPUT_FILES/<name>/` with `bus_data.csv`, `generators_data.csv` and `line_data.csv`
(`;`-delimited), then `RunConfig(case = "<name>")`. Transient runs additionally need a
`gen_dynamic_data*.csv` and, for `fault_type = SC`, a `contingencies.csv`; mixed-fleet runs need a
`gfm_dynamic_data.csv`. Column reference: `docs/parameter_reference.md`.

Case data does not have to live here at all — `load_system(cfg, path_main)` reads
`<path_main>/INPUT_FILES/<case>`, so any root with that one directory level works. The test suite
uses exactly that to read its own fixtures.
