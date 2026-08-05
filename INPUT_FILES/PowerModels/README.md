# MATPOWER / PGLib case files

Raw `.m` case files for the standalone MATPOWER parser (`Import_Matpower_Case`). They are
**not** in the in-house CSV schema — the parser converts them and writes the CSVs into the
run's `Inputs/` folder. Two provenances live here, and the distinction matters:

| Prefix | Source | Use |
|---|---|---|
| `case*.m` | MATPOWER distribution | Demo copies. The **pinned** copies live in `test/INPUT_FILES/PowerModels/`; these are yours to edit. |
| `pglib_case*.m` | [IEEE PES Power Grid Library](https://github.com/power-grid-lib/pglib-opf) v23.07, "Typical Operations" | Case-study material; nothing pins them. |

## Why the two `case39` files

The distinction still matters even though the tests now read their own copies, because it is what
makes the two files non-interchangeable.

`case39.m` (MATPOWER) is the file `test/runtests_matpower_parser.jl` asserts against — its uniform
quadratic gen costs (`0.01 / 0.30 / 0.20`) and its ±360° branch angle limits — and it is the driver
case for `test/runtests_powermodels_crosscheck.jl`, chosen there *because* its costs are strictly
convex, which makes the DC optimum unique and a per-quantity comparison against PowerModels
meaningful.

`pglib_case39.m` is the same network at a different operating point: costs are affine
(`c₂ = 0`, per-unit marginal costs 6.7–34.8 €/MWh) and branch angle limits are ±30°.
Copying it over `test/INPUT_FILES/PowerModels/case39.m` would break five assertions in the parser
test and remove the strict convexity the cross-check relies on. Keep them as separate files.

Editing `case39.m` *here* is now harmless to the suite — the tests read the frozen copy under
`test/INPUT_FILES/`. Re-pinning that copy is a deliberate act with its own protocol, documented in
`test/INPUT_FILES/README.md`.

## Adding more

Drop the file in, prefix it by provenance, and point `Import_Matpower_Case` at it. The larger
PGLib cases (118, 179, 300 buses) are useful for the steady-state paths; the transient paths
additionally need `gen_dynamic_data*.csv`, which these files do not carry.
