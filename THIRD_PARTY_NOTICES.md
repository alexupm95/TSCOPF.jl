# Third-party notices

The MIT licence in [`LICENSE`](LICENSE) covers the source code of this project.
Some bundled **data files** come from third parties and carry their own terms.
Those files, and their terms, are listed here.

## Network case files — `INPUT_FILES/PowerModels/`

### PGLib-OPF cases (`pglib_case*.m`)

From the [IEEE PES Power Grid Library — Optimal Power Flow](https://github.com/power-grid-lib/pglib-opf),
v23.07, "Typical Operations" benchmark group. Each file retains its original
header. The individual notices are:

| File | Copyright | Licence |
|---|---|---|
| `pglib_case14.m`, `pglib_case30.m`, `pglib_case118.m`, `pglib_case300.m` | © 1999 Richard D. Christie, University of Washington Electrical Engineering | CC BY 4.0 |
| `pglib_case24.m` | © 1979 The Institute of Electrical and Electronics Engineers (IEEE) | CC BY 4.0 |
| `pglib_case39.m` | © 1989 The Institute of Electrical and Electronics Engineers (IEEE) | CC BY 4.0 |
| `pglib_case89.m` | © 2015 Cédric Josz, Stéphane Fliscounakis, Jean Maeght (RTE France) | CC BY 4.0 |
| `pglib_case179.m` | Derived from an APRA-e Grid Optimization Competition model, itself based on EPRI-TR-104586 (December 1994) | Provided in the public domain (January 2018) |

Creative Commons Attribution 4.0 International:
<http://creativecommons.org/licenses/by/4.0/>

### MATPOWER cases

- `case39.m` — New England 39-bus case from the
  [MATPOWER](https://matpower.org/) distribution, which is released under the
  3-clause BSD licence. The file retains its original header and reference list.
- `case9.m` — **not** third-party. Written for this project as a MATPOWER-format
  mirror of `INPUT_FILES/9bus/*.csv`, so the PowerModels.jl cross-check solves
  exactly the same network as the in-house pipeline. Covered by `LICENSE`.

## Dynamic and case-study data — `INPUT_FILES/`

Machine dynamic data (`gen_dynamic_data*.csv`, `gfm_dynamic_data.csv`) and the
mixed synchronous + grid-forming cases under `9bus_gfm/` and `9bus_gfm_imax12/`
were assembled for this project, in part reproducing an independent in-house
reference implementation so that a cross-check compares physics rather than
input drift. They are covered by `LICENSE`.

The 9-bus network is the classic WSCC three-machine, nine-bus test system; the
39-bus network is the New England test system. Standard published parameters for
both appear in, among others:

- P. M. Anderson and A. A. Fouad, *Power System Control and Stability*.
- M. A. Pai, *Energy Function Analysis for Power System Stability*, Kluwer, 1989
  — pp. 229–233 give the 39-bus dynamic data.
- M. Pavella and P. G. Murthy, *Transient Stability of Power Systems: Theory and
  Practice* — WSCC 9-bus data.

## Funder logos — `utils/`

`logo_MICIU_AEI.png` and `logo_CM.png` are the institutional identifiers of the
Spanish Ministerio de Ciencia, Innovación y Universidades / Agencia Estatal de
Investigación (co-funded by the ERDF/EU) and of the Comunidad de Madrid. They
are reproduced solely to acknowledge the grants listed in `README.md`, as those
grants require. They are trademarks of their respective bodies and are **not**
covered by the MIT licence of this project; do not reuse them for any other
purpose.
