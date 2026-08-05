# `9bus_gfm_imax12` — mixed SG + GFM 9-bus, GFM Imax = 1.2

Steady-state data reproduced byte-for-byte from the reference implementation's
mixed-fleet case, so a cross-check between the two codes compares physics rather
than input drift:

| File | Source |
|------|--------|
| `bus_data.csv` | reference (SHA256 match) |
| `generators_data.csv` | reference (SHA256 match) |
| `line_data.csv` | reference (SHA256 match) |
| `gen_dynamic_data_reference.csv` | reference combined dyn CSV (kept for comparison only) |

Dynamic data is **split** for the TSCOPF module (same physics values as the reference):

| File | Content |
|------|---------|
| `gen_dynamic_data_full.csv` | SG rows only (gen ids 1–3, buses 1/2/3) |
| `gfm_dynamic_data.csv` | GFM rows (gen ids 4–6, buses 5/6/8), **Imax = 1.2** |
| `contingencies.csv` | Cont. 2 = bus 7 / circuit 6 (the reference default) |

```julia
RunConfig(
    case = "9bus_gfm_imax12",
    load_factor = 1.5,
    trans_stab = true,
    transient = TransientConfig(
        gen_dynamic_filename = "gen_dynamic_data_full.csv",
        gfm_dynamic_filename = "gfm_dynamic_data.csv",
        dyn_model = DynModelConfig(
            allow_gfm = true,
            gen_order = DQ_4TH,
            network_form = FULL_BUS,
            fault = FaultConfig(fault_type = SC, contingency_id = 2),
        ),
    ),
)
```
