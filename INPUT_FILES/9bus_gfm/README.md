# Mixed SG + GFM 9-bus case (derived from the reference implementation's mixed-fleet case).

Steady-state `generators_data.csv` lists all four units (buses 1, 2, 3, 5).

Dynamic parameters are **split**:
- `gen_dynamic_data_full.csv` — SG rows only (ids 1–3)
- `gfm_dynamic_data.csv` — GFM row only (id 4, bus 5)

The GFM unit carries `Imax = 1.2` pu on system base, so the current limiter is
built (the limiter is bypassed only at `Imax ≥ 20`, see `_GFM_IMAX_NO_LIMIT`).

Enable with:

```julia
RunConfig(
    case = "9bus_gfm",
    trans_stab = true,
    transient = TransientConfig(
        gen_dynamic_filename = "gen_dynamic_data_full.csv",
        gfm_dynamic_filename = "gfm_dynamic_data.csv",
        simulation = TsSimulationConfig(clearing_time = 0.15),
        dyn_model = DynModelConfig(
            allow_gfm = true,
            gen_order = DQ_4TH,
            network_form = FULL_BUS,
            mech_power_mode = USE_PM,
            bound_style_δ = :coi_box,
            fault = FaultConfig(fault_type = SC, contingency_id = 2),
        ),
    ),
)
```

Validated disturbance: **contingency 2** — three-phase short circuit at bus 7,
cleared after 150 ms by opening branch 5–7. This is the configuration used by
`test/runtests_gfm_transient.jl`.
