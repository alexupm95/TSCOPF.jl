#=
================================================================================
 DynModelConfig.jl  —  runtime selection of dynamic-model complexity
================================================================================
 Phase 0 (migration scaffolding). Defaults reproduce the current integrated
 behaviour: 2nd-order classical generator, Kron-reduced network, P_g in swing.

 Phase 1.1 wires `mech_power_mode` into Kron builders; Phase 1.2 routes builders
 via `dynamic_gen_model` in `AbstractDynamicGenModel.jl`.  Validation runs in
 `run_case!` so invalid combinations fail fast.
================================================================================
=#

# --- enumerations -------------------------------------------------------------

"""
    GenOrder

Generator swing-equation order. `CLASSICAL_2ND` (rotor angle + speed
deviation) is implemented on Kron and FULL_BUS paths. `DQ_4TH` (dq-axis,
4th order, FULL_BUS only) implements the machine core with optional AVR and
optional TGOV1 governor on the DQ path.
"""
@enum GenOrder       CLASSICAL_2ND DQ_4TH

"""
    NetworkForm

Transient network representation. `KRON_REDUCED` eliminates non-generator
buses via Kron reduction; `FULL_BUS` keeps the full sparse nodal `Ybus` with
explicit bus voltage/angle states and requires a mandatory ACOPF warm start.
"""
@enum NetworkForm    KRON_REDUCED FULL_BUS

"""
    MechPowerMode

Mechanical power used in the swing equation. `USE_PG` (default) uses the
dispatch decision `P_g` directly; `USE_PM` introduces an explicit `P_m`
variable and requires `bound_style = :coi_box`. `FULL_BUS` requires `USE_PM`.
"""
@enum MechPowerMode  USE_PG USE_PM

"""
    GovernorLimiter

Valve/gate saturation treatment for the optional turbine governor
(`include_governor`). `GOV_NO_LIMIT` leaves the valve state unbounded (linear
governor, cleanest duals); `GOV_SMOOTH` applies the reference sqrt-based smooth
min/max anti-windup clamp (physically saturating, nonconvex); `GOV_HARD_BOUND`
adds explicit ≤-form bounds on the valve state (clean bound duals but can render
a strong transient infeasible). Only consulted when `include_governor = true`.
"""
@enum GovernorLimiter  GOV_NO_LIMIT GOV_SMOOTH GOV_HARD_BOUND

# --- configuration struct -----------------------------------------------------

"""
    DynModelConfig

Physics, network form, and disturbance selection for a transient-stability
run (`RunConfig.transient.dyn_model`).

Core knobs: `gen_order` (machine model), `network_form` (Kron-reduced vs
full-bus nodal), `mech_power_mode` (`P_g` vs explicit `P_m` in the swing
equation), `zip_load_p` / `zip_load_q` (independent constant-Z/I/P demand
splits for `FULL_BUS`, each `(Z, I, P)` and each summing to 1),
`bound_style` (`:swing_propagated` or `:coi_box`), and `fault::FaultConfig`.

The two ZIP vectors are independent so the active and reactive demand can follow
different TSO conventions — e.g. REE's constant-current active / constant-admittance
reactive is `zip_load_p = (0.0, 1.0, 0.0)` with `zip_load_q = (1.0, 0.0, 0.0)`.

Optional COI-referenced speed box: `constrain_Δω_COI` enables it, `Δω_tol_pu` sets a
symmetric half-width, and `Δω_tol_pu_lower` / `Δω_tol_pu_upper` set independent
below/above limits (positive magnitudes, p.u.).

Validation rules enforced in `validate_dyn_config!`: `USE_PM` requires
`bound_style = :coi_box`; `FULL_BUS` requires `USE_PM` plus a solved ACOPF
warm start; TSC-DCOPF + `FULL_BUS` is not implemented.

See the user guide, section 4.
"""
Base.@kwdef struct DynModelConfig
    # Machine / network complexity (Phase 1+ will dispatch on these)
    gen_order::GenOrder           = CLASSICAL_2ND
    network_form::NetworkForm     = KRON_REDUCED
    mech_power_mode::MechPowerMode = USE_PG

    # DQ_4TH only: include (1+Δω) in stator/Pe algebraic equations (RMS-style); false neglects it.
    dq_speed_dev_in_algebra::Bool = true

    # Optional control layers (dq full-network only, Phase 5–6)
    include_avr::Bool             = false
    include_governor::Bool        = false
    # Valve saturation treatment for the turbine governor (only read when include_governor).
    governor_limiter::GovernorLimiter = GOV_NO_LIMIT

    # Mixed SG + GFM fleet (FULL_BUS + DQ_4TH). GFM params live in a separate
    # `gfm_dynamic_data.csv` (see TransientConfig.gfm_dynamic_filename), not in
    # gen_dynamic_data*.csv. Phase G0 loads/validates; transient physics is G1+.
    allow_gfm::Bool               = false

    # First ODE step of each fault/post-fault window for FULL_BUS swing (δ/Δω),
    # EMF, AVR, and governor. `:trapezoidal` (default) matches the pinned SG
    # AVR+TG FULL_BUS path; `:backward_euler` matches the reference implementation
    # BE-at-t=1 (optional for investigation). Remainder steps stay trapezoidal.
    ode_first_step::Symbol        = :trapezoidal

    # Time discretisation of the GFM measurement filters (P/Q/V) and the Q–V PI
    # integrator. Scoped to those two families only: GFM δ and every SG family keep
    # following `ode_first_step` above.
    #   :backward_euler        — BE at every step (DEFAULT). Matches the reference
    #                            implementation, and is the only scheme that stays
    #                            affordable as the horizon grows (see below).
    #   :follow_ode_first_step — BE at t=1 of each window iff `ode_first_step` is
    #                            `:backward_euler`, trapezoidal otherwise and always
    #                            for t ≥ 2 — i.e. the same rule the SG families use.
    #   :trapezoidal           — trapezoidal at every step.
    #
    # Why BE is the default despite trapezoidal being second-order accurate: the
    # trapezoidal row carries `u_{t-1}` (Pe/Qe, nonlinear in V, θ, δ, Id, Iq) into every
    # filter equation, and the resulting Jacobian/Hessian fill-in compounds with horizon
    # length. Measured on 9bus_gfm, SC at bus 7 cleared in 150 ms, t_step = 0.01 s:
    #
    #   horizon   :backward_euler        :follow_ode_first_step
    #   0.6 s     139 it                 97 it,  0.074 s/it
    #   3.0 s     205 it, 0.198 s/it     161 it, 17.6  s/it   (≈70× total solve time)
    #
    # Accuracy cost of the default is small: worst rotor-angle gap between the two was
    # 2.5° absolute / 1.8° relative to COI over 3 s (≈2 % of swing), objective 1.6e-5
    # relative. Use the trapezoidal variants for accuracy studies on short horizons.
    #
    # Stability note: BE is L-stable and never rings; trapezoidal oscillates
    # step-to-step once `t_step > 2·Tf` (warned in the builder).
    gfm_integrator::Symbol        = :backward_euler

    # ZIP load model for the FULL_BUS KCL path. Active and reactive demand carry
    # INDEPENDENT splits, because TSOs do not generally use the same one for both
    # (e.g. REE recommends constant-current P with constant-admittance Q).
    # Coefficient order is (Z, I, P) in BOTH vectors:
    #   [1] = constant impedance (∝ V_t²), [2] = constant current (∝ V_t·V_nom),
    #   [3] = constant power (∝ V_nom²).  Each vector must sum to 1 on its own.
    # Ignored on KRON_REDUCED, where loads are folded into Y_red as constant admittance.
    # WARNING — the reference implementation stores the vector as (P, I, Z) in the KCL
    # formula (`ZIP[1]*V² + ZIP[2]*V_t*V + ZIP[3]*V_t²`). Its `[0,0,1]` is therefore
    # constant-Z, which maps to package `(1,0,0)` — NOT package `(0,0,1)` (const-P).
    zip_load_p::NTuple{3, Float64} = (1.0, 0.0, 0.0)  # active-demand split (Z, I, P)
    zip_load_q::NTuple{3, Float64} = (1.0, 0.0, 0.0)  # reactive-demand split (Z, I, P)

    # Stability bound style: DQ_4TH requires :coi_box (enforced in validate_dyn_config!)
    bound_style::Symbol           = :swing_propagated
    # Optional box bounds on Δω_i − Δω_COI (off by default). `Δω_tol_pu` is the
    # symmetric half-width; the two overrides set independent below/above limits as
    # positive magnitudes, mirroring δ_tol_deg / δ_tol_deg_lower / δ_tol_deg_upper.
    # e.g. lower=0.05, upper=0.03 → the signed pair (-0.05, +0.03).
    constrain_Δω_COI::Bool      = false
    Δω_tol_pu::Float64            = 0.5
    Δω_tol_pu_lower::Union{Nothing, Float64} = nothing  # below COI [p.u.]; default → Δω_tol_pu
    Δω_tol_pu_upper::Union{Nothing, Float64} = nothing  # above COI [p.u.]; default → Δω_tol_pu

    # Disturbance specification (SC via contingencies.csv; GL / OB via explicit ids)
    fault::FaultConfig            = FaultConfig()
end

# --- FULL_BUS coupling hints (ACOPF warm start or synthetic flat start) -------

"""
    SteadyStateHints

Operating-point snapshot used to set `start=` on FULL_BUS dynamic variables.
Populated by `extract_opf_solved_hints` after an ACOPF pre-solve, or by
`build_flat_start_hints` when `RunConfig.use_acopf_warmstart=false`.
"""
struct SteadyStateHints
    val_V::Dict{Int, Float64}
    val_θ::Dict{Int, Float64}
    val_Pg::Dict{Int, Float64}
    val_Qg::Dict{Int, Float64}
end

"""Extract `JuMP.value` on steady-state V, θ, P_g, Q_g after the warm-start solve."""
function extract_opf_solved_hints(opf_dict::OrderedDict{Symbol, Any})::SteadyStateHints
    return SteadyStateHints(
        Dict(i => Float64(JuMP.value(v)) for (i, v) in opf_dict[:vars][:V]),
        Dict(i => Float64(JuMP.value(v)) for (i, v) in opf_dict[:vars][:θ]),
        Dict(i => Float64(JuMP.value(v)) for (i, v) in opf_dict[:vars][:P_g]),
        Dict(i => Float64(JuMP.value(v)) for (i, v) in opf_dict[:vars][:Q_g]),
    )
end

# --- validation ---------------------------------------------------------------

"""Return `true` when every optional machine column needed by `dyn` is finite."""
function has_required_dyn_columns(dyn::DynModelConfig, DGEN_DYN::DataFrame)::Bool
    req = required_dyn_column_names(dyn)
    return all(col -> col in propertynames(DGEN_DYN) && all(isfinite, DGEN_DYN[!, col]), req)
end

"""Canonical DataFrame column names required for the chosen dynamic configuration."""
function required_dyn_column_names(dyn::DynModelConfig)::Vector{Symbol}
    # E_int is never read from CSV — internal voltage E is a JuMP variable from init.
    cols = Symbol[:bus, :Xd_tr, :H, :D]
    if dyn.gen_order == DQ_4TH
        append!(cols, [:Xq_tr, :Xd, :Xq, :Td, :Tq, :Ra])
    end
    if dyn.include_avr
        append!(cols, [:T_exc, :K_exc])
    end
    if dyn.include_governor
        append!(cols, [:R, :T1, :T2, :T3])
    end
    return unique(cols)
end

# validate_dyn_config!(cfg::RunConfig) lives in engine.jl (after RunConfig is defined).

"""Symmetric speed-deviation tolerance tuple `(−Δω_tol_pu, +Δω_tol_pu)` in p.u."""
function Δω_tol_tuple(Δω_tol_pu::Float64)::Tuple{Float64, Float64}
    return (-Δω_tol_pu, Δω_tol_pu)
end

"""
    Δω_tol_tuple(dyn::DynModelConfig) -> (lower, upper)

Signed speed-deviation tolerance pair in p.u., resolved from `Δω_tol_pu` and the optional
`Δω_tol_pu_lower` / `Δω_tol_pu_upper` overrides. Both overrides are **positive magnitudes**,
so `lower = 0.05`, `upper = 0.03` yields `(-0.05, 0.03)` — the same convention as
`δ_tol_deg_lower` / `δ_tol_deg_upper` in `common_ts_parameters`.
"""
function Δω_tol_tuple(dyn::DynModelConfig)::Tuple{Float64, Float64}
    lo = something(dyn.Δω_tol_pu_lower, dyn.Δω_tol_pu)
    hi = something(dyn.Δω_tol_pu_upper, dyn.Δω_tol_pu)
    lo > 0 || throw(ArgumentError("Δω_tol_pu_lower / Δω_tol_pu must be positive."))
    hi > 0 || throw(ArgumentError("Δω_tol_pu_upper / Δω_tol_pu must be positive."))
    return (-lo, hi)
end

"""
    mechanical_power_for_swing(mech_power_mode, P_g, dyn_model_dict)

JuMP variables used as mechanical power in swing and propagated COI bounds.
`USE_PG` → dispatch `P_g`; `USE_PM` → explicit `P_m` (requires prior init).
"""
function mechanical_power_for_swing(
    mech_power_mode::MechPowerMode,
    P_g::OrderedDict{Int, JuMP.VariableRef},
    dyn_model_dict::OrderedDict{Symbol, Any},
)::OrderedDict{Int, JuMP.VariableRef}
    if mech_power_mode == USE_PG
        return P_g
    else
        haskey(dyn_model_dict, :vars) && haskey(dyn_model_dict[:vars], :P_m) ||
            throw(ArgumentError("mech_power_mode=USE_PM but :P_m is not in dyn_model_dict[:vars]."))
        return dyn_model_dict[:vars][:P_m]
    end
end

"""Store `P_g` and swing mechanical-power refs after pre-fault init."""
function register_mech_power_refs!(
    dyn_model_dict::OrderedDict{Symbol, Any},
    P_g::OrderedDict{Int, JuMP.VariableRef},
    mech_power_mode::MechPowerMode,
)
    dyn_model_dict[:refs] = OrderedDict{Symbol, Any}(
        :P_g    => P_g,
        :P_mech => mechanical_power_for_swing(mech_power_mode, P_g, dyn_model_dict),
    )
    dyn_model_dict[:mech_power_mode] = mech_power_mode
    return dyn_model_dict
end
