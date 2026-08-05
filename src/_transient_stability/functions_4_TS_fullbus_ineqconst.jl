#=
================================================================================
 functions_4_TS_fullbus_ineqconst.jl — FullBus TS inequality builders
================================================================================
=#

# ===================================================================================
# δ-COI stability bounds — `bound_style` dispatch (mirrors Kron intent)
# ===================================================================================

"""
Attach the δ-COI stability bounds for the fault-on window, dispatching on `bound_style`:

- `:coi_box` — a plain box on the COI-relative angle: δ_tol[1] ≤ δ_g − δ_COI ≤ δ_tol[2].
- otherwise  — the "modified"/swing-propagated bound, which additionally couples the angle
  band to the swing dynamics (P_mech, Pe, Δω, ω_syn, Δt) and the initial state (δ_0, Δω_0).

Both store the resulting lower/upper constraint families under the `*_tf_*` keys.
"""
function _add_fullbus_δ_COI_bounds_fault!(
    model::JuMP.Model,
    dyn_model_dict::OrderedDict{Symbol, Any},
    active_gen::Vector{Int64},
    DGEN_DYN::DataFrame,
    P_mech::OrderedDict{Int64, JuMP.VariableRef},
    δ::OrderedDict,
    δCOI::OrderedDict,
    Δω::OrderedDict,
    Pe::OrderedDict,
    time_window::Vector{Float64},
    δ_tol::Tuple{Float64, Float64},
    δ_0::OrderedDict{Int64, JuMP.VariableRef},
    Δω_0::Float64,
    P_g::OrderedDict{Int64, JuMP.VariableRef},
    ω_syn::Float64,
    Δt::Float64,
)
    bound_style = get(dyn_model_dict[:meta], :bound_style, :coi_box)
    build_lo, build_up = δ_COI_ineq_toggle_flags(dyn_model_dict, :tf)
    if build_lo || build_up
        if bound_style == :coi_box
            lower, upper = ineq_const_kron_δ_COI_generic!(
                model, active_gen, δ, δCOI, time_window, δ_tol;
                build_lower=build_lo, build_upper=build_up)
        else
            lower, upper = ineq_const_kron_δ_COI_generic_modified!(
                model, active_gen, DGEN_DYN, P_mech, δ, δCOI, Δω, Pe,
                time_window, δ_tol, δ_0, Δω_0, ω_syn, Δt;
                build_lower=build_lo, build_upper=build_up)
        end
        store_δ_COI_ineq!(dyn_model_dict, :tf, lower, upper)
    end
    return nothing
end

"""
Post-fault counterpart of `_add_fullbus_δ_COI_bounds_fault!`.

Same `bound_style` dispatch; the modified style is anchored on the last fault-on step
(`δ_ant`, `Δω_ant`, `Pe_ant`) instead of the pre-fault equilibrium. Stores the bounds
under the `*_tpf_*` keys.
"""
function _add_fullbus_δ_COI_bounds_postf!(
    model::JuMP.Model,
    dyn_model_dict::OrderedDict{Symbol, Any},
    active_gen::Vector{Int64},
    DGEN_DYN::DataFrame,
    P_mech::OrderedDict{Int64, JuMP.VariableRef},
    δ::OrderedDict,
    δCOI::OrderedDict,
    Δω::OrderedDict,
    Pe::OrderedDict,
    time_window::Vector{Float64},
    δ_tol::Tuple{Float64, Float64},
    δ_ant::OrderedDict{Int64, JuMP.VariableRef},
    Δω_ant::OrderedDict{Int64, JuMP.VariableRef},
    Pe_ant::OrderedDict{Int64, JuMP.VariableRef},
    ω_syn::Float64,
    Δt::Float64,
)
    bound_style = get(dyn_model_dict[:meta], :bound_style, :coi_box)
    build_lo, build_up = δ_COI_ineq_toggle_flags(dyn_model_dict, :tpf)
    if build_lo || build_up
        if bound_style == :coi_box
            lower, upper = ineq_const_kron_δ_COI_generic!(
                model, active_gen, δ, δCOI, time_window, δ_tol;
                build_lower=build_lo, build_upper=build_up)
        else
            lower, upper = ineq_const_kron_δ_COI_generic_modified!(
                model, active_gen, DGEN_DYN, P_mech, δ, δCOI, Δω, Pe,
                time_window, δ_tol, δ_ant, Δω_ant, Pe_ant, ω_syn, Δt;
                build_lower=build_lo, build_upper=build_up)
        end
        store_δ_COI_ineq!(dyn_model_dict, :tpf, lower, upper)
    end
    return nothing
end
"""Attach forced lower bounds on bus voltage magnitudes (FullBus windows)."""
function attach_fullbus_V_lower_bounds!(
    model::JuMP.Model,
    dyn_model_dict::OrderedDict{Symbol, Any},
    V_bus,
    V_min::Float64,
    manifest_key::Symbol,
)
    meta = dyn_model_dict[:meta]
    if bound_encoding_from_meta(meta) == VARIABLE
        for bus in keys(V_bus)
            for t in keys(V_bus[bus])
                JuMP.set_lower_bound(V_bus[bus][t], V_min)
            end
        end
        register_bound_manifest!(meta, manifest_key, :lower, V_bus, :per_bus_time)
        return nothing
    end
    dyn_model_dict[:ineq_const][manifest_key] =
        ineq_const_fullbus_V_lower!(model, V_bus, V_min)
    return nothing
end

"""Explicit ≤-form lower bounds on bus voltage magnitude: V ≥ min_lim."""
function ineq_const_fullbus_V_lower!(
    model::JuMP.Model,
    V_bus::OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}},
    min_lim::Float64=0.0,
)
    lower = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    for bus in keys(V_bus)
        lower[bus] = OrderedDict{Int, JuMP.ConstraintRef}()
        for t in keys(V_bus[bus])
            lower[bus][t] = JuMP.@constraint(model, min_lim - V_bus[bus][t] ≤ 0.0)
        end
    end
    return lower
end
