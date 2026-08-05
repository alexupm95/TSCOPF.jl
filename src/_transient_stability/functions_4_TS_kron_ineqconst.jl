#=
================================================================================
 functions_4_TS_kron_ineqconst.jl — Kron TS inequality builders
================================================================================
=#

# ===================================================================================
#                         Inequality Constraints (shared)
# ===================================================================================

function ineq_const_kron_Δω_COI_generic!(model::JuMP.Model,
    active_gen::Vector{Int64},
    Δω::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    ΔωCOI::OrderedDict{Int, JuMP.VariableRef},
    time_window::Vector{Float64},
    Δω_tol::Tuple{Float64, Float64}
    )

    ineq_const_Δω_COI_lower = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    ineq_const_Δω_COI_upper = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()

    for gen in active_gen
        ineq_const_Δω_COI_lower[gen] = OrderedDict{Int, JuMP.ConstraintRef}()
        ineq_const_Δω_COI_upper[gen] = OrderedDict{Int, JuMP.ConstraintRef}()
        for t in eachindex(time_window)
            ineq_const_Δω_COI_lower[gen][t] = JuMP.@constraint(model, Δω_tol[1] - (Δω[gen][t] - ΔωCOI[t]) <= 0.0)
            ineq_const_Δω_COI_upper[gen][t] = JuMP.@constraint(model, (Δω[gen][t] - ΔωCOI[t]) - Δω_tol[2] <= 0.0)
        end
    end

    return ineq_const_Δω_COI_lower, ineq_const_Δω_COI_upper
end

function ineq_const_kron_δ_COI_generic!(model::JuMP.Model,
    active_gen::Vector{Int64},
    δ::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    δCOI::OrderedDict{Int, JuMP.VariableRef},
    time_window::Vector{Float64},
    δ_tol::Tuple{Float64, Float64};
    build_lower::Bool=true,
    build_upper::Bool=true,
    )

    ineq_const_δ_COI_lower  = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    ineq_const_δ_COI_upper  = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()

    # Loop to create the variables
    for gen in active_gen
        # Initialize inner dict for this generator
        ineq_const_δ_COI_lower[gen]  = OrderedDict{Int, JuMP.ConstraintRef}()
        ineq_const_δ_COI_upper[gen]  = OrderedDict{Int, JuMP.ConstraintRef}()

        for t in eachindex(time_window)
            if build_lower
            ineq_const_δ_COI_lower[gen][t] = JuMP.@constraint(model, δ_tol[1] - (δ[gen][t] - δCOI[t]) <= 0.0)
            end
            if build_upper
            ineq_const_δ_COI_upper[gen][t] = JuMP.@constraint(model, (δ[gen][t] - δCOI[t]) - δ_tol[2] <= 0.0)
            end
        end
    end

    return ineq_const_δ_COI_lower, ineq_const_δ_COI_upper

end

# Fault
function ineq_const_kron_δ_COI_generic_modified!(model::JuMP.Model,
    active_gen::Vector{Int64},
    DGEN_DYN::DataFrame,
    Pg::OrderedDict{Int64, JuMP.VariableRef},
    δ::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    δCOI::OrderedDict{Int, JuMP.VariableRef},
    Δω::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    Pe::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    time_window::Vector{Float64},
    δ_tol::Tuple{Float64, Float64},
    δ_0::OrderedDict{Int64, JuMP.VariableRef},
    Δω_0::Float64,
    ω_syn::Float64,
    Δt::Float64;
    build_lower::Bool=true,
    build_upper::Bool=true,
    )

    ineq_const_δ_COI_lower  = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    ineq_const_δ_COI_upper  = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()

    for gen in active_gen
        H = DGEN_DYN.H[gen]
        D = DGEN_DYN.D[gen]

        ineq_const_δ_COI_lower[gen]  = OrderedDict{Int, JuMP.ConstraintRef}()
        ineq_const_δ_COI_upper[gen]  = OrderedDict{Int, JuMP.ConstraintRef}()

        for t in eachindex(time_window)
            if t == 1
                if build_lower
                ineq_const_δ_COI_lower[gen][t] = JuMP.@constraint(model,
                    - δ_0[gen]
                    - ((4*H*ω_syn*Δt)/(4*H + D*Δt))*Δω_0
                    - ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*Pg[gen]
                    + ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*Pe[gen][t]
                    + δCOI[t] + δ_tol[1]
                    ≤ 0.0
                )
                end

                if build_upper
                ineq_const_δ_COI_upper[gen][t] = JuMP.@constraint(model,
                    δ_0[gen]
                    + ((4*H*ω_syn*Δt)/(4*H + D*Δt))*Δω_0
                    + ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*Pg[gen]
                    - ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*Pe[gen][t]
                    - δCOI[t] - δ_tol[2]
                    ≤ 0.0
                )
                end
            else
                if build_lower
                ineq_const_δ_COI_lower[gen][t] = JuMP.@constraint(model,
                    - δ[gen][t-1]
                    - ((4*H*ω_syn*Δt)/(4*H + D*Δt))*Δω[gen][t-1]
                    - ((ω_syn*(Δt^2))/(4*H + D*Δt))*Pg[gen]
                    + ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*(Pe[gen][t] + Pe[gen][t-1])
                    + δCOI[t] + δ_tol[1]
                    ≤ 0.0
                )
                end

                if build_upper
                ineq_const_δ_COI_upper[gen][t] = JuMP.@constraint(model,
                    δ[gen][t-1]
                    + ((4*H*ω_syn*Δt)/(4*H + D*Δt))*Δω[gen][t-1]
                    + ((ω_syn*(Δt^2))/(4*H + D*Δt))*Pg[gen]
                    - ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*(Pe[gen][t] + Pe[gen][t-1])
                    - δCOI[t] - δ_tol[2]
                    ≤ 0.0
                )
                end
            end
        end
    end

    return ineq_const_δ_COI_lower, ineq_const_δ_COI_upper

end

# Post-Fault
function ineq_const_kron_δ_COI_generic_modified!(model::JuMP.Model,
    active_gen::Vector{Int64},
    DGEN_DYN::DataFrame,
    Pg::OrderedDict{Int64, JuMP.VariableRef},
    δ::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    δCOI::OrderedDict{Int, JuMP.VariableRef},
    Δω::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    Pe::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    time_window::Vector{Float64},
    δ_tol::Tuple{Float64, Float64},
    δ_0::OrderedDict{Int64, JuMP.VariableRef},
    Δω_0::OrderedDict{Int64, JuMP.VariableRef},
    Pe_0::OrderedDict{Int64, JuMP.VariableRef},
    ω_syn::Float64,
    Δt::Float64;
    build_lower::Bool=true,
    build_upper::Bool=true,
    )

    ineq_const_δ_COI_lower  = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    ineq_const_δ_COI_upper  = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()

    for gen in active_gen
        H = DGEN_DYN.H[gen]
        D = DGEN_DYN.D[gen]

        ineq_const_δ_COI_lower[gen]  = OrderedDict{Int, JuMP.ConstraintRef}()
        ineq_const_δ_COI_upper[gen]  = OrderedDict{Int, JuMP.ConstraintRef}()

        for t in eachindex(time_window)
            if t == 1
                if build_lower
                ineq_const_δ_COI_lower[gen][t] = JuMP.@constraint(model,
                    - δ_0[gen]
                    - ((4*H*ω_syn*Δt)/(4*H + D*Δt))*Δω_0[gen]
                    - ((ω_syn*(Δt^2))/(4*H + D*Δt))*Pg[gen]
                    + ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*(Pe[gen][t] + Pe_0[gen])
                    + δCOI[t] + δ_tol[1]
                    ≤ 0.0
                )
                end

                if build_upper
                ineq_const_δ_COI_upper[gen][t] = JuMP.@constraint(model,
                    δ_0[gen]
                    + ((4*H*ω_syn*Δt)/(4*H + D*Δt))*Δω_0[gen]
                    + ((ω_syn*(Δt^2))/(4*H + D*Δt))*Pg[gen]
                    - ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*(Pe[gen][t] + Pe_0[gen])
                    - δCOI[t] - δ_tol[2]
                    ≤ 0.0
                )
                end
            else
                if build_lower
                ineq_const_δ_COI_lower[gen][t] = JuMP.@constraint(model,
                    - δ[gen][t-1]
                    - ((4*H*ω_syn*Δt)/(4*H + D*Δt))*Δω[gen][t-1]
                    - ((ω_syn*(Δt^2))/(4*H + D*Δt))*Pg[gen]
                    + ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*(Pe[gen][t] + Pe[gen][t-1])
                    + δCOI[t] + δ_tol[1]
                    ≤ 0.0
                )
                end

                if build_upper
                ineq_const_δ_COI_upper[gen][t] = JuMP.@constraint(model,
                    δ[gen][t-1]
                    + ((4*H*ω_syn*Δt)/(4*H + D*Δt))*Δω[gen][t-1]
                    + ((ω_syn*(Δt^2))/(4*H + D*Δt))*Pg[gen]
                    - ((ω_syn*(Δt^2))/(2*(4*H + D*Δt)))*(Pe[gen][t] + Pe[gen][t-1])
                    - δCOI[t] - δ_tol[2]
                    ≤ 0.0
                )
                end
            end
        end
    end

    return ineq_const_δ_COI_lower, ineq_const_δ_COI_upper

end

"""Read `TsBuilderConfig` δ-COI ineq toggles stored on `dyn_model_dict[:meta][:ineq_cons]`."""
function δ_COI_ineq_toggle_flags(
    dyn_model_dict::OrderedDict{Symbol, Any},
    window::Symbol,
)::Tuple{Bool, Bool}
    ineq_cons = get(dyn_model_dict[:meta], :ineq_cons, nothing)
    suffix = window == :tf ? "tf" : "tpf"
    key_lo = Symbol("ineq_const_δ_COI_$(suffix)_lower")
    key_up = Symbol("ineq_const_δ_COI_$(suffix)_upper")
    if ineq_cons === nothing
        return true, true
    end
    return get(ineq_cons, key_lo, true), get(ineq_cons, key_up, true)
end

"""Store δ-COI ineq families under the fault (`:tf`) or post-fault (`:tpf`) keys."""
function store_δ_COI_ineq!(
    dyn_model_dict::OrderedDict{Symbol, Any},
    window::Symbol,
    lower::OrderedDict,
    upper::OrderedDict,
)
    suffix = window == :tf ? "tf" : "tpf"
    build_lo, build_up = δ_COI_ineq_toggle_flags(dyn_model_dict, window)
    build_lo && (dyn_model_dict[:ineq_const][Symbol("ineq_const_δ_COI_$(suffix)_lower")] = lower)
    build_up && (dyn_model_dict[:ineq_const][Symbol("ineq_const_δ_COI_$(suffix)_upper")] = upper)
    return nothing
end

# ===================================================================================
# δ-COI stability bounds — `bound_style` dispatch (shared by every network form)
# ===================================================================================
# Both flavours are built from the same two families above, so the choice belongs in
# one place rather than in each builder. Kron, Kron-linear, classical FULL_BUS and DQ
# FULL_BUS all route through these two entry points; a run that declares `:coi_box`
# now gets the box on every path.

"""
Attach the δ-COI stability bounds for the fault-on window, dispatching on `bound_style`:

- `:coi_box` — a plain box on the COI-relative angle: δ_tol[1] ≤ δ_g − δ_COI ≤ δ_tol[2].
- otherwise  — the "modified"/swing-propagated bound, which additionally couples the angle
  band to the swing dynamics (P_mech, Pe, Δω, ω_syn, Δt) and the initial state (δ_0, Δω_0).

Both store the resulting lower/upper constraint families under the `*_tf_*` keys.
"""
function _add_δ_COI_bounds_fault!(
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
Post-fault counterpart of `_add_δ_COI_bounds_fault!`.

Same `bound_style` dispatch; the modified style is anchored on the last fault-on step
(`δ_ant`, `Δω_ant`, `Pe_ant`) instead of the pre-fault equilibrium. Stores the bounds
under the `*_tpf_*` keys.
"""
function _add_δ_COI_bounds_postf!(
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
