#=
================================================================================
 functions_4_TS_avr.jl — optional first-order AVR (exciter) for DQ_4TH
================================================================================
 Voltage regulator for the 4th-order dq machine.  The exciter sees the generator
 terminal voltage `V` and drives the field voltage `E_fd` that feeds the Eq EMF
 ODE.  Wired into the DQ FULL_BUS path only (`include_avr=true`).

 AVR model (reference implementation) — first-order lag K_exc/(1 + T_exc·s), NOT an
 integrator; the −E_fd_unlim self-decay term is what makes the pre-fault link
 below the steady state of the same ODE:

     Pre-fault:  E_fd = K_exc · (V_ref − V_terminal)
     Transient:  T_exc · d(E_fd_unlim)/dt = K_exc · (V_ref − V_terminal) − E_fd_unlim
                 E_fd = smooth clamp(E_fd_unlim; E_min, E_max)

 Discretization is trapezoidal (module convention).  The t=1 step of each window
 is anchored to the previous-window **saturated** E_fd (pre-fault scalar for the
 fault window; last fault-on E_fd_tf for post-fault).
================================================================================
=#

# Smoothing parameter for the sqrt min/max (matches governor `_GOV_SMOOTH_RHO`).
const _AVR_SMOOTH_RHO = 1.0e-4

"""Index `x[gen]` when `x` is a per-generator dict, else return the scalar `x`."""
_avr_at(x::AbstractDict, gen::Int) = x[gen]
_avr_at(x, ::Int) = x

_avr_start(v::JuMP.VariableRef)::Float64 = (s = JuMP.start_value(v); isnothing(s) ? 0.0 : Float64(s))

# ===================================================================================
# Variables
# ===================================================================================

"""Constant voltage set-point `V_ref[g]` (one scalar per active SG).

Warm-started at `val_V[bus] + E_fd_start/K_exc` (reference AVR), using the
ACOPF warm-start bus voltage dictionary — not `JuMP.start_value(V)`, which is
still ~1.0 when AVR init runs (before OPF re-stamp). Falls back to
`start_value(V)` only if `val_V` lacks that bus.
"""
function var_avr_V_ref!(
    model::JuMP.Model,
    active_gen::Vector{Int64},
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    V::OrderedDict{Int, JuMP.VariableRef},
    E_fd::OrderedDict{Int, JuMP.VariableRef},
    val_V::Dict,
)
    V_ref = OrderedDict{Int, JuMP.VariableRef}()
    for gen in active_gen
        bus = DGEN.bus[gen]
        K_exc = DGEN_DYN.K_exc[gen]
        start_Efd = _avr_start(E_fd[gen])
        v_seed = haskey(val_V, bus) ? Float64(val_V[bus]) : _avr_start(V[bus])
        V_ref[gen] = JuMP.@variable(model, base_name = "V_ref[$gen]")
        JuMP.set_start_value(V_ref[gen], v_seed + start_Efd / K_exc)
    end
    return V_ref
end

"""Time-indexed pre-saturation field voltage `E_fd_unlim`, warm-started at pre-fault `E_fd`."""
function var_avr_E_fd_unlim_time!(
    model::JuMP.Model,
    active_gen::Vector{Int64},
    var_name::String,
    time_window::Vector{Float64},
    E_fd::OrderedDict{Int, JuMP.VariableRef},
)
    E_fd_unlim = var_kron_gen_time_generic!(model, active_gen, var_name, time_window)
    for gen in active_gen
        seed = _avr_start(E_fd[gen])
        for t in eachindex(time_window)
            JuMP.set_start_value(E_fd_unlim[gen][t], seed)
        end
    end
    return E_fd_unlim
end

# ===================================================================================
# Pre-fault link: E_fd = K_exc · (V_ref − V_terminal)
# ===================================================================================

"""Pin pre-fault field voltage to the exciter steady-state gain."""
function eq_const_avr_Efd_init!(
    model::JuMP.Model,
    active_gen::Vector{Int64},
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    V::OrderedDict{Int, JuMP.VariableRef},
    V_ref::OrderedDict{Int, JuMP.VariableRef},
    E_fd::OrderedDict{Int, JuMP.VariableRef},
)
    eq = OrderedDict{Int, JuMP.ConstraintRef}()
    for gen in active_gen
        bus = DGEN.bus[gen]
        K_exc = DGEN_DYN.K_exc[gen]
        eq[gen] = JuMP.@constraint(model, E_fd[gen] - K_exc * (V_ref[gen] - V[bus]) == 0.0)
    end
    return eq
end

# ===================================================================================
# Exciter ODE (first step: trap or BE via ode_first_step; trap thereafter)
# ===================================================================================

"""
Exciter update for one window (fault or post-fault).

First step uses `ode_first_step` (`:trapezoidal` default, or `:backward_euler`);
remaining steps use trapezoidal. The ODE integrates `E_fd_unlim`.  The
previous-step anchor uses the **saturated** output `E_fd_sat` (the reference
convention), not the raw unlimited state.

`V_terminal` is the per-step generator-bus voltage from `V_tf` / `V_tpf`.
`E_fd_prev0` / `V_prev0` supply the t=1 anchor: fault window → pre-fault
`(E_fd, V)`; post-fault → last fault-on `(E_fd_tf, V_tf)`.
"""
function eq_const_avr_exciter!(
    model::JuMP.Model,
    active_gen::Vector{Int64},
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    E_fd_unlim::OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}},
    E_fd_sat::OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}},
    V_ref::OrderedDict{Int, JuMP.VariableRef},
    V_terminal::OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}},
    E_fd_prev0::OrderedDict{Int, JuMP.VariableRef},
    V_prev0,
    time_window::Vector{Float64},
    Δt::Float64;
    ode_first_step::Symbol=:trapezoidal,
)
    eq = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    for gen in active_gen
        bus = DGEN.bus[gen]
        K_exc = DGEN_DYN.K_exc[gen]
        T_exc = DGEN_DYN.T_exc[gen]
        c = Δt / (2 * T_exc)
        eq[gen] = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
        for t in eachindex(time_window)
            E_fd_prev = t == 1 ? E_fd_prev0[gen] : E_fd_sat[gen][t - 1]
            V_curr = V_terminal[bus][t]
            if t == 1 && ode_first_step === :backward_euler
                # --- BACKWARD EULER FOR STEP 1 (reference GFM path) ---
                eq[gen][t] = JuMP.@constraint(model,
                    E_fd_unlim[gen][t] * (1 + Δt / T_exc) - E_fd_prev
                    - ((K_exc*Δt) / T_exc) * (V_ref[gen] - V_curr) == 0.0)
            elseif t == 1
                # --- TRAPEZOIDAL AT t=1 (SG pin default) ---
                V_prev = _avr_at(V_prev0, bus)
                eq[gen][t] = JuMP.@constraint(model,
                    E_fd_unlim[gen][t] * (1 + c) - E_fd_prev * (1 - c)
                    - (K_exc * c) * (2 * V_ref[gen] - V_curr - V_prev) == 0.0)
            else
                # --- TRAPEZOIDAL FOR REMAINDER ---
                V_prev = V_terminal[bus][t - 1]
                eq[gen][t] = JuMP.@constraint(model,
                    E_fd_unlim[gen][t] * (1 + c) - E_fd_prev * (1 - c)
                    - (K_exc * c) * (2 * V_ref[gen] - V_curr - V_prev) == 0.0)
            end
        end
    end
    return eq
end

# ===================================================================================
# Field-voltage saturation (smooth sqrt min/max)
# ===================================================================================

"""Per-generator field-voltage `(E_min, E_max)` from resolved `var_limit_specs[:E]`."""
function _avr_field_limit_dicts(
    active_gen::Vector{Int64},
    specs::OrderedDict{Symbol, Tuple{Any, Any}},
)::Tuple{Dict{Int, Float64}, Dict{Int, Float64}}
    emin, emax = specs[:E]
    e_min = Dict{Int, Float64}()
    e_max = Dict{Int, Float64}()
    for (idx, gen) in enumerate(active_gen)
        e_min[gen] = emin isa AbstractVector ? emin[idx] : Float64(emin)
        e_max[gen] = emax isa AbstractVector ? emax[idx] : Float64(emax)
    end
    return e_min, e_max
end

"""
Smooth anti-windup: `E_fd = smoothmax(smoothmin(E_fd_unlim, E_max), E_min)` via sqrt.

Introduces a separate limited-output variable `E_fd` tied to the raw state
`E_fd_unlim` by one equality per (gen, t).  Physically saturating but nonconvex.
"""
function apply_avr_field_limit_smooth!(
    model::JuMP.Model,
    active_gen::Vector{Int64},
    E_fd_unlim::OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}},
    time_window::Vector{Float64},
    e_min::Dict{Int, Float64},
    e_max::Dict{Int, Float64};
    out_name::String="E_fd",
)
    ρ = _AVR_SMOOTH_RHO
    E_fd = OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}}()
    eq = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    for gen in active_gen
        Emax = e_max[gen]
        Emin = e_min[gen]
        E_fd[gen] = OrderedDict{Int, JuMP.VariableRef}()
        eq[gen] = OrderedDict{Int, JuMP.ConstraintRef}()
        for t in eachindex(time_window)
            # Window-specific name (E_fd_tf / E_fd_tpf) so dumps do not collide.
            E_fd[gen][t] = JuMP.@variable(model, base_name = "$out_name[$gen, $t]",
                start = _avr_start(E_fd_unlim[gen][t]))
            u = E_fd_unlim[gen][t]
            E_aux = JuMP.@expression(model, (u + Emax - sqrt((u - Emax)^2 + ρ)) / 2)
            eq[gen][t] = JuMP.@constraint(model,
                E_fd[gen][t] == (E_aux + Emin + sqrt((E_aux - Emin)^2 + ρ)) / 2)
        end
    end
    return E_fd, eq
end

# ===================================================================================
# Orchestrators — called from the DQ FULL_BUS builder
# ===================================================================================

"""
Attach AVR pre-fault variables and the steady-state exciter link.

Call from `Define_Initial_Condition_4_dq!` when `include_avr=true`.
`val_V` seeds `V_ref` the reference way: `V_ref ← val_V[bus] + E_fd/K_exc`.
"""
function Attach_Avr_init!(
    model::JuMP.Model,
    dyn_model_dict::OrderedDict{Symbol, Any},
    active_gen::Vector{Int64},
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    V::OrderedDict{Int, JuMP.VariableRef},
    val_V::Dict,
)
    E_fd = dyn_model_dict[:vars][:E_fd]
    V_ref = var_avr_V_ref!(model, active_gen, DGEN, DGEN_DYN, V, E_fd, val_V)
    dyn_model_dict[:vars][:V_ref] = V_ref
    dyn_model_dict[:eq_const][:eq_const_Efd_init] =
        eq_const_avr_Efd_init!(model, active_gen, DGEN, DGEN_DYN, V, V_ref, E_fd)
    return nothing
end

"""
Attach the exciter to the **fault-on** window; returns saturated time-varying `E_fd_tf`.

The t=1 anchor uses pre-fault `(E_fd, V_terminal)`.
"""
function Attach_Avr_fault!(
    model::JuMP.Model,
    dyn_model_dict::OrderedDict{Symbol, Any},
    active_gen::Vector{Int64},
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    V::OrderedDict{Int, JuMP.VariableRef},
    V_tf::OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}},
    time_window::Vector{Float64},
    Δt::Float64,
)
    E_fd = dyn_model_dict[:vars][:E_fd]
    V_ref = dyn_model_dict[:vars][:V_ref]
    specs = dyn_model_dict[:meta][:var_limit_specs]
    e_min, e_max = _avr_field_limit_dicts(active_gen, specs)

    E_fd_unlim_tf = var_avr_E_fd_unlim_time!(model, active_gen, "E_fd_unlim_tf", time_window, E_fd)
    dyn_model_dict[:vars][:E_fd_unlim_tf] = E_fd_unlim_tf

    E_fd_tf, sat_eq = apply_avr_field_limit_smooth!(
        model, active_gen, E_fd_unlim_tf, time_window, e_min, e_max; out_name="E_fd_tf")
    dyn_model_dict[:vars][:E_fd_tf] = E_fd_tf
    dyn_model_dict[:eq_const][:eq_const_E_fd_tf] = sat_eq

    V_prev0 = OrderedDict(bus => V[bus] for bus in keys(V_tf))
    ode_fs = get(dyn_model_dict[:meta], :ode_first_step, :trapezoidal)
    dyn_model_dict[:eq_const][:eq_const_E_fd_unlim_tf] = eq_const_avr_exciter!(
        model, active_gen, DGEN, DGEN_DYN, E_fd_unlim_tf, E_fd_tf, V_ref, V_tf, E_fd, V_prev0,
        time_window, Δt; ode_first_step=ode_fs)
    return E_fd_tf
end

"""
Attach the exciter to the **post-fault** window; returns saturated `E_fd_tpf`.

Reuses `V_ref` from init; t=1 anchors to the last fault-on saturated exciter state
and terminal voltage.
"""
function Attach_Avr_postfault!(
    model::JuMP.Model,
    dyn_model_dict::OrderedDict{Symbol, Any},
    active_gen::Vector{Int64},
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    V_tf::OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}},
    V_tpf::OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}},
    time_window::Vector{Float64},
    Δt::Float64,
)
    E_fd = dyn_model_dict[:vars][:E_fd]
    V_ref = dyn_model_dict[:vars][:V_ref]
    specs = dyn_model_dict[:meta][:var_limit_specs]
    e_min, e_max = _avr_field_limit_dicts(active_gen, specs)

    E_fd_last = OrderedDict(g => last(inner).second for (g, inner) in dyn_model_dict[:vars][:E_fd_tf])
    V_last = OrderedDict(bus => last(inner).second for (bus, inner) in V_tf)

    E_fd_unlim_tpf = var_avr_E_fd_unlim_time!(model, active_gen, "E_fd_unlim_tpf", time_window, E_fd)
    dyn_model_dict[:vars][:E_fd_unlim_tpf] = E_fd_unlim_tpf

    E_fd_tpf, sat_eq = apply_avr_field_limit_smooth!(
        model, active_gen, E_fd_unlim_tpf, time_window, e_min, e_max; out_name="E_fd_tpf")
    dyn_model_dict[:vars][:E_fd_tpf] = E_fd_tpf
    dyn_model_dict[:eq_const][:eq_const_E_fd_tpf] = sat_eq

    dyn_model_dict[:eq_const][:eq_const_E_fd_unlim_tpf] = eq_const_avr_exciter!(
        model, active_gen, DGEN, DGEN_DYN, E_fd_unlim_tpf, E_fd_tpf, V_ref, V_tpf, E_fd_last, V_last,
        time_window, Δt; ode_first_step=get(dyn_model_dict[:meta], :ode_first_step, :trapezoidal))
    return E_fd_tpf
end
