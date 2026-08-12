#=
================================================================================
 functions_4_TS_kron_ineqconst.jl — Kron TS inequality builders
================================================================================
=#

# ===================================================================================
#                         Inequality Constraints (shared)
# ===================================================================================

"""
Speed-deviation corridor, `Δω_tol[1] ≤ Δω_g[t] − Δω_ref[t] ≤ Δω_tol[2]`.

`ΔωCOI = nothing` drops the reference term, leaving a box on the raw Δω of the swing
equation (`bound_style_Δω = :abs`). That is not the same constraint as the COI-relative
box with the COI held at zero: the COI is a free variable that the optimiser can shift, so
the absolute box also pins the fleet's common-mode frequency excursion, not just the
spread between machines.
"""
function ineq_const_kron_Δω_COI_generic!(model::JuMP.Model,
    active_gen::Vector{Int64},
    Δω::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    ΔωCOI::Union{Nothing, OrderedDict{Int, JuMP.VariableRef}},
    time_window::Vector{Float64},
    Δω_tol::Tuple{Float64, Float64}
    )

    ineq_const_Δω_COI_lower = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    ineq_const_Δω_COI_upper = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()

    for gen in active_gen
        ineq_const_Δω_COI_lower[gen] = OrderedDict{Int, JuMP.ConstraintRef}()
        ineq_const_Δω_COI_upper[gen] = OrderedDict{Int, JuMP.ConstraintRef}()
        for t in eachindex(time_window)
            Δω_rel = ΔωCOI === nothing ? Δω[gen][t] : Δω[gen][t] - ΔωCOI[t]
            ineq_const_Δω_COI_lower[gen][t] = JuMP.@constraint(model, Δω_tol[1] - Δω_rel <= 0.0)
            ineq_const_Δω_COI_upper[gen][t] = JuMP.@constraint(model, Δω_rel - Δω_tol[2] <= 0.0)
        end
    end

    return ineq_const_Δω_COI_lower, ineq_const_Δω_COI_upper
end

"""
Plain box on the angle of every generator relative to a per-time reference series:
`δ_tol[1] ≤ δ_g[t] − δ_ref[t] ≤ δ_tol[2]`.

`δ_ref` is keyed by time step, which fits both references the package supports: the COI
variable series `δCOI`, and the inner per-time dict `δ[ref]` of a reference *machine*.
Pass `skip_gen = ref` in the latter case — the reference's own row would read `0 ≤ δ_tol`,
a constraint that can never bind and whose dual carries no information.
"""
function ineq_const_kron_δ_COI_generic!(model::JuMP.Model,
    active_gen::Vector{Int64},
    δ::OrderedDict{Int64, OrderedDict{Int64, JuMP.VariableRef}},
    δCOI::OrderedDict{Int, JuMP.VariableRef},
    time_window::Vector{Float64},
    δ_tol::Tuple{Float64, Float64};
    build_lower::Bool=true,
    build_upper::Bool=true,
    skip_gen::Union{Nothing, Int}=nothing,
    )

    ineq_const_δ_COI_lower  = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()
    ineq_const_δ_COI_upper  = OrderedDict{Int, OrderedDict{Int, JuMP.ConstraintRef}}()

    # Loop to create the variables
    for gen in active_gen
        gen == skip_gen && continue
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

"""
Store δ stability ineq families under the fault (`:tf`) or post-fault (`:tpf`) keys.

`family` picks the key prefix, and with it the dual-export rows: `"δ_COI"` for the
COI-referenced corridors, `"δ_ref"` for the machine-referenced ones. They are kept apart
so that a COI-vs-reference-machine comparison shows up as two distinct sets of dual
columns, rather than one column whose meaning silently depends on the run configuration.
"""
function store_δ_COI_ineq!(
    dyn_model_dict::OrderedDict{Symbol, Any},
    window::Symbol,
    lower::OrderedDict,
    upper::OrderedDict;
    family::String="δ_COI",
)
    suffix = window == :tf ? "tf" : "tpf"
    build_lo, build_up = δ_COI_ineq_toggle_flags(dyn_model_dict, window)
    build_lo && (dyn_model_dict[:ineq_const][Symbol("ineq_const_$(family)_$(suffix)_lower")] = lower)
    build_up && (dyn_model_dict[:ineq_const][Symbol("ineq_const_$(family)_$(suffix)_upper")] = upper)
    return nothing
end

"""
    _sg_ids(dyn_model_dict, gens) -> Vector{Int64}

`gens` with the grid-forming units removed, for the paths that index `DGEN_DYN` by
generator id.

`DGEN_DYN` carries synchronous-machine rows only, so `DGEN_DYN.H[g]` for a converter id is
a `BoundsError` on a small fleet and — worse — the wrong machine's inertia on a large one.
Every H-weighted quantity (the COI series, the `:highest_H` ranking, the swing-propagated
band) must therefore be built over this set, whatever the corridor itself spans.

On Kron, Kron-linear and classical FULL_BUS `meta[:gfm_gens]` never exists and this is the
identity, so those paths keep their exact behaviour.
"""
function _sg_ids(dyn_model_dict::OrderedDict{Symbol, Any}, gens::Vector{Int64})::Vector{Int64}
    gfm = get(dyn_model_dict[:meta], :gfm_gens, Int64[])
    isempty(gfm) && return gens
    return Int64[g for g in gens if g ∉ gfm]
end

"""
    resolve_δ_reference(dyn_model_dict, gens, DGEN_DYN; ref_candidates=gens)
        -> Union{Nothing, Int}

Generator whose angle the δ corridor is measured against, or `nothing` for the
COI-referenced styles (`:coi_box`, `:swing_propagated`), which use the `δCOI` series
instead.

`gens` is the set the corridor is built over — the *surviving* units after a GL trip, and
on the DQ path grid-forming converters as well as machines: bounding `δ_g − δ_ref` needs no
inertia from `g`, only an angle in the same frame. `ref_candidates` is the (narrower) set
the reference may be *chosen* from, and defaults to `gens` for the SG-only paths. Callers
on the mixed-fleet path pass the SG subset, because `:highest_H` ranks by inertia and a
converter has none.

`:highest_H` therefore picks the largest-inertia machine that is still synchronised, and a
tripped one can never be selected. `:ref_gen` accepts any surviving unit — including a
converter, whose angle is as good a reference as a rotor position — so the `H > 0` check
applies only when the resolved reference is a machine.

The result is cached in `meta[:δ_ref_gen_resolved]`; a second window that would resolve
differently throws rather than silently re-defining the corridor half-way through the
horizon.

Configuration errors (missing id, id not among `gens`) are caught earlier by
`validate_δ_reference!`, which runs before the warm-start solve; the checks here are the
build-time backstop for callers that reach the builders directly.
"""
function resolve_δ_reference(
    dyn_model_dict::OrderedDict{Symbol, Any},
    gens::Vector{Int64},
    DGEN_DYN::DataFrame;
    ref_candidates::Vector{Int64}=gens,
)::Union{Nothing, Int}
    meta = dyn_model_dict[:meta]
    style = get(meta, :bound_style_δ, :coi_box)
    style ∈ (:highest_H, :ref_gen) || return nothing

    length(gens) ≥ 2 || throw(ArgumentError(
        "bound_style_δ=:$(style) needs at least two synchronised units: the reference " *
        "carries no row of its own, so a single-unit set yields an empty corridor."))

    ref = if style == :ref_gen
        id = get(meta, :δ_ref_gen_id, nothing)
        id === nothing && throw(ArgumentError(
            "bound_style_δ=:ref_gen requires δ_ref_gen_id (a generator id from gen_dynamic_data)."))
        Int(id)
    else
        isempty(ref_candidates) && throw(ArgumentError(
            "bound_style_δ=:highest_H ranks by inertia, so the reference is chosen from " *
            "the synchronous machines; none survived the disturbance."))
        # argmax over the surviving machines — H is indexed by generator id, not row order.
        ref_candidates[argmax([Float64(DGEN_DYN.H[g]) for g in ref_candidates])]
    end

    ref ∈ gens || throw(ArgumentError(
        "δ reference generator $ref is not among the units the corridor is built over " *
        "($(gens)); it is out of service or tripped by the disturbance."))
    # A converter carries no inertia row; the inertia check is a machine-only sanity test.
    if ref ∉ get(meta, :gfm_gens, Int64[])
        Float64(DGEN_DYN.H[ref]) > 0.0 || throw(ArgumentError(
            "δ reference generator $ref has H = 0; pick a machine with inertia."))
    end

    prev = get(meta, :δ_ref_gen_resolved, nothing)
    prev === nothing || prev == ref || throw(ArgumentError(
        "δ reference generator changed between windows ($prev → $ref); the corridor must " *
        "keep one definition over the whole horizon."))
    meta[:δ_ref_gen_resolved] = ref
    return ref
end

# ===================================================================================
# δ stability bounds — `bound_style_δ` dispatch (shared by every network form)
# ===================================================================================
# Every flavour is built from the two families above, so the choice belongs in one place
# rather than in each builder. Kron, Kron-linear, classical FULL_BUS and DQ FULL_BUS all
# route through these two entry points.
#
# The reference machine is resolved here too, not in the builders: these functions already
# receive the generator set the corridor spans (post-GL survivors, SG-only on the DQ path),
# the angle variables, and DGEN_DYN — everything `resolve_δ_reference` needs.

"""
    _attach_δCOI!(model, dyn_model_dict, δ, gens, DGEN_DYN, time_window, window) -> OrderedDict

Centre-of-inertia angle series for `window` (`:tf` or `:tpf`), in one of two forms.

When the δ corridor is measured against the COI (`:coi_box`, `:swing_propagated`), the COI
is a **variable** with an H-weighted defining equality, as before — the multiplier of that
equality is exported as `dual_δCOI`, and the optional `bound_δCOI_*` box needs something
with bounds to attach to.

Otherwise the COI constrains nothing, so building it as a variable would add one column
and one row per time step that only ever pin themselves. It is built as a plain `AffExpr`
instead: the solver never sees it, no `dual_δCOI` row exists, but `JuMP.value` still
resolves it, so `swing_debug.csv` keeps reporting a real `δ_rel_COI` and a
reference-machine run stays directly comparable with a COI run.

Returns an empty dict when the machine set carries no inertia at all, which only happens
on degenerate fixtures; the caller uses the result solely as the COI-branch reference.

The COI is inertia-weighted, so it is always built over the synchronous machines alone —
`gens` may include grid-forming converters on the DQ path and they are dropped here.
"""
function _attach_δCOI!(
    model::JuMP.Model,
    dyn_model_dict::OrderedDict{Symbol, Any},
    δ::OrderedDict,
    gens::Vector{Int64},
    DGEN_DYN::DataFrame,
    time_window::Vector{Float64},
    window::Symbol,
)
    suffix = window == :tf ? "tf" : "tpf"
    meta = dyn_model_dict[:meta]
    sg_gens = _sg_ids(dyn_model_dict, gens)
    coi_referenced = get(meta, :constrain_δ, true) &&
        get(meta, :bound_style_δ, :coi_box) ∈ (:coi_box, :swing_propagated)

    if coi_referenced
        δCOI = var_kron_COI_time_generic!(model, "δCOI_$(suffix)", time_window)
        dyn_model_dict[:vars][Symbol("δCOI_", suffix)] = δCOI
        dyn_model_dict[:eq_const][Symbol("eq_const_δCOI_", suffix)] =
            eq_const_kron_COI_generic!(model, δ, δCOI, sg_gens, DGEN_DYN, time_window)
        return δCOI
    end

    H_total = sum(Float64(DGEN_DYN.H[g]) for g in sg_gens; init=0.0)
    H_total > 0.0 || return OrderedDict{Int, JuMP.AffExpr}()

    expr = OrderedDict{Int, JuMP.AffExpr}(
        t => sum(Float64(DGEN_DYN.H[g]) * δ[g][t] for g in sg_gens) / H_total
        for t in eachindex(time_window))
    haskey(dyn_model_dict, :expressions) ||
        (dyn_model_dict[:expressions] = OrderedDict{Symbol, Any}())
    dyn_model_dict[:expressions][Symbol("δCOI_", suffix)] = expr
    return expr
end

"""
Box the fault-on rotor angles, dispatching on `bound_style_δ`:

- `:coi_box` — plain box on the COI-relative angle: δ_tol[1] ≤ δ_g − δ_COI ≤ δ_tol[2].
- `:highest_H` / `:ref_gen` — same box against a reference *machine*: δ_g − δ_ref. The
  reference carries no row, so the corridor spans the n−1 remaining machines.
- `:swing_propagated` — the "modified" bound, which additionally couples the angle band
  to the swing dynamics (P_mech, Pe, Δω, ω_syn, Δt) and the initial state (δ_0, Δω_0).

COI-referenced styles store under the `δ_COI_tf_*` keys, machine-referenced ones under
`δ_ref_tf_*`.

`active_gen` may carry grid-forming units on the DQ path. The machine-referenced styles
bound them alongside the machines — same reference, same `δ_tol` — while the COI-referenced
styles stay synchronous-machine only, since their reference is inertia-weighted.
"""
function _add_δ_bounds_fault!(
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
    get(dyn_model_dict[:meta], :constrain_δ, true) || return nothing
    bound_style_δ = get(dyn_model_dict[:meta], :bound_style_δ, :coi_box)
    build_lo, build_up = δ_COI_ineq_toggle_flags(dyn_model_dict, :tf)
    build_lo || build_up || return nothing

    if bound_style_δ ∈ (:highest_H, :ref_gen)
        # Corridor spans every synchronised unit; the reference is picked from the machines.
        ref = resolve_δ_reference(dyn_model_dict, active_gen, DGEN_DYN;
            ref_candidates=_sg_ids(dyn_model_dict, active_gen))
        # `δ[ref]` is the reference unit's own time series — the same
        # `time step => VariableRef` shape the COI series has.
        lower, upper = ineq_const_kron_δ_COI_generic!(
            model, active_gen, δ, δ[ref], time_window, δ_tol;
            build_lower=build_lo, build_upper=build_up, skip_gen=ref)
        store_δ_COI_ineq!(dyn_model_dict, :tf, lower, upper; family="δ_ref")
    elseif bound_style_δ == :coi_box
        lower, upper = ineq_const_kron_δ_COI_generic!(
            model, _sg_ids(dyn_model_dict, active_gen), δ, δCOI, time_window, δ_tol;
            build_lower=build_lo, build_upper=build_up)
        store_δ_COI_ineq!(dyn_model_dict, :tf, lower, upper)
    else
        lower, upper = ineq_const_kron_δ_COI_generic_modified!(
            model, _sg_ids(dyn_model_dict, active_gen), DGEN_DYN, P_mech, δ, δCOI, Δω, Pe,
            time_window, δ_tol, δ_0, Δω_0, ω_syn, Δt;
            build_lower=build_lo, build_upper=build_up)
        store_δ_COI_ineq!(dyn_model_dict, :tf, lower, upper)
    end
    return nothing
end

"""
Post-fault counterpart of `_add_δ_bounds_fault!`.

Same `bound_style_δ` dispatch; the swing-propagated style is anchored on the last fault-on
step (`δ_ant`, `Δω_ant`, `Pe_ant`) instead of the pre-fault equilibrium. Stores the bounds
under the `*_tpf_*` keys.
"""
function _add_δ_bounds_postf!(
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
    get(dyn_model_dict[:meta], :constrain_δ, true) || return nothing
    bound_style_δ = get(dyn_model_dict[:meta], :bound_style_δ, :coi_box)
    build_lo, build_up = δ_COI_ineq_toggle_flags(dyn_model_dict, :tpf)
    build_lo || build_up || return nothing

    if bound_style_δ ∈ (:highest_H, :ref_gen)
        ref = resolve_δ_reference(dyn_model_dict, active_gen, DGEN_DYN;
            ref_candidates=_sg_ids(dyn_model_dict, active_gen))
        lower, upper = ineq_const_kron_δ_COI_generic!(
            model, active_gen, δ, δ[ref], time_window, δ_tol;
            build_lower=build_lo, build_upper=build_up, skip_gen=ref)
        store_δ_COI_ineq!(dyn_model_dict, :tpf, lower, upper; family="δ_ref")
    elseif bound_style_δ == :coi_box
        lower, upper = ineq_const_kron_δ_COI_generic!(
            model, _sg_ids(dyn_model_dict, active_gen), δ, δCOI, time_window, δ_tol;
            build_lower=build_lo, build_upper=build_up)
        store_δ_COI_ineq!(dyn_model_dict, :tpf, lower, upper)
    else
        lower, upper = ineq_const_kron_δ_COI_generic_modified!(
            model, _sg_ids(dyn_model_dict, active_gen), DGEN_DYN, P_mech, δ, δCOI, Δω, Pe,
            time_window, δ_tol, δ_ant, Δω_ant, Pe_ant, ω_syn, Δt;
            build_lower=build_lo, build_upper=build_up)
        store_δ_COI_ineq!(dyn_model_dict, :tpf, lower, upper)
    end
    return nothing
end

# ===================================================================================
# Δω stability bounds — `bound_style_Δω` dispatch (shared by every network form)
# ===================================================================================

"""
    _add_Δω_bounds!(model, dyn_model_dict, window, gens, DGEN_DYN, Δω, time_window)

Attach the optional speed corridor for `window` (`:tf` or `:tpf`), dispatching on
`bound_style_Δω`:

- `:coi_box` — creates the `ΔωCOI_<window>` variable and its H-weighted defining equality,
  then boxes `Δω_g − Δω_COI`. Families land under `ineq_const_Δω_COI_<window>_*`.
- `:abs` — boxes the raw `Δω_g`. No COI variable and no defining equality are created at
  all, because nothing would reference them. Families land under
  `ineq_const_Δω_abs_<window>_*`.

Grid-forming converters are bounded under `:abs` only. Their `Δω` is a real variable in the
same dict and the same per-unit convention — the droop row `Δω = mp·(P_set − P_meas)` feeds
the angle integrator through the same `ω_syn` the swing equation uses — so an absolute band
is meaningful for them. `:coi_box` stays synchronous-machine only, because its reference is
inertia-weighted and a converter has no `H` to weight with.

Does nothing when `constrain_Δω` is false, which is the default.
"""
function _add_Δω_bounds!(
    model::JuMP.Model,
    dyn_model_dict::OrderedDict{Symbol, Any},
    window::Symbol,
    gens::Vector{Int64},
    DGEN_DYN::DataFrame,
    Δω::OrderedDict,
    time_window::Vector{Float64},
)
    meta = dyn_model_dict[:meta]
    get(meta, :constrain_Δω, false) || return nothing

    suffix = window == :tf ? "tf" : "tpf"
    Δω_tol = meta[:Δω_tol]
    style = get(meta, :bound_style_Δω, :coi_box)

    ΔωCOI = nothing
    family = "Δω_abs"
    # `:abs` spans every synchronised unit; `:coi_box` is inertia-weighted, so SG-only.
    corridor_gens = style == :abs ? gens : _sg_ids(dyn_model_dict, gens)
    if style == :coi_box
        ΔωCOI = var_kron_COI_time_generic!(model, "ΔωCOI_$(suffix)", time_window)
        dyn_model_dict[:vars][Symbol("ΔωCOI_", suffix)] = ΔωCOI
        dyn_model_dict[:eq_const][Symbol("eq_const_ΔωCOI_", suffix)] =
            eq_const_kron_COI_generic!(model, Δω, ΔωCOI, corridor_gens, DGEN_DYN, time_window)
        family = "Δω_COI"
    end

    dyn_model_dict[:ineq_const][Symbol("ineq_const_$(family)_$(suffix)_lower")],
    dyn_model_dict[:ineq_const][Symbol("ineq_const_$(family)_$(suffix)_upper")] =
        ineq_const_kron_Δω_COI_generic!(model, corridor_gens, Δω, ΔωCOI, time_window, Δω_tol)
    return nothing
end
