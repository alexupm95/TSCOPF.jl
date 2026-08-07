
# ===================================================================================
#                   PRINT THE OPTIMIZATION MODEL IN TXT FILE
# ===================================================================================

"""`true` when the dynamic sub-model is the DQ_4TH FULL_BUS machine path."""
function _is_dq_4th_model(dyn_model_dict::OrderedDict{Symbol, Any})::Bool
    return get(get(dyn_model_dict, :meta, OrderedDict{Symbol, Any}()), :gen_order, "") == "DQ_4TH"
end

"""Pre-fault variable dicts for `dynamic_model_details.txt`, including mechanical power."""
function _export_prefault_var_dicts(dyn_model_dict::OrderedDict{Symbol, Any})::Vector{Any}
    dicts = Any[]
    if _is_dq_4th_model(dyn_model_dict)
        haskey(dyn_model_dict[:vars], :E_fd) && push!(dicts, dyn_model_dict[:vars][:E_fd])
        for sym in (:Ed, :Eq, :Id, :Iq)
            haskey(dyn_model_dict[:vars], sym) && push!(dicts, dyn_model_dict[:vars][sym])
        end
    else
        haskey(dyn_model_dict[:vars], :E) && push!(dicts, dyn_model_dict[:vars][:E])
    end
    haskey(dyn_model_dict[:vars], :δ) && push!(dicts, dyn_model_dict[:vars][:δ])
    # USE_PM → explicit P_m; USE_PG → OPF dispatch P_g (aliased in :refs[:P_mech]).
    if haskey(dyn_model_dict, :refs) && haskey(dyn_model_dict[:refs], :P_mech)
        push!(dicts, dyn_model_dict[:refs][:P_mech])
    elseif haskey(dyn_model_dict[:vars], :P_m)
        push!(dicts, dyn_model_dict[:vars][:P_m])
    end
    # Control set-points: constant over the horizon, created only by the AVR / governor.
    # They are not dq-specific — the governor also runs on the classical FULL_BUS path.
    for sym in (:V_ref, :P_ref)
        haskey(dyn_model_dict[:vars], sym) && push!(dicts, dyn_model_dict[:vars][sym])
    end
    # GFM pre-fault algebraic (Phase G1); absent when allow_gfm=false.
    for sym in (:P_meas, :Q_meas, :V_meas, :E_int, :V_set)
        haskey(dyn_model_dict[:vars], sym) && push!(dicts, dyn_model_dict[:vars][sym])
    end
    return dicts
end

"""Governor state trajectories (`Pv_raw_tf`, `Pv_tf`, `Pm_tf`, …) for the model TXT export.

Independent of `gen_order`: the TGOV1 governor is attached on the classical FULL_BUS path
as well as on DQ, so these must not sit behind the dq gate."""
function _export_gov_time_var_dicts(
    dyn_model_dict::OrderedDict{Symbol, Any},
    suffix::String,
)::Vector{Any}
    dicts = Any[]
    for stem in ("Pv_raw", "Pv", "Pm")
        key = Symbol(string(stem, "_", suffix))
        haskey(dyn_model_dict[:vars], key) && push!(dicts, dyn_model_dict[:vars][key])
    end
    return dicts
end

"""Fault/post-fault dq machine trajectories (`Ed_tf`, `Eq_tf`, …) for model TXT export."""
function _export_dq_time_var_dicts(
    dyn_model_dict::OrderedDict{Symbol, Any},
    suffix::String,
)::Vector{Any}
    dicts = Any[]
    for stem in ("Ed", "Eq", "Id", "Iq", "Te", "E_fd", "E_fd_unlim")
        key = Symbol(string(stem, "_", suffix))
        haskey(dyn_model_dict[:vars], key) && push!(dicts, dyn_model_dict[:vars][key])
    end
    return dicts
end

"""GFM time-window trajectories (`P_meas_tf`, `E_droop_tf`, …) registered by `Attach_GFM_*!`."""
function _export_gfm_time_var_dicts(
    dyn_model_dict::OrderedDict{Symbol, Any},
    suffix::String,
)::Vector{Any}
    dicts = Any[]
    for stem in ("P_meas", "Q_meas", "V_meas", "E_int_raw", "E_int", "E_droop_raw", "E_droop")
        key = Symbol(string(stem, "_", suffix))
        haskey(dyn_model_dict[:vars], key) && push!(dicts, dyn_model_dict[:vars][key])
    end
    return dicts
end

"""Print GFM equality blocks: gen → t → Vector{ConstraintRef} (reference-style visibility)."""
function _export_gfm_eq_vector_block!(
    io::IO,
    title::AbstractString,
    block,
)
    bar = "=" ^ max(length(title) + 1, 1)
    println(io, bar)
    println(io, title)
    println(io, bar)
    for (gen, by_t) in block
        for (t, crefs) in by_t
            for (k, cref) in enumerate(crefs)
                println(io, "G$(gen)[t=$t]#$k: ", cref)
            end
        end
    end
    println(io, "\n")
    return nothing
end

"""Append GFM init + fault/post-fault vars/eqs to `dynamic_model_details.txt`."""
function _export_gfm_appendix!(
    io::IO,
    dyn_model_dict::OrderedDict{Symbol, Any},
)
    vars = get(dyn_model_dict, :vars, nothing)
    eqs = get(dyn_model_dict, :eq_const, nothing)
    vars === nothing && return nothing
    eqs === nothing && return nothing

    has_gfm = any(haskey(vars, k) for k in
        (:P_meas, :P_meas_tf, :P_meas_tpf, :E_droop_tf, :E_droop_tpf))
    has_gfm || return nothing

    println(io, "\n")
    println(io, "============================================================")
    println(io, "GFM converter variables and constraints (appendix)")
    println(io, "============================================================")
    if haskey(dyn_model_dict, :meta)
        meta = dyn_model_dict[:meta]
        haskey(meta, :gfm_gens) && println(io, "gfm_gens: ", meta[:gfm_gens])
        haskey(meta, :gfm_phase) && println(io, "gfm_phase: ", meta[:gfm_phase])
    end
    println(io, "\n")

    for (label, key) in (
        ("Pre-Fault — GFM P_meas", :P_meas),
        ("Pre-Fault — GFM Q_meas", :Q_meas),
        ("Pre-Fault — GFM V_meas", :V_meas),
        ("Pre-Fault — GFM E_int", :E_int),
        ("Pre-Fault — GFM V_set", :V_set),
    )
        haskey(vars, key) || continue
        println(io, "=================================")
        println(io, "Variables: $label")
        println(io, "=================================")
        for (gen, v) in vars[key]
            println(io, "$gen: ", v)
        end
        println(io, "\n")
    end

    for (label, key) in (
        ("Fault Period — GFM P_meas_tf", :P_meas_tf),
        ("Fault Period — GFM Q_meas_tf", :Q_meas_tf),
        ("Fault Period — GFM V_meas_tf", :V_meas_tf),
        ("Fault Period — GFM E_int_raw_tf", :E_int_raw_tf),
        ("Fault Period — GFM E_int_tf", :E_int_tf),
        ("Fault Period — GFM E_droop_raw_tf", :E_droop_raw_tf),
        ("Fault Period — GFM E_droop_tf", :E_droop_tf),
        ("Post-Fault Period — GFM P_meas_tpf", :P_meas_tpf),
        ("Post-Fault Period — GFM Q_meas_tpf", :Q_meas_tpf),
        ("Post-Fault Period — GFM V_meas_tpf", :V_meas_tpf),
        ("Post-Fault Period — GFM E_int_raw_tpf", :E_int_raw_tpf),
        ("Post-Fault Period — GFM E_int_tpf", :E_int_tpf),
        ("Post-Fault Period — GFM E_droop_raw_tpf", :E_droop_raw_tpf),
        ("Post-Fault Period — GFM E_droop_tpf", :E_droop_tpf),
    )
        haskey(vars, key) || continue
        println(io, "=================================")
        println(io, "Variables: $label")
        println(io, "=================================")
        for (gen, inner) in vars[key]
            println(io, " ******* Gen $gen ****** ")
            for (t, v) in inner
                println(io, "$t: ", v)
            end
            println(io, "\n")
        end
    end

    for (title, key) in (
        ("Equality Constraints: GFM Init — P_meas", :eq_const_gfm_Pmeas_init),
        ("Equality Constraints: GFM Init — Q_meas", :eq_const_gfm_Qmeas_init),
        ("Equality Constraints: GFM Init — V_meas", :eq_const_gfm_Vmeas_init),
        ("Equality Constraints: GFM Init — V_set equilibrium", :eq_const_gfm_Vset_init),
    )
        haskey(eqs, key) || continue
        println(io, "=====================================")
        println(io, title)
        println(io, "=====================================")
        for (gen, c) in eqs[key]
            println(io, "$gen: ", c)
        end
        println(io, "\n")
    end

    if haskey(eqs, :eq_const_gfm_tf)
        _export_gfm_eq_vector_block!(io,
            "Fault Period — GFM dynamics (filter/droop/PI/limiter)",
            eqs[:eq_const_gfm_tf])
    end
    if haskey(eqs, :eq_const_gfm_tpf)
        _export_gfm_eq_vector_block!(io,
            "Post-Fault Period — GFM dynamics (filter/droop/PI/limiter)",
            eqs[:eq_const_gfm_tpf])
    end
    return nothing
end

"""Print machine/control constraints not covered by the classical 2nd-order export.

Covers the dq machine (DQ_4TH only), the AVR (DQ only) and the turbine governor. The
governor runs on the **classical** FULL_BUS path too, so this block must not be gated on
`_is_dq_4th_model` — every family is selected by `haskey`, and the dq/AVR keys are simply
absent on a classical run.
"""
function _export_machine_control_eq_const_appendix!(
    io::IO,
    dyn_model_dict::OrderedDict{Symbol, Any},
)
    for (title, key) in (
        ("DQ Init — Subtransient EMF Ed", :eq_const_Ed_init),
        ("DQ Init — Subtransient EMF Eq", :eq_const_Eq_init),
        ("DQ Init — Stator Voltage Vd", :eq_const_Vd_init),
        ("DQ Init — Stator Voltage Vq", :eq_const_Vq_init),
        ("DQ Init — Exciter E_fd", :eq_const_Efd_init),
        ("Governor Init — Set-point P_ref", :eq_const_Pref_init),
    )
        haskey(dyn_model_dict[:eq_const], key) || continue
        println(io, "=====================================")
        println(io, "Equality Constraints: $title")
        println(io, "=====================================")
        for (i, c) in dyn_model_dict[:eq_const][key]
            println(io, "$i: ", c)
        end
        println(io, "\n")
    end
    for (title, key_tf, key_tpf) in (
        ("DQ Te = Ed·Id + Eq·Iq", :eq_const_Te_tf, :eq_const_Te_tpf),
        ("DQ Stator Vd", :eq_const_Vd_tf, :eq_const_Vd_tpf),
        ("DQ Stator Vq", :eq_const_Vq_tf, :eq_const_Vq_tpf),
        ("DQ EMF Ed dynamics", :eq_const_Ed_tf, :eq_const_Ed_tpf),
        ("DQ EMF Eq dynamics", :eq_const_Eq_tf, :eq_const_Eq_tpf),
        ("AVR Exciter ODE (E_fd_unlim)", :eq_const_E_fd_unlim_tf, :eq_const_E_fd_unlim_tpf),
        ("AVR Field Softsat (E_fd)", :eq_const_E_fd_tf, :eq_const_E_fd_tpf),
        ("Governor Valve ODE (P_valve_raw)", :eq_const_gov_valve_tf, :eq_const_gov_valve_tpf),
        ("Governor Valve Softsat (P_valve)", :eq_const_gov_valve_limit_tf, :eq_const_gov_valve_limit_tpf),
        ("Governor Mech Power ODE (P_mech)", :eq_const_gov_mech_tf, :eq_const_gov_mech_tpf),
    )
        for (period, key) in (("Fault Period", key_tf), ("Post-Fault Period", key_tpf))
            haskey(dyn_model_dict[:eq_const], key) || continue
            println(io, "=====================================")
            println(io, "Equality Constraints: $title ($period)")
            println(io, "=====================================")
            for i in eachindex(dyn_model_dict[:eq_const][key])
                println(io, " ******* Gen $i ****** ")
                for (t, c) in dyn_model_dict[:eq_const][key][i]
                    println(io, "$t: ", c)
                end
                println(io, "\n")
            end
        end
    end
    # GOV_HARD_BOUND under the CONSTRAINT encoding emits explicit ≤-form valve bounds.
    # (Under the VARIABLE encoding they sit on `Pv_raw` and are dumped by
    # `Export_Variable_Bounds!` instead, so nothing is listed here.)
    for (period, win) in (("Fault Period", "tf"), ("Post-Fault Period", "tpf")),
        (side, side_label) in ((:lower, "Lower Bound"), (:upper, "Upper Bound"))
        key = Symbol("ineq_const_gov_valve_$(win)_$(side)")
        haskey(dyn_model_dict[:ineq_const], key) || continue
        println(io, "=====================================")
        println(io, "Inequality Constraints: Governor Valve $side_label ($period)")
        println(io, "=====================================")
        for i in eachindex(dyn_model_dict[:ineq_const][key])
            println(io, " ******* Gen $i ****** ")
            for (t, c) in dyn_model_dict[:ineq_const][key][i]
                println(io, "$t: ", c)
            end
            println(io, "\n")
        end
    end
    return nothing
end

function _export_mech_power_mode_label(dyn_model_dict::OrderedDict{Symbol, Any})::String
    if !haskey(dyn_model_dict, :mech_power_mode)
        return "mech_power_mode: (not set)"
    end
    mode = dyn_model_dict[:mech_power_mode]
    if mode == USE_PG
        return "mech_power_mode: USE_PG (swing mechanical power = OPF P_g)"
    else
        return "mech_power_mode: USE_PM (swing mechanical power = explicit P_m)"
    end
end

"""
Model-metadata block printed once at the top of `dynamic_model_details.txt`.

Shared by the Kron, Kron-linear and FULL_BUS exports, so the constraint listings that
follow are never interrupted by configuration lines. `bound_style` names the δ-COI
constraint form that was actually built, and the ZIP splits (FULL_BUS only) describe the
load model inside the nodal balances — neither is recorded anywhere else in the results
tree.
"""
function _export_dyn_meta_header(dyn_model_dict::OrderedDict{Symbol, Any})::Vector{String}
    lines = String[_export_mech_power_mode_label(dyn_model_dict)]
    meta = get(dyn_model_dict, :meta, OrderedDict{Symbol, Any}())
    haskey(meta, :bound_style) && push!(lines, "bound_style: $(meta[:bound_style])")
    haskey(meta, :ode_first_step) && push!(lines, "ode_first_step: $(meta[:ode_first_step])")
    if get(meta, :constrain_Δω_COI, false)
        # Signed pair — may be asymmetric (Δω_tol_pu_lower / Δω_tol_pu_upper).
        lo, hi = meta[:Δω_tol]
        push!(lines, "constrain_Δω_COI: true (Δω_tol = [$lo, +$hi] p.u.)")
    else
        push!(lines, "constrain_Δω_COI: false")
    end
    haskey(meta, :zip_load_p) && push!(lines, "zip_load_p (Z,I,P): $(meta[:zip_load_p])")
    haskey(meta, :zip_load_q) && push!(lines, "zip_load_q (Z,I,P): $(meta[:zip_load_q])")
    return lines
end

"""Metadata for pre-fault coupling exports (TXT header + CSV provenance)."""
function _prefault_coupling_metadata(dyn_model_dict::OrderedDict{Symbol, Any})
    meta = get(dyn_model_dict, :meta, OrderedDict{Symbol, Any}())
    # Only the FULL_BUS builders record a coupling source; the Kron paths seed
    # constants and never reach the pre-fault starts export.
    init_src = get(meta, :coupling_init_source, "none")
    return (
        gen_order = string(get(meta, :gen_order, "CLASSICAL_2ND")),
        network_form = string(get(meta, :network_form, "KRON_REDUCED")),
        mech_power_mode_label = _export_mech_power_mode_label(dyn_model_dict),
        include_avr = Bool(get(meta, :include_avr, false)),
        include_governor = Bool(get(meta, :include_governor, false)),
        coupling_init_source = string(init_src),
    )
end

"""One row per active generator: steady-state variables that couple OPF to dynamics."""
function _prefault_coupling_table(
    dyn_model_dict::OrderedDict{Symbol, Any},
    base_MVA::Float64,
)::DataFrame
    vars = dyn_model_dict[:vars]
    refs = get(dyn_model_dict, :refs, OrderedDict{Symbol, Any}())
    is_dq = _is_dq_4th_model(dyn_model_dict)
    has_avr = haskey(vars, :V_ref)
    has_gov = haskey(vars, :P_ref)
    P_mech = refs[:P_mech]
    P_g = refs[:P_g]
    gen_ids = sort(collect(keys(vars[:δ])))

    rows = NamedTuple[]
    for g in gen_ids
        push!(rows, (
            gen = "G$g",
            delta_rad = JuMP.value(vars[:δ][g]),
            E_pu = is_dq ? missing :
                (haskey(vars, :E) && haskey(vars[:E], g) ? JuMP.value(vars[:E][g]) : missing),
            E_fd_pu = is_dq && haskey(vars, :E_fd) && haskey(vars[:E_fd], g) ?
                JuMP.value(vars[:E_fd][g]) : missing,
            Ed_pu = is_dq && haskey(vars, :Ed) && haskey(vars[:Ed], g) ?
                JuMP.value(vars[:Ed][g]) : missing,
            Eq_pu = is_dq && haskey(vars, :Eq) && haskey(vars[:Eq], g) ?
                JuMP.value(vars[:Eq][g]) : missing,
            Id_pu = is_dq && haskey(vars, :Id) && haskey(vars[:Id], g) ?
                JuMP.value(vars[:Id][g]) : missing,
            Iq_pu = is_dq && haskey(vars, :Iq) && haskey(vars[:Iq], g) ?
                JuMP.value(vars[:Iq][g]) : missing,
            P_mech_pu = haskey(P_mech, g) ? JuMP.value(P_mech[g]) : missing,
            P_mech_MW = haskey(P_mech, g) ? JuMP.value(P_mech[g]) * base_MVA : missing,
            P_g_pu = JuMP.value(P_g[g]),
            P_g_MW = JuMP.value(P_g[g]) * base_MVA,
            V_ref_pu = has_avr && haskey(vars[:V_ref], g) ? JuMP.value(vars[:V_ref][g]) : missing,
            P_ref_pu = has_gov && haskey(vars[:P_ref], g) ? JuMP.value(vars[:P_ref][g]) : missing,
        ))
    end
    table = DataFrame(rows)
    # `E` is the classical internal voltage behind x'_d; the dq machine carries Ed/Eq
    # instead, so the column would be empty on every DQ row. Drop it rather than export
    # a blank column. (The TXT writer only reads `E_pu` on the non-DQ branch.)
    is_dq && select!(table, Not(:E_pu))
    return table
end

function _print_prefault_txt_section(
    io::IO,
    title::String,
    gens::AbstractVector,
    values::AbstractVector,
)
    println(io, "==========================")
    println(io, "         $title")
    println(io, "==========================")
    for (g, v) in zip(gens, values)
        println(io, "$g: ", v isa Missing ? "—" : v)
    end
    println(io, "\n")
    return nothing
end

"""Human-readable pre-fault coupling block for `dynamic_model_short_results.txt`."""
function _write_prefault_short_results_txt!(
    io::IO,
    table::DataFrame,
    metadata,
)
    println(io, "==========================")
    println(io, "   Pre-fault coupling (t=0)")
    println(io, "==========================")
    println(io, "gen_order: ", metadata.gen_order)
    println(io, "network_form: ", metadata.network_form)
    println(io, "coupling_init_source: ", metadata.coupling_init_source)
    println(io, metadata.mech_power_mode_label)
    metadata.include_avr && println(io, "include_avr: true")
    metadata.include_governor && println(io, "include_governor: true")
    println(io, "\n")

    gens = table.gen
    if metadata.gen_order == "DQ_4TH"
        _print_prefault_txt_section(io, "E_fd [p.u.]", gens, table.E_fd_pu)
    else
        _print_prefault_txt_section(io, "E [p.u.]", gens, table.E_pu)
    end
    _print_prefault_txt_section(io, "δ [deg]", gens, rad2deg.(table.delta_rad))
    _print_prefault_txt_section(io, "P_mech [MW]", gens, table.P_mech_MW)
    _print_prefault_txt_section(io, "P_g [MW]", gens, table.P_g_MW)

    if metadata.gen_order == "DQ_4TH"
        for (label, col) in (
            ("Ed [p.u.]", :Ed_pu), ("Eq [p.u.]", :Eq_pu),
            ("Id [p.u.]", :Id_pu), ("Iq [p.u.]", :Iq_pu),
        )
            _print_prefault_txt_section(io, label, gens, table[!, col])
        end
    end

    metadata.include_avr &&
        _print_prefault_txt_section(io, "V_ref [p.u.]", gens, table.V_ref_pu)
    if metadata.include_governor
        _print_prefault_txt_section(io, "P_ref [p.u.]", gens, table.P_ref_pu)
        println(io, "Governor equilibrium (t=0): P_valve_0 = P_mech, P_mech_0 = P_mech")
        println(io, "\n")
    end
    return nothing
end

"""Write machine-precision pre-fault coupling table to `CSV/prefault_coupling.csv`."""
function Save_Prefault_Coupling_CSV!(
    table::DataFrame,
    path_names::OrderedDict{Symbol, String},
)
    pf_ts_csv = path_names[:pf_TS_CSV]
    mkpath(pf_ts_csv)
    CSV.write(joinpath(pf_ts_csv, "prefault_coupling.csv"), table; delim=';')
    println("Pre-fault coupling variables saved to: ", pf_ts_csv)
    return nothing
end

# ===================================================================================
# Prefault coupling START values (Dispatch_WarmStart/, before joint optimize!)
# ===================================================================================

"""`JuMP.start_value` as Float64, or `missing` if unset."""
function _start_or_missing(v::JuMP.VariableRef)::Union{Float64, Missing}
    s = JuMP.start_value(v)
    return s === nothing ? missing : Float64(s)
end

"""Start value of `vars[sym][g]` when present; else `missing`."""
function _var_start(
    vars::OrderedDict{Symbol, Any},
    sym::Symbol,
    g::Int,
)::Union{Float64, Missing}
    haskey(vars, sym) || return missing
    d = vars[sym]
    (d isa AbstractDict && haskey(d, g)) || return missing
    return _start_or_missing(d[g])
end

"""Variable ref `vars[sym][g]` when present; else `nothing`."""
function _var_ref_or_nothing(
    vars::OrderedDict{Symbol, Any},
    sym::Symbol,
    g::Int,
)
    haskey(vars, sym) || return nothing
    d = vars[sym]
    (d isa AbstractDict && haskey(d, g)) || return nothing
    return d[g]
end

"""
One row per pre-fault machine: JuMP `start=` values that link OPF → dynamics.

Defensive over classical / DQ ± AVR/TG / GFM — absent families are `missing`.
"""
function _prefault_coupling_starts_table(
    dyn_model_dict::OrderedDict{Symbol, Any},
)::DataFrame
    vars = get(dyn_model_dict, :vars, OrderedDict{Symbol, Any}())
    meta = get(dyn_model_dict, :meta, OrderedDict{Symbol, Any}())
    gfm_set = Set{Int}(Int.(get(meta, :gfm_gens, Int[])))

    empty_cols = (
        gen=Int[], kind=String[],
        delta_rad=Union{Float64, Missing}[],
        E=Union{Float64, Missing}[], E_fd=Union{Float64, Missing}[],
        Ed=Union{Float64, Missing}[], Eq=Union{Float64, Missing}[],
        Id=Union{Float64, Missing}[], Iq=Union{Float64, Missing}[],
        P_m=Union{Float64, Missing}[],
        V_ref=Union{Float64, Missing}[], P_ref=Union{Float64, Missing}[],
        P_meas=Union{Float64, Missing}[], Q_meas=Union{Float64, Missing}[],
        V_meas=Union{Float64, Missing}[], E_int=Union{Float64, Missing}[],
        V_set=Union{Float64, Missing}[],
    )
    (!haskey(vars, :δ) || !(vars[:δ] isa AbstractDict) || isempty(vars[:δ])) &&
        return DataFrame(empty_cols)

    rows = NamedTuple[]
    for g in sort(collect(keys(vars[:δ])))
        is_gfm = g in gfm_set ||
            (haskey(vars, :P_meas) && vars[:P_meas] isa AbstractDict && haskey(vars[:P_meas], g))
        push!(rows, (
            gen = g,
            kind = is_gfm ? "GFM" : "SG",
            delta_rad = _var_start(vars, :δ, g),
            E = _var_start(vars, :E, g),
            E_fd = _var_start(vars, :E_fd, g),
            Ed = _var_start(vars, :Ed, g),
            Eq = _var_start(vars, :Eq, g),
            Id = _var_start(vars, :Id, g),
            Iq = _var_start(vars, :Iq, g),
            P_m = _var_start(vars, :P_m, g),
            V_ref = _var_start(vars, :V_ref, g),
            P_ref = _var_start(vars, :P_ref, g),
            P_meas = _var_start(vars, :P_meas, g),
            Q_meas = _var_start(vars, :Q_meas, g),
            V_meas = _var_start(vars, :V_meas, g),
            E_int = _var_start(vars, :E_int, g),
            V_set = _var_start(vars, :V_set, g),
        ))
    end
    return DataFrame(rows)
end

"""Dump-style TXT section: only gens that have `sym`, labeled with JuMP variable names."""
function _print_prefault_starts_section!(
    io::IO,
    title::String,
    vars::OrderedDict{Symbol, Any},
    sym::Symbol,
    gen_ids::AbstractVector{Int},
)
    pairs = Tuple{String, Union{Float64, Missing}}[]
    for g in gen_ids
        v = _var_ref_or_nothing(vars, sym, g)
        v === nothing && continue
        push!(pairs, (string(JuMP.name(v)), _start_or_missing(v)))
    end
    isempty(pairs) && return nothing
    println(io, "==========================")
    println(io, "         $title")
    println(io, "==========================")
    for (i, (nm, val)) in enumerate(pairs)
        println(io, "$i: $nm = ", val isa Missing ? "—" : val)
    end
    println(io, "\n")
    return nothing
end

"""Human-readable pre-fault START snapshot (before joint TSC optimize!)."""
function _write_prefault_starts_txt!(
    io::IO,
    dyn_model_dict::OrderedDict{Symbol, Any},
    table::DataFrame,
)
    meta = _prefault_coupling_metadata(dyn_model_dict)
    vars = get(dyn_model_dict, :vars, OrderedDict{Symbol, Any}())
    println(io, "========================================")
    println(io, "  Prefault coupling START values (t=0)")
    println(io, "  (JuMP start= before joint optimize!)")
    println(io, "========================================")
    println(io, "gen_order: ", meta.gen_order)
    println(io, "network_form: ", meta.network_form)
    println(io, "coupling_init_source: ", meta.coupling_init_source)
    println(io, meta.mech_power_mode_label)
    meta.include_avr && println(io, "include_avr: true")
    meta.include_governor && println(io, "include_governor: true")
    println(io, "\n")

    isempty(table) && return nothing
    gen_ids = Int.(table.gen)

    # Order matches typical dynamic_model_details pre-fault listing.
    for (title, sym) in (
        ("E_fd", :E_fd),
        ("E (classical)", :E),
        ("Ed", :Ed),
        ("Eq", :Eq),
        ("Id", :Id),
        ("Iq", :Iq),
        ("δ", :δ),
        ("P_m / P_set", :P_m),
        ("P_meas", :P_meas),
        ("Q_meas", :Q_meas),
        ("V_meas", :V_meas),
        ("E_int", :E_int),
        ("V_set", :V_set),
        ("V_ref", :V_ref),
        ("P_ref", :P_ref),
    )
        _print_prefault_starts_section!(io, title, vars, sym, gen_ids)
    end
    return nothing
end

"""
Write pre-fault JuMP `start=` values to `Dispatch_WarmStart/` (not Transient_Stability).

Called after TS assembly when ACOPF warm-start was used, before joint `optimize!`.
"""
function Save_Prefault_Coupling_Starts!(
    dyn_model_dict::OrderedDict{Symbol, Any},
    path_names::OrderedDict{Symbol, String},
)
    ws_dir = path_names[:pf_dispatch_warmstart]
    ws_csv = path_names[:pf_dispatch_warmstart_CSV]
    mkpath(ws_dir)
    mkpath(ws_csv)

    table = _prefault_coupling_starts_table(dyn_model_dict)
    CSV.write(joinpath(ws_csv, "prefault_coupling_starts.csv"), table; delim=';')
    open(joinpath(ws_dir, "prefault_coupling_starts.txt"), "w") do io
        _write_prefault_starts_txt!(io, dyn_model_dict, table)
    end
    println("Prefault coupling START values saved to: ", ws_dir)
    return nothing
end

function _export_fault_COI_var_dicts(dyn_model_dict::OrderedDict{Symbol, Any})::Vector{Any}
    dicts = Any[dyn_model_dict[:vars][:δCOI_tf]]
    if haskey(dyn_model_dict[:vars], :ΔωCOI_tf)
        push!(dicts, dyn_model_dict[:vars][:ΔωCOI_tf])
    end
    return dicts
end

function _export_postf_COI_var_dicts(dyn_model_dict::OrderedDict{Symbol, Any})::Vector{Any}
    dicts = Any[]
    if haskey(dyn_model_dict[:vars], :δCOI_tpf)
        push!(dicts, dyn_model_dict[:vars][:δCOI_tpf])
    end
    if haskey(dyn_model_dict[:vars], :ΔωCOI_tpf)
        push!(dicts, dyn_model_dict[:vars][:ΔωCOI_tpf])
    end
    return dicts
end

# Inertia H for generator `gen` from SG-only `DGEN_DYN` (0 for GFM / missing).
function _inertia_H(DGEN_DYN::DataFrame, gen::Int)::Float64
    if :id in propertynames(DGEN_DYN)
        i = findfirst(==(gen), Int.(DGEN_DYN.id))
        i === nothing && return 0.0
        return Float64(DGEN_DYN.H[i])
    end
    (1 <= gen <= nrow(DGEN_DYN)) || return 0.0
    return Float64(DGEN_DYN.H[gen])
end

# Total SG inertia over `gens`, or `nothing` for an all-GFM fleet.
#
# `nothing` rather than an error: an inertia-free fleet is a legitimate study case
# (100 % converter-based), and the H-weighted COI is simply undefined there. Every
# caller then drops the COI-weighted correction term, whose numerator is itself zero
# when every H is zero, so the remaining quantities stay correct. Throwing here used
# to abort *all* post-processing after a successful solve.
function _H_total(DGEN_DYN::DataFrame, gens)::Union{Nothing, Float64}
    s = sum(_inertia_H(DGEN_DYN, Int(g)) for g in gens; init=0.0)
    s > 0.0 || return nothing
    return s
end

# Build ΔωCOI and Δω_COI trajectories; use model ΔωCOI vars when constrained.
function _extract_ΔωCOI_trajectories(
    dyn_model_dict::OrderedDict{Symbol, Any},
    Δωt::OrderedDict{Int, Vector{Float64}},
    DGEN_DYN::DataFrame,
    active_gen::Vector{Int},
    t_window_len::Int,
)::Tuple{Vector{Float64}, OrderedDict{Int, Vector{Float64}}}
    if haskey(dyn_model_dict[:vars], :ΔωCOI_tf)
        ΔωCOI_tf_values = [JuMP.value(v) for (_, v) in dyn_model_dict[:vars][:ΔωCOI_tf]]
        if haskey(dyn_model_dict[:vars], :ΔωCOI_tpf)
            ΔωCOI_tpf_values = [JuMP.value(v) for (_, v) in dyn_model_dict[:vars][:ΔωCOI_tpf]]
            ΔωCOIt = vcat(ΔωCOI_tf_values, ΔωCOI_tpf_values)
        else
            ΔωCOIt = ΔωCOI_tf_values
        end
    else
        ΔωCOIt = zeros(Float64, t_window_len)
        for (k, Δω_vector) in Δωt
            Hk = _inertia_H(DGEN_DYN, Int(k))
            Hk == 0.0 && continue  # GFM / no inertia — SG-only COI
            ΔωCOIt .+= Hk .* Δω_vector
        end
        # `nothing` = all-GFM fleet: the SG-weighted COI has no members, so the
        # reference stays at zero and Δω_COI degenerates to the absolute speed
        # deviation. The accumulator above is already all-zero in that case.
        H_total = _H_total(DGEN_DYN, active_gen)
        H_total === nothing || (ΔωCOIt ./= H_total)
    end
    Δω_COIt = OrderedDict{Int, Vector{Float64}}()
    for k in keys(Δωt)
        Δω_COIt[k] = Δωt[k] .- ΔωCOIt
    end
    return ΔωCOIt, Δω_COIt
end

# Function to write the AC-OPF model in a txt file
function Export_Dynamic_Model_tsred(model::Model, 
    path_names::OrderedDict{Symbol, String},
    dyn_model_dict::OrderedDict{Symbol, Any}
    )

    pf_ts = path_names[:pf_TS]

    # Open the file for writing
    open(joinpath(pf_ts, "model_summary.txt"), "w") do io
        # Print the model to the file
        show(io, model)
    end

    vector_dict_var_pref      = _export_prefault_var_dicts(dyn_model_dict)
    vector_dict_var_fault     = [dyn_model_dict[:vars][:Pe_tf], dyn_model_dict[:vars][:δ_tf], dyn_model_dict[:vars][:Δω_tf]]
    vector_dict_var_fault_COI = _export_fault_COI_var_dicts(dyn_model_dict)

    vector_dict_var_postf = []
    if haskey(dyn_model_dict[:vars], :Pe_tpf) push!(vector_dict_var_postf, dyn_model_dict[:vars][:Pe_tpf]) end
    if haskey(dyn_model_dict[:vars], :δ_tpf) push!(vector_dict_var_postf, dyn_model_dict[:vars][:δ_tpf]) end
    if haskey(dyn_model_dict[:vars], :Δω_tpf) push!(vector_dict_var_postf, dyn_model_dict[:vars][:Δω_tpf]) end
    if _is_dq_4th_model(dyn_model_dict)
        append!(vector_dict_var_fault, _export_dq_time_var_dicts(dyn_model_dict, "tf"))
        append!(vector_dict_var_postf, _export_dq_time_var_dicts(dyn_model_dict, "tpf"))
        append!(vector_dict_var_fault, _export_gfm_time_var_dicts(dyn_model_dict, "tf"))
        append!(vector_dict_var_postf, _export_gfm_time_var_dicts(dyn_model_dict, "tpf"))
    end
    # Governor states exist on classical FULL_BUS as well — never gate them on gen_order.
    append!(vector_dict_var_fault, _export_gov_time_var_dicts(dyn_model_dict, "tf"))
    append!(vector_dict_var_postf, _export_gov_time_var_dicts(dyn_model_dict, "tpf"))
    vector_dict_var_postf_COI = _export_postf_COI_var_dicts(dyn_model_dict)

    open(joinpath(pf_ts, "dynamic_model_details.txt"), "w") do io

        # ---------------------------
        # Variables used in the model
        # ---------------------------
        begin
            for line in _export_dyn_meta_header(dyn_model_dict)
                println(io, line)
            end
            println(io, "\n")
            println(io, "=================================")
            println(io, "Variables in the Pre-Fault Period")
            println(io, "=================================")
            for i in eachindex(vector_dict_var_pref)
                for (j, info) in vector_dict_var_pref[i]
                    println(io, "$j: ", info)
                end
            end
            println(io, "\n")

            println(io, "=============================")
            println(io, "Variables in the Fault Period")
            println(io, "=============================")
            for i in eachindex(vector_dict_var_fault)
                for (j, infoj) in vector_dict_var_fault[i]
                    for (k, infok) in infoj
                        println(io, "$k: ", infok)
                    end
                end
            end
            for i in eachindex(vector_dict_var_fault_COI)
                for (j, info) in vector_dict_var_fault_COI[i]
                    println(io, "$j: ", info)
                end
            end
            println(io, "\n")

            println(io, "==================================")
            println(io, "Variables in the Post-Fault Period")
            println(io, "==================================")
            for i in eachindex(vector_dict_var_postf)
                for (j, infoj) in vector_dict_var_postf[i]
                    for (k, infok) in infoj
                        println(io, "$k: ", infok)
                    end
                end
            end
            for i in eachindex(vector_dict_var_postf_COI)
                for (j, info) in vector_dict_var_postf_COI[i]
                    println(io, "$j: ", info)
                end
            end
            println(io, "\n")

            exprs = get(dyn_model_dict, :expressions, nothing)
            if exprs !== nothing && haskey(exprs, :Qe_tf)
                println(io, "=======================================================")
                println(io, "Expressions — Generator Reactive Power Qe (fault-on)")
                println(io, "=======================================================")
                for (gen_id, inner) in exprs[:Qe_tf]
                    for (t, ex) in inner
                        println(io, "G$gen_id[t=$t]: ", ex)
                    end
                end
                if haskey(exprs, :Qe_tpf)
                    println(io, "\n=======================================================")
                    println(io, "Expressions — Generator Reactive Power Qe (post-fault)")
                    println(io, "=======================================================")
                    for (gen_id, inner) in exprs[:Qe_tpf]
                        for (t, ex) in inner
                            println(io, "G$gen_id[t=$t]: ", ex)
                        end
                    end
                end
                println(io, "\n")
            end
        end

        # ---------------------
        # Equality constraints
        # ---------------------
        if haskey(dyn_model_dict[:eq_const], :eq_const_P_init)
            println(io, "==================================================================")
            println(io, "Equality Constraints Initial Active Electrical Power of Generators")
            println(io, "==================================================================")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_P_init] 
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_Q_init)
            println(io, "====================================================================")
            println(io, "Equality Constraints Initial Reactive Electrical Power of Generators ")
            println(io, "====================================================================")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_Q_init] 
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_Pm_init)
            println(io, "===========================================================")
            println(io, "Equality Constraints Initial Mechanical Power of Generators ")
            println(io, "===========================================================")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_Pm_init] 
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        println(io, "=====================================")
        println(io, "Equality Constraints Angle of the COI ")
        println(io, "=====================================")
        if haskey(dyn_model_dict[:eq_const], :eq_const_δCOI_tf)
            println(io, "------------")
            println(io, "Fault Period ")
            println(io, "------------")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_δCOI_tf]
                println(io, "$i: ", info) 
            end
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_δCOI_tpf)
            println(io, "------------------")
            println(io, "Post-Fault Period ")
            println(io, "------------------")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_δCOI_tpf]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_ΔωCOI_tf) || haskey(dyn_model_dict[:eq_const], :eq_const_ΔωCOI_tpf)
            println(io, "===========================================")
            println(io, "Equality Constraints Speed Deviation of COI ")
            println(io, "===========================================")
            if haskey(dyn_model_dict[:eq_const], :eq_const_ΔωCOI_tf)
                println(io, "------------")
                println(io, "Fault Period ")
                println(io, "------------")
                for (i, info) in dyn_model_dict[:eq_const][:eq_const_ΔωCOI_tf]
                    println(io, "$i: ", info)
                end
            end
            if haskey(dyn_model_dict[:eq_const], :eq_const_ΔωCOI_tpf)
                println(io, "------------------")
                println(io, "Post-Fault Period ")
                println(io, "------------------")
                for (i, info) in dyn_model_dict[:eq_const][:eq_const_ΔωCOI_tpf]
                    println(io, "$i: ", info)
                end
            end
            println(io, "\n")
        end

        println(io, "=====================================")
        println(io, "Equality Constraints Electrical Power ")
        println(io, "=====================================")
        if haskey(dyn_model_dict[:eq_const], :eq_const_Pe_tf)
            println(io, "------------")
            println(io, "Fault Period ")
            println(io, "------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_Pe_tf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_Pe_tf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_Pe_tpf)
            println(io, "------------------")
            println(io, "Post-Fault Period ")
            println(io, "------------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_Pe_tpf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_Pe_tpf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
            println(io, "\n")
        end

        println(io, "===========================================")
        println(io, "Equality Constraints Angle - Swing Equation")
        println(io, "===========================================")
        if haskey(dyn_model_dict[:eq_const], :eq_const_δ_tf)
            println(io, "------------")
            println(io, "Fault Period ")
            println(io, "------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_δ_tf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_δ_tf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_δ_tpf)
            println(io, "------------------")
            println(io, "Post-Fault Period ")
            println(io, "------------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_δ_tpf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_δ_tpf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
            println(io, "\n")
        end

        println(io, "=====================================================")
        println(io, "Equality Constraints Speed Deviation - Swing Equation")
        println(io, "=====================================================")
        if haskey(dyn_model_dict[:eq_const], :eq_const_Δω_tf)
            println(io, "------------")
            println(io, "Fault Period ")
            println(io, "------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_Δω_tf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_Δω_tf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_Δω_tpf)
            println(io, "------------------")
            println(io, "Post-Fault Period ")
            println(io, "------------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_Δω_tpf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_Δω_tpf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
            println(io, "\n")
        end

        _export_machine_control_eq_const_appendix!(io, dyn_model_dict)

        # ---------------------
        # Inequality constraints
        # ---------------------
        println(io, "===================================================")
        println(io, "Inequality Constraints Angle in Relation to the COI")
        println(io, "===================================================")
        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_COI_tf_lower)
            println(io, "--------------------------")
            println(io, "Fault Period - Lower Bound")
            println(io, "--------------------------")
            for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tf_lower])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tf_lower][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_COI_tf_upper)
            println(io, "--------------------------")
            println(io, "Fault Period - Upper Bound")
            println(io, "--------------------------")
            for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tf_upper])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tf_upper][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_COI_tpf_lower)
            println(io, "-------------------------------")
            println(io, "Post-Fault Period - Lower Bound")
            println(io, "-------------------------------")
            for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tpf_lower])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tpf_lower][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_COI_tpf_upper)
            println(io, "-------------------------------")
            println(io, "Post-Fault Period - Upper Bound")
            println(io, "-------------------------------")
            for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tpf_upper])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tpf_upper][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tf_lower) ||
           haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tpf_lower)
            println(io, "============================================================")
            println(io, "Inequality Constraints Speed Deviation in Relation to COI")
            println(io, "============================================================")
            if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tf_lower)
                println(io, "--------------------------")
                println(io, "Fault Period - Lower Bound")
                println(io, "--------------------------")
                for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tf_lower])
                    println(io, " ******* Gen $i ****** ")
                    for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tf_lower][i]
                        println(io, "$j: ", info)
                    end
                    println(io, "\n")
                end
            end
            if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tf_upper)
                println(io, "--------------------------")
                println(io, "Fault Period - Upper Bound")
                println(io, "--------------------------")
                for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tf_upper])
                    println(io, " ******* Gen $i ****** ")
                    for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tf_upper][i]
                        println(io, "$j: ", info)
                    end
                    println(io, "\n")
                end
            end
            if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tpf_lower)
                println(io, "-------------------------------")
                println(io, "Post-Fault Period - Lower Bound")
                println(io, "-------------------------------")
                for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tpf_lower])
                    println(io, " ******* Gen $i ****** ")
                    for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tpf_lower][i]
                        println(io, "$j: ", info)
                    end
                    println(io, "\n")
                end
            end
            if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tpf_upper)
                println(io, "-------------------------------")
                println(io, "Post-Fault Period - Upper Bound")
                println(io, "-------------------------------")
                for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tpf_upper])
                    println(io, " ******* Gen $i ****** ")
                    for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tpf_upper][i]
                        println(io, "$j: ", info)
                    end
                    println(io, "\n")
                end
            end
        end

        println(io, "==========================================================")
        println(io, "Inequality Constraints Variables - Initial Operating Point")
        println(io, "==========================================================")
        if haskey(dyn_model_dict[:ineq_const], :ineq_const_E_lower)
            println(io, "------------------------------------------")
            println(io, "Internal voltage magnitude E - Lower Bound")
            println(io, "------------------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_E_lower] 
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_E_upper)
            println(io, "------------------------------------------")
            println(io, "Internal voltage magnitude E - Upper Bound")
            println(io, "------------------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_E_upper]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_E_fd_lower)
            println(io, "------------------------------------------")
            println(io, "Field voltage E_fd - Lower Bound")
            println(io, "------------------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_E_fd_lower]
                println(io, "$i: ", info)
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_E_fd_upper)
            println(io, "------------------------------------------")
            println(io, "Field voltage E_fd - Upper Bound")
            println(io, "------------------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_E_fd_upper]
                println(io, "$i: ", info)
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_lower)
            println(io, "---------------------------")
            println(io, "Rotor Angle δ - Lower Bound")
            println(io, "---------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_lower]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_upper)
            println(io, "---------------------------")
            println(io, "Rotor Angle δ - Upper Bound")
            println(io, "---------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_upper]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_P_m_lower)
            println(io, "----------------------------------")
            println(io, "Mechanical Power Pm - Lower Bound")
            println(io, "---------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_P_m_lower]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_P_m_upper)
            println(io, "---------------------------------")
            println(io, "Mechanical Power Pm - Upper Bound")
            println(io, "---------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_P_m_upper]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

    end

    println("Dynamic Model successfully saved as TXT file in: ", path_names[:pf_TS])

end

"""Append FULL_BUS-specific variables and network-balance constraints to the export."""
function _export_fullbus_network_appendix!(
    io::IO,
    dyn_model_dict::OrderedDict{Symbol, Any},
)
    for (label, key) in (
        ("Fault Period — Bus Voltage Magnitude V_tf", :V_tf),
        ("Fault Period — Bus Voltage Angle θ_tf", :θ_tf),
        ("Fault Period — Generator Reactive Power Qe_tf", :Qe_tf),
        ("Fault Period — Subtransient EMF Ed_tf", :Ed_tf),
        ("Fault Period — Subtransient EMF Eq_tf", :Eq_tf),
        ("Fault Period — dq Current Id_tf", :Id_tf),
        ("Fault Period — dq Current Iq_tf", :Iq_tf),
        ("Fault Period — Electrical Torque Te_tf", :Te_tf),
        ("Fault Period — AVR E_fd_unlim_tf", :E_fd_unlim_tf),
        ("Fault Period — AVR E_fd_tf", :E_fd_tf),
        ("Post-Fault Period — Bus Voltage Magnitude V_tpf", :V_tpf),
        ("Post-Fault Period — Bus Voltage Angle θ_tpf", :θ_tpf),
        ("Post-Fault Period — Generator Reactive Power Qe_tpf", :Qe_tpf),
        ("Post-Fault Period — Subtransient EMF Ed_tpf", :Ed_tpf),
        ("Post-Fault Period — Subtransient EMF Eq_tpf", :Eq_tpf),
        ("Post-Fault Period — dq Current Id_tpf", :Id_tpf),
        ("Post-Fault Period — dq Current Iq_tpf", :Iq_tpf),
        ("Post-Fault Period — Electrical Torque Te_tpf", :Te_tpf),
        ("Post-Fault Period — AVR E_fd_unlim_tpf", :E_fd_unlim_tpf),
        ("Post-Fault Period — AVR E_fd_tpf", :E_fd_tpf),
        ("Fault Period — Governor Valve Raw P_valve_raw_tf", :Pv_raw_tf),
        ("Fault Period — Governor Valve P_valve_tf", :Pv_tf),
        ("Fault Period — Governor Mech Power P_mech_tf", :Pm_tf),
        ("Post-Fault Period — Governor Valve Raw P_valve_raw_tpf", :Pv_raw_tpf),
        ("Post-Fault Period — Governor Valve P_valve_tpf", :Pv_tpf),
        ("Post-Fault Period — Governor Mech Power P_mech_tpf", :Pm_tpf),
    )
        if !haskey(dyn_model_dict[:vars], key)
            continue
        end
        println(io, "=================================")
        println(io, "Variables: $label")
        println(io, "=================================")
        for (bus_or_gen, inner) in dyn_model_dict[:vars][key]
            println(io, " ******* ID $bus_or_gen ****** ")
            for (t, v) in inner
                println(io, "$t: ", v)
            end
            println(io, "\n")
        end
    end

    for (label, key) in (
        ("Fault Period — Generator Qe", :eq_const_Qe_tf),
        ("Fault Period — DQ Te", :eq_const_Te_tf),
        ("Fault Period — DQ Stator Vd", :eq_const_Vd_tf),
        ("Fault Period — DQ Stator Vq", :eq_const_Vq_tf),
        ("Fault Period — DQ EMF Ed", :eq_const_Ed_tf),
        ("Fault Period — DQ EMF Eq", :eq_const_Eq_tf),
        ("Fault Period — Active Power Balance", :eq_const_Pbalance_tf),
        ("Fault Period — Reactive Power Balance", :eq_const_Qbalance_tf),
        ("Post-Fault Period — Generator Qe", :eq_const_Qe_tpf),
        ("Post-Fault Period — DQ Te", :eq_const_Te_tpf),
        ("Post-Fault Period — DQ Stator Vd", :eq_const_Vd_tpf),
        ("Post-Fault Period — DQ Stator Vq", :eq_const_Vq_tpf),
        ("Post-Fault Period — DQ EMF Ed", :eq_const_Ed_tpf),
        ("Post-Fault Period — DQ EMF Eq", :eq_const_Eq_tpf),
        ("Post-Fault Period — Active Power Balance", :eq_const_Pbalance_tpf),
        ("Post-Fault Period — Reactive Power Balance", :eq_const_Qbalance_tpf),
    )
        if !haskey(dyn_model_dict[:eq_const], key)
            continue
        end
        println(io, "=====================================")
        println(io, "Equality Constraints: $label")
        println(io, "=====================================")
        for (id, inner) in dyn_model_dict[:eq_const][key]
            println(io, " ******* Bus/Gen $id ****** ")
            for (t, c) in inner
                println(io, "$t: ", c)
            end
            println(io, "\n")
        end
    end
    return nothing
end

"""
    Export_Variable_Bounds!(model, outdir)

Write `variable_bounds.txt` listing every JuMP variable that has a VariableRef
lower and/or upper bound (including ±Inf). Unbounded vars are counted only in
the header. Does **not** list constraint-encoded boxes (`BoundEncoding::CONSTRAINT`).

Parity artefact with the reference `Transient_Stability/variable_bounds.txt`.
"""
function Export_Variable_Bounds!(model::Model, outdir::AbstractString)
    mkpath(outdir)
    vars = JuMP.all_variables(model)
    n_all = length(vars)

    rows = NamedTuple{(:name, :lb, :ub, :finite_lb, :finite_ub),
        Tuple{String, String, String, Bool, Bool}}[]
    n_unbounded = 0
    n_finite_box = 0
    n_one_sided = 0

    for v in vars
        has_lb = JuMP.has_lower_bound(v)
        has_ub = JuMP.has_upper_bound(v)
        if !has_lb && !has_ub
            n_unbounded += 1
            continue
        end
        lb = has_lb ? JuMP.lower_bound(v) : NaN
        ub = has_ub ? JuMP.upper_bound(v) : NaN
        flb = has_lb && isfinite(lb)
        fub = has_ub && isfinite(ub)
        if flb && fub
            n_finite_box += 1
        elseif flb || fub
            n_one_sided += 1
        end
        push!(rows, (
            name = String(JuMP.name(v)),
            lb = has_lb ? string(lb) : "—",
            ub = has_ub ? string(ub) : "—",
            finite_lb = flb,
            finite_ub = fub,
        ))
    end
    sort!(rows; by = r -> r.name)

    path = joinpath(outdir, "variable_bounds.txt")
    open(path, "w") do io
        println(io, "source: TSCOPF package")
        println(io, "artefact: JuMP VariableRef bounds only (not constraint-encoded boxes)")
        println(io, "num_variables              = $n_all")
        println(io, "num_with_any_var_bound     = $(length(rows))")
        println(io, "num_finite_box_lb_and_ub   = $n_finite_box")
        println(io, "num_one_sided_finite       = $n_one_sided")
        println(io, "num_unbounded_no_var_bound = $n_unbounded")
        println(io, "num_±Inf_only_or_partial   = $(length(rows) - n_finite_box - n_one_sided)")
        println(io)
        println(io, "name\tlb\tub\tfinite_lb\tfinite_ub")
        for r in rows
            println(io, "$(r.name)\t$(r.lb)\t$(r.ub)\t$(r.finite_lb)\t$(r.finite_ub)")
        end
    end
    println("Variable bounds saved to: ", path)
    return path
end

"""
    Export_Dynamic_Model_fullbus(model, path_names, dyn_model_dict)

Full-network classical export: reuses the Kron TXT layout for shared families,
then appends bus voltages, Qe, and nodal KCL constraints.  Called from
`export_dynamic_model!` **before** `optimize!` so the assembled model is auditable.
"""
function Export_Dynamic_Model_fullbus(
    model::Model,
    path_names::OrderedDict{Symbol, String},
    dyn_model_dict::OrderedDict{Symbol, Any},
)
    Export_Dynamic_Model_tsred(model, path_names, dyn_model_dict)
    pf_ts = path_names[:pf_TS]
    open(joinpath(pf_ts, "dynamic_model_details.txt"), "a") do io
        println(io, "\n")
        println(io, "============================================================")
        println(io, "FULL_BUS network-specific variables and constraints (appendix)")
        println(io, "============================================================")
        # Model metadata (bound_style, ZIP splits, …) is printed once by
        # `_export_dyn_meta_header` at the top of the file, not here between the
        # shared constraint listing and the network one.
        println(io, "network_form: FULL_BUS")
        println(io, "\n")
        _export_fullbus_network_appendix!(io, dyn_model_dict)
        _export_gfm_appendix!(io, dyn_model_dict)
    end
    println("FULL_BUS dynamic model successfully saved as TXT file in: ", pf_ts)
    return nothing
end

# Function to write the DC-OPF model in a txt file
function Export_Dynamic_Model_tsredlinear(model::Model, 
    path_names::OrderedDict{Symbol, String},
    dyn_model_dict::OrderedDict{Symbol, Any}
    )

    pf_ts = path_names[:pf_TS]

    # Open the file for writing
    open(joinpath(pf_ts, "model_summary.txt"), "w") do io
        # Print the model to the file
        show(io, model)
    end

    vector_dict_var_pref      = _export_prefault_var_dicts(dyn_model_dict)
    vector_dict_var_fault     = [dyn_model_dict[:vars][:Pe_tf], dyn_model_dict[:vars][:δ_tf], dyn_model_dict[:vars][:Δω_tf]]
    vector_dict_var_fault_COI = _export_fault_COI_var_dicts(dyn_model_dict)

    vector_dict_var_postf = []
    if haskey(dyn_model_dict[:vars], :Pe_tpf) push!(vector_dict_var_postf, dyn_model_dict[:vars][:Pe_tpf]) end
    if haskey(dyn_model_dict[:vars], :δ_tpf) push!(vector_dict_var_postf, dyn_model_dict[:vars][:δ_tpf]) end
    if haskey(dyn_model_dict[:vars], :Δω_tpf) push!(vector_dict_var_postf, dyn_model_dict[:vars][:Δω_tpf]) end
    vector_dict_var_postf_COI = _export_postf_COI_var_dicts(dyn_model_dict)

    open(joinpath(pf_ts, "dynamic_model_details.txt"), "w") do io

        # ---------------------------
        # Variables used in the model
        # ---------------------------
        begin
            for line in _export_dyn_meta_header(dyn_model_dict)
                println(io, line)
            end
            println(io, "\n")
            println(io, "=================================")
            println(io, "Variables in the Pre-Fault Period")
            println(io, "=================================")
            for i in eachindex(vector_dict_var_pref)
                for (j, info) in vector_dict_var_pref[i]
                    println(io, "$j: ", info)
                end
            end
            println(io, "\n")

            println(io, "=============================")
            println(io, "Variables in the Fault Period")
            println(io, "=============================")
            for i in eachindex(vector_dict_var_fault)
                for (j, infoj) in vector_dict_var_fault[i]
                    for (k, infok) in infoj
                        println(io, "$k: ", infok)
                    end
                end
            end
            for i in eachindex(vector_dict_var_fault_COI)
                for (j, info) in vector_dict_var_fault_COI[i]
                    println(io, "$j: ", info)
                end
            end
            println(io, "\n")

            println(io, "==================================")
            println(io, "Variables in the Post-Fault Period")
            println(io, "==================================")
            for i in eachindex(vector_dict_var_postf)
                for (j, infoj) in vector_dict_var_postf[i]
                    for (k, infok) in infoj
                        println(io, "$k: ", infok)
                    end
                end
            end
            for i in eachindex(vector_dict_var_postf_COI)
                for (j, info) in vector_dict_var_postf_COI[i]
                    println(io, "$j: ", info)
                end
            end
            println(io, "\n")
        end

        # ---------------------
        # Equality constraints
        # ---------------------
        if haskey(dyn_model_dict[:eq_const], :eq_const_P_init)
            println(io, "==================================================================")
            println(io, "Equality Constraints Initial Active Electrical Power of Generators")
            println(io, "==================================================================")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_P_init] 
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_Q_init)
            println(io, "====================================================================")
            println(io, "Equality Constraints Initial Reactive Electrical Power of Generators ")
            println(io, "====================================================================")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_Q_init] 
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_Pm_init)
            println(io, "===========================================================")
            println(io, "Equality Constraints Initial Mechanical Power of Generators ")
            println(io, "===========================================================")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_Pm_init] 
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        println(io, "=====================================")
        println(io, "Equality Constraints Angle of the COI ")
        println(io, "=====================================")
        if haskey(dyn_model_dict[:eq_const], :eq_const_δCOI_tf)
            println(io, "------------")
            println(io, "Fault Period ")
            println(io, "------------")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_δCOI_tf]
                println(io, "$i: ", info) 
            end
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_δCOI_tpf)
            println(io, "------------------")
            println(io, "Post-Fault Period ")
            println(io, "------------------")
            for (i, info) in dyn_model_dict[:eq_const][:eq_const_δCOI_tpf]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_ΔωCOI_tf) || haskey(dyn_model_dict[:eq_const], :eq_const_ΔωCOI_tpf)
            println(io, "===========================================")
            println(io, "Equality Constraints Speed Deviation of COI ")
            println(io, "===========================================")
            if haskey(dyn_model_dict[:eq_const], :eq_const_ΔωCOI_tf)
                println(io, "------------")
                println(io, "Fault Period ")
                println(io, "------------")
                for (i, info) in dyn_model_dict[:eq_const][:eq_const_ΔωCOI_tf]
                    println(io, "$i: ", info)
                end
            end
            if haskey(dyn_model_dict[:eq_const], :eq_const_ΔωCOI_tpf)
                println(io, "------------------")
                println(io, "Post-Fault Period ")
                println(io, "------------------")
                for (i, info) in dyn_model_dict[:eq_const][:eq_const_ΔωCOI_tpf]
                    println(io, "$i: ", info)
                end
            end
            println(io, "\n")
        end

        println(io, "=====================================")
        println(io, "Equality Constraints Electrical Power ")
        println(io, "=====================================")
        if haskey(dyn_model_dict[:eq_const], :eq_const_Pe_tf)
            println(io, "------------")
            println(io, "Fault Period ")
            println(io, "------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_Pe_tf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_Pe_tf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_Pe_tpf)
            println(io, "------------------")
            println(io, "Post-Fault Period ")
            println(io, "------------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_Pe_tpf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_Pe_tpf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
            println(io, "\n")
        end

        println(io, "===========================================")
        println(io, "Equality Constraints Angle - Swing Equation")
        println(io, "===========================================")
        if haskey(dyn_model_dict[:eq_const], :eq_const_δ_tf)
            println(io, "------------")
            println(io, "Fault Period ")
            println(io, "------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_δ_tf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_δ_tf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_δ_tpf)
            println(io, "------------------")
            println(io, "Post-Fault Period ")
            println(io, "------------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_δ_tpf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_δ_tpf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
            println(io, "\n")
        end

        println(io, "=====================================================")
        println(io, "Equality Constraints Speed Deviation - Swing Equation")
        println(io, "=====================================================")
        if haskey(dyn_model_dict[:eq_const], :eq_const_Δω_tf)
            println(io, "------------")
            println(io, "Fault Period ")
            println(io, "------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_Δω_tf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_Δω_tf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:eq_const], :eq_const_Δω_tpf)
            println(io, "------------------")
            println(io, "Post-Fault Period ")
            println(io, "------------------")
            for i in eachindex(dyn_model_dict[:eq_const][:eq_const_Δω_tpf])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:eq_const][:eq_const_Δω_tpf][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
            println(io, "\n")
        end

        _export_machine_control_eq_const_appendix!(io, dyn_model_dict)

        # ---------------------
        # Inequality constraints
        # ---------------------
        println(io, "===================================================")
        println(io, "Inequality Constraints Angle in Relation to the COI")
        println(io, "===================================================")
        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_COI_tf_lower)
            println(io, "--------------------------")
            println(io, "Fault Period - Lower Bound")
            println(io, "--------------------------")
            for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tf_lower])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tf_lower][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_COI_tf_upper)
            println(io, "--------------------------")
            println(io, "Fault Period - Upper Bound")
            println(io, "--------------------------")
            for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tf_upper])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tf_upper][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_COI_tpf_lower)
            println(io, "-------------------------------")
            println(io, "Post-Fault Period - Lower Bound")
            println(io, "-------------------------------")
            for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tpf_lower])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tpf_lower][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_COI_tpf_upper)
            println(io, "-------------------------------")
            println(io, "Post-Fault Period - Upper Bound")
            println(io, "-------------------------------")
            for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tpf_upper])
                println(io, " ******* Gen $i ****** ")
                for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_COI_tpf_upper][i]
                    println(io, "$j: ", info)
                end
                println(io, "\n")
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tf_lower) ||
           haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tpf_lower)
            println(io, "============================================================")
            println(io, "Inequality Constraints Speed Deviation in Relation to COI")
            println(io, "============================================================")
            if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tf_lower)
                println(io, "--------------------------")
                println(io, "Fault Period - Lower Bound")
                println(io, "--------------------------")
                for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tf_lower])
                    println(io, " ******* Gen $i ****** ")
                    for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tf_lower][i]
                        println(io, "$j: ", info)
                    end
                    println(io, "\n")
                end
            end
            if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tf_upper)
                println(io, "--------------------------")
                println(io, "Fault Period - Upper Bound")
                println(io, "--------------------------")
                for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tf_upper])
                    println(io, " ******* Gen $i ****** ")
                    for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tf_upper][i]
                        println(io, "$j: ", info)
                    end
                    println(io, "\n")
                end
            end
            if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tpf_lower)
                println(io, "-------------------------------")
                println(io, "Post-Fault Period - Lower Bound")
                println(io, "-------------------------------")
                for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tpf_lower])
                    println(io, " ******* Gen $i ****** ")
                    for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tpf_lower][i]
                        println(io, "$j: ", info)
                    end
                    println(io, "\n")
                end
            end
            if haskey(dyn_model_dict[:ineq_const], :ineq_const_Δω_COI_tpf_upper)
                println(io, "-------------------------------")
                println(io, "Post-Fault Period - Upper Bound")
                println(io, "-------------------------------")
                for i in eachindex(dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tpf_upper])
                    println(io, " ******* Gen $i ****** ")
                    for (j, info) in dyn_model_dict[:ineq_const][:ineq_const_Δω_COI_tpf_upper][i]
                        println(io, "$j: ", info)
                    end
                    println(io, "\n")
                end
            end
        end

        println(io, "==========================================================")
        println(io, "Inequality Constraints Variables - Initial Operating Point")
        println(io, "==========================================================")
        if haskey(dyn_model_dict[:ineq_const], :ineq_const_E_lower)
            println(io, "------------------------------------------")
            println(io, "Internal voltage magnitude E - Lower Bound")
            println(io, "------------------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_E_lower] 
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_E_upper)
            println(io, "------------------------------------------")
            println(io, "Internal voltage magnitude E - Upper Bound")
            println(io, "------------------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_E_upper]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_E_fd_lower)
            println(io, "------------------------------------------")
            println(io, "Field voltage E_fd - Lower Bound")
            println(io, "------------------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_E_fd_lower]
                println(io, "$i: ", info)
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_E_fd_upper)
            println(io, "------------------------------------------")
            println(io, "Field voltage E_fd - Upper Bound")
            println(io, "------------------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_E_fd_upper]
                println(io, "$i: ", info)
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_lower)
            println(io, "---------------------------")
            println(io, "Rotor Angle δ - Lower Bound")
            println(io, "---------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_lower]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_δ_upper)
            println(io, "---------------------------")
            println(io, "Rotor Angle δ - Upper Bound")
            println(io, "---------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_δ_upper]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_P_m_lower)
            println(io, "----------------------------------")
            println(io, "Mechanical Power Pm - Lower Bound")
            println(io, "---------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_P_m_lower]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

        if haskey(dyn_model_dict[:ineq_const], :ineq_const_P_m_upper)
            println(io, "---------------------------------")
            println(io, "Mechanical Power Pm - Upper Bound")
            println(io, "---------------------------------")
            for (i, info) in dyn_model_dict[:ineq_const][:ineq_const_P_m_upper]
                println(io, "$i: ", info) 
            end
            println(io, "\n")
        end

    end

    println("Dynamic Model successfully saved as TXT file in: ", path_names[:pf_TS])

end

# ===================================================================================
#                  PRINT THE DUALS OF THE OPTIMIZATION PROBLEM
# ===================================================================================

# Function to obtain the duals of the Dynamic Simulation
function Save_Duals_Dynamic_Model_tsred(model::Model,
    path_names::OrderedDict{Symbol, String},
    dyn_model_dict::OrderedDict{Symbol, Any},
    dyn_parameters_dict::OrderedDict{Symbol, Any};
    save_ts_plots::Bool=false,
    )

    pf_ts = path_names[:pf_TS]
    pf_ts_duals_csv = path_names[:pf_TS_CSV_duals]
    pf_ts_figures_duals = path_names[:pf_TS_figures_duals]

    # `run_case!` treats ITERATION_LIMIT / ALMOST_LOCALLY_SOLVED as "solved enough" to
    # export primal trajectories, but a solve that stopped early carries no meaningful
    # dual certificate. Exporting it anyway would put numbers into the CSVs that look
    # like prices and are not. Ask JuMP directly rather than re-deriving from the status.
    if !_has_usable_duals(model)
        @warn "Dual solution unavailable (termination: $(JuMP.termination_status(model)), " *
              "dual status: $(JuMP.dual_status(model))). Skipping the TS dual export."
        return nothing
    end

    # =================================================================================================
    #                     Extract constraint duals (registry-driven, Phase 1.3)
    # =================================================================================================
    # At build time, `build_dual_registry!` stored in `dyn_model_dict[:dual_registry]`
    # the subset of `CLASSICAL_KRON_DUAL_SPECS` whose JuMP keys are present.
    # `extract_registered_duals` walks that list and returns a Dict keyed by
    # `export_name` (:dual_Pe, :dual_δ_COI_lower, …).
    #
    # Optional families (Pm_init for USE_PM, ΔωCOI for constrain_Δω_COI, E bounds
    # for TSC-ACOPF, …) appear only when the builder created those constraints.
    #
    # Local variables below are kept so the TXT / CSV / XLSX writers further down
    # remain unchanged; each is `nothing` when the family was not in the model.
    _duals = extract_registered_duals(dyn_model_dict)
    _getdual(sym) = get(_duals, sym, nothing)

    # --- pre-fault initial-condition duals (λ on P/Q/Pm balance at t = 0) -------
    dual_Pe_init = _getdual(:dual_Pe_init)
    dual_Qe_init = _getdual(:dual_Qe_init)
    dual_Pm_init = _getdual(:dual_Pm_init)   # USE_PM only

    # --- COI reference equalities (λ on δ_COI and optional Δω_COI constraints) --
    dual_δCOI = _getdual(:dual_δCOI)
    dual_ΔωCOI = _getdual(:dual_ΔωCOI)      # constrain_Δω_COI only

    # --- discretized swing / Pe equalities (per generator, fault + post-fault) ---
    dual_Pe = _getdual(:dual_Pe)
    dual_δ = _getdual(:dual_δ)
    dual_Δω = _getdual(:dual_Δω)

    # --- transient-stability inequality duals (δ and Δω w.r.t. COI) ------------
    dual_δ_COI_lower = _getdual(:dual_δ_COI_lower)
    dual_δ_COI_upper = _getdual(:dual_δ_COI_upper)
    dual_Δω_COI_lower = _getdual(:dual_Δω_COI_lower)
    dual_Δω_COI_upper = _getdual(:dual_Δω_COI_upper)

    # --- variable bound duals (explicit inequalities on E, δ, P_m) --------------
    dual_LB_E = _getdual(:dual_LB_E)        # TSC-ACOPF
    dual_UB_E = _getdual(:dual_UB_E)
    dual_LB_δ = _getdual(:dual_LB_δ)        # TSC-DCOPF linear path
    dual_UB_δ = _getdual(:dual_UB_δ)
    dual_LB_Pm = _getdual(:dual_LB_Pm)      # USE_PM only
    dual_UB_Pm = _getdual(:dual_UB_Pm)

    # --- FULL_BUS network duals (Qe trajectories, nodal KCL balances) ----------
    dual_Qe = _getdual(:dual_Qe)
    dual_Pbalance = _getdual(:dual_Pbalance)
    dual_Qbalance = _getdual(:dual_Qbalance)
    dual_LB_V = _getdual(:dual_LB_V)

    # --- turbine-governor duals (set-point + valve / mech ODEs) ----------------
    dual_Pref_init = _getdual(:dual_Pref_init)   # include_governor only
    dual_gov_valve = _getdual(:dual_gov_valve)
    dual_gov_mech = _getdual(:dual_gov_mech)

    # --- DQ_4TH machine duals (init, E_fd bounds, algebra, EMF dynamics) --------
    dual_Ed_init = _getdual(:dual_Ed_init)
    dual_Eq_init = _getdual(:dual_Eq_init)
    dual_Vd_init = _getdual(:dual_Vd_init)
    dual_Vq_init = _getdual(:dual_Vq_init)
    dual_LB_E_fd = _getdual(:dual_LB_E_fd)
    dual_UB_E_fd = _getdual(:dual_UB_E_fd)
    dual_Te = _getdual(:dual_Te)
    dual_Vd = _getdual(:dual_Vd)
    dual_Vq = _getdual(:dual_Vq)
    dual_Ed = _getdual(:dual_Ed)
    dual_Eq = _getdual(:dual_Eq)
    dual_Vref_init = _getdual(:dual_Vref_init)   # include_avr only
    dual_avr_E_fd = _getdual(:dual_avr_E_fd)
    dual_avr_E_fd_sat = _getdual(:dual_avr_E_fd_sat)


    # ========== WRITE TO TXT FILE ==========
    open(joinpath(pf_ts, "dynamic_model_duals.txt"), "w") do io
        _write_solve_status_header!(io, model, "Transient-stability dual solution")

        function write_dual_active_power(io, name, vec)
            println(io, "======================================")
            println(io, "          $name:")
            println(io, "======================================")
            for (i, val) in enumerate(vec)
                println(io, "[$i] =\t €/MW $val")
            end
            println(io)  # empty line between sections
        end

        function write_dual_reactive_power(io, name, vec)
            println(io, "======================================")
            println(io, "          $name:")
            println(io, "======================================")
            for (i, val) in enumerate(vec)
                println(io, "[$i] =\t €/MVAr $val")
            end
            println(io)  # empty line between sections
        end

        function write_dual_apparent_power(io, name, vec)
            println(io, "======================================")
            println(io, "          $name:")
            println(io, "======================================")
            for (i, val) in enumerate(vec)
                println(io, "[$i] =\t €/MVA $val")
            end
            println(io)  # empty line between sections
        end

        function write_dual_voltage_pu(io, name, vec)
            println(io, "======================================")
            println(io, "          $name:")
            println(io, "======================================")
            for (i, val) in enumerate(vec)
                println(io, "[$i] =\t €/p.u. $val")
            end
            println(io)  # empty line between sections
        end

        function write_dual_angle_rad(io, name, vec)
            println(io, "======================================")
            println(io, "          $name:")
            println(io, "======================================")
            for (i, val) in enumerate(vec)
                println(io, "[$i] =\t €/rad $val")
            end
            println(io)  # empty line between sections
        end

        function write_dual_speed_pu(io, name, vec)
            println(io, "======================================")
            println(io, "          $name:")
            println(io, "======================================")
            for (i, val) in enumerate(vec)
                println(io, "[$i] =\t €/p.u. $val")
            end
            println(io)  # empty line between sections
        end

        function write_dual_others(io, name, vec)
            println(io, "======================================")
            println(io, "          $name:")
            println(io, "======================================")
            for (i, val) in enumerate(vec)
                println(io, "[$i] =\t $val")
            end
            println(io)  # empty line between sections
        end

        dual_Pe_init !== nothing && write_dual_active_power(io, "dual_Pe_init", dual_Pe_init)
        dual_Qe_init !== nothing && write_dual_reactive_power(io, "dual_Qe_init", dual_Qe_init)
        dual_Pm_init !== nothing && write_dual_active_power(io, "dual_Pm_init", dual_Pm_init)
        dual_δCOI !== nothing && write_dual_angle_rad(io, "dual_eq_δCOI", dual_δCOI)
        dual_ΔωCOI !== nothing && write_dual_speed_pu(io, "dual_eq_ΔωCOI", dual_ΔωCOI)
        dual_Pe !== nothing && for (i, info) in dual_Pe write_dual_active_power(io, "dual_eq_Pe[G$i]", info) end
        dual_Qe !== nothing && for (i, info) in dual_Qe write_dual_reactive_power(io, "dual_eq_Qe[G$i]", info) end
        dual_Pbalance !== nothing && for (i, info) in dual_Pbalance write_dual_active_power(io, "dual_eq_Pbalance[bus$i]", info) end
        dual_Qbalance !== nothing && for (i, info) in dual_Qbalance write_dual_reactive_power(io, "dual_eq_Qbalance[bus$i]", info) end
        dual_LB_V !== nothing && for (i, info) in dual_LB_V write_dual_voltage_pu(io, "dual_ineq_V_lower[bus$i]", info) end
        dual_δ !== nothing && for (i, info) in dual_δ write_dual_angle_rad(io, "dual_eq_δ[G$i]", info) end
        dual_Δω !== nothing && for (i, info) in dual_Δω write_dual_speed_pu(io, "dual_eq_Δω[G$i]", info) end
        dual_δ_COI_lower !== nothing && for (i, info) in dual_δ_COI_lower write_dual_angle_rad(io, "dual_ineq_δ_COI_lower[G$i]", info) end
        dual_δ_COI_upper !== nothing && for (i, info) in dual_δ_COI_upper write_dual_angle_rad(io, "dual_ineq_δ_COI_upper[G$i]", info) end
        dual_Δω_COI_lower !== nothing && for (i, info) in dual_Δω_COI_lower write_dual_speed_pu(io, "dual_ineq_Δω_COI_lower[G$i]", info) end
        dual_Δω_COI_upper !== nothing && for (i, info) in dual_Δω_COI_upper write_dual_speed_pu(io, "dual_ineq_Δω_COI_upper[G$i]", info) end
        dual_LB_E !== nothing && write_dual_voltage_pu(io, "dual_LB_E", dual_LB_E)
        dual_UB_E !== nothing && write_dual_voltage_pu(io, "dual_UB_E", dual_UB_E)
        dual_LB_δ !== nothing && write_dual_angle_rad(io, "dual_LB_δ", dual_LB_δ)
        dual_UB_δ !== nothing && write_dual_angle_rad(io, "dual_UB_δ", dual_UB_δ)
        dual_LB_Pm !== nothing && write_dual_active_power(io, "dual_LB_Pm", dual_LB_Pm)
        dual_UB_Pm !== nothing && write_dual_active_power(io, "dual_UB_Pm", dual_UB_Pm)
        dual_Ed_init !== nothing && write_dual_voltage_pu(io, "dual_Ed_init", dual_Ed_init)
        dual_Eq_init !== nothing && write_dual_voltage_pu(io, "dual_Eq_init", dual_Eq_init)
        dual_Vd_init !== nothing && write_dual_voltage_pu(io, "dual_Vd_init", dual_Vd_init)
        dual_Vq_init !== nothing && write_dual_voltage_pu(io, "dual_Vq_init", dual_Vq_init)
        dual_LB_E_fd !== nothing && write_dual_voltage_pu(io, "dual_LB_E_fd", dual_LB_E_fd)
        dual_UB_E_fd !== nothing && write_dual_voltage_pu(io, "dual_UB_E_fd", dual_UB_E_fd)
        dual_Te !== nothing && for (i, info) in dual_Te write_dual_active_power(io, "dual_eq_Te[G$i]", info) end
        dual_Vd !== nothing && for (i, info) in dual_Vd write_dual_voltage_pu(io, "dual_eq_Vd[G$i]", info) end
        dual_Vq !== nothing && for (i, info) in dual_Vq write_dual_voltage_pu(io, "dual_eq_Vq[G$i]", info) end
        dual_Ed !== nothing && for (i, info) in dual_Ed write_dual_voltage_pu(io, "dual_eq_Ed[G$i]", info) end
        dual_Eq !== nothing && for (i, info) in dual_Eq write_dual_voltage_pu(io, "dual_eq_Eq[G$i]", info) end
        dual_Vref_init !== nothing && write_dual_voltage_pu(io, "dual_Vref_init", dual_Vref_init)
        dual_avr_E_fd !== nothing && for (i, info) in dual_avr_E_fd write_dual_voltage_pu(io, "dual_eq_E_fd[G$i]", info) end
        dual_avr_E_fd_sat !== nothing && for (i, info) in dual_avr_E_fd_sat write_dual_voltage_pu(io, "dual_eq_E_fd_sat[G$i]", info) end
    end
    println("Duals of the dynamic model successfully saved as TXT file in: ", path_names[:pf_TS])

    # ========== SAVE IN A CSV FILE ==========
    # Registry-driven: every catalog entry that declares a `csv_file` and whose family
    # exists in this run is written here. Adding a `DualRegistryEntry` is therefore the
    # only step needed to export a new constraint family — this block never needs edits.
    # (It replaced a hand-maintained list that silently skipped registered families such
    # as the optional tf/tpf bound duals and the governor valve limiter.)
    begin
        written = String[]
        for entry in dual_spec_catalog(dyn_model_dict)
            entry.csv_file === nothing && continue
            val = _getdual(entry.export_name)
            (val === nothing || isempty(val)) && continue
            csv_path = joinpath(pf_ts_duals_csv, entry.csv_file)
            if entry.layout in (GEN_INDEXED, TIME_INDEXED, TIME_INDEXED_MERGE)
                # Flat vector → single column named after the file stem, matching the
                # historical headers (`dual_delta_COI` in dual_delta_COI.csv, …).
                col = Symbol(first(splitext(entry.csv_file)))
                CSV.write(csv_path, DataFrame(col => val); delim = ';')
            else
                # Per-generator / per-bus trajectories → one `Gen_<id>` column per key.
                save_ordered_dict_to_csv(val, csv_path)
            end
            push!(written, entry.csv_file)
        end

        println("Duals of the dynamic model successfully saved as CSV file in: ",
            pf_ts_duals_csv, " (", length(written), " files)")
    end

    # ========== SAVE IN A XLSX FILE ==========
    # Map registry export names → keyword names expected by Save_Duals_2_Excel_tsred
    Save_Duals_2_Excel_tsred(path_names; duals_to_xlsx_kwargs(_duals, dyn_model_dict)...)

    # δ-COI dual figures are gated on `save_ts_plots` like every other SVG; without the
    # gate a default run with Plots loaded produced figures nobody asked for, and a run
    # without Plots left `Figures_Duals/` empty.
    if save_ts_plots
        mkpath(pf_ts_figures_duals)
        invoke_save_dual_ts_constraint_svgs!(
            dyn_parameters_dict[:time][:t_window_total],
            dual_δ_COI_lower,
            dual_δ_COI_upper,
            pf_ts_figures_duals,
        )
    end

end

# A helper function to convert OrderedDict{Int, Vector{Float64}} to a CSV
function save_ordered_dict_to_csv(data_dict, filename)
    # Convert the dictionary to a DataFrame
    # Column names will be "Gen_1", "Gen_2", etc.
    df = DataFrame()
    for (gen_id, values) in data_dict
        df[!, Symbol("Gen_$gen_id")] = values
    end
    CSV.write(filename, df; delim = ';')
end

# Write registry-extracted duals to `Transient_Stability/OPF_Duals_Results.xlsx`.
# Accepts any `*_xlsx` kwargs from `duals_to_xlsx_kwargs` (vectors or OrderedDicts)
# so new registry entries (GFM, V_ref bounds, …) do not require signature edits.
function Save_Duals_2_Excel_tsred(path_names; kwargs...)
    file_path = joinpath(path_names[:pf_TS], "OPF_Duals_Results.xlsx")

    function to_df(data_dict)
        if isnothing(data_dict) || isempty(data_dict)
            return nothing
        end
        df = DataFrame()
        for (gen_id, values) in data_dict
            df[!, Symbol("Gen_$gen_id")] = values
        end
        return df
    end

    function sheet_name(kw::Symbol)::String
        s = String(kw)
        endswith(s, "_xlsx") && (s = chop(s; tail=5))
        startswith(s, "dual_") && (s = chop(s; head=5, tail=0))
        return s * "_Duals"
    end

    sheets = Pair{String, DataFrame}[]
    for (kw, data) in kwargs
        data === nothing && continue
        if data isa AbstractDict
            df = to_df(data)
            df === nothing && continue
            push!(sheets, sheet_name(kw) => df)
        elseif data isa AbstractVector
            isempty(data) && continue
            push!(sheets, sheet_name(kw) => DataFrame(Value = data))
        else
            @warn "Skipping Excel dual sheet for $kw: unsupported type $(typeof(data))"
        end
    end

    if !isempty(sheets)
        XLSX.writetable(file_path, sheets..., overwrite = true)
        println("Duals successfully saved to: $file_path")
    else
        @warn "No dual data was provided. Excel file not created."
    end
    return nothing
end

# ===================================================================================
#                   MANAGE RESULTS FROM THE DYNAMIC SIMULATION
# ===================================================================================
# Function to save results across the whole time window (fault and post-fault)
# Accelerating power of each generator relative to the COI, expressed in pu of
# base_MVA. Keyed by generator index — which MUST match DGEN_DYN row indices —
# NOT by enumeration position: under a GL generator trip `active_gen` is
# non-contiguous, so a position-based H lookup would weight the wrong machine
# (review finding C2). Unit-tested in test/runtests_fast_unit.jl.
function _pacc_relative_to_coi(
    Pacc::OrderedDict{Int, Vector{Float64}},
    PaccCOI_total::Vector{Float64},
    DGEN_DYN::DataFrame,
    H_total::Union{Nothing, Float64},
    base_MVA::Float64,
)
    Pacc_COI = OrderedDict{Int, Vector{Float64}}()
    for (i, info) in Pacc
        # All-GFM fleet: every Hi is zero, so the COI term drops out and the
        # COI-relative accelerating power collapses to the absolute one.
        if H_total === nothing
            Pacc_COI[i] = info ./ base_MVA
            continue
        end
        Hi = _inertia_H(DGEN_DYN, Int(i))
        Pacc_COI[i] = (info .- ((Hi .* PaccCOI_total) ./ H_total)) ./ base_MVA
    end
    return Pacc_COI
end

"""
    _compute_pacc_trajectories(Pet, Pmt, δ_COIt, DGEN_DYN, active_gen, base_MVA)

Accelerating power (MW), COI-relative Pacc (pu then scaled to MW), and potential
energy from ∫Pacc_COI dδ. `Pmt` must be keyed like `Pet` with matching time lengths.
"""
function _compute_pacc_trajectories(
    Pet::OrderedDict{Int, Vector{Float64}},
    Pmt::OrderedDict{Int, Vector{Float64}},
    δ_COIt::OrderedDict{Int, Vector{Float64}},
    DGEN_DYN::DataFrame,
    active_gen,
    base_MVA::Float64,
)
    H_total = _H_total(DGEN_DYN, active_gen)
    Pacc = OrderedDict{Int, Vector{Float64}}()
    PaccCOI = OrderedDict{Int, Vector{Float64}}()
    first_gen = true
    for (i, Pe_vals) in Pet
        Pm_vals = Pmt[i]
        length(Pm_vals) == length(Pe_vals) ||
            throw(ArgumentError("Pm and Pe length mismatch for generator $i"))
        pacc_i = Pm_vals .- Pe_vals
        Pacc[i] = pacc_i
        if first_gen
            PaccCOI[1] = copy(pacc_i)
            first_gen = false
        else
            PaccCOI[1] .+= pacc_i
        end
    end
    Pacc_COI = _pacc_relative_to_coi(Pacc, PaccCOI[1], DGEN_DYN, H_total, base_MVA)

    Vpe = OrderedDict{Int, Vector{Float64}}()
    for k in keys(Pacc_COI)
        haskey(δ_COIt, k) || continue
        # Classical PE is SG-only (GFM has H=0 and is omitted from Vke).
        _inertia_H(DGEN_DYN, Int(k)) == 0.0 && continue
        integral = Float64[]
        for (idx, _) in enumerate(Pacc_COI[k])
            if idx == 1
                push!(integral, 0.0)
            else
                push!(integral, -trapz(δ_COIt[k][1:idx], Pacc_COI[k][1:idx]))
            end
        end
        Vpe[k] = integral
    end
    for (i, info) in Pacc_COI
        Pacc_COI[i] = info .* base_MVA
    end
    return Pacc, PaccCOI, Pacc_COI, Vpe
end

# Function to save results across the whole time window for the dynamic model (fault and post-fault).
# Model-agnostic: handles both the linear (TSC-DCOPF) and nonlinear (TSC-ACOPF) reduced models,
# since both populate the same dyn_model_dict structure. (Renamed from
# Save_Results_Dynamic_Model_tsred_linear, which misleadingly implied linear-only — audit SF-2.)
function Save_Results_Dynamic_Model(model::Model,
    path_names::OrderedDict{Symbol, String},
    dyn_model_dict::OrderedDict{Symbol, Any},
    dyn_parameters_dict::OrderedDict{Symbol, Any},
    base_MVA::Float64,
    P_m::OrderedDict{Int, JuMP.VariableRef},
    DGEN_DYN::DataFrame;
    save_ts_plots::Bool=false,
    save_ts_debug_csv::Bool=false,
    )

    active_gen = dyn_model_dict[:active_gen]     # Indices of active generators

    pf_ts = path_names[:pf_TS]
    mkpath(pf_ts)

    coupling_meta = _prefault_coupling_metadata(dyn_model_dict)
    coupling_table = _prefault_coupling_table(dyn_model_dict, base_MVA)

    # ====================================
    # Save short results in TXT
    # ====================================
    open(joinpath(pf_ts, "dynamic_model_short_results.txt"), "w") do io
        _write_prefault_short_results_txt!(io, coupling_table, coupling_meta)
    end
    Save_Prefault_Coupling_CSV!(coupling_table, path_names)

    # ====================================
    #                P_e
    # ====================================
    Pe_tf_values  = OrderedDict(i => [JuMP.value(v) for (_, v) in inner] .* base_MVA for (i, inner) in dyn_model_dict[:vars][:Pe_tf])  # Values of Pe during the fault

    if haskey(dyn_model_dict[:vars], :Pe_tpf)
        Pe_tpf_values = OrderedDict(i => [JuMP.value(v) for (_, v) in inner] .* base_MVA for (i, inner) in dyn_model_dict[:vars][:Pe_tpf]) # Values of Pe in the post-fault

        Pet = OrderedDict{Int, Vector{Float64}}()
        for k in keys(Pe_tf_values)
            Pet[k] = vcat(Pe_tf_values[k], Pe_tpf_values[k])
        end
    else
        Pet = Pe_tf_values
    end

    Qet = _extract_gen_time_trajectory(
        dyn_model_dict, :Qe_tf, :Qe_tpf; scale=base_MVA)

    # ====================================
    #                 δ
    # ====================================
    δ_tf_values  = OrderedDict(i => [JuMP.value(v) for (_, v) in inner] for (i, inner) in dyn_model_dict[:vars][:δ_tf])   # Values of δ during the fault
    δCOI_tf_values  = [JuMP.value(v) for (i, v) in dyn_model_dict[:vars][:δCOI_tf]]  # Values of δ_COI during the fault

    if haskey(dyn_model_dict[:vars], :δ_tpf)
        δ_tpf_values = OrderedDict(i => [JuMP.value(v) for (_, v) in inner] for (i, inner) in dyn_model_dict[:vars][:δ_tpf]) # Values of Pe in the post-fault
        δCOI_tpf_values = [JuMP.value(v) for (i, v) in dyn_model_dict[:vars][:δCOI_tpf]] # Values of δ_COI in the post-fault

        δt = OrderedDict{Int, Vector{Float64}}()
        δ_COIt = OrderedDict{Int, Vector{Float64}}()  # δ individual in relation to COI across the whole simulation
        δCOIt = vcat(δCOI_tf_values, δCOI_tpf_values) # δ do COI across the whole simulation

        for k in keys(δ_tf_values)
            δt[k] = vcat(δ_tf_values[k], δ_tpf_values[k])
            δ_COIt[k] = δt[k] .- δCOIt
        end

    else
        δt = δ_tf_values
        δ_COIt = OrderedDict{Int, Vector{Float64}}()  # δ individual in relation to COI across the whole simulation
        δCOIt = δCOI_tf_values # δ do COI across the whole simulation

        for k in keys(δ_tf_values)
            δ_COIt[k] = δt[k] .- δCOIt
        end
    end

    # ====================================
    #                 Δω
    # ====================================
    Δω_tf_values  = OrderedDict(i => [JuMP.value(v) for (_, v) in inner] for (i, inner) in dyn_model_dict[:vars][:Δω_tf])  # Values of Δω during the fault

    if haskey(dyn_model_dict[:vars], :Δω_tpf)
        Δω_tpf_values = OrderedDict(i => [JuMP.value(v) for (_, v) in inner] for (i, inner) in dyn_model_dict[:vars][:Δω_tpf]) # Values of Pe in the post-fault

        Δωt = OrderedDict{Int, Vector{Float64}}()

        for k in keys(Δω_tf_values)
            Δωt[k] = vcat(Δω_tf_values[k], Δω_tpf_values[k])
        end

    else
        Δωt = Δω_tf_values
    end

    ΔωCOIt, Δω_COIt = _extract_ΔωCOI_trajectories(
        dyn_model_dict, Δωt, DGEN_DYN, active_gen, length(dyn_parameters_dict[:time][:t_window_total]))


    # ====================================
    #              RoCoF
    # ====================================
    time_RoCoF = []
    RoCoF = OrderedDict{Int, Vector{Float64}}()  # RoCoF individual across the whole simulation
    for (i, info) in Δωt
        time_RoCoF, RoCoF[i] = Calculate_Derivative_CFD(dyn_parameters_dict[:time][:t_window_total], dyn_parameters_dict[:common][:f_syn] .* (1.0 .+ info))
    end

    time_RoCoF, RoCoFCOI = Calculate_Derivative_CFD(dyn_parameters_dict[:time][:t_window_total], dyn_parameters_dict[:common][:f_syn] .* (1.0 .+ ΔωCOIt))

    # ====================================
    #          Kinetic Energy
    # ====================================
    Vke = OrderedDict{Int, Vector{Float64}}()  # Vke individual across the whole simulation
    for (i, info) in Δω_COIt
        Hi = _inertia_H(DGEN_DYN, Int(i))
        Hi == 0.0 && continue  # GFM has no classical kinetic energy term
        Vke[i] = Hi .* dyn_parameters_dict[:common][:ω_syn] .* (info .^2)
    end

    # ====================================
    #        Accelerating Power
    # ====================================
    t_window_total = dyn_parameters_dict[:time][:t_window_total]
    Pmt = _mechanical_power_trajectories(
        dyn_model_dict, Pet, P_m, base_MVA, length(t_window_total))
    Pacc, PaccCOI, Pacc_COI, Vpe = _compute_pacc_trajectories(
        Pet, Pmt, δ_COIt, DGEN_DYN, active_gen, base_MVA)

    # ====================================
    #           SAVE FIGURES
    # ====================================
    if save_ts_plots
        Save_Dynamic_Results_Plots_tsred(
            t_window_total, Pmt, Pet, Qet, δt, δ_COIt, δCOIt, Δωt, Δω_COIt, ΔωCOIt,
            time_RoCoF, RoCoF, RoCoFCOI, Vke, Pacc, Pacc_COI, PaccCOI, Vpe,
            base_MVA, dyn_parameters_dict[:common][:f_syn], dyn_parameters_dict[:common][:δ_tol],
            path_names)
    end
    t_clear_fault = get(dyn_parameters_dict[:time], :t_clear_fault, nothing)

    # ====================================
    #           SAVE CSV FILES
    # ====================================
    Save_Dynamic_Results_CSV_tsred(
        t_window_total, Pmt, Pet, Qet, δt, δ_COIt, δCOIt, Δωt, Δω_COIt, ΔωCOIt,
        time_RoCoF, RoCoF, RoCoFCOI, Vke, Pacc, Pacc_COI, PaccCOI, Vpe,
        base_MVA, dyn_parameters_dict[:common][:f_syn], path_names;
        has_governor = haskey(dyn_model_dict[:vars], :Pm_tf),
        governed_gens = haskey(dyn_model_dict[:vars], :Pm_tf) ?
            collect(Int, keys(dyn_model_dict[:vars][:Pm_tf])) : Int[])

    # Pre-fault internal EMF magnitude. `:E` exists on the classical Kron path as well as
    # on classical FULL_BUS, but the writer used to sit inside `Save_FullBus_Network_Results!`
    # — so a Kron run solved for E and then exported nothing. (DQ_4TH has no `:E`; it uses
    # Ed/Eq, written by `Save_Dq_Generator_Results!`.)
    if haskey(dyn_model_dict[:vars], :E)
        E_container = dyn_model_dict[:vars][:E]
        CSV.write(joinpath(path_names[:pf_TS_CSV], "generator_internal_voltage.csv"),
            DataFrame(generator = collect(keys(E_container)),
                      E_pu = [JuMP.value(v) for (_, v) in E_container]); delim = ';')
    end

    # Governor trajectories (no-op unless include_governor produced Pm_tf).
    Save_Governor_Trajectories_CSV(dyn_model_dict, dyn_parameters_dict, base_MVA, path_names;
        t_clear_fault=t_clear_fault, save_ts_plots=save_ts_plots)

    if get(get(dyn_model_dict, :meta, OrderedDict{Symbol, Any}()), :network_form, "") == "FULL_BUS"
        Save_FullBus_Network_Results!(
            dyn_model_dict, dyn_parameters_dict, base_MVA, path_names;
            t_clear_fault=t_clear_fault, save_ts_plots=save_ts_plots)
        if _is_dq_4th_model(dyn_model_dict)
            Save_Dq_Generator_Results!(
                dyn_model_dict, dyn_parameters_dict, base_MVA, path_names;
                t_clear_fault=t_clear_fault, save_ts_plots=save_ts_plots)
            Save_Gfm_Converter_Results!(
                dyn_model_dict, dyn_parameters_dict, path_names;
                t_clear_fault=t_clear_fault, save_ts_plots=save_ts_plots)
        end
    end

    if save_ts_debug_csv
        Save_TS_Debug_CSV!(dyn_model_dict, dyn_parameters_dict, DGEN_DYN, path_names)
    end

end

# Function to save the transient stability results in CSV files
function Save_Dynamic_Results_CSV_tsred(t_window_total::Vector{Float64},
    Pmt::OrderedDict{Int, Vector{Float64}},
    Pet::OrderedDict{Int, Vector{Float64}},
    Qet::Union{Nothing, OrderedDict{Int, Vector{Float64}}},
    δt::OrderedDict{Int, Vector{Float64}},
    δ_COIt::OrderedDict{Int, Vector{Float64}},
    δCOIt::Vector{Float64},
    Δωt::OrderedDict{Int, Vector{Float64}},
    Δω_COIt::OrderedDict{Int, Vector{Float64}},
    ΔωCOIt::Vector{Float64},
    time_RoCoF::Vector{Float64},
    RoCoF::OrderedDict{Int, Vector{Float64}},
    RoCoFCOI::Vector{Float64},
    Vke::OrderedDict{Int, Vector{Float64}},
    Pacc::OrderedDict{Int, Vector{Float64}},
    Pacc_COI::OrderedDict{Int, Vector{Float64}},
    PaccCOI::OrderedDict{Int, Vector{Float64}},
    Vpe::OrderedDict{Int, Vector{Float64}},
    base_MVA::Float64,
    f_syn::Float64,
    path_names::OrderedDict{Symbol, String};
    has_governor::Bool = false,
    governed_gens::Vector{Int} = Int[],
    )

    pf_ts_csv = path_names[:pf_TS_CSV]
    
    # ========================================================
    # Names of the variables
    # ========================================================

    gen_names = ["G$(i)" for (i, inner) in Pet]

    delta_names = ["G$(i)" for (i, inner) in δt]
    push!(delta_names, "COI")

    omega_names = ["G$(i)" for (i, inner) in Δωt]
    push!(omega_names, "COI")

    # ========================================================
    # Creating the matrices
    # ========================================================
    # Electrical Power
    Pe_matrix = zeros(Float64, length(t_window_total), length(Pet))
    aux_count = 0
    for (gen_id, values) in Pet
        aux_count += 1
        Pe_matrix[:,aux_count] = values
    end
    df_Pe = DataFrame(hcat(t_window_total, Pe_matrix), vcat("t", gen_names))

    Pm_matrix = zeros(Float64, length(t_window_total), length(Pmt))
    aux_count = 0
    for (gen_id, values) in Pmt
        aux_count += 1
        Pm_matrix[:, aux_count] = values
    end
    pm_gen_names = ["G$(i)" for (i, _) in Pmt]
    df_Pm = DataFrame(hcat(t_window_total, Pm_matrix), vcat("t", pm_gen_names))

    df_Qe = nothing
    if Qet !== nothing
        Qe_matrix = zeros(Float64, length(t_window_total), length(Qet))
        aux_count = 0
        for (gen_id, values) in Qet
            aux_count += 1
            Qe_matrix[:, aux_count] = values
        end
        q_gen_names = ["G$(i)" for (i, _) in Qet]
        df_Qe = DataFrame(hcat(t_window_total, Qe_matrix), vcat("t", q_gen_names))
    end

    # Accelerating Power
    Pacc_matrix = zeros(Float64, length(t_window_total), length(Pacc)+1)
    aux_count = 0
    for (gen_id, values) in Pacc
        aux_count += 1
        Pacc_matrix[:,aux_count] = values
    end
    Pacc_matrix[:,end] = PaccCOI[1]
    df_Pacc = DataFrame(hcat(t_window_total, Pacc_matrix), vcat("t", delta_names))

    # Accelerating Power in relation to COI
    Pacc_COI_matrix = zeros(Float64, length(t_window_total), length(Pacc_COI))
    aux_count = 0
    for (gen_id, values) in Pacc_COI
        aux_count += 1
        Pacc_COI_matrix[:,aux_count] = values
    end
    df_Pacc_COI = DataFrame(hcat(t_window_total, Pacc_COI_matrix), vcat("t", gen_names))

    # Absolute values of δ
    δ_matrix = zeros(Float64, length(t_window_total), length(δt)+1)
    aux_count = 0
    for (gen_id, values) in δt
        aux_count += 1
        δ_matrix[:,aux_count] = rad2deg.(values)
    end
    δ_matrix[:,end] = rad2deg.(δCOIt)
    df_δ = DataFrame(hcat(t_window_total, δ_matrix), vcat("t", delta_names))

    # δ values in relation to the COI
    δ_COI_matrix = zeros(Float64, length(t_window_total), length(δ_COIt))
    aux_count = 0
    for (gen_id, values) in δ_COIt
        aux_count += 1
        δ_COI_matrix[:,aux_count] = rad2deg.(values)
    end
    df_δ_COI = DataFrame(hcat(t_window_total, δ_COI_matrix), vcat("t", gen_names))
    
    # Absolute values of Δω
    Δω_matrix = zeros(Float64, length(t_window_total), length(Δωt)+1)
    aux_count = 0
    for (gen_id, values) in Δωt
        aux_count += 1
        Δω_matrix[:,aux_count] = values
    end
    Δω_matrix[:,end] = ΔωCOIt
    df_Δω = DataFrame(hcat(t_window_total, Δω_matrix), vcat("t", omega_names))

    # Δω values in relation to the COI
    Δω_COI_matrix = zeros(Float64, length(t_window_total), length(Δω_COIt))
    aux_count = 0
    for (gen_id, values) in Δω_COIt
        aux_count += 1
        Δω_COI_matrix[:,aux_count] = values
    end
    df_Δω_COI = DataFrame(hcat(t_window_total, Δω_COI_matrix), vcat("t", gen_names))

    # Frequency values
    f_matrix = zeros(Float64, length(t_window_total), length(Δωt)+1)
    aux_count = 0
    for (gen_id, values) in Δωt
        aux_count += 1
        f_matrix[:,aux_count] = f_syn .* (1.0 .+ values)
    end
    f_matrix[:,end] = f_syn .* (1.0 .+ ΔωCOIt)
    df_f = DataFrame(hcat(t_window_total, f_matrix), vcat("t", omega_names))

    # Frequency values in relation to the COI
    f_COI_matrix = zeros(Float64, length(t_window_total), length(Δω_COIt))
    aux_count = 0
    for (gen_id, values) in Δω_COIt
        aux_count += 1
        f_COI_matrix[:,aux_count] = f_syn .* (1.0 .+ values)
    end
    df_f_COI = DataFrame(hcat(t_window_total, f_COI_matrix), vcat("t", gen_names))
    
    # RoCoF values
    RoCoF_matrix = zeros(Float64, length(time_RoCoF), length(RoCoF)+1)
    aux_count = 0
    for (gen_id, values) in RoCoF
        aux_count += 1
        RoCoF_matrix[:,aux_count] = values
    end
    RoCoF_matrix[:,end] = RoCoFCOI
    df_RoCoF = DataFrame(hcat(time_RoCoF, RoCoF_matrix), vcat("t", omega_names))

    # Vke values (SG inertia only — GFM keys omitted)
    Vke_matrix = zeros(Float64, length(t_window_total), length(Vke))
    aux_count = 0
    for (gen_id, values) in Vke
        aux_count += 1
        Vke_matrix[:,aux_count] = values
    end
    vke_names = ["G$(i)" for (i, _) in Vke]
    df_Vke = DataFrame(hcat(t_window_total, Vke_matrix), vcat("t", vke_names))

    # Vpe values (may omit GFM when δ_COI is SG-only)
    Vpe_matrix = zeros(Float64, length(t_window_total), length(Vpe))
    aux_count = 0
    for (gen_id, values) in Vpe
        aux_count += 1
        Vpe_matrix[:,aux_count] = values
    end
    vpe_names = ["G$(i)" for (i, _) in Vpe]
    df_Vpe = DataFrame(hcat(t_window_total, Vpe_matrix), vcat("t", vpe_names))

    # Total transient energy Ve = Vke + Vpe, over the generators that have both terms
    # (SG only: GFM units carry no classical kinetic energy). This is the quantity the
    # equal-area / energy-function reading of the swing curves rests on, and it used to
    # exist only inside "Total Energy vs time.svg" — computed in the plot and discarded
    # with it, so a run without `save_ts_plots` produced no record of it at all.
    ve_ids = [i for i in keys(Vpe) if haskey(Vke, i)]
    Ve_matrix = zeros(Float64, length(t_window_total), length(ve_ids))
    for (j, gen_id) in enumerate(ve_ids)
        Ve_matrix[:, j] = Vke[gen_id] .+ Vpe[gen_id]
    end
    df_Ve = DataFrame(hcat(t_window_total, Ve_matrix),
        vcat("t", ["G$(i)" for i in ve_ids]))

    # ========================================================
    # Saving the CSVs
    # ========================================================
    CSV.write(joinpath(pf_ts_csv, "electrical_power.csv"), df_Pe; delim=';')
    # With a governor, P_m(t) is the governor mechanical-power state, so for an SG-only
    # fleet this file would duplicate `governor_P_mech.csv` byte for byte and is skipped.
    # It is *not* a duplicate once GFM units are present: `governor_P_mech.csv` covers the
    # governed SGs only, while `Pmt` also carries the converters' constant P_set — which
    # otherwise reached no file at all. So suppress only when every column is governed.
    gov_covers_all = has_governor && length(governed_gens) == length(Pmt) &&
        all(g -> g in governed_gens, keys(Pmt))
    gov_covers_all || CSV.write(joinpath(pf_ts_csv, "mechanical_power.csv"), df_Pm; delim=';')
    df_Qe !== nothing && CSV.write(joinpath(pf_ts_csv, "electrical_reactive_power.csv"), df_Qe; delim=';')
    CSV.write(joinpath(pf_ts_csv, "accelerating_power.csv"), df_Pacc; delim=';')
    CSV.write(joinpath(pf_ts_csv, "accelerating_power_COI.csv"), df_Pacc_COI; delim=';')
    CSV.write(joinpath(pf_ts_csv, "angle_abs.csv"), df_δ; delim=';')
    CSV.write(joinpath(pf_ts_csv, "angle_rel_COI.csv"), df_δ_COI; delim=';')
    CSV.write(joinpath(pf_ts_csv, "speed_dev.csv"), df_Δω; delim=';')
    CSV.write(joinpath(pf_ts_csv, "speed_dev_rel_COI.csv"), df_Δω_COI; delim=';')
    CSV.write(joinpath(pf_ts_csv, "frequency.csv"), df_f; delim=';')
    CSV.write(joinpath(pf_ts_csv, "frequency_rel_COI.csv"), df_f_COI; delim=';')
    CSV.write(joinpath(pf_ts_csv, "RoCoF.csv"), df_RoCoF; delim=';')
    CSV.write(joinpath(pf_ts_csv, "kinetic_energy.csv"), df_Vke; delim=';')
    CSV.write(joinpath(pf_ts_csv, "potential_energy.csv"), df_Vpe; delim=';')
    CSV.write(joinpath(pf_ts_csv, "total_energy.csv"), df_Ve; delim=';')

    println("Results of the dynamic model successfully saved as CSV files in: ", pf_ts_csv)

end

# ===================================================================================
#                   FULL_BUS network trajectories (bus P/Q/V/θ)
# ===================================================================================

"""Merge fault-on and post-fault per-generator time series from `vars` or `expressions`."""
function _extract_gen_time_trajectory(
    dyn_model_dict::OrderedDict{Symbol, Any},
    key_tf::Symbol,
    key_tpf::Symbol;
    scale::Float64=1.0,
)
    for source in (:vars, :expressions)
        store = get(dyn_model_dict, source, nothing)
        store === nothing && continue
        haskey(store, key_tf) || continue
        tf_vals = OrderedDict(
            gen => scale .* [JuMP.value(v) for (_, v) in inner]
            for (gen, inner) in store[key_tf])
        if haskey(store, key_tpf)
            tpf_vals = OrderedDict(
                gen => scale .* [JuMP.value(v) for (_, v) in inner]
                for (gen, inner) in store[key_tpf])
            return OrderedDict(k => vcat(tf_vals[k], tpf_vals[k]) for k in keys(tf_vals))
        end
        return tf_vals
    end
    return nothing
end

"""
    _mechanical_power_trajectories(dyn_model_dict, Pet, P_m, base_MVA, n_time)

Per-generator mechanical power [MW] over the merged fault+post-fault window.
Governor runs use `:Pm_tf`/`:Pm_tpf`; otherwise constant pre-fault `P_m` per generator.
"""
function _mechanical_power_trajectories(
    dyn_model_dict::OrderedDict{Symbol, Any},
    Pet::OrderedDict{Int, Vector{Float64}},
    P_m::OrderedDict{Int, JuMP.VariableRef},
    base_MVA::Float64,
    n_time::Int,
)::OrderedDict{Int, Vector{Float64}}
    vars = get(dyn_model_dict, :vars, nothing)
    if vars !== nothing && haskey(vars, :Pm_tf)
        traj = _extract_gen_time_trajectory(dyn_model_dict, :Pm_tf, :Pm_tpf; scale=base_MVA)
        if traj !== nothing
            # Governor trajectories cover SG only; GFM uses constant P_set (= P_m).
            Pmt = OrderedDict{Int, Vector{Float64}}()
            for gen_id in keys(Pet)
                if haskey(traj, gen_id)
                    Pmt[gen_id] = traj[gen_id]
                else
                    Pm_mw = JuMP.value(P_m[gen_id]) * base_MVA
                    Pmt[gen_id] = fill(Pm_mw, n_time)
                end
            end
            return Pmt
        end
    end
    Pmt = OrderedDict{Int, Vector{Float64}}()
    for gen_id in keys(Pet)
        Pm_mw = JuMP.value(P_m[gen_id]) * base_MVA
        Pmt[gen_id] = fill(Pm_mw, n_time)
    end
    return Pmt
end

"""
    Save_Governor_Trajectories_CSV(dyn_model_dict, dyn_parameters_dict, base_MVA, path_names)

Write the turbine-governor trajectories to CSV (and SVG when `save_ts_plots`) if a
governor is present in the run; no-op otherwise. Reuses `_extract_gen_time_trajectory`
for the fault + post-fault merge.

Produces `governor_P_mech.csv`, `governor_P_valve.csv`, and `governor_P_valve_raw.csv`
(columns `t` + `G<id>`, powers in MW) plus `governor_P_ref.csv` (the constant
per-generator set-point, in p.u.). `P_valve_raw` is the integrator state ahead of the
valve limiter, so `P_valve_raw − P_valve` is the active clamp under `GOV_SMOOTH`.
"""
function Save_Governor_Trajectories_CSV(
    dyn_model_dict::OrderedDict{Symbol, Any},
    dyn_parameters_dict::OrderedDict{Symbol, Any},
    base_MVA::Float64,
    path_names::OrderedDict{Symbol, String};
    t_clear_fault=nothing,
    save_ts_plots::Bool=false,
)
    haskey(dyn_model_dict[:vars], :Pm_tf) || return nothing  # no governor in this run

    pf_ts_csv = path_names[:pf_TS_CSV]
    mkpath(pf_ts_csv)
    pf_figures = path_names[:pf_TS_figures]
    save_ts_plots && mkpath(pf_figures)
    t = dyn_parameters_dict[:time][:t_window_total]

    for (label, key_tf, key_tpf, title, ylab, svg_name) in (
        ("P_mech", :Pm_tf, :Pm_tpf,
            "Governor Mechanical Power", "P_m (MW)", "governor_P_mech.svg"),
        ("P_valve", :Pv_tf, :Pv_tpf,
            "Governor Valve Output", "P_valve (MW)", "governor_P_valve.svg"),
        # Raw integrator state ahead of the valve limiter. Identical to P_valve under
        # GOV_NO_LIMIT / GOV_HARD_BOUND; under GOV_SMOOTH the difference is the clamp.
        ("P_valve_raw", :Pv_raw_tf, :Pv_raw_tpf,
            "Governor Valve (unsaturated)", "P_valve_raw (MW)", "governor_P_valve_raw.svg"),
    )
        traj = _extract_gen_time_trajectory(dyn_model_dict, key_tf, key_tpf; scale=base_MVA)
        traj === nothing && continue
        gen_names = ["G$(g)" for g in keys(traj)]
        mat = zeros(Float64, length(t), length(traj))
        for (j, (_, vals)) in enumerate(traj)
            mat[:, j] = vals
        end
        CSV.write(joinpath(pf_ts_csv, "governor_$(label).csv"),
            DataFrame(hcat(t, mat), vcat("t", gen_names)); delim = ';')
        if save_ts_plots
            _save_fullbus_svg(t, traj, title, ylab, pf_figures, svg_name;
                t_clear_fault=t_clear_fault, label_prefix="G")
        end
    end

    if haskey(dyn_model_dict[:vars], :P_ref)
        P_ref = dyn_model_dict[:vars][:P_ref]
        CSV.write(joinpath(pf_ts_csv, "governor_P_ref.csv"),
            DataFrame(gen = ["G$(g)" for g in keys(P_ref)],
                      P_ref_pu = [JuMP.value(v) for (_, v) in P_ref]); delim = ';')
    end

    println("Governor trajectories saved to: ", pf_ts_csv)
    return nothing
end

"""
    Save_Dq_Generator_Results!(dyn_model_dict, dyn_parameters_dict, base_MVA, path_names)

Write dq-machine trajectories (`E_fd`, `Ed/Eq/Id/Iq/Te`) to CSV and optional SVG plots.
Called from `Save_Results_Dynamic_Model` when `meta[:gen_order] == "DQ_4TH"`.
"""
function Save_Dq_Generator_Results!(
    dyn_model_dict::OrderedDict{Symbol, Any},
    dyn_parameters_dict::OrderedDict{Symbol, Any},
    base_MVA::Float64,
    path_names::OrderedDict{Symbol, String};
    t_clear_fault=nothing,
    save_ts_plots::Bool=false,
)
    pf_ts_csv = path_names[:pf_TS_CSV]
    mkpath(pf_ts_csv)
    t_window = dyn_parameters_dict[:time][:t_window_total]
    pf_figures = path_names[:pf_TS_figures]
    save_ts_plots && mkpath(pf_figures)

    if haskey(dyn_model_dict[:vars], :V_ref)
        CSV.write(joinpath(pf_ts_csv, "avr_V_ref.csv"),
            DataFrame(gen = ["G$(g)" for g in keys(dyn_model_dict[:vars][:V_ref])],
                      V_ref_pu = [JuMP.value(v) for (_, v) in dyn_model_dict[:vars][:V_ref]]);
            delim = ';')
    end

    if haskey(dyn_model_dict[:vars], :E_fd)
        CSV.write(joinpath(pf_ts_csv, "dq_E_fd.csv"),
            DataFrame(gen = ["G$(g)" for g in keys(dyn_model_dict[:vars][:E_fd])],
                      E_fd_pu = [JuMP.value(v) for (_, v) in dyn_model_dict[:vars][:E_fd]]);
            delim = ';')
    end

    prefault_rows = Vector{NamedTuple}()
    for (label, sym) in (
        ("Ed_pu", :Ed), ("Eq_pu", :Eq), ("Id_pu", :Id), ("Iq_pu", :Iq),
    )
        haskey(dyn_model_dict[:vars], sym) || continue
        for (g, v) in dyn_model_dict[:vars][sym]
            push!(prefault_rows, (gen = "G$g", variable = label, value = JuMP.value(v)))
        end
    end
    # Terminal dq voltages at t = 0 are expressions, not variables, so they live in
    # `:expressions` — same rows, same file, so the pre-fault algebraic state is complete.
    exprs_store = get(dyn_model_dict, :expressions, OrderedDict{Symbol, Any}())
    for (label, sym) in (("Vd_pu", :Vd_init), ("Vq_pu", :Vq_init))
        haskey(exprs_store, sym) || continue
        for (g, e) in exprs_store[sym]
            push!(prefault_rows, (gen = "G$g", variable = label, value = JuMP.value(e)))
        end
    end
    if !isempty(prefault_rows)
        CSV.write(joinpath(pf_ts_csv, "dq_prefault_algebraic.csv"), DataFrame(prefault_rows); delim = ';')
    end

    dq_series = (
        ("dq_Ed_pu", :Ed_tf, :Ed_tpf, 1.0, "Subtransient EMF Ed", "Ed (p.u.)", "dq_Ed.svg"),
        ("dq_Eq_pu", :Eq_tf, :Eq_tpf, 1.0, "Subtransient EMF Eq", "Eq (p.u.)", "dq_Eq.svg"),
        ("dq_Id_pu", :Id_tf, :Id_tpf, 1.0, "dq Current Id", "Id (p.u.)", "dq_Id.svg"),
        ("dq_Iq_pu", :Iq_tf, :Iq_tpf, 1.0, "dq Current Iq", "Iq (p.u.)", "dq_Iq.svg"),
        ("dq_Te_MW", :Te_tf, :Te_tpf, base_MVA, "Electrical Torque Te", "T_e (MW)", "dq_Te.svg"),
        ("dq_E_fd_pu", :E_fd_tf, :E_fd_tpf, 1.0, "Field Voltage E_fd", "E_fd (p.u.)", "dq_E_fd_traj.svg"),
        # Terminal dq voltages: JuMP expressions, picked up by `_extract_gen_time_trajectory`
        # from `:expressions` exactly like the nodal injections.
        ("dq_Vd_pu", :Vd_tf, :Vd_tpf, 1.0, "Terminal Voltage Vd", "Vd (p.u.)", "dq_Vd.svg"),
        ("dq_Vq_pu", :Vq_tf, :Vq_tpf, 1.0, "Terminal Voltage Vq", "Vq (p.u.)", "dq_Vq.svg"),
        # Pre-saturation exciter output (AVR only); equals E_fd wherever the softsat
        # clamp is inactive, so the gap between the two shows when the AVR is limiting.
        ("dq_E_fd_unlim_pu", :E_fd_unlim_tf, :E_fd_unlim_tpf, 1.0,
            "Unsaturated Field Voltage E_fd_unlim", "E_fd_unlim (p.u.)", "dq_E_fd_unlim.svg"),
    )
    for (fname, key_tf, key_tpf, scale, title, ylab, svg_name) in dq_series
        traj = _extract_gen_time_trajectory(dyn_model_dict, key_tf, key_tpf; scale=scale)
        traj === nothing && continue
        _write_fullbus_csv(joinpath(pf_ts_csv, "$(fname).csv"), t_window, traj, "G")
        if save_ts_plots
            _save_fullbus_svg(t_window, traj, title, ylab, pf_figures, svg_name;
                t_clear_fault=t_clear_fault, label_prefix="G")
        end
    end

    println("DQ generator trajectories saved as CSV in: ", pf_ts_csv,
        save_ts_plots ? " (figures in $pf_figures)" : "")
    return nothing
end

"""
    Save_Gfm_Converter_Results!(dyn_model_dict, dyn_parameters_dict, path_names; …)

Write grid-forming converter trajectories to CSV and optional SVG plots.

Covers the measured states (`P_meas`, `Q_meas`, `V_meas`), both Q–V PI states raw and
clipped, and the current loading `|I| / Imax` — the channel that says whether the limiter
is actually saturating (`1.0` = hard against `Imax`). Two overlay figures put each raw PI
state against its clipped counterpart, so the visible gap is the anti-windup clip at work.

No-op unless the run carried GFM units.
"""
function Save_Gfm_Converter_Results!(
    dyn_model_dict::OrderedDict{Symbol, Any},
    dyn_parameters_dict::OrderedDict{Symbol, Any},
    path_names::OrderedDict{Symbol, String};
    t_clear_fault=nothing,
    save_ts_plots::Bool=false,
)
    meta = get(dyn_model_dict, :meta, OrderedDict{Symbol, Any}())
    gfm_gens = Vector{Int}(get(meta, :gfm_gens, Int[]))
    isempty(gfm_gens) && return nothing
    params = get(meta, :gfm_params, nothing)
    params === nothing && return nothing

    pf_ts_csv = path_names[:pf_TS_CSV]
    mkpath(pf_ts_csv)
    t_window = dyn_parameters_dict[:time][:t_window_total]
    pf_figures = path_names[:pf_TS_figures]
    save_ts_plots && mkpath(pf_figures)

    # --- pre-fault converter set-points and t=0 filter states -------------------
    # `V_set` is the Q–V droop reference the PI integrator works against, and the four
    # t=0 scalars are the initial conditions every transient filter starts from. None of
    # them reached a results file before: they appeared only in the pre-solve `start=`
    # dump under Dispatch_WarmStart/, which records what the solver was *seeded* with,
    # not what it converged to.
    setpoint_cols = (
        ("V_set_pu", :V_set), ("E_int_pu", :E_int), ("P_meas_pu", :P_meas),
        ("Q_meas_pu", :Q_meas), ("V_meas_pu", :V_meas),
    )
    present_cols = [(name, sym) for (name, sym) in setpoint_cols
                    if haskey(dyn_model_dict[:vars], sym)]
    if !isempty(present_cols)
        df_set = DataFrame(gen = ["GFM$(g)" for g in gfm_gens])
        for (name, sym) in present_cols
            container = dyn_model_dict[:vars][sym]
            df_set[!, name] = [haskey(container, g) ? JuMP.value(container[g]) : missing
                               for g in gfm_gens]
        end
        CSV.write(joinpath(pf_ts_csv, "gfm_setpoints.csv"), df_set; delim = ';')
    end

    gfm_series = (
        ("gfm_P_meas", :P_meas_tf, :P_meas_tpf, "Filtered Active Power", "P_meas (p.u.)",
            "gfm_P_meas.svg"),
        ("gfm_Q_meas", :Q_meas_tf, :Q_meas_tpf, "Filtered Reactive Power", "Q_meas (p.u.)",
            "gfm_Q_meas.svg"),
        ("gfm_V_meas", :V_meas_tf, :V_meas_tpf, "Filtered Terminal Voltage", "V_meas (p.u.)",
            "gfm_V_meas.svg"),
        ("gfm_E_int", :E_int_tf, :E_int_tpf, "Q–V PI Integrator (clipped)", "E_int (p.u.)",
            "gfm_E_int.svg"),
        ("gfm_E_int_raw", :E_int_raw_tf, :E_int_raw_tpf, "Q–V PI Integrator (raw)",
            "E_int_raw (p.u.)", "gfm_E_int_raw.svg"),
        ("gfm_E_droop", :E_droop_tf, :E_droop_tpf, "Internal Voltage (clipped)",
            "E_droop (p.u.)", "gfm_E_droop.svg"),
        ("gfm_E_droop_raw", :E_droop_raw_tf, :E_droop_raw_tpf, "Internal Voltage (raw)",
            "E_droop_raw (p.u.)", "gfm_E_droop_raw.svg"),
    )
    traj_cache = Dict{String, OrderedDict{Int, Vector{Float64}}}()
    for (fname, key_tf, key_tpf, title, ylab, svg_name) in gfm_series
        traj = _extract_gen_time_trajectory(dyn_model_dict, key_tf, key_tpf)
        traj === nothing && continue
        traj_cache[fname] = traj
        _write_fullbus_csv(joinpath(pf_ts_csv, "$(fname).csv"), t_window, traj, "GFM")
        if save_ts_plots
            _save_fullbus_svg(t_window, traj, title, ylab, pf_figures, svg_name;
                t_clear_fault=t_clear_fault, label_prefix="GFM ")
        end
    end

    # |I| / Imax is not a stored variable: rebuild it from the dq currents, which the DQ
    # builder creates for every unit, restricted to the converter ids.
    Id = _extract_gen_time_trajectory(dyn_model_dict, :Id_tf, :Id_tpf)
    Iq = _extract_gen_time_trajectory(dyn_model_dict, :Iq_tf, :Iq_tpf)
    if Id !== nothing && Iq !== nothing
        loading = OrderedDict{Int, Vector{Float64}}()
        for gen in gfm_gens
            (haskey(Id, gen) && haskey(Iq, gen) && haskey(params, gen)) || continue
            Imax = params[gen].Imax
            Imax > 0 || continue
            loading[gen] = sqrt.(Id[gen] .^ 2 .+ Iq[gen] .^ 2) ./ Imax
        end
        if !isempty(loading)
            _write_fullbus_csv(joinpath(pf_ts_csv, "gfm_current_loading.csv"),
                t_window, loading, "GFM")
            if save_ts_plots
                _save_fullbus_svg(t_window, loading, "Converter Current Loading",
                    "|I| / Imax (-)", pf_figures, "gfm_current_loading.svg";
                    t_clear_fault=t_clear_fault, label_prefix="GFM ")
            end
        end
    end

    # Raw vs clipped overlays: the gap between the pair is the anti-windup clip acting.
    if save_ts_plots
        for (raw_key, clip_key, title, ylab, svg_name) in (
            ("gfm_E_int_raw", "gfm_E_int", "Q–V PI Integrator: raw vs clipped",
                "E_int (p.u.)", "gfm_E_int_clip.svg"),
            ("gfm_E_droop_raw", "gfm_E_droop", "Internal Voltage: raw vs clipped",
                "E_droop (p.u.)", "gfm_E_droop_clip.svg"),
        )
            (haskey(traj_cache, raw_key) && haskey(traj_cache, clip_key)) || continue
            _save_fullbus_svg(t_window, traj_cache[clip_key], title, ylab,
                pf_figures, svg_name; t_clear_fault=t_clear_fault,
                label_prefix="GFM ", overlay=traj_cache[raw_key],
                overlay_label_suffix=" raw")
        end
    end

    println("GFM converter trajectories saved as CSV in: ", pf_ts_csv,
        save_ts_plots ? " (figures in $pf_figures)" : "")
    return nothing
end

# ===================================================================================
# Per-(window, generator, step) diagnostic dumps  —  Transient_Stability/CSV/Debug/
# ===================================================================================
# Column layout mirrors the reference implementation's Debug/ folder so a parity check is
# a direct CSV diff. Each row carries the value, its distance to the relevant limit, and
# the dual, so "was this binding at t, and what did it cost?" is one line rather than a
# join across four files. Nothing here is stored by the builders: the limiter algebra and
# the filter coefficients are recomputed from solved primal values plus `meta[:gfm_params]`.

"""`(period_label, suffix, time_vector)` for each window present in the run."""
function _ts_debug_windows(dyn_parameters_dict::OrderedDict{Symbol, Any})
    t = dyn_parameters_dict[:time]
    out = Tuple{String, String, Vector{Float64}}[]
    haskey(t, :t_window_fault) && push!(out, ("fault", "tf", t[:t_window_fault]))
    if haskey(t, :t_window_postf) && !isempty(t[:t_window_postf])
        push!(out, ("postfault", "tpf", t[:t_window_postf]))
    end
    return out
end

"""Solved value of `container[gen][t]`, or `NaN` when the entry is absent."""
function _dbg_val(container, gen::Int, t::Int)
    container === nothing && return NaN
    haskey(container, gen) || return NaN
    inner = container[gen]
    haskey(inner, t) || return NaN
    return Float64(JuMP.value(inner[t]))
end

"""Dual of `dyn_model_dict[store][key][gen][t]`, or `NaN` when unavailable."""
function _dbg_dual(dyn_model_dict::OrderedDict{Symbol, Any}, store::Symbol,
                   key::Symbol, gen::Int, t::Int)
    s = get(dyn_model_dict, store, nothing)
    s === nothing && return NaN
    haskey(s, key) || return NaN
    fam = s[key]
    haskey(fam, gen) || return NaN
    haskey(fam[gen], t) || return NaN
    try
        return Float64(JuMP.dual(fam[gen][t]))
    catch
        return NaN
    end
end

_dbg_vars(dyn_model_dict, name::String, suffix::String) =
    get(dyn_model_dict[:vars], Symbol(name, "_", suffix), nothing)

"""
Filter states, coefficients and duals — one row per (window, gen, step, channel).

`α` / `β` are the coefficients of the update actually written into the model, which
depends on `meta[:gfm_integrator]` (and, under `:follow_ode_first_step`, on
`meta[:ode_first_step]` at `t = 1`):

| scheme | row | α | β |
|---|---|---|---|
| backward Euler | `x_t = α·x_prev + β·u_t` | `Tf/(Tf+Δt)` | `Δt/(Tf+Δt)` |
| trapezoidal | `x_t = α·x_prev + β·(u_t + u_prev)` | `(1−c)/(1+c)` | `c/(1+c)`, `c = Δt/(2Tf)` |

The trapezoidal row needs `u_prev`, which is deliberately not its own column (the header
is asserted verbatim by the parity test): it is `u_t` on the previous row of the same
`(period, gen, channel)` group.
"""
function _write_gfm_filter_debug!(io_path, dyn_model_dict, windows, gfm_gens, params, Δt)
    rows = NamedTuple[]
    meta = dyn_model_dict[:meta]
    gfm_int = get(meta, :gfm_integrator, :backward_euler)
    ode_fs = get(meta, :ode_first_step, :trapezoidal)
    be_at(t) = gfm_int === :backward_euler ||
        (gfm_int === :follow_ode_first_step && t == 1 && ode_fs === :backward_euler)
    for (period, sfx, tvec) in windows
        chans = (
            ("P", _dbg_vars(dyn_model_dict, "P_meas", sfx), _dbg_vars(dyn_model_dict, "Pe", sfx),
                Symbol("eq_const_gfm_filter_P_", sfx)),
            ("Q", _dbg_vars(dyn_model_dict, "Q_meas", sfx), _dbg_vars(dyn_model_dict, "Qe", sfx),
                Symbol("eq_const_gfm_filter_Q_", sfx)),
            ("V", _dbg_vars(dyn_model_dict, "V_meas", sfx), nothing,
                Symbol("eq_const_gfm_filter_V_", sfx)),
        )
        V_bus = _dbg_vars(dyn_model_dict, "V", sfx)
        prev_anchor = _gfm_debug_prev_anchor(dyn_model_dict, sfx)
        for gen in gfm_gens
            p = params[gen]
            Tf = p.Tf
            c = Tf <= 1e-9 ? 0.0 : Δt / (2.0 * Tf)
            for (ch, state, input, dual_key) in chans
                for t in eachindex(tvec)
                    α, β = if Tf <= 1e-9
                        0.0, 1.0                                  # pass-through bypass
                    elseif be_at(t)
                        Tf / (Tf + Δt), Δt / (Tf + Δt)
                    else
                        (1 - c) / (1 + c), c / (1 + c)
                    end
                    x_t = _dbg_val(state, gen, t)
                    x_prev = t == 1 ? prev_anchor(ch, gen) : _dbg_val(state, gen, t - 1)
                    u_t = ch == "V" ?
                        (V_bus === nothing ? NaN : _dbg_val(V_bus, p.bus, t)) :
                        _dbg_val(input, gen, t)
                    push!(rows, (
                        period = period, gen = gen, t_idx = t, t_s = tvec[t], channel = ch,
                        Tf = Tf, Δt = Δt, Tf_over_Δt = Tf / Δt, α = α, β = β,
                        x_t = x_t, x_prev = x_prev, u_t = u_t,
                        x_minus_u = x_t - u_t, x_minus_prev = x_t - x_prev,
                        dual_filter = _dbg_dual(dyn_model_dict, :eq_const, dual_key, gen, t),
                    ))
                end
            end
        end
    end
    isempty(rows) || CSV.write(io_path, DataFrame(rows); delim = ';')
    return length(rows)
end

"""`t = 1` previous value for a filter channel: pre-fault scalar, or last fault-on step."""
function _gfm_debug_prev_anchor(dyn_model_dict::OrderedDict{Symbol, Any}, sfx::String)
    vars = dyn_model_dict[:vars]
    return function (ch::String, gen::Int)
        key = ch == "P" ? :P_meas : ch == "Q" ? :Q_meas : :V_meas
        if sfx == "tf"
            c = get(vars, key, nothing)
            (c === nothing || !haskey(c, gen)) && return NaN
            return Float64(JuMP.value(c[gen]))
        end
        c = get(vars, Symbol(key, "_tf"), nothing)
        (c === nothing || !haskey(c, gen)) && return NaN
        return Float64(JuMP.value(last(c[gen])[2]))
    end
end

"""Current-limiter geometry, margins and duals — one row per (window, gen, step)."""
function _write_gfm_limiter_debug!(io_path, dyn_model_dict, windows, gfm_gens, params)
    rows = NamedTuple[]
    for (period, sfx, tvec) in windows
        V_bus = _dbg_vars(dyn_model_dict, "V", sfx)
        θ_bus = _dbg_vars(dyn_model_dict, "θ", sfx)
        δ_w = _dbg_vars(dyn_model_dict, "δ", sfx)
        Id_w = _dbg_vars(dyn_model_dict, "Id", sfx)
        Iq_w = _dbg_vars(dyn_model_dict, "Iq", sfx)
        E_dr = _dbg_vars(dyn_model_dict, "E_droop", sfx)
        for gen in gfm_gens
            p = params[gen]
            Xl, Imax = p.Xl, p.Imax
            Imax_Xl = Imax * Xl
            bypassed = Imax >= _GFM_IMAX_NO_LIMIT
            for t in eachindex(tvec)
                Vmag = _dbg_val(V_bus, p.bus, t)
                θ = _dbg_val(θ_bus, p.bus, t)
                δ = _dbg_val(δ_w, gen, t)
                Ed = _dbg_val(E_dr, gen, t)
                Id = _dbg_val(Id_w, gen, t)
                Iq = _dbg_val(Iq_w, gen, t)
                Vd = Vmag * sin(δ - θ)
                Vq = Vmag * cos(δ - θ)
                ΔVd = Ed - Vq
                ΔVq = Vd
                ΔVraw = sqrt(ΔVd^2 + ΔVq^2)
                Iraw = ΔVraw / Xl
                Iout = sqrt(Id^2 + Iq^2)
                den_Xl = bypassed ? ΔVraw : max(ΔVraw, Imax_Xl)
                scale = bypassed ? 1.0 : (den_Xl == 0 ? 1.0 : Imax_Xl / den_Xl)
                push!(rows, (
                    period = period, gen = gen, t_idx = t, t_s = tvec[t], bus = p.bus,
                    Xl = Xl, Imax = Imax, Vd = Vd, Vq = Vq, ΔVd = ΔVd, ΔVq = ΔVq,
                    ΔVraw = ΔVraw, ΔVlim = Imax_Xl, ΔV_margin = Imax_Xl - ΔVraw,
                    Iraw = Iraw, Iout = Iout, I_margin = Imax - Iout,
                    Iout_over_Imax = Imax == 0 ? NaN : Iout / Imax,
                    transition_distance_I = Iraw - Imax,
                    Iraw_Xl = ΔVraw, Iout_Xl = Xl * Iout, Imax_Xl = Imax_Xl,
                    den_Xl = den_Xl, scale = scale,
                    dual_c_Id = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_gfm_limiter_Id_", sfx), gen, t),
                    dual_c_Iq = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_gfm_limiter_Iq_", sfx), gen, t),
                    dual_c_Pe = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_gfm_Pe_", sfx), gen, t),
                    dual_c_Qe = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_gfm_Qe_", sfx), gen, t),
                ))
            end
        end
    end
    isempty(rows) || CSV.write(io_path, DataFrame(rows); delim = ';')
    return length(rows)
end

"""Q–V PI states, clip distances and duals — one row per (window, gen, step)."""
function _write_gfm_voltage_debug!(io_path, dyn_model_dict, windows, gfm_gens, params)
    rows = NamedTuple[]
    for (period, sfx, tvec) in windows
        V_meas = _dbg_vars(dyn_model_dict, "V_meas", sfx)
        Q_meas = _dbg_vars(dyn_model_dict, "Q_meas", sfx)
        E_int_raw = _dbg_vars(dyn_model_dict, "E_int_raw", sfx)
        E_int = _dbg_vars(dyn_model_dict, "E_int", sfx)
        E_dr_raw = _dbg_vars(dyn_model_dict, "E_droop_raw", sfx)
        E_dr = _dbg_vars(dyn_model_dict, "E_droop", sfx)
        V_set = get(dyn_model_dict[:vars], :V_set, nothing)
        for gen in gfm_gens
            p = params[gen]
            vset = (V_set === nothing || !haskey(V_set, gen)) ? NaN :
                Float64(JuMP.value(V_set[gen]))
            for t in eachindex(tvec)
                err = vset - p.mq * _dbg_val(Q_meas, gen, t) - _dbg_val(V_meas, gen, t)
                eir = _dbg_val(E_int_raw, gen, t); ei = _dbg_val(E_int, gen, t)
                edr = _dbg_val(E_dr_raw, gen, t); ed = _dbg_val(E_dr, gen, t)
                push!(rows, (
                    period = period, gen = gen, t_idx = t, t_s = tvec[t],
                    Emin = p.Emin, Emax = p.Emax, voltage_error = err,
                    E_int_raw = eir, E_int = ei,
                    E_int_to_Emax = p.Emax - ei, E_int_to_Emin = ei - p.Emin,
                    E_droop_raw = edr, E_droop = ed,
                    E_droop_to_Emax = p.Emax - ed, E_droop_to_Emin = ed - p.Emin,
                    dual_Eint_raw = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_gfm_Eint_raw_", sfx), gen, t),
                    dual_Eint_clip = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_gfm_Eint_clip_", sfx), gen, t),
                    dual_Edroop_raw = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_gfm_Edroop_raw_", sfx), gen, t),
                    dual_Edroop_clip = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_gfm_Edroop_clip_", sfx), gen, t),
                ))
            end
        end
    end
    isempty(rows) || CSV.write(io_path, DataFrame(rows); delim = ';')
    return length(rows)
end

"""
SG swing states with their δ–COI (and Δω–COI) corridor margins.

The reference columns come first so a diff lines up; the package extras follow. `δ_util`
is the corridor analogue of `Iout_over_Imax` in the limiter file: `1.0` means the machine
sits on the stability corridor, and the matching dual is what holding it there costs.
`dual_accel` is `NaN` by construction — the package inlines the accelerating power into
the swing rows instead of carrying a separate `a` constraint, so the *value* is available
but there is no row to price.
"""
function _write_swing_debug!(io_path, dyn_model_dict, dyn_parameters_dict, DGEN_DYN,
                             windows, sg_gens)
    rows = NamedTuple[]
    meta = dyn_model_dict[:meta]
    Δt = dyn_parameters_dict[:time][:t_step]
    # δ_tol is the (lower, upper) corridor in **radians**, matching the δ variables here;
    # angle_rel_COI.csv reports degrees, so compare ratios, not raw values.
    δ_tol = get(dyn_parameters_dict[:common], :δ_tol, (NaN, NaN))
    Δω_tol = get(meta, :Δω_tol, (NaN, NaN))
    has_Δω_corridor = get(meta, :constrain_Δω_COI, false)
    for (period, sfx, tvec) in windows
        δ_w = _dbg_vars(dyn_model_dict, "δ", sfx)
        Δω_w = _dbg_vars(dyn_model_dict, "Δω", sfx)
        Pe_w = _dbg_vars(dyn_model_dict, "Pe", sfx)
        Pm_w = _dbg_vars(dyn_model_dict, "Pm", sfx)
        δCOI = get(dyn_model_dict[:vars], Symbol("δCOI_", sfx), nothing)
        ΔωCOI = get(dyn_model_dict[:vars], Symbol("ΔωCOI_", sfx), nothing)
        P_m0 = get(dyn_model_dict[:vars], :P_m, nothing)
        for gen in sg_gens
            H = Float64(DGEN_DYN.H[gen]); D = Float64(DGEN_DYN.D[gen])
            for t in eachindex(tvec)
                δ = _dbg_val(δ_w, gen, t)
                Δω = _dbg_val(Δω_w, gen, t)
                δ_prev = t == 1 ? NaN : _dbg_val(δ_w, gen, t - 1)
                Δω_prev = t == 1 ? NaN : _dbg_val(Δω_w, gen, t - 1)
                Pm = Pm_w !== nothing ? _dbg_val(Pm_w, gen, t) :
                    (P_m0 !== nothing && haskey(P_m0, gen) ?
                        Float64(JuMP.value(P_m0[gen])) : NaN)
                Pe = _dbg_val(Pe_w, gen, t)
                a = H == 0 ? NaN : (Pm - Pe - D * Δω) / (2 * H)
                coi = δCOI === nothing || !haskey(δCOI, t) ? NaN :
                    Float64(JuMP.value(δCOI[t]))
                δ_rel = δ - coi
                ω_coi = ΔωCOI === nothing || !haskey(ΔωCOI, t) ? NaN :
                    Float64(JuMP.value(ΔωCOI[t]))
                Δω_rel = Δω - ω_coi
                push!(rows, (
                    period = period, gen = gen, t_idx = t, t_s = tvec[t],
                    H = H, D = D, Δt = Δt, a = a,
                    Δω_curr = Δω, Δω_prev = Δω_prev, Δω_step = Δω - Δω_prev,
                    Δω_step_over_Δt = (Δω - Δω_prev) / Δt,
                    δ_curr = δ, δ_prev = δ_prev, δ_step = δ - δ_prev,
                    δ_step_over_Δt = (δ - δ_prev) / Δt,
                    dual_accel = NaN,
                    dual_delta = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_δ_", sfx), gen, t),
                    dual_omega = _dbg_dual(dyn_model_dict, :eq_const,
                        Symbol("eq_const_Δω_", sfx), gen, t),
                    # --- package extras: distance to the stability corridor -------------
                    δ_rel_COI = δ_rel, δ_tol_lo = δ_tol[1], δ_tol_hi = δ_tol[2],
                    δ_margin_lo = δ_rel - δ_tol[1], δ_margin_hi = δ_tol[2] - δ_rel,
                    δ_util = _corridor_utilisation(δ_rel, δ_tol),
                    dual_δ_COI_lower = _dbg_dual(dyn_model_dict, :ineq_const,
                        Symbol("ineq_const_δ_COI_", sfx, "_lower"), gen, t),
                    dual_δ_COI_upper = _dbg_dual(dyn_model_dict, :ineq_const,
                        Symbol("ineq_const_δ_COI_", sfx, "_upper"), gen, t),
                    Δω_rel_COI = Δω_rel,
                    Δω_margin_lo = has_Δω_corridor ? Δω_rel - Δω_tol[1] : NaN,
                    Δω_margin_hi = has_Δω_corridor ? Δω_tol[2] - Δω_rel : NaN,
                    Δω_util = has_Δω_corridor ? _corridor_utilisation(Δω_rel, Δω_tol) : NaN,
                    dual_Δω_COI_lower = _dbg_dual(dyn_model_dict, :ineq_const,
                        Symbol("ineq_const_Δω_COI_", sfx, "_lower"), gen, t),
                    dual_Δω_COI_upper = _dbg_dual(dyn_model_dict, :ineq_const,
                        Symbol("ineq_const_Δω_COI_", sfx, "_upper"), gen, t),
                ))
            end
        end
    end
    isempty(rows) || CSV.write(io_path, DataFrame(rows); delim = ';')
    return length(rows)
end

"""
Fraction of the corridor a deviation uses: `1.0` = sitting on the limit.

Each side is measured against its own tolerance, so an asymmetric corridor
(`δ_tol_deg_lower` ≠ `δ_tol_deg_upper`) reports honestly on both sides.
"""
function _corridor_utilisation(x::Float64, tol::Tuple{Float64, Float64})::Float64
    (isnan(x) || isnan(tol[1]) || isnan(tol[2])) && return NaN
    lim = x < 0 ? abs(tol[1]) : abs(tol[2])
    lim == 0 && return NaN
    return abs(x) / lim
end

"""
    Save_TS_Debug_CSV!(dyn_model_dict, dyn_parameters_dict, DGEN_DYN, path_names)

Write the per-(window, generator, step) diagnostic dumps under
`Transient_Stability/CSV/Debug/`. Gated by `RunConfig.save_ts_debug_csv` (default `false`)
because the filter file alone is three rows per converter per step.
"""
function Save_TS_Debug_CSV!(
    dyn_model_dict::OrderedDict{Symbol, Any},
    dyn_parameters_dict::OrderedDict{Symbol, Any},
    DGEN_DYN::DataFrame,
    path_names::OrderedDict{Symbol, String},
)
    windows = _ts_debug_windows(dyn_parameters_dict)
    isempty(windows) && return nothing
    pf_debug = joinpath(path_names[:pf_TS_CSV], "Debug")
    mkpath(pf_debug)

    meta = get(dyn_model_dict, :meta, OrderedDict{Symbol, Any}())
    gfm_gens = Vector{Int}(get(meta, :gfm_gens, Int[]))
    params = get(meta, :gfm_params, nothing)
    Δt = dyn_parameters_dict[:time][:t_step]
    written = String[]

    if !isempty(gfm_gens) && params !== nothing
        n = _write_gfm_filter_debug!(joinpath(pf_debug, "gfm_filter_debug.csv"),
            dyn_model_dict, windows, gfm_gens, params, Δt)
        n > 0 && push!(written, "gfm_filter_debug.csv")
        n = _write_gfm_limiter_debug!(joinpath(pf_debug, "gfm_limiter_debug.csv"),
            dyn_model_dict, windows, gfm_gens, params)
        n > 0 && push!(written, "gfm_limiter_debug.csv")
        n = _write_gfm_voltage_debug!(joinpath(pf_debug, "gfm_voltage_debug.csv"),
            dyn_model_dict, windows, gfm_gens, params)
        n > 0 && push!(written, "gfm_voltage_debug.csv")
    end

    sg_gens = Vector{Int}(get(meta, :sg_gens, Int[]))
    if isempty(sg_gens)
        sg_gens = Vector{Int}(get(dyn_model_dict, :active_gen, Int[]))
    end
    if !isempty(sg_gens)
        n = _write_swing_debug!(joinpath(pf_debug, "swing_debug.csv"),
            dyn_model_dict, dyn_parameters_dict, DGEN_DYN, windows, sg_gens)
        n > 0 && push!(written, "swing_debug.csv")
    end

    println("TS debug CSVs saved in: ", pf_debug, " (", join(written, ", "), ")")
    return nothing
end

"""Concatenate fault-on and post-fault per-bus time series."""
function _merge_bus_time_dict(
    tf::OrderedDict{Int, Vector{Float64}},
    tpf::Union{Nothing, OrderedDict{Int, Vector{Float64}}},
)
    out = OrderedDict{Int, Vector{Float64}}()
    for k in keys(tf)
        out[k] = tpf === nothing ? tf[k] : vcat(tf[k], tpf[k])
    end
    return out
end

"""Extract per-bus trajectories from nested JuMP variable containers."""
function _fullbus_values_from_vars(
    tf_vars::OrderedDict,
    tpf_vars::Union{Nothing, OrderedDict};
    scale::Float64=1.0,
    to_deg::Bool=false,
)
    tf_vals = OrderedDict(
        bus => scale .* [JuMP.value(v) for (_, v) in inner]
        for (bus, inner) in tf_vars)
    tpf_vals = tpf_vars === nothing ? nothing : OrderedDict(
        bus => scale .* [JuMP.value(v) for (_, v) in inner]
        for (bus, inner) in tpf_vars)
    merged = _merge_bus_time_dict(tf_vals, tpf_vals)
    to_deg && (merged = OrderedDict(k => rad2deg.(v) for (k, v) in merged))
    return merged
end

"""Extract per-bus nodal injection trajectories from stored JuMP expressions."""
function _fullbus_values_from_exprs(
    tf_exprs::OrderedDict,
    tpf_exprs::Union{Nothing, OrderedDict};
    scale::Float64=1.0,
)
    tf_vals = OrderedDict(
        bus => scale .* [JuMP.value(expr) for (_, expr) in inner]
        for (bus, inner) in tf_exprs)
    tpf_vals = tpf_exprs === nothing ? nothing : OrderedDict(
        bus => scale .* [JuMP.value(expr) for (_, expr) in inner]
        for (bus, inner) in tpf_exprs)
    return _merge_bus_time_dict(tf_vals, tpf_vals)
end

"""
    Save_FullBus_Network_Results!(dyn_model_dict, dyn_parameters_dict, base_MVA, path_names)

Save bus-level active/reactive power, voltage magnitude, and angle as CSV and SVG.
Called from `Save_Results_Dynamic_Model` when `meta[:network_form] == "FULL_BUS"`.
"""
function Save_FullBus_Network_Results!(
    dyn_model_dict::OrderedDict{Symbol, Any},
    dyn_parameters_dict::OrderedDict{Symbol, Any},
    base_MVA::Float64,
    path_names::OrderedDict{Symbol, String};
    t_clear_fault=nothing,
    save_ts_plots::Bool=false,
)
    t_window = dyn_parameters_dict[:time][:t_window_total]
    pf_ts_csv = path_names[:pf_TS_CSV]
    mkpath(pf_ts_csv)
    pf_figures = path_names[:pf_TS_figures]
    save_ts_plots && mkpath(pf_figures)

    V_t = _fullbus_values_from_vars(
        dyn_model_dict[:vars][:V_tf],
        get(dyn_model_dict[:vars], :V_tpf, nothing))
    θ_t = _fullbus_values_from_vars(
        dyn_model_dict[:vars][:θ_tf],
        get(dyn_model_dict[:vars], :θ_tpf, nothing);
        to_deg=true)
    exprs = dyn_model_dict[:expressions]
    P_bus = _fullbus_values_from_exprs(
        exprs[:P_inj_tf], get(exprs, :P_inj_tpf, nothing); scale=base_MVA)
    Q_bus = _fullbus_values_from_exprs(
        exprs[:Q_inj_tf], get(exprs, :Q_inj_tpf, nothing); scale=base_MVA)

    Pet = _extract_gen_time_trajectory(dyn_model_dict, :Pe_tf, :Pe_tpf; scale=base_MVA)
    Qet = _extract_gen_time_trajectory(dyn_model_dict, :Qe_tf, :Qe_tpf; scale=base_MVA)

    _write_fullbus_csv(joinpath(pf_ts_csv, "bus_active_power.csv"), t_window, P_bus, "Bus_")
    _write_fullbus_csv(joinpath(pf_ts_csv, "bus_reactive_power.csv"), t_window, Q_bus, "Bus_")
    _write_fullbus_csv(joinpath(pf_ts_csv, "bus_voltage_magnitude.csv"), t_window, V_t, "Bus_")
    _write_fullbus_csv(joinpath(pf_ts_csv, "bus_voltage_angle.csv"), t_window, θ_t, "Bus_")

    if Pet !== nothing
        _write_fullbus_csv(joinpath(pf_ts_csv, "generator_active_power.csv"), t_window, Pet, "G")
    end
    if Qet !== nothing
        _write_fullbus_csv(joinpath(pf_ts_csv, "generator_reactive_power.csv"), t_window, Qet, "G")
    end

    if save_ts_plots
        _save_fullbus_svg(t_window, P_bus, "Bus Active Power", "P (MW)", pf_figures,
            "Bus active power vs time.svg"; t_clear_fault=t_clear_fault)
        _save_fullbus_svg(t_window, Q_bus, "Bus Reactive Power", "Q (MVAr)", pf_figures,
            "Bus reactive power vs time.svg"; t_clear_fault=t_clear_fault)
        _save_fullbus_svg(t_window, V_t, "Bus Voltage Magnitude", "V (p.u.)", pf_figures,
            "Bus voltage magnitude vs time.svg"; t_clear_fault=t_clear_fault)
        _save_fullbus_svg(t_window, θ_t, "Bus Voltage Angle", "θ (deg)", pf_figures,
            "Bus voltage angle vs time.svg"; t_clear_fault=t_clear_fault)
        Pet !== nothing && _save_fullbus_svg(t_window, Pet, "Generator Active Power", "P_e (MW)", pf_figures,
            "Generator active power vs time.svg"; t_clear_fault=t_clear_fault, label_prefix="G")
        Qet !== nothing && _save_fullbus_svg(t_window, Qet, "Generator Reactive Power", "Q_e (MVAr)", pf_figures,
            "Generator reactive power vs time.svg"; t_clear_fault=t_clear_fault, label_prefix="G")
    end

    println("FULL_BUS network trajectories saved as CSV in: ", pf_ts_csv,
        save_ts_plots ? " (figures in $pf_figures)" : "")
    return nothing
end

"""
    _write_fullbus_csv(filename, t_window, data, prefix)

Write `t` + one column per id of `data`, columns ordered by sorted id and named
`"\$(prefix)\$(id)"` (e.g. `Bus_5`, `G2`, `GFM3`).

The header is built here, from the same sorted key vector used to lay out the
matrix, so a caller cannot pass names derived from a *different* dict and have
the columns silently mislabelled — which is what the earlier
`col_names::Vector{String}` signature allowed.
"""
function _write_fullbus_csv(
    filename::String,
    t_window::Vector{Float64},
    data::OrderedDict{Int, Vector{Float64}},
    prefix::String,
)
    ids = sort(collect(keys(data)))
    mat = hcat([data[i] for i in ids]...)
    df = DataFrame(hcat(t_window, mat), vcat("t", ["$(prefix)$(i)" for i in ids]))
    CSV.write(filename, df; delim=';')
    return nothing
end

