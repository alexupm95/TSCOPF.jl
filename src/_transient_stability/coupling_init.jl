#=
================================================================================
 coupling_init.jl — steady-state / dynamics coupling initialisation
================================================================================
 FULL_BUS TSC-ACOPF can seed coupling variables from a solved ACOPF warm start
 (`extract_opf_solved_hints`) or from a synthetic flat start (V=1, θ=0, P_g/Q_g
 from case setpoints) when `RunConfig.use_acopf_warmstart=false`.
================================================================================
=#

"""How pre-fault coupling hints were built (`:acopf_warmstart` or `:flat_start`)."""
const CouplingInitSource = Union{typeof(:acopf_warmstart), typeof(:flat_start)}

"""Human-readable label for exports and `dyn_model_dict[:meta]`."""
coupling_init_source_label(src::CouplingInitSource)::String = string(src)

"""
    build_flat_start_hints(DBUS, DGEN, base_MVA) -> SteadyStateHints

Flat-start recipe: all buses at `V=1` p.u. and `θ=0` rad; active generators at
`P_g = pg_spe/base_MVA` and `Q_g = qg_spe/base_MVA` from `generators_data.csv`.
"""
function build_flat_start_hints(
    DBUS::DataFrame,
    DGEN::DataFrame,
    base_MVA::Real,
)::SteadyStateHints
    val_V = Dict{Int, Float64}(bus => 1.0 for bus in DBUS.bus)
    val_θ = Dict{Int, Float64}(bus => 0.0 for bus in DBUS.bus)
    val_Pg = Dict{Int, Float64}()
    val_Qg = Dict{Int, Float64}()
    bMVA = Float64(base_MVA)
    for gen in 1:nrow(DGEN)
        DGEN.g_status[gen] == 1 || continue
        gid = Int(DGEN.id[gen])
        val_Pg[gid] = Float64(DGEN.pg_spe[gen]) / bMVA
        val_Qg[gid] = Float64(DGEN.qg_spe[gen]) / bMVA
    end
    return SteadyStateHints(val_V, val_θ, val_Pg, val_Qg)
end

"""Copy coupling hints onto steady-state OPF JuMP variables (`start=` only)."""
function apply_steady_state_hints_to_opf!(
    opf_dict::OrderedDict{Symbol, Any},
    hints::SteadyStateHints,
)
    for (bus, v) in opf_dict[:vars][:V]
        haskey(hints.val_V, bus) && JuMP.set_start_value(v, hints.val_V[bus])
    end
    for (bus, v) in opf_dict[:vars][:θ]
        haskey(hints.val_θ, bus) && JuMP.set_start_value(v, hints.val_θ[bus])
    end
    for (gen, v) in opf_dict[:vars][:P_g]
        haskey(hints.val_Pg, gen) && JuMP.set_start_value(v, hints.val_Pg[gen])
    end
    for (gen, v) in opf_dict[:vars][:Q_g]
        haskey(hints.val_Qg, gen) && JuMP.set_start_value(v, hints.val_Qg[gen])
    end
    return nothing
end
