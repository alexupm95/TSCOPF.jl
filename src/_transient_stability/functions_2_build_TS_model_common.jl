#=
================================================================================
 functions_2_build_TS_model_common.jl  —  TS dynamic-model orchestration
================================================================================
 Single entry point for attaching transient-stability constraints to an existing
 steady-state OPF model.

 Phase 1.2 routing
 -----------------
   RunConfig.dyn_model  →  dynamic_gen_model(dyn; linearize)
                         →  assemble_dynamic_model!(gen_model, …)
                         →  export_dynamic_model!(gen_model, …)

 `linearize` is `true` for TSC-DCOPF (Taylor Pe) and `false` for TSC-ACOPF.
================================================================================
=#

# ==================================================================================
# Per-model assembly hooks (extend with new `AbstractDynamicGenModel` subtypes)
# ==================================================================================

"""
    assemble_dynamic_model!(gen_model::ClassicalKronModel, …)

Attach all fault-on / post-fault TS constraints for the classical Kron path.
Delegates to `Make_Dynamic_Model_tsred!` or `Make_Dynamic_Model_tsredlinear!`
depending on `gen_model.linearize`.  Mechanical-power and Δω-COI options are
threaded from the factory product (copied from `DynModelConfig` at build time).
"""
function assemble_dynamic_model!(
    gen_model::ClassicalKronModel,
    model::Model,
    opf_dict::OrderedDict{Symbol, Any},
    path_names::OrderedDict{Symbol, String},
    DBUS::DataFrame,
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    DCIR::DataFrame,
    bus_gen_circ_dict_ON::OrderedDict,
    base_MVA::Float64,
    nBUS::Int64,
    nGEN::Int64,
    nCIR::Int64,
    ts_fault_details::OrderedDict{Symbol, Any};
    simulation::TsSimulationConfig,
    ts_builder::TsBuilderConfig,
    δ_ref::Union{Nothing, OrderedDict{Int64, Float64}}=nothing,
)
    if gen_model.linearize
        δ_ref === nothing && throw(ArgumentError(
            "δ_ref is required when building a linearized ClassicalKronModel (TSC-DCOPF)."))
        return Make_Dynamic_Model_tsredlinear!(
            model, opf_dict, path_names, DBUS, DGEN, DGEN_DYN, DCIR,
            bus_gen_circ_dict_ON, base_MVA, nBUS, nGEN, nCIR,
            ts_fault_details, δ_ref;
            simulation=simulation,
            ts_builder=ts_builder,
            mech_power_mode=gen_model.mech_power_mode,
            constrain_Δω_COI=gen_model.constrain_Δω_COI,
            Δω_tol=gen_model.Δω_tol,
            bound_style=gen_model.bound_style,
        )
    else
        return Make_Dynamic_Model_tsred!(
            model, opf_dict, path_names, DBUS, DGEN, DGEN_DYN, DCIR,
            bus_gen_circ_dict_ON, base_MVA, nBUS, nGEN, nCIR, ts_fault_details;
            simulation=simulation,
            ts_builder=ts_builder,
            mech_power_mode=gen_model.mech_power_mode,
            constrain_Δω_COI=gen_model.constrain_Δω_COI,
            Δω_tol=gen_model.Δω_tol,
            bound_style=gen_model.bound_style,
        )
    end
end

"""
    assemble_dynamic_model!(gen_model::ClassicalFullBusModel, …)

Full-network classical 2nd-order path (Phase 3).  TSC-ACOPF only.
"""
function assemble_dynamic_model!(
    gen_model::ClassicalFullBusModel,
    model::Model,
    opf_dict::OrderedDict{Symbol, Any},
    path_names::OrderedDict{Symbol, String},
    DBUS::DataFrame,
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    DCIR::DataFrame,
    bus_gen_circ_dict_ON::OrderedDict,
    base_MVA::Float64,
    nBUS::Int64,
    nGEN::Int64,
    nCIR::Int64,
    ts_fault_details::OrderedDict{Symbol, Any};
    simulation::TsSimulationConfig,
    ts_builder::TsBuilderConfig,
    δ_ref::Union{Nothing, OrderedDict{Int64, Float64}}=nothing,
    steady_state_hints::SteadyStateHints,
    DGFM::Union{Nothing, DataFrame}=nothing,
)
    δ_ref !== nothing && @warn "δ_ref is ignored for ClassicalFullBusModel (ACOPF only)."
    DGFM !== nothing && nrow(DGFM) > 0 && throw(ArgumentError(
        "GFM units require gen_order=DQ_4TH (ClassicalFullBusModel does not host GFM)."))
    return Make_Dynamic_Model_fullbus!(
        model, opf_dict, path_names, DBUS, DGEN, DGEN_DYN, DCIR,
        bus_gen_circ_dict_ON, base_MVA, nBUS, nGEN, nCIR, ts_fault_details;
        simulation=simulation,
        ts_builder=ts_builder,
        mech_power_mode=gen_model.mech_power_mode,
        bound_style=gen_model.bound_style,
        constrain_Δω_COI=gen_model.constrain_Δω_COI,
        Δω_tol=gen_model.Δω_tol,
        zip_load_p=gen_model.zip_load_p,
        zip_load_q=gen_model.zip_load_q,
        include_governor=gen_model.include_governor,
        governor_limiter=gen_model.governor_limiter,
        ode_first_step=gen_model.ode_first_step,
        steady_state_hints=steady_state_hints,
    )
end

"""
    assemble_dynamic_model!(gen_model::DqFullBusModel, …)

Fourth-order dq machine on the full network (TSC-ACOPF only).
Optional `DGFM` hosts GFM units (Phase G0 stubs; physics in G1+).
"""
function assemble_dynamic_model!(
    gen_model::DqFullBusModel,
    model::Model,
    opf_dict::OrderedDict{Symbol, Any},
    path_names::OrderedDict{Symbol, String},
    DBUS::DataFrame,
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    DCIR::DataFrame,
    bus_gen_circ_dict_ON::OrderedDict,
    base_MVA::Float64,
    nBUS::Int64,
    nGEN::Int64,
    nCIR::Int64,
    ts_fault_details::OrderedDict{Symbol, Any};
    simulation::TsSimulationConfig,
    ts_builder::TsBuilderConfig,
    δ_ref::Union{Nothing, OrderedDict{Int64, Float64}}=nothing,
    steady_state_hints::SteadyStateHints,
    DGFM::Union{Nothing, DataFrame}=nothing,
)
    δ_ref !== nothing && @warn "δ_ref is ignored for DqFullBusModel (ACOPF only)."
    return Make_Dynamic_Model_dqfullbus!(
        model, opf_dict, path_names, DBUS, DGEN, DGEN_DYN, DCIR,
        bus_gen_circ_dict_ON, base_MVA, nBUS, nGEN, nCIR, ts_fault_details;
        simulation=simulation,
        ts_builder=ts_builder,
        mech_power_mode=gen_model.mech_power_mode,
        bound_style=gen_model.bound_style,
        constrain_Δω_COI=gen_model.constrain_Δω_COI,
        Δω_tol=gen_model.Δω_tol,
        zip_load_p=gen_model.zip_load_p,
        zip_load_q=gen_model.zip_load_q,
        dq_speed_dev_in_algebra=gen_model.dq_speed_dev_in_algebra,
        include_avr=gen_model.include_avr,
        include_governor=gen_model.include_governor,
        governor_limiter=gen_model.governor_limiter,
        ode_first_step=gen_model.ode_first_step,
        gfm_integrator=gen_model.gfm_integrator,
        steady_state_hints=steady_state_hints,
        DGFM=DGFM,
    )
end

"""Write the human-readable dynamic-model dump (TXT) for the selected model type."""
function export_dynamic_model!(
    gen_model::ClassicalFullBusModel,
    model::Model,
    path_names::OrderedDict{Symbol, String},
    dyn_model_dict::OrderedDict{Symbol, Any},
)
    Export_Dynamic_Model_fullbus(model, path_names, dyn_model_dict)
    Export_Variable_Bounds!(model, path_names[:pf_TS])
    return nothing
end

function export_dynamic_model!(
    gen_model::DqFullBusModel,
    model::Model,
    path_names::OrderedDict{Symbol, String},
    dyn_model_dict::OrderedDict{Symbol, Any},
)
    Export_Dynamic_Model_fullbus(model, path_names, dyn_model_dict)
    Export_Variable_Bounds!(model, path_names[:pf_TS])
    return nothing
end

# Export for Kron models
function export_dynamic_model!(
    gen_model::ClassicalKronModel,
    model::Model,
    path_names::OrderedDict{Symbol, String},
    dyn_model_dict::OrderedDict{Symbol, Any},
)
    if gen_model.linearize
        Export_Dynamic_Model_tsredlinear(model, path_names, dyn_model_dict)
    else
        Export_Dynamic_Model_tsred(model, path_names, dyn_model_dict)
    end
    Export_Variable_Bounds!(model, path_names[:pf_TS])
    return nothing
end

# ==================================================================================
# Top-level builder — called from engine.jl when cfg.trans_stab == true
# ==================================================================================

"""
    Build_Dynamic_Model!(…, dyn::DynModelConfig; linearize, δ_ref, …)

Orchestration sequence:
  1. Read contingency row from `contingencies.csv`
  2. `dynamic_gen_model(dyn; linearize)` — pick concrete physics path
  3. `assemble_dynamic_model!` — add JuMP constraints
  4. `register_dyn_model_meta!` — stamp model type into `dyn_model_dict[:meta]`
  5. `export_dynamic_model!` — write model structure TXT for inspection
"""
function Build_Dynamic_Model!(
    path_names::OrderedDict{Symbol, String},
    model::Model,
    opf_dict::OrderedDict{Symbol, Any},
    DBUS::DataFrame,
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    DCIR::DataFrame,
    bus_gen_circ_dict_ON::OrderedDict,
    base_MVA::Float64,
    nBUS::Int64,
    nGEN::Int64,
    nCIR::Int64,
    transient::TransientConfig;
    linearize::Bool,
    δ_ref::Union{Nothing, OrderedDict{Int64, Float64}}=nothing,
    steady_state_hints::Union{Nothing, SteadyStateHints}=nothing,
    DGFM::Union{Nothing, DataFrame}=nothing,
)
    dyn = transient.dyn_model
    ts_fault_details = build_fault_details(dyn.fault, path_names[:pf_input_files])
    gen_model = dynamic_gen_model(dyn; linearize=linearize)

    ts_fault_details[:reduced_model] = (network_form(gen_model) == KRON_REDUCED)

    # Phase G2: GFM fault/post-fault attachers run inside Define_*_dq!; COI/swing/EMF SG-only.

    if gen_model isa ClassicalFullBusModel || gen_model isa DqFullBusModel
        steady_state_hints === nothing && throw(ArgumentError(
            "FULL_BUS requires SteadyStateHints from the mandatory ACOPF warm start."))
        model, dyn_model_dict, dyn_parameters_dict = assemble_dynamic_model!(
            gen_model, model, opf_dict, path_names, DBUS, DGEN, DGEN_DYN, DCIR,
            bus_gen_circ_dict_ON, base_MVA, nBUS, nGEN, nCIR, ts_fault_details;
            simulation=transient.simulation,
            ts_builder=transient.builder,
            steady_state_hints=steady_state_hints,
            DGFM=DGFM)
    else
        model, dyn_model_dict, dyn_parameters_dict = assemble_dynamic_model!(
            gen_model, model, opf_dict, path_names, DBUS, DGEN, DGEN_DYN, DCIR,
            bus_gen_circ_dict_ON, base_MVA, nBUS, nGEN, nCIR, ts_fault_details;
            simulation=transient.simulation,
            ts_builder=transient.builder,
            δ_ref=δ_ref)
    end
    register_dyn_model_meta!(dyn_model_dict, gen_model)
    export_dynamic_model!(gen_model, model, path_names, dyn_model_dict)

    return model, dyn_model_dict, dyn_parameters_dict
end
