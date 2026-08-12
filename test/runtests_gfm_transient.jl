#=
================================================================================
 test/runtests_gfm_transient.jl — mixed SG + GFM transient solve (Phase G3.3)
================================================================================
 Heavy tier. One end-to-end SC solve on `9bus_gfm` (3 SG + 1 GFM at bus 5,
 Imax = 1.2 pu so the current limiter is actually built), contingency 2:
 three-phase fault at bus 7 cleared after 150 ms by opening branch 5–7.

 Guards the GFM knob/dual wiring: named equality families reach the model, the
 GFM boxes are materialised in the configured encoding, and both encodings give
 the same optimum.
================================================================================
=#

using Test
using JuMP
using CSV
using DataFrames
using DataStructures: OrderedDict

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))

const GFM_SOLVED = (OPTIMAL, LOCALLY_SOLVED, ALMOST_LOCALLY_SOLVED, ITERATION_LIMIT)
const GFM_ZIP = (1.0, 0.0, 0.0)

function gfm_config(; encoding::BoundEncoding = TSCOPF.CONSTRAINT,
                     builder_kwargs::NamedTuple = NamedTuple(),
                     ode_first_step::Symbol = :trapezoidal,
                     save_ts_debug_csv::Bool = false,
                     save_ts_plots::Bool = false,
                     bound_style_δ::Symbol = :coi_box,
                     δ_ref_gen_id::Union{Nothing, Int} = nothing,
                     constrain_Δω::Bool = false,
                     bound_style_Δω::Symbol = :coi_box,
                     fault::FaultConfig = FaultConfig(fault_type = SC, contingency_id = 2))
    return RunConfig(;
        trans_stab = true,
        case = "9bus_gfm",
        solver_name = "Ipopt",
        silent_solver = true,
        save_duals = true,
        save_ts_plots = save_ts_plots,
        save_ts_debug_csv = save_ts_debug_csv,
        save_optim_matrices = false,
        overwrite_results = TEST_OVERWRITE_RESULTS,
        load_factor = 1.5,
        dispatch = DispatchConfig(type_model = "ACOPF", use_matrix = true),
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data_full.csv",
            gfm_dynamic_filename = "gfm_dynamic_data.csv",
            simulation = TsSimulationConfig(
                δ_tol_deg = 100.0, t_end_sim = 0.6, t_step = 0.02,
                t_start_fault = 0.01, clearing_time = 0.15),
            builder = TsBuilderConfig(; bound_encoding = encoding, builder_kwargs...),
            dyn_model = DynModelConfig(
                allow_gfm = true,
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style_δ = bound_style_δ,
                δ_ref_gen_id = δ_ref_gen_id,
                constrain_Δω = constrain_Δω,
                bound_style_Δω = bound_style_Δω,
                zip_load_p = GFM_ZIP,
                zip_load_q = GFM_ZIP,
                dq_speed_dev_in_algebra = true,
                ode_first_step = ode_first_step,
                fault = fault,
            ),
        ),
    )
end

function gfm_run(; kwargs...)
    cfg = gfm_config(; kwargs...)
    validate_dyn_config!(cfg)
    sys = load_fixture_system(cfg)
    return run_fixture_case!(cfg, sys)
end

@testset "GFM mixed-fleet transient (G3.3)" begin
    res = gfm_run(; encoding = TSCOPF.CONSTRAINT)
    @test res.status in GFM_SOLVED
    @test res.obj_MVA !== nothing && isfinite(res.obj_MVA)

    dd = res.dyn_model_dict
    @test dd[:meta][:n_gfm] == 1
    @test dd[:meta][:gfm_gens] == [4]
    @test dd[:meta][:sg_gens] == [1, 2, 3]
    # Imax = 1.2 < _GFM_IMAX_NO_LIMIT: the limiter must be built, not bypassed.
    @test !haskey(dd[:meta], :gfm_limiter_bypassed)

    eqs = dd[:eq_const]
    for window in ("tf", "tpf")
        @test haskey(eqs, Symbol("eq_const_gfm_", window))          # flat appendix vector
        for fam in TSCOPF._GFM_EQ_FAMILIES
            key = Symbol("eq_const_gfm_", fam, "_", window)
            @test haskey(eqs, key)
            @test haskey(eqs[key], 4)                               # the GFM unit
            @test !haskey(eqs[key], 1)                              # never an SG
        end
    end

    # CONSTRAINT encoding → explicit ≤-rows, no bound-manifest entries.
    ineqs = dd[:ineq_const]
    for name in ("P_meas", "Q_meas", "V_meas", "E_int", "E_droop", "Id", "Iq")
        for side in ("lower", "upper")
            @test haskey(ineqs, Symbol("ineq_const_gfm_", name, "_tf_", side))
            @test haskey(ineqs, Symbol("ineq_const_gfm_", name, "_tpf_", side))
        end
    end
    @test haskey(ineqs, :ineq_const_gfm_δ_lower)
    @test !TSCOPF.bound_manifest_entry_present(dd, :ineq_const_gfm_Id_tf_lower)

    # Registered dual families reached the export registry.
    registry = dd[:dual_registry]
    exported = Set(e.export_name for e in registry)
    for name in (:dual_gfm_droop, :dual_gfm_limiter_Id, :dual_gfm_limiter_Iq,
                 :dual_gfm_Eint_clip, :dual_gfm_Edroop_clip, :dual_gfm_Pe, :dual_gfm_Qe,
                 :dual_LB_gfm_Id_tf, :dual_UB_gfm_Id_tf)
        @test name in exported
    end

    @testset "VARIABLE encoding parity" begin
        res_v = gfm_run(; encoding = TSCOPF.VARIABLE)
        @test res_v.status in GFM_SOLVED
        @test res_v.obj_MVA !== nothing && isfinite(res_v.obj_MVA)
        # Same optimum, different bookkeeping: boxes move from ≤-rows to the manifest.
        @test isapprox(res_v.obj_MVA, res.obj_MVA; rtol = 1e-4)
        dv = res_v.dyn_model_dict
        @test !haskey(dv[:ineq_const], :ineq_const_gfm_Id_tf_lower)
        @test TSCOPF.bound_manifest_entry_present(dv, :ineq_const_gfm_Id_tf_lower)
        @test TSCOPF.bound_manifest_entry_present(dv, :ineq_const_gfm_E_droop_tpf_upper)
        exported_v = Set(e.export_name for e in dv[:dual_registry])
        @test :dual_UB_gfm_Id_tf in exported_v
    end

    @testset "ode_first_step drives the GFM first step (G4.3)" begin
        res_be = gfm_run(; ode_first_step = :backward_euler)
        @test res_be.status in GFM_SOLVED
        @test res_be.dyn_model_dict[:meta][:ode_first_step] === :backward_euler
        @test res.dyn_model_dict[:meta][:ode_first_step] === :trapezoidal
        # One step of one window: the optimum must move, but only slightly. No move at
        # all would mean the flag never reached the converter rows.
        @test res_be.obj_MVA != res.obj_MVA
        @test isapprox(res_be.obj_MVA, res.obj_MVA; rtol = 1e-2)
    end

    @testset "GFM trajectories and debug CSVs (G4.1 / G4.2)" begin
        plots_on = TSCOPF.plots_extension_loaded()
        res_dbg = gfm_run(; save_ts_debug_csv = true, save_ts_plots = plots_on)
        @test res_dbg.status in GFM_SOLVED

        csv_dir = res_dbg.path_names[:pf_TS_CSV]
        dbg_dir = joinpath(csv_dir, "Debug")
        tw = res_dbg.dyn_parameters_dict[:time]
        n_steps = length(tw[:t_window_fault]) + length(tw[:t_window_postf])
        n_gfm = length(res_dbg.dyn_model_dict[:meta][:gfm_gens])
        n_sg = length(res_dbg.dyn_model_dict[:meta][:sg_gens])

        # --- G4.2: converter trajectories now reach CSV (they never did before) ---
        for f in ("gfm_P_meas", "gfm_Q_meas", "gfm_V_meas", "gfm_E_int", "gfm_E_int_raw",
                  "gfm_E_droop", "gfm_E_droop_raw", "gfm_current_loading")
            @test isfile(joinpath(csv_dir, "$(f).csv"))
        end
        loading = CSV.read(joinpath(csv_dir, "gfm_current_loading.csv"), DataFrame; delim = ';')
        @test nrow(loading) == n_steps
        @test all(loading.GFM4 .<= 1.0 + 1e-6)      # the limiter caps |I| at Imax
        @test maximum(loading.GFM4) > 0.99          # and it saturates on this case

        if plots_on
            fig_dir = res_dbg.path_names[:pf_TS_figures]
            for f in ("gfm_P_meas.svg", "gfm_Q_meas.svg", "gfm_V_meas.svg", "gfm_E_int.svg",
                      "gfm_E_droop.svg", "gfm_current_loading.svg",
                      "gfm_E_int_clip.svg", "gfm_E_droop_clip.svg")
                @test isfile(joinpath(fig_dir, f))
            end
        end

        # --- G4.1: debug dumps, reference column layout ---
        headers = Dict(
            "gfm_filter_debug.csv" =>
                "period;gen;t_idx;t_s;channel;Tf;Δt;Tf_over_Δt;α;β;x_t;x_prev;u_t;" *
                "x_minus_u;x_minus_prev;dual_filter",
            "gfm_voltage_debug.csv" =>
                "period;gen;t_idx;t_s;Emin;Emax;voltage_error;E_int_raw;E_int;" *
                "E_int_to_Emax;E_int_to_Emin;E_droop_raw;E_droop;E_droop_to_Emax;" *
                "E_droop_to_Emin;dual_Eint_raw;dual_Eint_clip;dual_Edroop_raw;" *
                "dual_Edroop_clip",
        )
        for (f, head) in headers
            p = joinpath(dbg_dir, f)
            @test isfile(p)
            @test first(readlines(p)) == head
        end
        @test isfile(joinpath(dbg_dir, "gfm_limiter_debug.csv"))
        @test isfile(joinpath(dbg_dir, "swing_debug.csv"))

        filt = CSV.read(joinpath(dbg_dir, "gfm_filter_debug.csv"), DataFrame; delim = ';')
        @test nrow(filt) == 3 * n_gfm * n_steps          # one row per P/Q/V channel
        @test Set(filt.channel) == Set(["P", "Q", "V"])
        @test Set(filt.period) == Set(["fault", "postfault"])

        lim = CSV.read(joinpath(dbg_dir, "gfm_limiter_debug.csv"), DataFrame; delim = ';')
        @test nrow(lim) == n_gfm * n_steps
        @test all(lim.Iout_over_Imax .<= 1.0 + 1e-6)
        @test all(lim.scale .<= 1.0 + 1e-9)
        # Where the converter sits on its limit, the limiter row must carry a price.
        binding = findall(lim.Iout_over_Imax .> 1.0 - 1e-4)
        @test !isempty(binding)
        @test any(abs.(lim.dual_c_Id[binding]) .> 0.0)

        volt = CSV.read(joinpath(dbg_dir, "gfm_voltage_debug.csv"), DataFrame; delim = ';')
        @test nrow(volt) == n_gfm * n_steps
        @test all(volt.E_int .>= volt.Emin .- 1e-6)      # clipped state inside the band
        @test all(volt.E_int .<= volt.Emax .+ 1e-6)

        swing = CSV.read(joinpath(dbg_dir, "swing_debug.csv"), DataFrame; delim = ';')
        @test nrow(swing) == n_sg * n_steps
        @test all(swing.δ_util[.!isnan.(swing.δ_util)] .<= 1.0 + 1e-6)  # inside the corridor
        # Cross-check against the independently written trajectory file: swing_debug is in
        # radians, angle_abs.csv in degrees.
        ang = CSV.read(joinpath(csv_dir, "angle_abs.csv"), DataFrame; delim = ';')
        g1 = swing[swing.gen .== 1, :]
        @test nrow(g1) == n_steps
        @test isapprox(rad2deg.(g1.δ_curr), ang.G1; rtol = 1e-8)
    end

    @testset "box toggles remove only their own rows" begin
        res_off = gfm_run(; encoding = TSCOPF.CONSTRAINT,
            builder_kwargs = (bound_gfm_Id_tf = false, bound_gfm_δ = false))
        @test res_off.status in GFM_SOLVED
        io = res_off.dyn_model_dict[:ineq_const]
        @test !haskey(io, :ineq_const_gfm_Id_tf_lower)   # switched off
        @test !haskey(io, :ineq_const_gfm_Id_tf_upper)
        @test !haskey(io, :ineq_const_gfm_δ_lower)
        @test haskey(io, :ineq_const_gfm_Id_tpf_lower)   # neighbours untouched
        @test haskey(io, :ineq_const_gfm_Iq_tf_lower)
        @test haskey(io, :ineq_const_gfm_P_meas_tf_lower)
        # Guard-rails do not shape the optimum: same objective as the full run.
        @test isapprox(res_off.obj_MVA, res.obj_MVA; rtol = 1e-4)
    end

    @testset "stability corridors span the converter" begin
        sys_gfm = load_fixture_system(gfm_config())

        # The base run above is `:coi_box` with no speed corridor, so it already answers
        # the "SG-only where it must stay SG-only" half of this: the converter carries no
        # inertia and so no row against an inertia-weighted reference.
        @test 4 ∉ keys(dd[:ineq_const][:ineq_const_δ_COI_tf_lower])
        @test Set(keys(dd[:ineq_const][:ineq_const_δ_COI_tpf_upper])) == Set([1, 2, 3])

        # --- machine-referenced δ + absolute Δω: both span the mixed fleet -------------
        res_h = gfm_run(; bound_style_δ = :highest_H,
            constrain_Δω = true, bound_style_Δω = :abs, save_ts_debug_csv = true)
        @test res_h.status in GFM_SOLVED
        dh = res_h.dyn_model_dict
        active = dh[:active_gen]

        # :highest_H ranks by inertia, so the reference is a machine even here.
        ref = dh[:meta][:δ_ref_gen_resolved]
        sg = dh[:meta][:sg_gens]
        @test ref ∈ sg
        @test ref == sg[argmax([Float64(sys_gfm.DGEN_DYN.H[g]) for g in sg])]

        δ_rows = keys(dh[:ineq_const][:ineq_const_δ_ref_tf_lower])
        @test 4 ∈ δ_rows                                   # the converter is bounded
        @test ref ∉ δ_rows                                 # the reference is not
        @test length(δ_rows) == length(active) - 1
        @test 4 ∈ keys(dh[:ineq_const][:ineq_const_δ_ref_tpf_upper])

        ω_rows = keys(dh[:ineq_const][:ineq_const_Δω_abs_tf_lower])
        @test Set(ω_rows) == Set(active)                   # :abs bounds every unit
        @test !haskey(dh[:vars], :ΔωCOI_tf)                # and forms no COI at all

        # Duals ride the merged families: the converter is just another column.
        dual_dir = res_h.path_names[:pf_TS_CSV_duals]
        dual_df = CSV.read(joinpath(dual_dir, "dual_delta_ref_upper.csv"),
            DataFrame; delim = ';')
        @test "Gen_4" ∈ names(dual_df)
        @test "Gen_$(ref)" ∉ names(dual_df)
        ω_df = CSV.read(joinpath(dual_dir, "dual_Delta_Omega_abs_upper.csv"),
            DataFrame; delim = ';')
        @test "Gen_4" ∈ names(ω_df)

        # swing_debug stays SG-only on purpose: its columns (H, D, accelerating power) have
        # no meaning for a droop converter. The corridor duals live in the CSVs above.
        tw_h = res_h.dyn_parameters_dict[:time]
        n_steps_h = length(tw_h[:t_window_fault]) + length(tw_h[:t_window_postf])
        swing_h = CSV.read(
            joinpath(res_h.path_names[:pf_TS_CSV], "Debug", "swing_debug.csv"),
            DataFrame; delim = ';')
        @test nrow(swing_h) == length(sg) * n_steps_h

        # --- a converter as the reference, with an SG-only speed corridor --------------
        res_g = gfm_run(; bound_style_δ = :ref_gen, δ_ref_gen_id = 4,
            constrain_Δω = true, bound_style_Δω = :coi_box)
        @test res_g.status in GFM_SOLVED
        dg = res_g.dyn_model_dict
        @test dg[:meta][:δ_ref_gen_resolved] == 4
        @test Set(keys(dg[:ineq_const][:ineq_const_δ_ref_tf_lower])) == Set([1, 2, 3])
        # :coi_box on the speed side is inertia-weighted, so the converter is left out.
        @test Set(keys(dg[:ineq_const][:ineq_const_Δω_COI_tf_lower])) == Set([1, 2, 3])
        @test haskey(dg[:vars], :ΔωCOI_tf)
    end

    @testset "a GL disturbance can trip the converter itself" begin
        # Nothing in the GL path is unit-type aware: it zeroes g_status on a deepcopy,
        # recomputes active_gen, and re-partitions SG/GFM from the survivors. So the
        # converter drops out of the dynamic model with no GFM-specific code. The corridor
        # is :highest_H here precisely because that style *would* bound a converter — this
        # pins that a tripped one is excluded for being gone, not for being a converter.
        res_gl = gfm_run(; bound_style_δ = :highest_H,
            fault = FaultConfig(fault_type = GL, gl_gen_ids = [4]))
        @test res_gl.status in GFM_SOLVED
        dgl = res_gl.dyn_model_dict

        @test dgl[:active_gen] == [1, 2, 3]
        @test dgl[:meta][:sg_gens] == [1, 2, 3]
        @test isempty(dgl[:meta][:gfm_gens])     # none active…
        @test dgl[:meta][:n_gfm] == 1            # …though the fleet still has one

        # The disconnection has to mean something: the converter is dispatched
        # pre-contingency and only then lost. A trip of an unloaded unit would be vacuous.
        @test res_gl.RGEN !== nothing
        row_4 = findfirst(==(4), res_gl.RGEN.id)
        @test row_4 !== nothing
        @test res_gl.RGEN.p_g[row_4] > 1.0

        # No converter rows anywhere in the transient: the GFM window body is skipped
        # wholesale when no converter survives.
        for fam in TSCOPF._GFM_EQ_FAMILIES, window in ("tf", "tpf")
            @test !haskey(dgl[:eq_const], Symbol("eq_const_gfm_", fam, "_", window))
        end
        @test !haskey(dgl[:ineq_const], :ineq_const_gfm_Id_tf_lower)

        # …and the corridor spans the survivors only, reference excluded as always.
        ref_gl = dgl[:meta][:δ_ref_gen_resolved]
        δ_rows_gl = keys(dgl[:ineq_const][:ineq_const_δ_ref_tf_lower])
        @test 4 ∉ δ_rows_gl
        @test ref_gl ∉ δ_rows_gl
        @test length(δ_rows_gl) == 2
    end

    @testset "validate_δ_reference! on a mixed fleet" begin
        # No solve: these are the data-aware checks that must fire before the warm start.
        cfg = gfm_config(; bound_style_δ = :ref_gen, δ_ref_gen_id = 4)
        sys = load_fixture_system(cfg)

        # A converter is a legal reference now — it has no H row, and none is needed.
        @test validate_δ_reference!(cfg, sys.DGEN, sys.DGEN_DYN, sys.DGFM) === nothing
        # …but only while it is synchronised.
        tripped = gfm_config(; bound_style_δ = :ref_gen, δ_ref_gen_id = 4,
            fault = FaultConfig(fault_type = GL, gl_gen_ids = [4]))
        @test_throws ArgumentError validate_δ_reference!(
            tripped, sys.DGEN, sys.DGEN_DYN, sys.DGFM)
        # An id belonging to no unit at all is still rejected.
        missing_id = gfm_config(; bound_style_δ = :ref_gen, δ_ref_gen_id = 99)
        @test_throws ArgumentError validate_δ_reference!(
            missing_id, sys.DGEN, sys.DGEN_DYN, sys.DGFM)
        # :highest_H needs no converter data and stays valid.
        @test validate_δ_reference!(gfm_config(; bound_style_δ = :highest_H),
            sys.DGEN, sys.DGEN_DYN, sys.DGFM) === nothing
    end
end
