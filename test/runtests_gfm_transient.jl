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

function gfm_run(; encoding::BoundEncoding = TSCOPF.CONSTRAINT,
                   builder_kwargs::NamedTuple = NamedTuple(),
                   ode_first_step::Symbol = :trapezoidal,
                   save_ts_debug_csv::Bool = false,
                   save_ts_plots::Bool = false)
    cfg = RunConfig(;
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
                bound_style = :coi_box,
                zip_load_p = GFM_ZIP,
                zip_load_q = GFM_ZIP,
                dq_speed_dev_in_algebra = true,
                ode_first_step = ode_first_step,
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ),
        ),
    )
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
end
