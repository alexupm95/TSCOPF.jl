using Test
using TSCOPF
using JuMP
using DataFrames
using DataStructures: OrderedDict

const PROJECT_ROOT = dirname(@__DIR__)
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(@__DIR__, "test_common.jl"))
include(joinpath(@__DIR__, "tsc_main_style_config.jl"))

@testset "GFM Phase G0/G1" begin
    input_gfm = fixture_case("9bus_gfm")
    DGFM = TSCOPF.Read_GFM_Dynamic_Data(input_gfm; sys_base_MVA = 100.0)
    @test nrow(DGFM) == 1
    @test DGFM.id == [4]
    @test DGFM.bus == [5]
    @test DGFM.Imax ≈ [1.2]
    @test DGFM.Xl ≈ [0.15]

    raw = DataFrame(
        id = [1], bus = [1], Xl = [0.1], mq = [0.05], Kpv = [0.0], Kiv = [1.0],
        Emax = [1.2], Emin = [0.0], mp = [0.01], Pmax = [1.0], Pmin = [0.0],
        Tf = [0.01], Imax = [2.0], InvBase = [50.0],
    )
    scaled = TSCOPF.Parse_GFM_Dynamic_DataFrame(raw; sys_base_MVA = 100.0)
    @test scaled.Xl[1] ≈ 0.2
    @test scaled.Imax[1] ≈ 1.0
    @test scaled.mp[1] ≈ 0.02
    @test scaled.Pmax[1] ≈ 0.5

    cfg_load = RunConfig(
        case = "9bus_gfm",
        trans_stab = true,
        transient = TransientConfig(
            gen_dynamic_filename = "gen_dynamic_data_full.csv",
            dyn_model = DynModelConfig(
                allow_gfm = true,
                gen_order = DQ_4TH,
                network_form = FULL_BUS,
                mech_power_mode = USE_PM,
                bound_style = :coi_box,
            ),
        ),
    )
    @test validate_dyn_config!(cfg_load) === nothing
    sys = load_fixture_system(cfg_load)
    @test sys.DGFM !== nothing
    @test nrow(sys.DGFM) == 1
    @test nrow(sys.DGEN_DYN) == 3
    @test nrow(sys.DGEN) == 4
    @test Set(Int.(sys.DGEN_DYN.id)) == Set([1, 2, 3])
    @test Set(Int.(sys.DGFM.id)) == Set([4])
    @test sg_active_gens(collect(1:4), sys.DGFM) == [1, 2, 3]
    @test gfm_active_gens(collect(1:4), sys.DGFM) == [4]

    bad_gfm_kron = reconfigure_dyn(tsc_run_config();
        dyn_model = DynModelConfig(allow_gfm = true, gen_order = DQ_4TH,
            network_form = KRON_REDUCED))
    @test_throws ArgumentError validate_dyn_config!(bad_gfm_kron)

    # --- Phase G1: warm-start map ---
    δ0, E0, Id0, Iq0 = gfm_machine_warmstart(1.02, 0.0, 0.5, 0.1, 0.15)
    @test E0 > 0.0
    @test isfinite(δ0) && isfinite(Id0) && isfinite(Iq0)

    # --- Phase G1: Attach_GFM_init! ---
    model = Model()
    ddict = OrderedDict{Symbol, Any}(
        :vars => OrderedDict{Symbol, Any}(
            :δ => OrderedDict{Int, JuMP.VariableRef}(),
            :Ed => OrderedDict{Int, JuMP.VariableRef}(),
            :Eq => OrderedDict{Int, JuMP.VariableRef}(),
            :Id => OrderedDict{Int, JuMP.VariableRef}(),
            :Iq => OrderedDict{Int, JuMP.VariableRef}(),
            :P_m => OrderedDict{Int, JuMP.VariableRef}(),
        ),
        :eq_const => OrderedDict{Symbol, Any}(
            :eq_const_Ed_init => OrderedDict{Int, JuMP.ConstraintRef}(),
            :eq_const_Eq_init => OrderedDict{Int, JuMP.ConstraintRef}(),
            :eq_const_Vd_init => OrderedDict{Int, JuMP.ConstraintRef}(),
            :eq_const_Vq_init => OrderedDict{Int, JuMP.ConstraintRef}(),
            :eq_const_P_init => OrderedDict{Int, JuMP.ConstraintRef}(),
            :eq_const_Q_init => OrderedDict{Int, JuMP.ConstraintRef}(),
            :eq_const_Pm_init => OrderedDict{Int, JuMP.ConstraintRef}(),
        ),
        :meta => OrderedDict{Symbol, Any}(),
    )
    V = OrderedDict{Int, JuMP.VariableRef}()
    θ = OrderedDict{Int, JuMP.VariableRef}()
    P_g = OrderedDict{Int, JuMP.VariableRef}()
    Q_g = OrderedDict{Int, JuMP.VariableRef}()
    val_V = Dict{Int, Float64}()
    val_θ = Dict{Int, Float64}()
    val_Pg = Dict{Int, Float64}()
    val_Qg = Dict{Int, Float64}()
    for bus in (5,)
        V[bus] = @variable(model, base_name = "V[$bus]", start = 1.02)
        θ[bus] = @variable(model, base_name = "θ[$bus]", start = 0.0)
        val_V[bus] = 1.02
        val_θ[bus] = 0.0
    end
    for gen in (4,)
        P_g[gen] = @variable(model, base_name = "P_g[$gen]", start = 0.3)
        Q_g[gen] = @variable(model, base_name = "Q_g[$gen]", start = 0.05)
        val_Pg[gen] = 0.3
        val_Qg[gen] = 0.05
    end
    Attach_GFM_init!(
        model, ddict, [4], sys.DGEN, sys.DGFM,
        V, θ, P_g, Q_g, val_V, val_θ, val_Pg, val_Qg)
    @test ddict[:meta][:n_gfm] == 1
    @test ddict[:meta][:gfm_phase] == :G1_init
    @test haskey(ddict[:vars], :P_meas)
    @test haskey(ddict[:vars], :V_set)
    @test haskey(ddict[:vars][:δ], 4)
    @test !haskey(ddict[:vars], :gfm_init)  # G0 stub key gone
    @test length(ddict[:eq_const][:eq_const_gfm_Vset_init]) == 1

    # --- Phase G1: ACOPF GFM inequalities ---
    opf_model = Model()
    opf_dict = OrderedDict{Symbol, Any}(
        :vars => OrderedDict{Symbol, Any}(
            :V => OrderedDict{Int, JuMP.VariableRef}(),
            :P_g => OrderedDict{Int, JuMP.VariableRef}(),
            :Q_g => OrderedDict{Int, JuMP.VariableRef}(),
        ),
        :ineq_const => OrderedDict{Symbol, Any}(),
    )
    for bus in unique(Int.(sys.DGEN.bus))
        opf_dict[:vars][:V][bus] = @variable(opf_model, start = 1.0)
    end
    for gen in 1:nrow(sys.DGEN)
        opf_dict[:vars][:P_g][gen] = @variable(opf_model, start = 0.1)
        opf_dict[:vars][:Q_g][gen] = @variable(opf_model, start = 0.0)
    end
    attach_gfm_acopf_limits!(opf_model, opf_dict, sys.DGEN, sys.DGFM, 100.0)
    @test length(opf_dict[:ineq_const][:ineq_const_gfm_Imax]) == 1
    @test length(opf_dict[:ineq_const][:ineq_const_gfm_Emax]) == 1
    @test haskey(opf_dict[:ineq_const][:ineq_const_gfm_Imax], 4)

    # --- Phase G2 helpers ---
    @test TSCOPF._GFM_IMAX_NO_LIMIT == 20.0
    m = Model()
    x = @variable(m)
    y = @variable(m)
    @test smooth_max_expr(m, x, y) isa JuMP.GenericNonlinearExpr
    # Tf = 0 is the pass-through bypass, and must come before the trapezoidal 2·Tf
    # division on both branches.
    bypass = [TSCOPF._add_gfm_measurement_filter!(m, x, y, x, y, 0.0, 0.01;
                  backward_euler = be) for be in (true, false)]
    @test all(c -> c isa JuMP.ConstraintRef, bypass)
    @test string(JuMP.constraint_object(bypass[1]).func) ==
          string(JuMP.constraint_object(bypass[2]).func)   # same row either way
    # With a real Tf the two schemes must produce different rows.
    c_be = TSCOPF._add_gfm_measurement_filter!(m, x, y, x, y, 0.02, 0.01;
        backward_euler = true)
    c_tr = TSCOPF._add_gfm_measurement_filter!(m, x, y, x, y, 0.02, 0.01;
        backward_euler = false)
    @test string(JuMP.constraint_object(c_be).func) !=
          string(JuMP.constraint_object(c_tr).func)

    tc = TransientConfig()
    @test tc.gfm_dynamic_filename == "gfm_dynamic_data.csv"
    @test DynModelConfig().allow_gfm == false

    # --- GFM box limits: defaults must reproduce the hard-coded reference numbers ---
    # 9bus_gfm GFM row: Emin 0.0, Emax 1.2, Imax 1.2, Pmin 0.0, Pmax 1.0 (system base).
    specs = resolve_gfm_bound_limits(TsBoundLimitsConfig(), sys.DGFM, [4])
    @test specs[:gfm_P_meas][1] ≈ [-2.95] && specs[:gfm_P_meas][2] ≈ [2.95]  # floor 3.0 − 0.05
    @test specs[:gfm_Q_meas][1] ≈ [-2.95] && specs[:gfm_Q_meas][2] ≈ [2.95]  # floor > 1.5·Imax
    @test specs[:gfm_I][1] ≈ [-5.0] && specs[:gfm_I][2] ≈ [5.0]              # floor 5.0 > 1.8
    @test specs[:gfm_V_meas][1] ≈ [0.0] && specs[:gfm_V_meas][2] ≈ [2.5]
    @test specs[:gfm_δ][1] ≈ [-π] && specs[:gfm_δ][2] ≈ [π]
    @test specs[:gfm_E_raw][1] ≈ [-0.75] && specs[:gfm_E_raw][2] ≈ [1.95]    # ±(0.25 + 0.5)
    @test specs[:gfm_E_clip][1] ≈ [-0.05] && specs[:gfm_E_clip][2] ≈ [1.25]  # ±0.05
    @test specs[:gfm_E_raw_start][1] ≈ [-0.25] && specs[:gfm_E_raw_start][2] ≈ [1.45]
    @test specs[:gfm_E_clip_start][1] ≈ [0.0] && specs[:gfm_E_clip_start][2] ≈ [1.2]

    # Knobs actually move the boxes.
    tuned = resolve_gfm_bound_limits(
        TsBoundLimitsConfig(gfm_I_floor_pu = 1.0, gfm_P_meas_floor_pu = 0.5,
            gfm_E_clip_slack_pu = 0.2),
        sys.DGFM, [4])
    @test tuned[:gfm_I][2] ≈ [1.8]          # now 1.5·Imax; the floor no longer binds
    @test tuned[:gfm_P_meas][2] ≈ [1.575]   # now 1.5·(0.05 + Pmax)
    @test tuned[:gfm_E_clip][2] ≈ [1.4]

    @test_throws ArgumentError TSCOPF.validate_ts_bound_limits!(
        TsBoundLimitsConfig(gfm_V_meas_min_pu = 2.0, gfm_V_meas_max_pu = 1.0))
    @test_throws ArgumentError TSCOPF.validate_ts_bound_limits!(
        TsBoundLimitsConfig(gfm_I_floor_pu = 25.0))     # floor above ceiling
    @test_throws ArgumentError TSCOPF.validate_ts_bound_limits!(
        TsBoundLimitsConfig(gfm_PQ_bound_scale = 0.0))

    # --- GFM box toggles: same shape as the SG families, but default ON --------
    b_default = TsBuilderConfig()
    @test b_default.bound_gfm_δ
    @test b_default.bound_gfm_Id_tf && b_default.bound_gfm_Iq_tpf
    @test b_default.bound_gfm_P_meas_tf && b_default.bound_gfm_E_droop_tpf
    @test b_default.bound_Ed_tf == false          # SG families still default off
    param = TSCOPF.build_ts_input_param(b_default)
    for key in (:gfm_δ, :gfm_P_meas_tf, :gfm_Q_meas_tf, :gfm_V_meas_tf,
                :gfm_E_int_raw_tf, :gfm_E_int_tf, :gfm_E_droop_raw_tf,
                :gfm_E_droop_tf, :gfm_Id_tf, :gfm_Iq_tf,
                :gfm_P_meas_tpf, :gfm_Q_meas_tpf, :gfm_V_meas_tpf,
                :gfm_E_int_raw_tpf, :gfm_E_int_tpf, :gfm_E_droop_raw_tpf,
                :gfm_E_droop_tpf, :gfm_Id_tpf, :gfm_Iq_tpf)
        @test param[:var_bounds][key] === true
    end
    param_off = TSCOPF.build_ts_input_param(
        TsBuilderConfig(bound_gfm_Id_tf = false, bound_gfm_δ = false))
    @test param_off[:var_bounds][:gfm_Id_tf] === false
    @test param_off[:var_bounds][:gfm_δ] === false
    @test param_off[:var_bounds][:gfm_Iq_tf] === true   # neighbours untouched

    # --- debug-CSV plumbing (G4.1) ---------------------------------------------
    @test RunConfig().save_ts_debug_csv === false          # opt-in: dumps are large
    @test reconfigure(RunConfig(); save_ts_debug_csv = true).save_ts_debug_csv === true

    # Corridor utilisation: 1.0 means sitting on the limit, each side against its own
    # tolerance so an asymmetric corridor reports honestly.
    @test TSCOPF._corridor_utilisation(0.5, (-1.0, 1.0)) ≈ 0.5
    @test TSCOPF._corridor_utilisation(-0.5, (-1.0, 2.0)) ≈ 0.5   # lower side
    @test TSCOPF._corridor_utilisation(1.0, (-1.0, 2.0)) ≈ 0.5    # upper side
    @test TSCOPF._corridor_utilisation(2.0, (-1.0, 2.0)) ≈ 1.0    # on the corridor
    @test isnan(TSCOPF._corridor_utilisation(NaN, (-1.0, 1.0)))
    @test isnan(TSCOPF._corridor_utilisation(0.5, (0.0, 0.0)))

    win_both = TSCOPF._ts_debug_windows(OrderedDict{Symbol, Any}(
        :time => OrderedDict{Symbol, Any}(
            :t_window_fault => [0.01, 0.02], :t_window_postf => [0.03, 0.04])))
    @test length(win_both) == 2
    @test win_both[1][1] == "fault" && win_both[1][2] == "tf"
    @test win_both[2][1] == "postfault" && win_both[2][2] == "tpf"
    win_one = TSCOPF._ts_debug_windows(OrderedDict{Symbol, Any}(
        :time => OrderedDict{Symbol, Any}(
            :t_window_fault => [0.01, 0.02], :t_window_postf => Float64[])))
    @test length(win_one) == 1                             # GL/OB: single window

    # --- ode_first_step reaches the GFM window (G4.3) --------------------------
    # Same model structure either way; only the t=1 δ and E_int rows change form.
    function _gfm_fault_probe(ode_fs::Symbol; gfm_int::Symbol = :backward_euler)
        m = Model()
        dd = OrderedDict{Symbol, Any}(
            :vars => OrderedDict{Symbol, Any}(),
            :eq_const => OrderedDict{Symbol, Any}(),
            :ineq_const => OrderedDict{Symbol, Any}(),
            :meta => OrderedDict{Symbol, Any}(
                :ode_first_step => ode_fs, :gfm_integrator => gfm_int),
        )
        v = dd[:vars]
        g, b, T = 4, 5, 3
        for k in (:P_meas, :Q_meas, :V_meas, :E_int, :V_set, :P_m, :δ)
            v[k] = OrderedDict{Int, JuMP.VariableRef}(
                g => @variable(m, base_name = string(k)))
        end
        for k in (:V_tf, :θ_tf)
            v[k] = OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}}(
                b => OrderedDict{Int, JuMP.VariableRef}(
                    t => @variable(m, base_name = "$(k)[$t]") for t in 1:T))
        end
        for k in (:Pe_tf, :Qe_tf, :δ_tf, :Δω_tf, :Id_tf, :Iq_tf)
            v[k] = OrderedDict{Int, OrderedDict{Int, JuMP.VariableRef}}(
                g => OrderedDict{Int, JuMP.VariableRef}(
                    t => @variable(m, base_name = "$(k)[$t]") for t in 1:T))
        end
        Attach_GFM_fault!(m, dd, [g], sys.DGEN, sys.DGFM,
            0.01, 2π * 50.0, collect(Float64, 1:T))
        return m, dd
    end

    # `gfm_int` pinned to the follow arm: this pair checks that `ode_first_step` reaches
    # the GFM window, which only shows up on the filters/PI under that setting (the
    # package default `:backward_euler` deliberately makes them inert to it).
    m_be, dd_be = _gfm_fault_probe(:backward_euler; gfm_int = :follow_ode_first_step)
    m_tr, dd_tr = _gfm_fault_probe(:trapezoidal; gfm_int = :follow_ode_first_step)
    nc(m) = num_constraints(m; count_variable_in_set_constraints = false)
    @test nc(m_be) == nc(m_tr)                  # coefficients change, structure does not
    @test length(dd_be[:eq_const][:eq_const_gfm_delta_tf][4]) == 3
    @test length(dd_tr[:eq_const][:eq_const_gfm_delta_tf][4]) == 3
    row(dd, fam, t) = string(JuMP.constraint_object(dd[:eq_const][fam][4][t]).func)
    @test row(dd_be, :eq_const_gfm_delta_tf, 1) != row(dd_tr, :eq_const_gfm_delta_tf, 1)
    @test row(dd_be, :eq_const_gfm_Eint_raw_tf, 1) != row(dd_tr, :eq_const_gfm_Eint_raw_tf, 1)
    # Steps after the first are untouched by the flag.
    @test row(dd_be, :eq_const_gfm_delta_tf, 2) == row(dd_tr, :eq_const_gfm_delta_tf, 2)
    @test row(dd_be, :eq_const_gfm_Eint_raw_tf, 2) == row(dd_tr, :eq_const_gfm_Eint_raw_tf, 2)

    # --- gfm_integrator: the filters and the Q–V PI get their own three-way dial ------
    # It is GFM-only and independent of `ode_first_step`, which keeps driving δ.
    filt_fams = (:eq_const_gfm_filter_P_tf, :eq_const_gfm_filter_Q_tf,
                 :eq_const_gfm_filter_V_tf, :eq_const_gfm_Eint_raw_tf)
    m_all_be, dd_all_be = _gfm_fault_probe(:trapezoidal; gfm_int = :backward_euler)
    m_all_tr, dd_all_tr = _gfm_fault_probe(:backward_euler; gfm_int = :trapezoidal)

    # 1. Structure is invariant across all three schemes; only coefficients move.
    @test nc(m_all_be) == nc(m_be) == nc(m_all_tr)

    # 2. `:backward_euler` — every step BE, and `ode_first_step` cannot reach these rows.
    _, dd_all_be2 = _gfm_fault_probe(:backward_euler; gfm_int = :backward_euler)
    for fam in filt_fams, t in 1:3
        @test row(dd_all_be, fam, t) == row(dd_all_be2, fam, t)
    end

    # 3. `:trapezoidal` — every step trapezoidal, `ode_first_step` again inert.
    _, dd_all_tr2 = _gfm_fault_probe(:trapezoidal; gfm_int = :trapezoidal)
    for fam in filt_fams, t in 1:3
        @test row(dd_all_tr, fam, t) == row(dd_all_tr2, fam, t)
    end
    # ...and the two extremes genuinely differ at every step, t=1 included.
    for fam in filt_fams, t in 1:3
        @test row(dd_all_be, fam, t) != row(dd_all_tr, fam, t)
    end

    # 4. `:follow_ode_first_step` + `:backward_euler` — BE at t=1, trapezoidal after.
    for fam in filt_fams
        @test row(dd_be, fam, 1) == row(dd_all_be, fam, 1)      # t=1 matches BE-all
        @test row(dd_be, fam, 2) == row(dd_all_tr, fam, 2)      # t≥2 matches trap-all
        @test row(dd_be, fam, 3) == row(dd_all_tr, fam, 3)
    end
    # ...and with `:trapezoidal` it is trapezoidal throughout.
    for fam in filt_fams, t in 1:3
        @test row(dd_tr, fam, t) == row(dd_all_tr, fam, t)
    end

    # 5. δ never goes through the GFM dial — it stays on `ode_first_step` alone.
    for t in 1:3
        @test row(dd_all_be, :eq_const_gfm_delta_tf, t) ==
              row(dd_tr, :eq_const_gfm_delta_tf, t)             # both ode_fs=:trapezoidal
        @test row(dd_all_tr, :eq_const_gfm_delta_tf, t) ==
              row(dd_be, :eq_const_gfm_delta_tf, t)             # both ode_fs=:backward_euler
    end

    # 6. The knob reaches the model through the factory (DynModelConfig → DqFullBusModel).
    @test DynModelConfig().gfm_integrator === :backward_euler
    gm = TSCOPF.dynamic_gen_model(
        DynModelConfig(gen_order = DQ_4TH, network_form = FULL_BUS,
            mech_power_mode = USE_PM, bound_style = :coi_box,
            gfm_integrator = :backward_euler); linearize = false)
    @test gm.gfm_integrator === :backward_euler

    # A missing :var_bounds entry reads as ON, so bare-dict callers keep the box.
    meta_bare = OrderedDict{Symbol, Any}()
    @test TSCOPF._gfm_box_enabled(meta_bare, :gfm_Id_tf)
    meta_off = OrderedDict{Symbol, Any}(
        :var_bounds => OrderedDict{Symbol, Any}(:gfm_Id_tf => false))
    @test !TSCOPF._gfm_box_enabled(meta_off, :gfm_Id_tf)
    @test TSCOPF._gfm_box_enabled(meta_off, :gfm_Iq_tf)
end
