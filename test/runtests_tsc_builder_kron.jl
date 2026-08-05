#=
 Kron-reduced TSC-ACOPF builder regression (main.jl-style settings).

 Verifies:
 - pre-fault init equalities are always-on physics (P/Q; Pm under USE_PM)
 - mandatory fault/post-fault equalities are always built
 - δ-COI ineq / explicit bound toggles still gate builders
=#

const KRON_TSC_SOLVED_STATUSES = (MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ITERATION_LIMIT)

function run_tsc_builder_kron_case(cfg::RunConfig)
    sys = load_fixture_system(cfg)
    return run_fixture_case!(cfg, sys)
end

@testset "TSC builder — Kron reduced (main.jl style)" begin
    @testset "baseline solve" begin
        result = run_tsc_builder_kron_case(main_style_tsc_kron_config())
        @test result.status in KRON_TSC_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        @test dmd[:meta][:network_form] == "KRON_REDUCED"
        @test haskey(dmd[:eq_const], :eq_const_P_init)
        @test haskey(dmd[:eq_const], :eq_const_Q_init)
        @test haskey(dmd[:eq_const], :eq_const_Pm_init)
        @test haskey(dmd[:eq_const], :eq_const_δCOI_tf)
        @test haskey(dmd[:eq_const], :eq_const_Pe_tf)
        @test haskey(dmd[:eq_const], :eq_const_δ_tf)
        @test haskey(dmd[:eq_const], :eq_const_Δω_tf)
        @test haskey(dmd[:ineq_const], :ineq_const_δ_COI_tf_lower)
        @test haskey(dmd[:ineq_const], :ineq_const_δ_COI_tf_upper)

        short_txt_path = joinpath(result.path_names[:pf_TS], "dynamic_model_short_results.txt")
        @test isfile(short_txt_path)
        short_txt = read(short_txt_path, String)
        @test occursin("E [p.u.]", short_txt)
        @test occursin("P_mech [MW]", short_txt)

        csv_path = joinpath(result.path_names[:pf_TS_CSV], "prefault_coupling.csv")
        @test isfile(csv_path)
        coupling_df = CSV.read(csv_path, DataFrame; delim=';')
        @test "gen" in names(coupling_df)
        @test "delta_rad" in names(coupling_df)
        @test "E_pu" in names(coupling_df)
        @test "P_mech_pu" in names(coupling_df)
        @test nrow(coupling_df) == length(dmd[:active_gen])
    end

    @testset "USE_PG omits Pm init pin" begin
        cfg = reconfigure_dyn(main_style_tsc_kron_config();
            dyn_model = DynModelConfig(
                network_form = KRON_REDUCED,
                mech_power_mode = USE_PG,
                bound_style = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ))
        result = run_tsc_builder_kron_case(cfg)
        @test result.status in KRON_TSC_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        @test haskey(dmd[:eq_const], :eq_const_P_init)
        @test haskey(dmd[:eq_const], :eq_const_Q_init)
        @test !haskey(dmd[:eq_const], :eq_const_Pm_init)
        @test !haskey(dmd[:vars], :P_m)
    end

    @testset "δ-COI ineq toggles (fault window off)" begin
        cfg = main_style_tsc_kron_config(;
            builder = main_style_ts_builder(
                ineq_δ_COI_tf_lower = false,
                ineq_δ_COI_tf_upper = false,
            ))
        result = run_tsc_builder_kron_case(cfg)
        @test result.status in KRON_TSC_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        @test !haskey(dmd[:ineq_const], :ineq_const_δ_COI_tf_lower)
        @test !haskey(dmd[:ineq_const], :ineq_const_δ_COI_tf_upper)
        @test haskey(dmd[:ineq_const], :ineq_const_δ_COI_tpf_lower)
    end

    @testset "fault δ explicit bounds toggle" begin
        cfg = main_style_tsc_kron_config(;
            builder = main_style_ts_builder(
                bound_δ_tf = true,
                limits = TsBoundLimitsConfig(
                    δ_tf = TsBoundLimitPair(min = -9999.0, max = 9999.0))))
        result = run_tsc_builder_kron_case(cfg)
        @test result.status in KRON_TSC_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        @test haskey(dmd[:ineq_const], :ineq_const_δ_tf_lower)
        @test haskey(dmd[:ineq_const], :ineq_const_δ_tf_upper)
        @test !haskey(dmd[:ineq_const], :ineq_const_Δω_tf_lower)
    end

    @testset "fault Qe explicit bounds toggle (expression)" begin
        cfg = main_style_tsc_kron_config(;
            builder = main_style_ts_builder(
                bound_Qe_tf = true,
                limits = TsBoundLimitsConfig(
                    Qe_tf = TsBoundLimitPair(min = -9999.0, max = 9999.0))))
        result = run_tsc_builder_kron_case(cfg)
        @test result.status in KRON_TSC_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        @test haskey(dmd[:expressions], :Qe_tf)
        @test haskey(dmd[:ineq_const], :ineq_const_Qe_tf_lower)
        @test haskey(dmd[:ineq_const], :ineq_const_Qe_tf_upper)
    end

    # `bound_style` used to be stored in meta and ignored: both Kron builders called the
    # swing-propagated form unconditionally, so a run could report `coi_box` and contain
    # the other constraint. The two forms are distinguishable in the exported model dump:
    # the propagated bound substitutes one swing step, so its rows carry Pe and Δω terms,
    # while the box is affine in δ and δCOI alone.
    @testset "bound_style selects the δ-COI constraint form" begin
        δ_coi_section(path) = begin
            txt = read(joinpath(path, "dynamic_model_details.txt"), String)
            i = findfirst("Inequality Constraints Angle in Relation to the COI", txt)
            @test i !== nothing
            txt[first(i):end]
        end

        # Baseline config is :coi_box (see main_style_tsc_kron_config).
        res_box = run_tsc_builder_kron_case(main_style_tsc_kron_config())
        @test res_box.status in KRON_TSC_SOLVED_STATUSES
        @test res_box.dyn_model_dict[:meta][:bound_style] === :coi_box
        box_txt = δ_coi_section(res_box.path_names[:pf_TS])
        @test !occursin("Pe_tf", box_txt)
        @test occursin("δCOI_tf", box_txt)

        cfg_swing = reconfigure_dyn(main_style_tsc_kron_config();
            dyn_model = DynModelConfig(
                network_form = KRON_REDUCED,
                mech_power_mode = USE_PG,          # USE_PM would force :coi_box
                bound_style = :swing_propagated,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                fault = FaultConfig(fault_type = SC, contingency_id = 2),
            ))
        res_swing = run_tsc_builder_kron_case(cfg_swing)
        @test res_swing.status in KRON_TSC_SOLVED_STATUSES
        @test res_swing.dyn_model_dict[:meta][:bound_style] === :swing_propagated
        swing_txt = δ_coi_section(res_swing.path_names[:pf_TS])
        @test occursin("Pe_tf", swing_txt)

        # The model metadata block is printed once, at the top, and states what was built.
        header = read(joinpath(res_box.path_names[:pf_TS], "dynamic_model_details.txt"), String)
        @test occursin("bound_style: coi_box", header)
    end

    @testset "GL gen trip — COI inertia over surviving set" begin
        cfg = reconfigure_dyn(main_style_tsc_kron_config();
            dyn_model = DynModelConfig(
                network_form = KRON_REDUCED,
                mech_power_mode = USE_PM,
                bound_style = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                fault = FaultConfig(fault_type = GL, gl_gen_ids = [3]),
            ))
        sys = load_fixture_system(cfg)
        result = run_fixture_case!(cfg, sys)
        @test result.status in KRON_TSC_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        @test 3 ∉ dmd[:active_gen]
        # COI-denominator identity is covered in fast-unit tests; after `run_case!`
        # the JuMP backend may already be released so `JuMP.value` is unavailable.
    end

    @testset "OB open branch — circuit 6" begin
        cfg = reconfigure_dyn(main_style_tsc_kron_config();
            dyn_model = DynModelConfig(
                network_form = KRON_REDUCED,
                mech_power_mode = USE_PM,
                bound_style = :coi_box,
                zip_load_p = (1.0, 0.0, 0.0),
                zip_load_q = (1.0, 0.0, 0.0),
                fault = FaultConfig(fault_type = OB, ob_branch_ids = [6]),
            ))
        sys = load_fixture_system(cfg)
        result = run_fixture_case!(cfg, sys)
        @test result.status in KRON_TSC_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        @test dmd[:meta][:fault_type] == "OB"
        @test dmd[:active_gen] == collect(1:sys.nGEN)
        @test !haskey(dmd[:eq_const], :eq_const_δ_tpf)

        ip_txt = read(joinpath(result.path_names[:pf_results_date], "input_parameters.txt"), String)
        @test occursin("Fault Category:                OB", ip_txt)
        @test occursin("Open Branch (no short-circuit): [6]", ip_txt)
    end
end
