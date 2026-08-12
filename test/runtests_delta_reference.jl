#=
 Machine-referenced δ corridor and absolute Δω corridor (Kron 9-bus).

 Verifies:
 - `:highest_H` resolves to the largest-inertia surviving machine, and a GL trip of that
   machine moves the reference rather than selecting a disconnected unit
 - the reference itself carries no corridor row (n−1 rows, not n)
 - the COI is built as an expression, not a variable, when nothing constrains it
 - `:ref_gen` honours the configured id
 - `bound_style_Δω = :abs` builds the absolute family and no COI variable
 - exports: `angle_rel_ref.csv`, and duals under the `δ_ref` names only
=#

const DELTA_REF_SOLVED_STATUSES = (MOI.OPTIMAL, MOI.LOCALLY_SOLVED, MOI.ITERATION_LIMIT)

function run_delta_reference_case(cfg::RunConfig)
    sys = load_fixture_system(cfg)
    return run_fixture_case!(cfg, sys)
end

delta_reference_kron_config(; kwargs...) = main_style_tsc_base(;
    network_form = KRON_REDUCED,
    mech_power_mode = USE_PM,
    kwargs...)

@testset "δ reference corridor + absolute Δω corridor (Kron 9-bus)" begin

    @testset ":highest_H picks the largest-inertia surviving machine" begin
        cfg = delta_reference_kron_config(bound_style_δ = :highest_H)
        result = run_delta_reference_case(cfg)
        @test result.status in DELTA_REF_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        sys = load_fixture_system(cfg)

        active = dmd[:active_gen]
        expected = active[argmax([Float64(sys.DGEN_DYN.H[g]) for g in active])]
        @test dmd[:meta][:δ_ref_gen_resolved] == expected

        # The machine-referenced family replaces the COI one; they never coexist.
        @test haskey(dmd[:ineq_const], :ineq_const_δ_ref_tf_lower)
        @test haskey(dmd[:ineq_const], :ineq_const_δ_ref_tf_upper)
        @test !haskey(dmd[:ineq_const], :ineq_const_δ_COI_tf_lower)
        @test !haskey(dmd[:ineq_const], :ineq_const_δ_COI_tpf_upper)

        # The reference carries no row against itself: n−1 generators, both windows.
        for key in (:ineq_const_δ_ref_tf_lower, :ineq_const_δ_ref_tpf_upper)
            gens = collect(keys(dmd[:ineq_const][key]))
            @test length(gens) == length(active) - 1
            @test expected ∉ gens
        end

        # COI still available for diagnostics, but as an expression the solver never sees.
        @test !haskey(dmd[:vars], :δCOI_tf)
        @test !haskey(dmd[:eq_const], :eq_const_δCOI_tf)
        @test haskey(dmd[:expressions], :δCOI_tf)
        @test haskey(dmd[:expressions], :δCOI_tpf)
    end

    @testset ":highest_H selects over survivors, not over the whole fleet" begin
        # Gen 1 carries the most inertia on the 9-bus fixture but is also the slack, which
        # `validate_fault_config!` refuses to trip — so drive the resolver directly with a
        # restricted machine set. That is exactly what the builders pass after a GL trip.
        sys = load_fixture_system(delta_reference_kron_config(bound_style_δ = :coi_box))
        H = sys.DGEN_DYN.H
        @test H[1] > H[2] > H[3]   # fixture assumption the rest of this testset rests on

        make_meta(; kwargs...) = OrderedDict{Symbol, Any}(
            :meta => OrderedDict{Symbol, Any}(:bound_style_δ => :highest_H, kwargs...))

        @test TSCOPF.resolve_δ_reference(make_meta(), Int64[1, 2, 3], sys.DGEN_DYN) == 1
        # Gen 1 gone: the reference moves down the inertia ranking rather than pointing at
        # a machine that is no longer synchronised.
        @test TSCOPF.resolve_δ_reference(make_meta(), Int64[2, 3], sys.DGEN_DYN) == 2
        @test TSCOPF.resolve_δ_reference(make_meta(), Int64[1, 3], sys.DGEN_DYN) == 1

        # One survivor leaves an empty corridor.
        @test_throws ArgumentError TSCOPF.resolve_δ_reference(
            make_meta(), Int64[3], sys.DGEN_DYN)

        # The resolved reference is cached, and a window that disagrees is an error rather
        # than a corridor that changes meaning half-way through the horizon.
        cached = make_meta()
        @test TSCOPF.resolve_δ_reference(cached, Int64[1, 2, 3], sys.DGEN_DYN) == 1
        @test cached[:meta][:δ_ref_gen_resolved] == 1
        @test_throws ArgumentError TSCOPF.resolve_δ_reference(
            cached, Int64[2, 3], sys.DGEN_DYN)

        # COI-referenced styles have no reference machine at all.
        coi_meta = OrderedDict{Symbol, Any}(
            :meta => OrderedDict{Symbol, Any}(:bound_style_δ => :coi_box))
        @test TSCOPF.resolve_δ_reference(coi_meta, Int64[1, 2, 3], sys.DGEN_DYN) === nothing
    end

    @testset "a GL trip drops the tripped machine from the corridor" begin
        # Gen 2 is not the slack, so it can be tripped; gen 1 stays the reference and the
        # corridor spans only what survives.
        cfg = delta_reference_kron_config(
            bound_style_δ = :highest_H,
            fault = FaultConfig(fault_type = GL, gl_gen_ids = [2]))
        dmd = run_delta_reference_case(cfg).dyn_model_dict

        @test 2 ∉ dmd[:active_gen]
        ref = dmd[:meta][:δ_ref_gen_resolved]
        @test ref ∈ dmd[:active_gen]
        gens = collect(keys(dmd[:ineq_const][:ineq_const_δ_ref_tf_lower]))
        @test 2 ∉ gens
        @test ref ∉ gens
        @test length(gens) == length(dmd[:active_gen]) - 1
    end

    @testset ":ref_gen honours the configured id" begin
        cfg = delta_reference_kron_config(bound_style_δ = :ref_gen, δ_ref_gen_id = 3)
        result = run_delta_reference_case(cfg)
        @test result.status in DELTA_REF_SOLVED_STATUSES
        dmd = result.dyn_model_dict
        @test dmd[:meta][:δ_ref_gen_resolved] == 3
        @test 3 ∉ keys(dmd[:ineq_const][:ineq_const_δ_ref_tf_lower])
    end

    @testset "reference-relative exports" begin
        cfg = delta_reference_kron_config(
            bound_style_δ = :highest_H, save_duals = true, save_ts_debug_csv = true)
        result = run_delta_reference_case(cfg)
        ref = result.dyn_model_dict[:meta][:δ_ref_gen_resolved]

        angle_path = joinpath(result.path_names[:pf_TS_CSV], "angle_rel_ref.csv")
        @test isfile(angle_path)
        angle_df = CSV.read(angle_path, DataFrame; delim=';')
        # The reference is flat at zero by construction; the others are not.
        @test all(≈(0.0; atol=1e-9), angle_df[!, "G$(ref)"])
        others = [n for n in names(angle_df) if n != "t" && n != "G$(ref)"]
        @test !isempty(others)
        @test any(c -> maximum(abs, angle_df[!, c]) > 1e-6, others)

        dual_dir = result.path_names[:pf_TS_CSV_duals]
        @test isfile(joinpath(dual_dir, "dual_delta_ref_upper.csv"))
        @test !isfile(joinpath(dual_dir, "dual_delta_COI_upper.csv"))
        dual_df = CSV.read(joinpath(dual_dir, "dual_delta_ref_upper.csv"),
            DataFrame; delim=';')
        @test "Gen_$(ref)" ∉ names(dual_df)

        debug_path = joinpath(result.path_names[:pf_TS_CSV], "Debug", "swing_debug.csv")
        @test isfile(debug_path)
        debug_df = CSV.read(debug_path, DataFrame; delim=';')
        @test all(==(ref), debug_df.δ_ref_gen)
        # The COI column survives the expression form — that is the whole point of it.
        @test all(isfinite, debug_df.δ_rel_COI)
    end

    @testset "bound_style_Δω = :abs boxes the raw speed deviation" begin
        cfg = delta_reference_kron_config(
            bound_style_δ = :coi_box, constrain_Δω = true, bound_style_Δω = :abs)
        result = run_delta_reference_case(cfg)
        @test result.status in DELTA_REF_SOLVED_STATUSES
        dmd = result.dyn_model_dict

        @test haskey(dmd[:ineq_const], :ineq_const_Δω_abs_tf_lower)
        @test haskey(dmd[:ineq_const], :ineq_const_Δω_abs_tpf_upper)
        @test !haskey(dmd[:ineq_const], :ineq_const_Δω_COI_tf_lower)
        # No COI is formed at all on the absolute path.
        @test !haskey(dmd[:vars], :ΔωCOI_tf)
        @test !haskey(dmd[:eq_const], :eq_const_ΔωCOI_tf)
    end

    @testset ":coi_box still builds the COI variable and its equality" begin
        cfg = delta_reference_kron_config(
            bound_style_δ = :coi_box, constrain_Δω = true, bound_style_Δω = :coi_box)
        dmd = run_delta_reference_case(cfg).dyn_model_dict
        @test haskey(dmd[:vars], :δCOI_tf)
        @test haskey(dmd[:eq_const], :eq_const_δCOI_tf)
        @test haskey(dmd[:vars], :ΔωCOI_tf)
        @test haskey(dmd[:ineq_const], :ineq_const_Δω_COI_tf_lower)
        @test !haskey(dmd[:ineq_const], :ineq_const_δ_ref_tf_lower)
        @test !haskey(dmd[:ineq_const], :ineq_const_Δω_abs_tf_lower)
    end

    @testset "validate_δ_reference! fires before the warm start" begin
        # These need the system data, which is exactly why they cannot live in
        # validate_dyn_config!. They must still throw without building a model.
        sys = load_fixture_system(delta_reference_kron_config(bound_style_δ = :coi_box))

        ok = delta_reference_kron_config(bound_style_δ = :ref_gen, δ_ref_gen_id = 2)
        @test validate_δ_reference!(ok, sys.DGEN, sys.DGEN_DYN, sys.DGFM) === nothing

        out_of_range = delta_reference_kron_config(
            bound_style_δ = :ref_gen, δ_ref_gen_id = 99)
        @test_throws ArgumentError validate_δ_reference!(
            out_of_range, sys.DGEN, sys.DGEN_DYN, sys.DGFM)

        tripped = delta_reference_kron_config(
            bound_style_δ = :ref_gen, δ_ref_gen_id = 2,
            fault = FaultConfig(fault_type = GL, gl_gen_ids = [2]))
        @test_throws ArgumentError validate_δ_reference!(
            tripped, sys.DGEN, sys.DGEN_DYN, sys.DGFM)

        # Two survivors is the minimum: the reference has no row of its own.
        too_few = delta_reference_kron_config(
            bound_style_δ = :highest_H,
            fault = FaultConfig(fault_type = GL, gl_gen_ids = [1, 2]))
        @test_throws ArgumentError validate_δ_reference!(
            too_few, sys.DGEN, sys.DGEN_DYN, sys.DGFM)

        # COI-referenced styles have no reference machine, so the check is a no-op.
        coi = delta_reference_kron_config(bound_style_δ = :coi_box)
        @test validate_δ_reference!(coi, sys.DGEN, sys.DGEN_DYN, sys.DGFM) === nothing
    end

    @testset "constrain_δ=false drops the angle corridor only" begin
        cfg = delta_reference_kron_config(
            bound_style_δ = :coi_box, constrain_δ = false, constrain_Δω = true)
        dmd = run_delta_reference_case(cfg).dyn_model_dict
        @test !haskey(dmd[:ineq_const], :ineq_const_δ_COI_tf_lower)
        @test !haskey(dmd[:ineq_const], :ineq_const_δ_ref_tf_lower)
        @test !haskey(dmd[:vars], :δCOI_tf)
        @test haskey(dmd[:ineq_const], :ineq_const_Δω_COI_tf_lower)
    end
end
