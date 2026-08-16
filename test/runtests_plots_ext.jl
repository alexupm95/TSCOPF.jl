#=
 test/runtests_plots_ext.jl — TSCOPFPlotsExt margin + SVG export smoke (no NLP solve)
 CI historically skipped plots (save_ts_plots=false everywhere); this catches
 extension dependency / Measures regressions.
=#

using Test
using TSCOPF
using DataStructures: OrderedDict

function _toy_ts_plot_payload(nt::Int=5)
    t = collect(0.0:0.1:(0.1 * (nt - 1)))
    v = fill(1.0, nt)
    z = zeros(nt)
    rocf_t = t[2:end]
    rocf_v = zeros(length(rocf_t))
    odv = OrderedDict{Int, Vector{Float64}}(1 => v, 2 => v)
    odz = OrderedDict{Int, Vector{Float64}}(1 => z, 2 => z)
    rocf = OrderedDict{Int, Vector{Float64}}(1 => rocf_v, 2 => rocf_v)
    return t, odv, odz, rocf_t, rocf, rocf_v
end

@testset "Plots extension" begin
    @test load_plots_extension!()
    @test plots_extension_loaded()

    @testset "SVG export with mm margins" begin
        t, odv, odz, rocf_t, rocf, rocf_coi = _toy_ts_plot_payload()
        tmp = mktempdir()
        pf = joinpath(tmp, "Figures")
        mkpath(pf)
        path_names = OrderedDict{Symbol, String}(:pf_TS_figures => pf)
        TSCOPF.Save_Dynamic_Results_Plots_tsred(
            t, odv, odv, odv, odz, odz, zeros(length(t)),
            odz, odz, zeros(length(t)),
            rocf_t, rocf, rocf_coi,
            odv, odz, odz, odz, odv,
            100.0, 50.0, (-π / 2, π / 2), path_names)
        @test isfile(joinpath(pf, "Electrical Power vs time.svg"))
        @test isfile(joinpath(pf, "Delta vs time.svg"))
        @test isfile(joinpath(pf, "Delta (ref COI) vs time.svg"))
        # No reference machine passed → the machine-referenced figure must not appear.
        @test !isfile(joinpath(pf, "Delta (ref G2) vs time.svg"))
    end

    @testset "machine-referenced angle figure" begin
        t, odv, odz, rocf_t, rocf, rocf_coi = _toy_ts_plot_payload()
        tmp = mktempdir()
        pf = joinpath(tmp, "Figures")
        mkpath(pf)
        path_names = OrderedDict{Symbol, String}(:pf_TS_figures => pf)
        # δ relative to G2: the reference row is flat at zero, the other machine offset.
        δ_reft = OrderedDict{Int, Vector{Float64}}(
            1 => fill(0.2, length(t)), 2 => zeros(length(t)))
        TSCOPF.Save_Dynamic_Results_Plots_tsred(
            t, odv, odv, odv, odz, odz, zeros(length(t)),
            odz, odz, zeros(length(t)),
            rocf_t, rocf, rocf_coi,
            odv, odz, odz, odz, odv,
            100.0, 50.0, (-π / 2, π / 2), path_names;
            δ_ref_gen = 2, δ_reft = δ_reft)
        @test isfile(joinpath(pf, "Delta (ref G2) vs time.svg"))
        # The COI figure is kept alongside it, not replaced.
        @test isfile(joinpath(pf, "Delta (ref COI) vs time.svg"))
    end

    @testset "corridor dual SVGs across families" begin
        t, odv, odz, _, _, _ = _toy_ts_plot_payload()
        tmp = mktempdir()
        pf = joinpath(tmp, "Figures_Duals")
        mkpath(pf)
        # A machine-referenced δ corridor (G2 is the reference, so it carries no row) plus an
        # absolute speed corridor. The `nothing` families stand for the COI pair this run
        # never built and must be skipped silently.
        families = [
            (title = "δ corridor (ref. G2) - lower", side = "lower", tag = "", ylabel = "€/rad",
             data = OrderedDict{Int, Vector{Float64}}(1 => odz[1])),
            (title = "δ corridor (ref. G2) - upper", side = "upper", tag = "", ylabel = "€/rad",
             data = nothing),
            (title = "Δω corridor - lower", side = "lower", tag = "Domega ", ylabel = "€/p.u.",
             data = OrderedDict{Int, Vector{Float64}}(1 => odv[1])),
        ]
        TSCOPF.invoke_save_dual_ts_constraint_svgs!(t, families, pf)
        @test isfile(joinpath(pf, "Duals Trans. Stab. Const G1 lower.svg"))
        @test isfile(joinpath(pf, "Duals Trans. Stab. Const Domega G1 lower.svg"))
        @test !isfile(joinpath(pf, "Duals Trans. Stab. Const G1 upper.svg"))
        @test !isfile(joinpath(pf, "Duals Trans. Stab. Const G2 lower.svg"))
    end
end
