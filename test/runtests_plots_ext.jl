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
    end
end
