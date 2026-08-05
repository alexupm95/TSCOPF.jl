#=
================================================================================
 test/runtests_fixture_isolation.jl — the suite must not read the demo tree
================================================================================
 `INPUT_FILES/` is demo and case-study data the user is invited to edit;
 `test/INPUT_FILES/` holds the frozen copies the pins are asserted against. A
 suite that reads the former re-couples the two and reintroduces the failure
 mode the split removed: an ordinary data edit turning CI red.

 Two shapes have to be caught. The obvious one names the tree
 (`joinpath(PROJECT_ROOT, "INPUT_FILES", ...)`). The dangerous one does not —
 `load_system(cfg, PROJECT_ROOT)` reads `<repo>/INPUT_FILES/<case>` while
 containing no such string, which is how the coupling would creep back in
 unnoticed.

 Costs no solve; runs first in the fast gate.
================================================================================
=#

using Test

@testset "Test inputs are isolated from the demo tree" begin

    @testset "fixtures are present" begin
        for case in ("9bus", "39bus", "9bus_gfm", "9bus_test_mfile")
            @test isdir(fixture_case(case))
        end
        @test isfile(fixture_matpower("case9.m"))
        @test isfile(fixture_matpower("case39.m"))
        # The coupled pair: the .m carries loads pre-scaled by 1.5, the CSVs raw.
        @test isfile(joinpath(fixture_case("9bus"), "bus_data.csv"))
    end

    @testset "no suite reaches back into INPUT_FILES/" begin
        # Exempt: this file; the fixture helpers themselves; and the two suites
        # that legitimately read the external _TSCOPF_simulations repo.
        exempt = ("runtests_fixture_isolation.jl", "test_paths.jl",
                  "runtests_simulation_pins.jl", "runtests_bound_encoding_tsc.jl")

        # Both patterns require PROJECT_ROOT, i.e. a path rooted at the repository
        # rather than at test/. That is the thing that is wrong — naming the tree is
        # not — so prose, docstrings and comments about `INPUT_FILES/` never fire and
        # need no stripping.
        for path in filter(p -> endswith(p, ".jl"), readdir(@__DIR__; join = true))
            basename(path) in exempt && continue
            for (n, line) in enumerate(eachline(path))
                # ponytail: line-scoped, so a call split across lines would slip
                # through. Every current call site is one line; tighten only if that
                # stops being true.
                offender = (occursin("PROJECT_ROOT", line) && occursin("\"INPUT_FILES\"", line)) ||
                           occursin(r"load_system\(.*PROJECT_ROOT", line)
                if offender
                    @test "$(basename(path)):$n reads the demo tree — use fixture_case / " *
                          "load_fixture_system (see test/INPUT_FILES/README.md)" == ""
                end
            end
        end
    end
end
