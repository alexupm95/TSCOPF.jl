#=
================================================================================
 test/runtests_optim_matrices.jl
   Optimization-matrix export (sparse COO) + save_optim_matrices policy
================================================================================
 Covers the three pieces wired together:

   1. resolve_save_optim_matrices — non-interactive policy. Honours the request
      for steady-state runs; forces it FALSE (with a warning) for TSC runs, where
      the Jacobian/Hessian have thousands of rows/columns. (Replaces the old
      Base.prompt that would hang any non-interactive run — the C5 fix.)
   2. save_sparse_coo_csv — writes only the structural nonzeros as row;col;value,
      round-tripping back to the same matrix.
   3. End-to-end: a steady-state AC-OPF with save_optim_matrices=true actually
      writes Jacobian_COO / Hessian_COO / Gradient / index legends under
      RESULTS/.../Optim_Matrices/.

 Run from the project root:

     julia test/runtests_optim_matrices.jl
================================================================================
=#
using Test

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))
# Guarded: the aggregate runner (test/runtests.jl) already includes this;
# the check keeps standalone runs working without redefining every helper.
@isdefined(TEST_OVERWRITE_RESULTS) || include(joinpath(PROJECT_ROOT, "test", "test_common.jl"))

@testset "Optim-matrix export (sparse COO) + save policy" begin

    # --- policy resolver: non-interactive, forces false for TSC --------------
    @testset "resolve_save_optim_matrices" begin
        @test resolve_save_optim_matrices(true,  false) == true   # steady-state honours request
        @test resolve_save_optim_matrices(false, false) == false
        @test resolve_save_optim_matrices(false, true)  == false
        # TSC + requested → forced false AND warns (never blocks on stdin)
        @test (@test_logs (:warn,) resolve_save_optim_matrices(true, true)) == false
    end

    # --- COO writer round-trips a sparse matrix, storing only nonzeros -------
    @testset "save_sparse_coo_csv round-trip" begin
        M = SparseArrays.sparse([1, 2, 3], [1, 3, 2], [10.0, -2.5, 4.0], 3, 3)
        tmp = mktempdir()
        try
            f = joinpath(tmp, "M_COO.csv")
            TSCOPF.save_sparse_coo_csv(M, f)
            df = CSV.read(f, DataFrame; delim=';')
            @test names(df) == ["row", "col", "value"]
            @test nrow(df) == 3                          # exactly nnz, not 9 dense entries
            Mr = SparseArrays.sparse(df.row, df.col, df.value, 3, 3)
            @test Mr == M                                # exact reconstruction
        finally
            rm(tmp; force=true, recursive=true)
        end
    end

    # --- end-to-end: steady-state AC-OPF writes the sparse export ------------
    @testset "AC-OPF writes Optim_Matrices/" begin
        cfg = dispatch_run_config(; type_model="ACOPF", use_matrix=true,
            solver_name="Ipopt", silent_solver=true,
            save_optim_matrices=true, save_duals=false)
        sys = load_fixture_system(cfg)
        res = run_fixture_case!(cfg, sys)
        @test res.status in (OPTIMAL, LOCALLY_SOLVED)

        odir = joinpath(res.path_names[:pf_results_date], "Optim_Matrices")
        @test isdir(odir)
        for fn in ("Jacobian_COO.csv", "Hessian_COO.csv", "Gradient_Lagrangian.csv",
                   "Variable_Index.csv", "Jacobian_Constraint_Index.csv")
            @test isfile(joinpath(odir, fn))
        end

        # Jacobian COO is well-formed and nonempty
        jac = CSV.read(joinpath(odir, "Jacobian_COO.csv"), DataFrame; delim=';')
        @test names(jac) == ["row", "col", "value"]
        @test nrow(jac) > 0
    end
end
