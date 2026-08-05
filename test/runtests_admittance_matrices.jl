#=
================================================================================
 test/runtests_admittance_matrices.jl
   Admittance / susceptance matrix builders — pure linear algebra, NO OPF solve
================================================================================
 Purpose
 -------
 Guard the sparse matrix builders in _common/functions_4_admittance_matrices.jl
 against regressions while they are refactored for sparse efficiency (ranked
 items 1/2/3/5). Every production matrix is compared against an INDEPENDENT,
 deliberately-naive dense reference built here in the test:

   - ref_ybus_dense         — textbook dense Ybus accumulation
   - ref_bbus               — A·diag(b)·Aᵀ with a dense incidence
   - ref_augment            — generator-internal augmentation, written out by hand
   - ref_schur              — Kron reduction via the DENSE inverse  Y22 - Y21·inv(Y11)·Y12

 Because the references re-implement the ORIGINAL (dense / inv-based) behaviour,
 running this test BEFORE the refactor confirms the references match the current
 code, and running it AFTER confirms the refactor preserves the matrices exactly
 (sparse-`lu` Schur is checked against the dense-`inv` Schur — the item-2 concern).

 Driver case: 39-bus (New England) — has generator dynamic data, transformers
 (tap) and series resistance r≠0, so it exercises the transformer Ybus path and
 the SIMPLE-vs-PowerModels susceptance split. Case-specific identities (Ybus
 symmetry, SIMPLE==PowerModels) are guarded on t_shift==0 / r==0.

 Run from the project root:

     julia test/runtests_admittance_matrices.jl
================================================================================
=#
using Test
using LinearAlgebra, SparseArrays
using Printf

const PROJECT_ROOT = dirname(@__DIR__)
include(joinpath(@__DIR__, "test_env.jl"))

const BASE_MVA = 100.0

# =============================================================================
#  Independent dense references (the "before" behaviour, re-derived by hand)
# =============================================================================
"""Dense Ybus by textbook accumulation (series y, line charging b/2, tap/shift, bus shunts)."""
function ref_ybus_dense(DBUS, DCIR, nBUS, nCIR, base_MVA)
    Y = zeros(ComplexF64, nBUS, nBUS)
    for i in 1:nCIR
        DCIR.l_status[i] == 1 || continue
        k = DCIR.from_bus[i]; m = DCIR.to_bus[i]
        ykm = 1 / (DCIR.l_res[i] + 1im * DCIR.l_reac[i])
        bsh = DCIR.l_sh_susp[i] / 2
        t = DCIR.t_tap[i]; sh = deg2rad(DCIR.t_shift[i])
        Y[k,k] += (1/t)^2 * ykm + 1im * bsh
        Y[k,m] += -(1/t) * ykm * exp(1im * sh)
        Y[m,k] += -(1/t) * ykm * exp(-1im * sh)
        Y[m,m] += ykm + 1im * bsh
    end
    for b in 1:nBUS
        Y[b,b] += (DBUS.g_sh[b] + 1im * DBUS.b_sh[b]) / base_MVA
    end
    return Y
end

"""Dense susceptance matrix B = A·diag(b₀)·Aᵀ; `pm=true` uses the PowerModels b = -x/(r²+x²)."""
function ref_bbus(DCIR, nBUS, nCIR; pm::Bool)
    A = zeros(Float64, nBUS, nCIR)
    for i in 1:nCIR
        A[DCIR.from_bus[i], i] += 1.0
        A[DCIR.to_bus[i],   i] -= 1.0
    end
    b0 = Vector{Float64}(undef, nCIR)
    for i in 1:nCIR
        b0[i] = pm ? -(DCIR.l_status[i] * DCIR.l_reac[i]) / (DCIR.l_res[i]^2 + DCIR.l_reac[i]^2) :
                     -DCIR.l_status[i] / DCIR.l_reac[i]
    end
    return A * Diagonal(b0) * A'
end

"""Augment a dense network admittance with generator internals (Y′ = -j/X′d) on extra nodes."""
function ref_augment(Ynet::Matrix{ComplexF64}, DGEN_DYN, active_gen, nBUS)
    nG = length(active_gen)
    Y = zeros(ComplexF64, nBUS + nG, nBUS + nG)
    Y[1:nBUS, 1:nBUS] .= Ynet
    for (idx, g) in enumerate(active_gen)
        yg = 1 / (1im * DGEN_DYN.Xd_tr[g])
        tb = DGEN_DYN.bus[g]; ib = nBUS + idx
        Y[tb,tb] += yg; Y[ib,ib] += yg
        Y[tb,ib] -= yg; Y[ib,tb] -= yg
    end
    return Y
end

"""Kron reduction via the dense inverse: eliminate the first nBUS nodes, keep the rest."""
function ref_schur(M::Matrix{ComplexF64}, nBUS)
    M11 = M[1:nBUS, 1:nBUS]; M12 = M[1:nBUS, nBUS+1:end]
    M21 = M[nBUS+1:end, 1:nBUS]; M22 = M[nBUS+1:end, nBUS+1:end]
    return M22 - M21 * inv(M11) * M12
end

# =============================================================================
#  Load the 9-bus system (with dynamics) — no OPF, just the data + matrices
# =============================================================================
const CASE = "39bus"   # transformers (tap) + r≠0 → exercises tap and the SIMPLE/PowerModels split
# These builders are internal to TSCOPF (not exported), so they must be qualified.
# They used to be called bare, which is why this file could not run at all.
DBUS, DGEN, DGEN_DYN, DCIR, _bm, _rbm =
    TSCOPF.Read_Input_Data(fixture_case(CASE), true)
const NBUS = length(DBUS.bus)
const NGEN = length(DGEN.id)
const NCIR = length(DCIR.from_bus)
const ACTIVE_GEN = findall(==(1), DGEN.g_status)
const BUS_FAULT = DGEN_DYN.bus[ACTIVE_GEN[1]]   # a real generator terminal bus

# Production matrices
Ybus       = TSCOPF.Calculate_Ybus_sparse(DBUS, DCIR, NBUS, NCIR, BASE_MVA)
Bbus_s     = TSCOPF.Calculate_Matrix_B(DBUS, DCIR, NBUS, NCIR)
Bbus_pm    = TSCOPF.Calculate_Matrix_B_PowerModels(DBUS, DCIR, NBUS, NCIR)
Yfault_sc  = TSCOPF.Calculate_Ybus_fault_SC_Kron(Ybus, DBUS, DGEN, DGEN_DYN, NBUS, ACTIVE_GEN, BASE_MVA, BUS_FAULT)
Yfault_lg  = TSCOPF.Calculate_Ybus_fault_LG_Kron(Ybus, DBUS, DGEN, DGEN_DYN, NBUS, ACTIVE_GEN, BASE_MVA)
Ypostf     = TSCOPF.Calculate_Ybus_postf_ClearFault_Kron(DBUS, DCIR, DGEN, DGEN_DYN, NBUS, NCIR, ACTIVE_GEN, BASE_MVA)
Yred_fault = TSCOPF.Reduce_Matrix(Yfault_sc, NBUS)
Yred_postf = TSCOPF.Reduce_Matrix(Ypostf,    NBUS)

# Network references (dense) used to assemble the augmented references
ynet         = ref_ybus_dense(DBUS, DCIR, NBUS, NCIR, BASE_MVA)
yload        = [(DBUS.p_d[b] - 1im * DBUS.q_d[b]) / BASE_MVA for b in 1:NBUS]
ynet_loaded  = copy(ynet); for b in 1:NBUS; ynet_loaded[b,b] += yload[b]; end
ynet_sc      = copy(ynet_loaded); ynet_sc[BUS_FAULT, BUS_FAULT] = FAULT_BUS_SHUNT   # SC overwrites
ynet_postf   = ynet_loaded   # post-fault: cleared topology + loads as constant-Z

Δ(A, B) = maximum(abs.(Matrix(A) .- Matrix(B)))

@testset "Admittance / susceptance matrices vs independent references" begin

    @testset "Calculate_Ybus_sparse == dense reference" begin
        @test Matrix(Ybus) ≈ ynet
        if all(iszero, DCIR.t_shift)                    # Ybus symmetric only without phase shifters
            @test issymmetric(Matrix(Ybus))
        end
        @printf("  Δ Ybus            = %.3e\n", Δ(Ybus, ynet))
    end

    @testset "Susceptance matrices (SIMPLE & PowerModels)" begin
        @test Matrix(Bbus_s)  ≈ ref_bbus(DCIR, NBUS, NCIR; pm=false)
        @test Matrix(Bbus_pm) ≈ ref_bbus(DCIR, NBUS, NCIR; pm=true)
        if all(iszero, DCIR.l_res)                       # SIMPLE and PowerModels agree only when r=0
            @test Matrix(Bbus_s) ≈ Matrix(Bbus_pm)
        end
        @printf("  Δ Bbus_simple     = %.3e\n", Δ(Bbus_s,  ref_bbus(DCIR, NBUS, NCIR; pm=false)))
        @printf("  Δ Bbus_PowerModels= %.3e\n", Δ(Bbus_pm, ref_bbus(DCIR, NBUS, NCIR; pm=true)))
    end

    @testset "Fault / post-fault augmented Ybus == reference" begin
        @test size(Yfault_sc) == (NBUS + NGEN, NBUS + NGEN)
        @test Matrix(Yfault_sc) ≈ ref_augment(ynet_sc,     DGEN_DYN, ACTIVE_GEN, NBUS)
        @test Matrix(Yfault_lg) ≈ ref_augment(ynet_loaded, DGEN_DYN, ACTIVE_GEN, NBUS)
        @test Matrix(Ypostf)    ≈ ref_augment(ynet_postf,  DGEN_DYN, ACTIVE_GEN, NBUS)
        @printf("  Δ Yfault_SC       = %.3e\n", Δ(Yfault_sc, ref_augment(ynet_sc,     DGEN_DYN, ACTIVE_GEN, NBUS)))
        @printf("  Δ Yfault_LG       = %.3e\n", Δ(Yfault_lg, ref_augment(ynet_loaded, DGEN_DYN, ACTIVE_GEN, NBUS)))
        @printf("  Δ Ypostf          = %.3e\n", Δ(Ypostf,    ref_augment(ynet_postf,  DGEN_DYN, ACTIVE_GEN, NBUS)))
    end

    @testset "Reduce_Matrix (Kron) == dense inv-based Schur" begin
        # item-2 check: sparse-lu Schur must match the dense inverse Schur
        @test size(Yred_fault) == (NGEN, NGEN)
        @test Matrix(Yred_fault) ≈ ref_schur(Matrix(Yfault_sc), NBUS) rtol=1e-8
        @test Matrix(Yred_postf) ≈ ref_schur(Matrix(Ypostf),    NBUS) rtol=1e-8
        @printf("  Δ Yred_fault      = %.3e\n", Δ(Yred_fault, ref_schur(Matrix(Yfault_sc), NBUS)))
        @printf("  Δ Yred_postf      = %.3e\n", Δ(Yred_postf, ref_schur(Matrix(Ypostf),    NBUS)))
    end

    @testset "FULL_BUS dynamic Ybus carries NO load (network only)" begin
        # The FULL_BUS path models loads in the nodal power balance via ZIP, so the
        # dynamic Ybus must be network-only — never carrying load admittances.
        Yfb_nofault = TSCOPF.Calculate_Ybus_fullbus_dynamics(Ybus)
        Yfb_fault   = TSCOPF.Calculate_Ybus_fullbus_dynamics(Ybus; bus_fault=BUS_FAULT)

        @test Matrix(Yfb_nofault) == Matrix(Ybus)        # equals the base network exactly (no loads)

        # SC fault OVERWRITES the faulted-bus diagonal with FAULT_BUS_SHUNT; everything else = network
        @test Yfb_fault[BUS_FAULT, BUS_FAULT] == FAULT_BUS_SHUNT
        D = Matrix(Yfb_fault) .- Matrix(Ybus)
        D[BUS_FAULT, BUS_FAULT] = 0
        @test all(iszero, D)

        # explicit no-load check on a genuine load bus: diagonal unchanged from the network
        load_bus = findfirst(>(0.0), DBUS.p_d)
        if load_bus !== nothing
            @test Yfb_nofault[load_bus, load_bus] == Ybus[load_bus, load_bus]
        end
        @printf("  Δ Ybus_fullbus(no fault) = %.3e\n", Δ(Yfb_nofault, Ybus))
    end
end
