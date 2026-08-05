# Function to check the coherence of the initial input data.
# Fails fast (throws) on an incoherent (type_model, solver, trans_stab) combo,
# rather than silently auto-correcting, so a misconfigured run never solves the
# wrong problem. Returns (trans_stab, solver_name) unchanged for call-site
# compatibility.
function Check_Coherence_Input_Data(trans_stab::Bool, type_model::String, solver_name::String)
    valid_models  = ("ACOPF", "DCOPF", "ED", "UC")
    valid_solvers = (
        "Ipopt", "Gurobi", "HiGHS",
        IPOPT_HSL_LINEAR_SOLVERS..., IPOPT_PARDISO_SOLVER,
        MADNLP_BACKEND_SOLVERS...,
    )
    models_str  = join(valid_models, ", ")    # build the list strings first to avoid
    solvers_str = join(valid_solvers, ", ")   # nesting quotes inside string interpolation

    type_model in valid_models ||
        throw(ArgumentError("Unknown type_model \"$type_model\". Valid: $models_str."))
    solver_name in valid_solvers ||
        throw(ArgumentError("Unknown solver_name \"$solver_name\". Valid: $solvers_str."))

    # ACOPF (and therefore TSC-ACOPF) is nonlinear/nonconvex: it MUST be solved
    # with an NLP solver. Gurobi and HiGHS are LP/QP/MILP solvers and cannot
    # handle the AC power-flow equations, so they are forbidden here.
    if type_model == "ACOPF" && (solver_name == "Gurobi" || solver_name == "HiGHS")
        throw(ArgumentError(
            "ACOPF / TSC-ACOPF is nonlinear and requires an NLP solver " *
            "(Ipopt, MadNLP, Ipopt/MadNLP+HSL, or Ipopt-pardiso). " *
            "Solver \"$solver_name\" (LP/QP) is not allowed for ACOPF."))
    end

    if ipopt_hsl_linear_solver(solver_name) !== nothing && !hsl_jll_available()
        throw(ArgumentError(
            "solver_name \"$solver_name\" requires optional HSL_jll " *
            "(`using Pkg; Pkg.add(\"HSL_jll\")`) and a valid HSL license."))
    end

    if solver_name == IPOPT_PARDISO_SOLVER && !pardiso_available()
        throw(ArgumentError(
            "solver_name \"Ipopt-pardiso\" requires IpoptSolverConfig.pardiso_lib_path or " *
            "ENV[\"JULIA_PARDISO_LIB\"] pointing to an existing libpardiso file."))
    end

    if solver_name in MADNLP_HSL_LINEAR_SOLVERS && !madnlp_hsl_ext_available()
        throw(ArgumentError(
            "solver_name \"$solver_name\" requires optional MadNLPHSL " *
            "(`using Pkg; Pkg.add(\"MadNLPHSL\")`) plus HSL_jll and a valid HSL license."))
    end

    # UC is a MILP: Gurobi only in this codebase (HiGHS MIP path not validated).
    if type_model == "UC" && solver_name != "Gurobi"
        throw(ArgumentError(
            "UC requires solver_name=\"Gurobi\" (MILP). Solver \"$solver_name\" is not allowed."))
    end

    # Transient stability is implemented only on top of ACOPF (nonlinear) and
    # DCOPF (linearised Taylor form). ED and UC have no TS formulation yet.
    if trans_stab && !(type_model in ("ACOPF", "DCOPF"))
        throw(ArgumentError(
            "Transient stability is only available with ACOPF or DCOPF, not \"$type_model\"."))
    end

    return trans_stab, solver_name
end

"""`true` when a Gurobi license and install are usable (UC / MILP tests)."""
function gurobi_available()::Bool
    fn = _GUROBI_AVAILABLE_FN[]
    if fn !== nothing
        return fn()
    end
    pkgid = Base.identify_package("Gurobi")
    pkgid === nothing && return false
    try
        gurobi = Base.require(pkgid)
        env = gurobi.Env()
        return env !== nothing
    catch
        return false
    end
end

# Resolve whether to actually export the optimization matrices (Jacobian, Hessian,
# Lagrangian gradient). They scale with the model size, so for a transient-stability
# run — thousands of dynamic variables and constraints — exporting them is forced OFF
# (with a warning) regardless of the requested value. This is intentionally
# NON-INTERACTIVE: it never blocks on stdin, so it is safe inside δ_tol / contingency
# sweeps, the test suite, and batch jobs (the previous version prompted with
# Base.prompt and would hang any non-interactive run).
function resolve_save_optim_matrices(save_requested::Bool, trans_stab::Bool)::Bool
    if save_requested && trans_stab
        @warn "save_optim_matrices was requested but is being forced to FALSE for a " *
              "transient-stability run: the Jacobian/Hessian of a TSC-OPF model have " *
              "thousands of rows/columns and would consume large amounts of memory and " *
              "disk. Export them by running a steady-state ED / DC-OPF / AC-OPF case."
        return false
    end
    return save_requested
end


# Function to check if the static and dynamic data of generators match in length
function Check_Gen_Sta_Dyn_Data(ngen_static::Int, ngen_dynamic::Int)
    if ngen_static != ngen_dynamic throw(ArgumentError("Dynamic and static data of generators do not match in length.")) end
end

"""
    validate_dgen_dyn_row_ids!(DGEN::DataFrame, DGEN_DYN::DataFrame)

Fail fast when dynamic rows are not aligned with static generator ids.

`DGEN_DYN.H[gen]` (and `.D`, `.Xd_tr`, …) index rows by generator id, so row `i`
must carry `id == i` and the same bus as `DGEN` row `i`.
"""
function validate_dgen_dyn_row_ids!(DGEN::DataFrame, DGEN_DYN::DataFrame)
    n = nrow(DGEN_DYN)
    n == nrow(DGEN) ||
        throw(ArgumentError(
            "DGEN_DYN has $n rows but DGEN has $(nrow(DGEN)); counts must match."))
    expected = collect(1:n)
    DGEN_DYN.id == expected ||
        throw(ArgumentError(
            "DGEN_DYN.id must equal 1:n (got $(DGEN_DYN.id)); row order must match generator ids."))
    DGEN.id == expected ||
        throw(ArgumentError(
            "DGEN.id must equal 1:n (got $(DGEN.id)); cannot align dynamic parameters by id."))
    for i in 1:n
        DGEN_DYN.bus[i] == DGEN.bus[i] ||
            throw(ArgumentError(
                "DGEN_DYN row $i bus $(DGEN_DYN.bus[i]) ≠ DGEN bus $(DGEN.bus[i]); " *
                "dynamic CSV row order must match generators_data.csv."))
    end
    return nothing
end

"""
    validate_dgen_dyn_subset_ids!(DGEN, DGEN_DYN)

When GFM is enabled, `DGEN_DYN` holds **SG rows only**. Each row's `id` must
exist in `DGEN` with a matching `bus`; ids must be unique.
"""
function validate_dgen_dyn_subset_ids!(DGEN::DataFrame, DGEN_DYN::DataFrame)
    nrow(DGEN_DYN) == 0 &&
        throw(ArgumentError("DGEN_DYN is empty; at least one SG dynamic row is required."))
    seen = Set{Int}()
    dgen_by_id = Dict{Int, Int}(Int(DGEN.id[i]) => i for i in 1:nrow(DGEN))
    for i in 1:nrow(DGEN_DYN)
        gid = Int(DGEN_DYN.id[i])
        gid in seen &&
            throw(ArgumentError("Duplicate generator id $gid in DGEN_DYN."))
        push!(seen, gid)
        haskey(dgen_by_id, gid) ||
            throw(ArgumentError("DGEN_DYN id $gid is not present in DGEN."))
        j = dgen_by_id[gid]
        DGEN_DYN.bus[i] == DGEN.bus[j] ||
            throw(ArgumentError(
                "DGEN_DYN id $gid bus $(DGEN_DYN.bus[i]) ≠ DGEN bus $(DGEN.bus[j])."))
    end
    return nothing
end

"""Validate `DGFM` rows against `DGEN` (unique ids, matching buses)."""
function validate_dgfm_ids!(DGEN::DataFrame, DGFM::DataFrame)
    nrow(DGFM) == 0 && return nothing
    seen = Set{Int}()
    dgen_by_id = Dict{Int, Int}(Int(DGEN.id[i]) => i for i in 1:nrow(DGEN))
    for i in 1:nrow(DGFM)
        gid = Int(DGFM.id[i])
        gid in seen &&
            throw(ArgumentError("Duplicate generator id $gid in DGFM (gfm_dynamic_data)."))
        push!(seen, gid)
        haskey(dgen_by_id, gid) ||
            throw(ArgumentError("DGFM id $gid is not present in DGEN."))
        j = dgen_by_id[gid]
        DGFM.bus[i] == DGEN.bus[j] ||
            throw(ArgumentError(
                "DGFM id $gid bus $(DGFM.bus[i]) ≠ DGEN bus $(DGEN.bus[j])."))
    end
    return nothing
end

"""
    validate_gen_dyn_partition!(DGEN, DGEN_DYN, DGFM)

Every `DGEN.id` must appear in exactly one of `DGEN_DYN` (SG) or `DGFM` (GFM).
"""
function validate_gen_dyn_partition!(
    DGEN::DataFrame,
    DGEN_DYN::DataFrame,
    DGFM::DataFrame,
)
    validate_dgen_dyn_subset_ids!(DGEN, DGEN_DYN)
    validate_dgfm_ids!(DGEN, DGFM)
    sg_ids = Set{Int}(Int.(DGEN_DYN.id))
    gfm_ids = Set{Int}(Int.(DGFM.id))
    overlap = intersect(sg_ids, gfm_ids)
    isempty(overlap) ||
        throw(ArgumentError(
            "Generator id(s) appear in both gen_dynamic_data and gfm_dynamic_data: " *
            join(sort!(collect(overlap)), ", ") * "."))
    all_ids = Set{Int}(Int.(DGEN.id))
    covered = union(sg_ids, gfm_ids)
    missing_ids = setdiff(all_ids, covered)
    isempty(missing_ids) ||
        throw(ArgumentError(
            "Generator id(s) missing from both gen_dynamic_data and gfm_dynamic_data: " *
            join(sort!(collect(missing_ids)), ", ") * "."))
    extra = setdiff(covered, all_ids)
    isempty(extra) ||
        throw(ArgumentError(
            "Dynamic id(s) not in DGEN: " * join(sort!(collect(extra)), ", ") * "."))
    return nothing
end

# Function to check if the Transient Stability is being required with ED or DCOPF
function Check_TS_w_ED_or_DCOPF(type_model::String, trans_stab::Bool)
    if (type_model == "ED" || type_model == "DCOPF") && trans_stab == true
        throw(ArgumentError("The code cannot run Transient Stability using $(type_model) as the dispatch model.")) 
    end
end

function Check_Length_Data_TS(_dataframe::DataFrame, active_gen::Vector{Int64})
    if nrow(_dataframe) != length(active_gen)
        throw(ArgumentError("The number of rows in the DataFrame and the number of active generators must be the same."))
    end
end

function Check_Length_Data_TS(_dataframe::DataFrame, active_gen::Int64)
    if nrow(_dataframe) != active_gen
        throw(ArgumentError("The number of rows in the DataFrame and the number of active generators must be the same."))
    end
end

# # ------------------
# # Some sanity checks
# # ------------------
# if !(haskey(bus_mapping, bus_fault))
#     throw(ArgumentError("The ID of the faulted bus does not exist in the CSV of input data."))
# end

# if maximum(circ_trip) > nCIR
#     throw(ArgumentError("The IDs of the faulted circuits are greater than the ids in DCIR."))
# end