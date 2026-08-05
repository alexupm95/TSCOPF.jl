# Dump package ACOPF warm-start V/θ/Pg/Qg with GFM limits (no TS assembly).
using TSCOPF
using JuMP
using CSV
using DataFrames
using OrderedCollections
import MathOptInterface as MOI

const PKG_ROOT = dirname(@__DIR__)
const OUTDIR = joinpath(PKG_ROOT, "RESULTS", "gfm_trap_imax_1.2", "WarmStart_Dump")
mkpath(OUTDIR)

hsl = try
    TSCOPF.hsl_jll_available()
catch
    false
end

cfg = RunConfig(
    case = "9bus_gfm_imax12",
    base_MVA = 100.0,
    load_factor = 1.5,
    trans_stab = true,
    solver_name = hsl ? "Ipopt-ma57" : "Ipopt",
    silent_solver = true,
    time_limit_sec = 600.0,
    overwrite_results = true,
    save_duals = false,
    save_matrices = false,
    save_ts_plots = false,
    save_optim_matrices = false,
    ipopt = IpoptSolverConfig(
        tol = 1e-8,
        max_iter = 350,
        print_level = 0,
        hessian_approximation = "limited-memory",
        limited_memory_max_history = 20,
        acceptable_tol = 1e-7,
        obj_scaling_factor = 1e-4,
        nlp_scaling_method = "gradient-based",
    ),
    dispatch = DispatchConfig(
        type_model = "ACOPF",
        cost_type = "quadratic",
        use_matrix = true,
        ineq_sbranch_upper = true,
        ineq_ang_diff_branch = true,
    ),
    transient = TransientConfig(
        gen_dynamic_filename = "gen_dynamic_data_full.csv",
        gfm_dynamic_filename = "gfm_dynamic_data.csv",
        dyn_model = DynModelConfig(
            allow_gfm = true,
            gen_order = DQ_4TH,
            network_form = FULL_BUS,
            mech_power_mode = USE_PM,
            include_avr = true,
            include_governor = true,
        ),
    ),
)

sys = load_system(cfg, PKG_ROOT)
path_names = TSCOPF.build_results_path_names(PKG_ROOT, OUTDIR, OUTDIR)
for k in (:pf_dispatch, :pf_dispatch_CSV, :pf_bus_matrices, :pf_inputs)
    mkpath(path_names[k])
end

model = Setup_Optim_Model(cfg.solver_name; silent=cfg.silent_solver)
TSCOPF.apply_ipopt_options!(model, cfg.ipopt)

model, obj_function, obj_function_MVA, opf_dict = TSCOPF.Build_SteadyState_Model!(
    path_names, model, cfg.dispatch,
    sys.DBUS, sys.DGEN, sys.DCIR, sys.bus_gen_circ_dict_ON,
    cfg.base_MVA, sys.nBUS, sys.nGEN, sys.nCIR;
    save_matrices=false,
)
attach_gfm_acopf_limits!(model, opf_dict, sys.DGEN, sys.DGFM, cfg.base_MVA)

optimize!(model)
status = termination_status(model)
hints = TSCOPF.extract_opf_solved_hints(opf_dict)
obj = Float64(JuMP.value(obj_function_MVA))

bus_rows = DataFrame(
    BUS = Int[], V_pu = Float64[], Theta_rad = Float64[], Theta_deg = Float64[]
)
for bus in sort(collect(keys(hints.val_V)))
    push!(bus_rows, (bus, hints.val_V[bus], hints.val_θ[bus], rad2deg(hints.val_θ[bus])))
end

gen_rows = DataFrame(
    ID = Int[], BUS = Int[], P_pu = Float64[], Q_pu = Float64[], P_MW = Float64[], Q_MVAr = Float64[]
)
for gen in sort(collect(keys(hints.val_Pg)))
    bus = Int(sys.DGEN.bus[gen])
    push!(gen_rows, (
        gen, bus, hints.val_Pg[gen], hints.val_Qg[gen],
        hints.val_Pg[gen] * cfg.base_MVA, hints.val_Qg[gen] * cfg.base_MVA
    ))
end

# DQ machine warmstarts for SG gens
sg = Int[]
for g in 1:sys.nGEN
    if sys.DGEN.g_status[g] == 1 && !(g in TSCOPF.gfm_id_set(sys.DGFM))
        push!(sg, g)
    end
end
dq_rows = DataFrame(
    ID=Int[], BUS=Int[], E_fd=Float64[], delta_rad=Float64[], delta_deg=Float64[],
    Ed=Float64[], Eq=Float64[], Id=Float64[], Iq=Float64[]
)
for gen in sg
    bus = Int(sys.DGEN.bus[gen])
    Efd, δ, Ed, Eq, Id, Iq = TSCOPF.dq_machine_warmstart(
        hints.val_V[bus], hints.val_θ[bus], hints.val_Pg[gen], hints.val_Qg[gen],
        sys.DGEN_DYN.Xd_tr[gen], sys.DGEN_DYN.Xq_tr[gen], sys.DGEN_DYN.Xd[gen],
        sys.DGEN_DYN.Xq[gen], sys.DGEN_DYN.Ra[gen],
    )
    push!(dq_rows, (gen, bus, Efd, δ, rad2deg(δ), Ed, Eq, Id, Iq))
end

CSV.write(joinpath(OUTDIR, "pkg_warmstart_buses.csv"), bus_rows; delim=';')
CSV.write(joinpath(OUTDIR, "pkg_warmstart_gens.csv"), gen_rows; delim=';')
CSV.write(joinpath(OUTDIR, "pkg_warmstart_dq.csv"), dq_rows; delim=';')
open(joinpath(OUTDIR, "pkg_warmstart_obj.txt"), "w") do io
    println(io, "status=$status")
    println(io, "objective=$obj")
end
println("PKG warmstart status=$status obj=$obj")
println("Wrote $OUTDIR/pkg_warmstart_*.csv")
