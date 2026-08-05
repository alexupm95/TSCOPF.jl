function mpc = case9
% CASE9  WSCC 3-generator, 9-bus system, mirrored from the repo's 9-bus CSV inputs.
%
% This MATPOWER case is a faithful copy of INPUT_FILES/9bus/*.csv so that the
% PowerModels benchmark solves *exactly* the same network the in-house TSC-OPF
% code solves. It is consumed only by scripts/generate_pm_ac_reference.jl
% and never by the in-house pipeline.
%
% Source CSVs (column-for-column):
%   bus_data.csv        -> mpc.bus
%   generators_data.csv -> mpc.gen, mpc.gencost
%   line_data.csv       -> mpc.branch
%
% Modelling choices copied from the CSVs:
%   * Line resistances r = 0 and line charging b = 0  (lossless, purely reactive
%     series branches). Active losses are therefore identically zero, so the
%     dispatched active power equals the total demand (315 MW at nominal load).
%   * Flat linear generation cost: c2 = 0, c1 = 50, c0 = 0 for every unit.
%     With equal marginal costs and a lossless network the optimal objective is
%     invariant to the dispatch split and equals 50 * sum(Pd) = 50 * 315 = 15750.
%   * Slack / reference bus = bus 1 (type 3); buses 2,3 are PV (type 2).
%
% baseMVA = 100. Loads at buses 5/6/8 = 125/90/100 MW (50/30/35 MVAr).

%% MATPOWER Case Format : Version 2
mpc.version = '2';

%%-----  Power Flow Data  -----%%
%% system MVA base
mpc.baseMVA = 100;

%% bus data
%	bus_i	type	Pd	Qd	Gs	Bs	area	Vm	Va	baseKV	zone	Vmax	Vmin
mpc.bus = [
	1	3	0	0	0	0	1	1	0	16.5	1	1.1	0.9;
	2	2	0	0	0	0	1	1	0	18	1	1.1	0.9;
	3	2	0	0	0	0	1	1	0	13.8	1	1.1	0.9;
	4	1	0	0	0	0	1	1	0	230	1	1.1	0.9;
	5	1	125	50	0	0	1	1	0	230	1	1.1	0.9;
	6	1	90	30	0	0	1	1	0	230	1	1.1	0.9;
	7	1	0	0	0	0	1	1	0	230	1	1.1	0.9;
	8	1	100	35	0	0	1	1	0	230	1	1.1	0.9;
	9	1	0	0	0	0	1	1	0	230	1	1.1	0.9;
];

%% generator data
%	bus	Pg	Qg	Qmax	Qmin	Vg	mBase	status	Pmax	Pmin	Pc1	Pc2	Qc1min	Qc1max	Qc2min	Qc2max	ramp_agc	ramp_10	ramp_30	ramp_q	apf
mpc.gen = [
	1	71.6	27	300	-300	1.04	100	1	250	0	0	0	0	0	0	0	0	0	0	0	0;
	2	163	6.7	300	-300	1.025	100	1	300	0	0	0	0	0	0	0	0	0	0	0	0;
	3	85	-10.9	300	-300	1.025	100	1	270	0	0	0	0	0	0	0	0	0	0	0	0;
];

%% branch data
%	fbus	tbus	r	x	b	rateA	rateB	rateC	ratio	angle	status	angmin	angmax
mpc.branch = [
	1	4	0	0.0576	0	9999	9999	9999	1	0	1	-60	60;
	2	7	0	0.0625	0	9999	9999	9999	1	0	1	-60	60;
	3	9	0	0.0586	0	9999	9999	9999	1	0	1	-60	60;
	4	5	0	0.085	0	9999	9999	9999	1	0	1	-60	60;
	4	6	0	0.092	0	9999	9999	9999	1	0	1	-60	60;
	5	7	0	0.161	0	9999	9999	9999	1	0	1	-60	60;
	6	9	0	0.17	0	9999	9999	9999	1	0	1	-60	60;
	7	8	0	0.072	0	9999	9999	9999	1	0	1	-60	60;
	8	9	0	0.1008	0	9999	9999	9999	1	0	1	-60	60;
];

%%-----  OPF Data  -----%%
%% generator cost data
%	2 = polynomial cost model, startup, shutdown, ncost, then coefficients
%	(decreasing degree). ncost = 3 -> [c2 c1 c0] = [0 50 0] => linear cost 50 EUR/MW.
mpc.gencost = [
	2	0	0	3	0	50	0;
	2	0	0	3	0	50	0;
	2	0	0	3	0	50	0;
];
