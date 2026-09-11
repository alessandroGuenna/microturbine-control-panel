function sched = computeSpeedSchedule(varargin)
%COMPUTESPEEDSCHEDULE  Efficiency-optimal shaft-speed schedule n_ref(P_el).
%
%   For the variable-speed single-shaft microturbine, this finds - for every
%   electrical power level - the shaft speed that MAXIMISES the electrical
%   efficiency eta_el, subject to:
%       * speed validity range      n in [n_min , n_max]   (with margin)
%       * turbine choking preserved  beta_t >= beta_crit*(1 + chokeMargin)
%       * compressor surge margin    (m - m_surge)/m_surge >= surgeMargin
%       * turbine-inlet temperature  T3 <= T3_max           (maxFuel limit)
%
%   The steady-state point solver reproduces the PLANT physics used in
%   microturbine_model_v2.slx exactly:
%       - compressor map : same scattered (beta,n)->flow data as the PS lookup
%       - compressor     : constant isentropic efficiency (adiabatic relation)
%       - plenum/ducts   : P3 = PR*P1*(1 - lambda*m_a^2)
%       - turbine        : CHOKED, m_eg = K_choke*P3/sqrt(T3) with design K_choke
%       - exhaust filter : P4 = P_amb + lambda_eg_filter*m_eg^2
%       - recuperator    : same 5-node metal model, UA ~ mdot^0.8 (Dittus-Boelter)
%       - combustion     : algebraic energy balance (same as CC block)
%
%   USAGE
%       sched = computeSpeedSchedule();                 % defaults + plots
%       sched = computeSpeedSchedule('surgeMargin',0.15,'plot',false);
%
%   NAME-VALUE OPTIONS (all optional)
%       'T1'           ambient / compressor-inlet temperature [K]     (288.15)
%       'chokeMargin'  fractional margin above beta_crit              (0.05)
%       'surgeMargin'  minimum compressor surge margin  [-]           (0.10)
%       'speedMargin'  fractional margin inside [n_min,n_max]         (0.03)
%       'nGrid'        # of speed grid points                         (61)
%       'prGrid'       # of pressure-ratio grid points                (81)
%       'nPower'       # of schedule power breakpoints                (40)
%       'plot'         make figures                                   (true)
%       'save'         save schedule to speedSchedule.mat             (false)
%       'beta','n','flow'          pre-built compressor-map vectors from
%                                  drawCompressorMap.m (n in rad/s). If given,
%                                  CompressorMap.mat is not re-loaded/re-cleaned.
%       'm_surge','beta_surge'     surge-line vectors (optional; else read .mat)
%
%   OUTPUT struct SCHED
%       .P_el_W , .P_el_kW   schedule power breakpoints
%       .n_ref_rpm           optimal speed at each breakpoint
%       .PR_ref , .T3_ref    pressure ratio / turbine-inlet temp on the schedule
%       .eta_el_ref          efficiency achieved on the schedule
%       .surgeMargin_ref , .chokeMargin_ref
%       .map                 full feasible operating map (for inspection/plots)
%       .lookup              ready-to-paste breakpoints/values for a 1-D lookup
%
%   NOTE: the physical constants below MUST be kept in sync with data_v2.m.

% ----------------------------------------------------------------------------
% 0) Options
% ----------------------------------------------------------------------------
p = inputParser;
p.addParameter('T1', 15 + 273.15);         % operating inlet temp the schedule is for [K]
p.addParameter('T1_design', 15 + 273.15);  % design inlet temp K_choke was fixed at [K]
p.addParameter('K_choke', []);             % fixed turbine constant (from data_v2); []=derive here
p.addParameter('chokeMargin', 0.2);
p.addParameter('surgeMargin', 0.10);
p.addParameter('speedMargin', 0.03);
p.addParameter('T3_min', 900);             % flame-stability floor on turbine-inlet temp [K]
p.addParameter('nGrid', 61);
p.addParameter('prGrid', 81);
p.addParameter('nPower', 40);
p.addParameter('P_min_kW', []);   % lower power bound of the schedule ([]=auto: 5% of Pmax)
p.addParameter('plot', true);
p.addParameter('save', false);
% Pre-computed compressor map vectors (as produced by drawCompressorMap.m). If
% supplied, the CompressorMap.mat load + concatenation + de-dup is skipped.
p.addParameter('beta', []);       % pressure-ratio vector  [-]
p.addParameter('n',    []);       % speed vector           [rad/s]
p.addParameter('flow', []);       % mass-flow vector       [kg/s]
p.addParameter('m_surge',    []); % surge-line flow  [kg/s] (optional; else read from .mat)
p.addParameter('beta_surge', []); % surge-line PR    [-]    (optional; else read from .mat)
p.parse(varargin{:});
o = p.Results;

% make sure designPoint.m / makeStation.m (same folder) are reachable
addpath(fileparts(mfilename('fullpath')));

% ----------------------------------------------------------------------------
% 1) Physical constants  (KEEP IN SYNC WITH data_v2.m)
% ----------------------------------------------------------------------------
c.P_amb   = 101325;
c.P1      = c.P_amb - 1e3;      % after inlet filter
c.T1      = o.T1;
c.eta_c   = 0.81;
c.eta_t   = 0.88;
c.k       = 1.4;
c.cp_air  = 1005;
c.cp_eg   = 1100;
c.lambda  = 0.313;             % combined duct+REC+CC pressure-drop coeff
c.eta_comb= 0.99;
c.LHV     = 47e6;
c.lambda_eg_filter = 15e3;
c.eta_mech= 0.97;
c.eta_gen = 0.98;
c.J       = 3e-3;
c.eps_REC = 0.75;              % only used by designPoint for the design K_choke

% recuperator (5-node, same as plant / data_v2.m)
c.ma0   = 0.440386860108652;
c.meg0  = 0.445815110936435;
c.UAc0  = 2571;
c.UAh0  = 2571;
c.Cm    = 25e3;                %(not needed at steady state)
c.Nrec  = 5;

% design reference & limits
PR_des   = 4;
n_des    = 65000;              % rpm
c.T3_des = 950 + 273.15;
c.T3_max = 960 + 273.15;
c.T3_min = o.T3_min;          % lean blow-out / flame-stability floor [K]

c.beta_crit = ((c.k+1)/2)^(c.k/(c.k-1));   % ~1.893

% ----------------------------------------------------------------------------
% 2) Compressor map  ->  interpolant  flow = F(beta, n[rad/s])  (== PS lookup)
% ----------------------------------------------------------------------------
% Use caller-supplied vectors if given (already built + de-duplicated by
% drawCompressorMap.m); otherwise load and build them from CompressorMap.mat.
if ~isempty(o.beta) && ~isempty(o.n) && ~isempty(o.flow)
    beta = o.beta(:);  nn = o.n(:);  flow = o.flow(:);
else
    S   = load(fullfile(fileparts(mfilename('fullpath')), '..', 'Allegati', 'CompressorMap.mat'));
    td  = S.tableData;                                 % 9 x 12
    spd = [48000, 54000, 60000, 66000, 72000] * 2*pi/60;
    flow = []; beta = []; nn = [];
    for kk = 1:5
        flow = [flow; td(:,2*kk-1)];  beta = [beta; td(:,2*kk)]; %#ok<AGROW>
        nn   = [nn;   repmat(spd(kk), size(td,1), 1)];           %#ok<AGROW>
    end
end

% surge line: caller-supplied, else from the .mat
if ~isempty(o.m_surge) && ~isempty(o.beta_surge)
    m_surge = o.m_surge(:);  beta_surge = o.beta_surge(:);
else
    if ~exist('td','var')
        td = getfield(load(fullfile(fileparts(mfilename('fullpath')), ...
            '..', 'Allegati', 'CompressorMap.mat')), 'tableData'); %#ok<GFLD>
    end
    m_surge = td(:,11);  beta_surge = td(:,12);
end

% The 72 krpm line has two tabulated points at the same PR (flat top near surge);
% scatteredInterpolant averages such duplicates - benign here, so silence the notice.
ws = warning('off', 'MATLAB:scatteredInterpolant:DupPtsAvValuesWarnId');
mapF = scatteredInterpolant(beta, nn, flow, 'linear', 'none');
warning(ws);

% mapped speed lines and per-speed PR limits, derived from whatever map we have
sp_rad        = unique(nn(:)).';
mapSpeeds_rpm = sp_rad * 60/(2*pi);
n_min_rpm     = min(mapSpeeds_rpm);
n_max_rpm     = max(mapSpeeds_rpm);
betaMin = zeros(size(sp_rad));  betaMax = zeros(size(sp_rad));
for kk = 1:numel(sp_rad)
    m = (nn == sp_rad(kk));
    betaMin(kk) = min(beta(m));  betaMax(kk) = max(beta(m));
end

% per-speed PR limits (interpolated vs rpm) so we never leave the map
prLoOf = @(nrpm) interp1(mapSpeeds_rpm, betaMin, nrpm, 'linear', 'extrap');
prHiOf = @(nrpm) interp1(mapSpeeds_rpm, betaMax, nrpm, 'linear', 'extrap');

% surge flow lookup handle (clamped outside the tabulated range)
surgeFlow = @(PR) surgeFlowLocal(PR, beta_surge, m_surge);

% ----------------------------------------------------------------------------
% 3) Design-point-consistent choke constant  (exactly as the plant uses it)
% ----------------------------------------------------------------------------
% K_choke is fixed turbine hardware. Prefer the value supplied by the caller
% (single source of truth, shared with data_v2's design point); otherwise derive
% it here at the DESIGN inlet temperature (not the operating T1 being scheduled).
if ~isempty(o.K_choke)
    c.K_choke = o.K_choke;
else
    pk = struct('P1',c.P1, 'PR_des',PR_des, 'T3_des',c.T3_des, ...
        'm_a_des',mapF(PR_des, n_des*2*pi/60), 'eta_c',c.eta_c, 'eta_t',c.eta_t, ...
        'lambda',c.lambda, 'eta_comb',c.eta_comb, 'LHV',c.LHV, ...
        'lambda_eg_filter',c.lambda_eg_filter, 'cp_air',c.cp_air, 'cp_eg',c.cp_eg, ...
        'UAc0',c.UAc0, 'UAh0',c.UAh0, 'ma0',c.ma0, 'meg0',c.meg0);
    c.K_choke = computeKchoke(pk, o.T1_design);
end

% ----------------------------------------------------------------------------
% 4) Sweep the (n, PR) grid, solve each steady-state point, apply feasibility
% ----------------------------------------------------------------------------
nMarginLo = n_min_rpm*(1+o.speedMargin);
nMarginHi = n_max_rpm*(1-o.speedMargin);
nvec = linspace(nMarginLo, nMarginHi, o.nGrid);

% storage (feasible points only)
FP = struct('n',[],'PR',[],'P',[],'eta',[],'T3',[],'SM',[],'bt',[],'ma',[]);
% full grid (for plotting speed lines) kept per-n
lines = struct('n',{},'P',{},'eta',{});

for i = 1:o.nGrid
    nrpm = nvec(i);
    nrad = nrpm*2*pi/60;
    PRvec = linspace(max(1.4, prLoOf(nrpm)), prHiOf(nrpm), o.prGrid);
    Pi = nan(1,o.prGrid);  Ei = nan(1,o.prGrid);
    for j = 1:o.prGrid
        PR = PRvec(j);
        r = solvePoint(nrad, PR, c, mapF);
        if ~r.ok, continue; end
        SM = (r.ma - surgeFlow(PR)) / surgeFlow(PR);
        % feasibility masks
        okChoke = r.betaT >= c.beta_crit*(1+o.chokeMargin);
        okSurge = SM      >= o.surgeMargin;
        okTemp  = r.T3    <= c.T3_des;
        okFlame = r.T3    >= c.T3_min;          % flame stability (lean blow-out)
        okFuel  = r.m_f   >  0;
        Pi(j) = r.P_el;  Ei(j) = r.eta_el;      % keep for speed-line plot
        if okChoke && okSurge && okTemp && okFlame && okFuel && isfinite(r.eta_el)
            FP.n(end+1)  = nrpm;   FP.PR(end+1) = PR;
            FP.P(end+1)  = r.P_el; FP.eta(end+1)= r.eta_el;
            FP.T3(end+1) = r.T3;   FP.SM(end+1) = SM;
            FP.bt(end+1) = r.betaT;FP.ma(end+1) = r.ma;
        end
    end
    lines(i).n = nrpm; lines(i).P = Pi; lines(i).eta = Ei; %#ok<AGROW>
end

if isempty(FP.P)
    error('No feasible operating points found - relax the margins.');
end

% ----------------------------------------------------------------------------
% 5) Build the schedule: for each power bin pick the max-efficiency speed
% ----------------------------------------------------------------------------
% Restrict the schedule to the USEFUL generating band. Below this the machine
% barely generates (eta_el ~ 0 or negative) and the efficiency-optimal speed is
% ill-defined, which would otherwise corrupt the monotonicity enforcement below.
Pmax = max(FP.P);
if isempty(o.P_min_kW)
    Pmin = 0.05*Pmax;                 % default: 5% of full power
else
    Pmin = o.P_min_kW*1e3;
end

Pmin = max(Pmin, min(FP.P(FP.eta > 0)));   % never below where eta_el turns positive
Ptar = linspace(Pmin, Pmax, o.nPower);
dP   = (Pmax - Pmin)/(o.nPower-1);

n_ref = nan(size(Ptar)); PR_ref = n_ref; T3_ref = n_ref;
eta_ref = n_ref; SM_ref = n_ref; bt_ref = n_ref;
for m = 1:numel(Ptar)
    win = abs(FP.P - Ptar(m)) <= 0.75*dP;
    if ~any(win), continue; end
    idx = find(win);
    [eta_ref(m), b] = max(FP.eta(idx));
    kbest = idx(b);
    n_ref(m)  = FP.n(kbest);   PR_ref(m) = FP.PR(kbest);
    T3_ref(m) = FP.T3(kbest);  SM_ref(m) = FP.SM(kbest);
    bt_ref(m) = FP.bt(kbest);
end

% drop empty bins, then smooth speed and enforce monotone increase with power
good = ~isnan(n_ref);
Ptar=Ptar(good); n_ref=n_ref(good); PR_ref=PR_ref(good);
T3_ref=T3_ref(good); eta_ref=eta_ref(good); SM_ref=SM_ref(good); bt_ref=bt_ref(good);

n_ref_raw = n_ref;
if numel(n_ref) >= 5
    n_ref = movmean(n_ref, 5);          % de-noise the argmax selection
end
n_ref = cummax(n_ref);                  % speed must not decrease with power

% ----------------------------------------------------------------------------
% 6) Package output
% ----------------------------------------------------------------------------
sched.P_el_W  = Ptar(:);
sched.P_el_kW = Ptar(:)/1e3;
sched.n_ref_rpm = n_ref(:);
sched.n_ref_raw_rpm = n_ref_raw(:);
sched.PR_ref = PR_ref(:);
sched.T3_ref = T3_ref(:);
sched.eta_el_ref = eta_ref(:);
sched.surgeMargin_ref = SM_ref(:);
sched.chokeMargin_ref = (bt_ref(:) - c.beta_crit)/c.beta_crit;
sched.map = FP;
sched.constants = c;
sched.options = o;
% plant solver ingredients (for scheduleIC.m -> flat-start initial conditions)
sched.mapF          = mapF;             % compressor-map interpolant  flow = F(PR, n[rad/s])
sched.mapSpeeds_rpm = mapSpeeds_rpm;    % mapped speed lines [rpm]
sched.betaMin       = betaMin;          % per-speed min PR
sched.betaMax       = betaMax;          % per-speed max PR
% ready-to-paste 1-D lookup (Simulink "1-D Lookup Table"): input P_el [W]
sched.lookup.breakpoints_P_el_W = sched.P_el_W.';
sched.lookup.table_n_ref_rpm    = sched.n_ref_rpm.';

if o.save
    outfile = fullfile(fileparts(mfilename('fullpath')), '..', 'speedSchedule.mat');
    save(outfile, '-struct', 'sched');
    fprintf('\nSchedule saved to %s\n', outfile);
end

% ----------------------------------------------------------------------------
% 7) Plots
% ----------------------------------------------------------------------------
if o.plot
    % (a) efficiency islands: eta_el vs power, one line per speed, + envelope
    figure('Name','Efficiency vs power'); hold on; grid on;
    cmap = parula(o.nGrid);
    for i = 1:o.nGrid
        plot(lines(i).P/1e3, lines(i).eta, '-', 'Color',[cmap(i,:) 0.35]);
    end
    plot(sched.P_el_kW, sched.eta_el_ref, 'k-', 'LineWidth', 2.2);
    scatter(FP.P/1e3, FP.eta, 6, FP.n, 'filled'); % feasible cloud
    cb=colorbar; cb.Label.String='shaft speed [rpm]';
    caxis([nMarginLo nMarginHi]);
    % clamp to the meaningful range: the faint per-speed lines run into
    % low/negative-power points where eta_el = P/(mf*LHV) diverges
    xlim([0, 1.05*max(FP.P)/1e3]);
    ylim([0, 1.15*max(FP.eta)]);
    xlabel('Electrical power [kW]'); ylabel('\eta_{el} [-]');
    title('Electrical efficiency - optimal-speed envelope (black)');

    % (b) the schedule itself
    figure('Name','Speed schedule');
    subplot(3,1,1); hold on; grid on;
    plot(sched.P_el_kW, sched.n_ref_rpm, 'b-', 'LineWidth', 2);
    yline(n_min_rpm,'r--'); yline(n_max_rpm,'r--');
    ylabel('n_{ref} [rpm]'); title('Optimal speed schedule n_{ref}(P_{el})');
    subplot(3,1,2); hold on; grid on;
    plot(sched.P_el_kW, sched.PR_ref, 'b-','LineWidth',1.5);
    ylabel('PR [-]');
    yyaxis right; plot(sched.P_el_kW, sched.T3_ref-273.15, 'r-','LineWidth',1.5);
    yline(c.T3_max-273.15,'r:'); yline(c.T3_min-273.15,'r:'); ylabel('T3 [\circC]');
    subplot(3,1,3); hold on; grid on;
    plot(sched.P_el_kW, sched.surgeMargin_ref*100, 'b-','LineWidth',1.5);
    yline(o.surgeMargin*100,'b:');
    ylabel('surge margin [%]');
    yyaxis right; plot(sched.P_el_kW, sched.eta_el_ref*100,'m-','LineWidth',1.5);
    ylabel('\eta_{el} [%]'); xlabel('Electrical power [kW]');

    % (c) operating points on the compressor map
    figure('Name','Operating points on map'); hold on; grid on;
    for kk = 1:numel(sp_rad)                       % speed lines, from the map vectors
        m = (nn == sp_rad(kk));
        [fm, ord] = sort(flow(m));  bm = beta(m);
        plot(fm, bm(ord), '-', 'Color',[.7 .7 .7]);
    end
    plot(m_surge, beta_surge, 'k--','LineWidth',1.2);
    scatter(FP.ma, FP.PR, 8, FP.eta, 'filled');
    cb=colorbar; cb.Label.String='\eta_{el}';
    xlabel('Corrected mass flow [kg/s]'); ylabel('Pressure ratio');
    title('Feasible operating cloud (colour = \eta_{el})');
end

end % ===================== main function =====================================


% solvePoint (steady-state point solver) and its fuelFor helper now live in
% solvePoint.m, shared with scheduleIC.m.

% ============================================================================
%  Compressor surge flow at a given PR (clamped outside the tabulated range)
% ============================================================================
function ms = surgeFlowLocal(PR, beta_surge, m_surge)
if PR <= beta_surge(1)
    ms = m_surge(1);
elseif PR >= beta_surge(end)
    ms = m_surge(end);
else
    ms = interp1(beta_surge, m_surge, PR, 'linear');
end
end
