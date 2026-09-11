clearvars -except T1_app appCompAxes

addpath('MATLAB functions\')
addpath('Allegati\')

% Data

if exist('T1_app','var')
    T1 = T1_app + 273.15;
else
    T1 = 15 + 273.15;
end

P_amb = 101325; % Pa
filter_P = 1e3; % Filter P loss in Pa
P1 = P_amb - filter_P;
eta_c = 0.81; % reference compressor efficiency
eta_t = 0.88; % reference turbine efficiency
k_air = 1.4; % Air
V = 0.025; % Mass accumulation volume in m^3
c_p_air = 1005; % J/kg K
c_p_eg = 1100; % Approximately, actually depends on air-to-fuel ratio
R_air = c_p_air * (k_air-1)/k_air;  % J/kg K, universal gas constant
R_eg  = c_p_eg  * (k_air-1)/k_air;
lambda = 0.313;
eps_REC = 0.75; % REC effectivness (constant)
eta_comb = 0.99;
LHV = 47e6; % J/kg
lambda_eg_filter = 15e3;
eta_mech = 0.97;
eta_gen = 0.98;
J = 3e-3; % kg m^2 Momentum of inertia

% Heat Recovery Boiler
c_p_w = 4184; % J/kg K
eps_HRB = 0.75;
T_cold_water = 15; % °C
hot_water_setpoint = 60; % °C
M_buffer = 20; % kg

% % Compressor map and lookup table
c_map = fullfile('Allegati', 'CompressorMap.mat');
DataCompMap = load(c_map);

speeds = [48000, 54000, 60000, 66000, 72000] * 2 * pi / 60; % rad/s

if exist('appCompAxes','var') && isgraphics(appCompAxes)
    drawCompressorMap(DataCompMap, speeds, appCompAxes);   % into the app
else
    drawCompressorMap(DataCompMap, speeds);                % standalone: own figure
end

%% ===== Design point & optimal-speed schedule =====
% initial guess of the design conditions
PR_des = 4;  n_des = 65000;  T3_des = 950 + 273.15;

% recuperator hardware constants
ma0  = 0.440386860108652;  meg0 = 0.445815110936435;
UAc0 = 2571;               UAh0 = 2571;

% Getting design air flow rate with compressor maps
% (PR_des, n_des) -> m_a_des
out = sim('designAirFlowRate');
m_a_des = out.yout{1}.Values.Data(end);
clear out

% bundle nominal design data + constants for computeKchoke
p = struct('P1',P1,'PR_des',PR_des,'n_des',n_des,'T3_des',T3_des,'m_a_des',m_a_des, ...
    'eta_c',eta_c,'eta_t',eta_t,'lambda',lambda,'eta_comb',eta_comb,'LHV',LHV, ...
    'lambda_eg_filter',lambda_eg_filter,'cp_air',c_p_air,'cp_eg',c_p_eg, ...
    'eta_mech',eta_mech,'eta_gen',eta_gen,'J',J,'UAc0',UAc0,'UAh0',UAh0,'ma0',ma0,'meg0',meg0);

% (1) FIXED hardware constant: K_choke at nominal 15 C -- never recomputed
K_choke = computeKchoke(p);   % you can specify (p, T1) for a different design ambient T

% handle for the design cycle at the operating T1
dpAt = @(PR,nn,T3,ma) designPoint(P1, T1, PR, eta_c, eta_t, ma, lambda, ...
    K_choke, eta_comb, LHV, T3, lambda_eg_filter, c_p_air, c_p_eg, nn, ...
    eta_mech, eta_gen, J, UAc0, UAh0, ma0, meg0);

fprintf("\nDesign point calculation (first guess)...........")

% (2) design cycle at the initial guess -> electrical power
dp = dpAt(PR_des, n_des, T3_des, m_a_des);

% Re-iterate to get the precise initial values
% using optimal speed schedule
% optimal-speed schedule built with the FIXED K_choke
T3_min = 650+273.15;

% schedule options shared by the run schedule AND the turndown calibration
schedOpts = {'T3_min', T3_min, 'plot', false, 'save', false, ...
    'beta', beta, 'n', n, 'flow', flow, 'm_surge', m_surge, 'beta_surge', beta_surge};

fprintf("\nCreating optimal speed schedule..........")
sched = computeSpeedSchedule('K_choke', K_choke, 'T1', T1, schedOpts{:});

P_el_max = 0.95 * max(sched.P_el_W);

% Initial condition
P0 = dp.power.P_el;

% (3) flat-start design point: governor speed at the load, PR solved for that load
[n_des, PR_des, T3_des, ic] = scheduleIC(sched, P0);
m_a_des = ic.ma;

fprintf("\nDesign point calculation (actual design conditions)...........")

% (4) actual steady-state cycle at that point (dpAt stamps the fixed K_choke)
dp = dpAt(PR_des, n_des, T3_des, m_a_des);

P_start = dp.power.P_el; % for the app

% T-S diagram of the thermodynamic cycle
if ~exist('appCompAxes','var')
    plotCycle(dp)     % only show the cycle window when run standalone
end

% Maximum fuel flow rate and max allowed T4p
T3_max = 960+273.15;

% Heat Recovery Boiler
m_w_des = eps_HRB * dp.stations(4).mdot * c_p_eg * ...
    ( (dp.recuperator.T_eg_out - 273.15) - T_cold_water ) / ...
    ( c_p_w * (hot_water_setpoint - T_cold_water) );

% Recuparot constant parameters
% the mass flow rates are obtained with designPoint.m at T1 = 15°C
dp.recuperator.ma0 = 0.440386860108652;  % kg/s
dp.recuperator.meg0 = 0.445815110936435; % kg/s
dp.recuperator.UAc0 = 2571;              % W/K, cold-side conductance at design flow
dp.recuperator.UAh0 = 2571;              % W/K, hot-side  conductance at design flow
dp.recuperator.Cm   = 25e3;              % J/K, metal capacity


% Limits the buffer of the scopes
mdl = 'microturbine_model_v2';
load_system(mdl);

scopes = find_system(mdl, 'MatchFilter', @Simulink.match.allVariants, ...
                     'LookUnderMasks','on', 'FollowLinks','on', ...
                     'BlockType', 'Scope');
for i = 1:numel(scopes)
    set_param(scopes{i}, 'Decimation', '50');
end

fprintf("\nLooking for the minimum deliverable power......\n")

%% ===== dynamic-turndown min-load clamp (cached across ambient) =====
% First run at a new plant builds turndownMap.mat (slow, closed-loop sweep);
% afterwards it just loads the fit. Delete Allegati\turndownMap.mat to rebuild.
T1_run = T1;  dp_run = dp;  sched_run = sched;   % findTurndown pollutes these
P_el_min = 0;                                    % clamp OFF during calibration (must probe low loads)
warning('off','MT:turbineNotChoked');

fitPmin = getTurndownFit(mdl, -10:9:35, K_choke, schedOpts);
T1 = T1_run;  dp = dp_run;  sched = sched_run;   % restore the real run point
clear sched_run schedOpts dp_run;

warning('on','MT:turbineNotChoked');

% minimum stable load at THIS ambient, +10% safety over the fitted turndown
P_el_min = 1.1 * polyval(fitPmin, T1 - 273.15) * 1e3;   % [W]

fprintf("\nThe model was initialized succesfully.\n")