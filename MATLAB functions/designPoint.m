function dp = designPoint(P1, T1, PR, eta_c, eta_t, mdot_x, ...
    lambda_total, K_choke, ...
    eta_comb, LHV, T3_target, lambda_eg_filter, cp_air, cp_eg, n_des, ...
    eta_mech, eta_gen, J, UAc0, UAh0, ma0, meg0)

%DESIGNPOINT  Steady thermodynamic cycle at a specified (PR, mdot, T3) design.
%   Uses the SHARED recuperator engine recupSteady5.m (the same 5-node flow-
%   scaled model as the plant and computeSpeedSchedule.m) so the design point
%   and the optimal-speed schedule are mutually consistent: designPoint fixes
%   T3 and derives K_choke; computeSpeedSchedule fixes K_choke and derives T3;
%   with the shared recuperator they describe the identical operating point.

if nargin < 15 || isempty(n_des), n_des = NaN; end
if nargin < 19 || isempty(UAc0),  UAc0 = 2571;                end
if nargin < 20 || isempty(UAh0),  UAh0 = 2571;                end
if nargin < 21 || isempty(ma0),   ma0  = 0.440386860108652;   end
if nargin < 22 || isempty(meg0),  meg0 = 0.445815110936435;   end

P_amb = 101325;
k_air = 1.4;
N     = 5;

%% ---- compressor outlet state (isentropic + efficiency) ----
T2s    = T1 * PR^((k_air-1)/k_air);
T2_des = T1 + (T2s - T1)/eta_c;

%% ---- P2' (plenum design pressure) and combined duct+REC+CC pressure drop ----
P2p    = PR * P1;
P3_des = P2p * (1 - lambda_total*mdot_x^2);

%% ---- iterative solve: P4' <-> mdot_eg  (with the shared 5-node recuperator) ----
beta_crit = ((k_air+1)/2)^(k_air/(k_air-1));
n_exp     = (k_air-1)/k_air;

mdot_eg  = mdot_x;
tol      = 1e-10;
max_iter = 200;

T4_des = T2_des;
T2_sec_des = T2_des;
T5_des = T2_des;
Tm0 = zeros(N,1);

for iter = 1:max_iter
    P4_des = P_amb + lambda_eg_filter * mdot_eg^2;
    beta   = P3_des / P4_des;
    if beta < beta_crit
        warning('designPoint:notChoked', ...
            'beta=%.3f is below beta_crit=%.3f -- choking assumption NOT valid.', ...
            beta, beta_crit);
    end
    T4_des = T3_target - eta_t*T3_target*(1 - 1/(beta^n_exp));

    [T2_sec_des, T5_des, Tm0] = recupSteady5(mdot_x, mdot_eg, T2_des, T4_des, ...
        UAc0, UAh0, ma0, meg0, cp_air, cp_eg, N);

    m_f_des = mdot_x*(cp_eg*T3_target - cp_air*T2_sec_des) / ...
              (LHV*eta_comb - cp_eg*T3_target);

    mdot_eg_new = mdot_x + m_f_des;
    
    if abs(mdot_eg_new - mdot_eg) < tol
        mdot_eg = mdot_eg_new;
        break;
    end
    mdot_eg = mdot_eg_new;
end

%% ==================== assemble output struct ====================
dp.stations(1) = makeStation('1',  'Compressor inlet (post-filter)', T1,        P1,     mdot_x,  cp_air);
dp.stations(2) = makeStation('2p', 'Compressor outlet / plenum',     T2_des,    P2p,    mdot_x,  cp_air);
dp.stations(3) = makeStation('3',  'CC outlet / turbine inlet',      T3_target, P3_des, mdot_eg, cp_eg);
dp.stations(4) = makeStation('4p', 'Turbine outlet / REC hot inlet', T4_des,    P4_des, mdot_eg, cp_eg);

%% ---- recuperator: boundary temperatures + metal profile (5-node) ----
dp.recuperator.T_air_in  = T2_des;       % cold side inlet (= compressor outlet)
dp.recuperator.T_air_out = T2_sec_des;   % cold side outlet (-> CC inlet)
dp.recuperator.T_eg_in   = T4_des;       % hot side inlet (= turbine outlet)
dp.recuperator.T_eg_out  = T5_des;       % hot side outlet (-> exhaust)
dp.recuperator.Tm0       = Tm0;          % metal node temperatures (steady)

%% ---- compressor: PR and power consumed ----
P_compressor = mdot_x * cp_air * (T2_des - T1);
dp.compressor.PR = PR;
dp.compressor.P_consumed = P_compressor;

%% ---- turbine: expansion ratio, power produced, K_choke ----
P_turbine = mdot_eg * cp_eg * (T3_target - T4_des);
dp.turbine.PR = beta;
dp.turbine.P_produced = P_turbine;
dp.turbine.K_choke = K_choke;

%% ---- power: net shaft, electrical, electrical efficiency ----
P_net_shaft = P_turbine - P_compressor;
P_el   = P_net_shaft * eta_mech * eta_gen;
eta_el = P_el / (m_f_des * LHV);
dp.power.P_net_shaft = P_net_shaft;
dp.power.P_el = P_el;
dp.power.eta_el = eta_el;

%% ---- rpm / kinetic energy ----
dp.shaft.rpm   = n_des;
dp.shaft.kinEn = 0.5 * J * (2*pi/60 * n_des)^2;

%% ---- fuel flow rate ----
dp.m_fuel = m_f_des;

%% ---- fluids: specific heats only ----
dp.fluids.cp_air = cp_air;
dp.fluids.cp_eg  = cp_eg;

end

%% ---------------------------------------------------------------------
function st = makeStation(id, description, T, P, mdot, cp)
    Tref = 273.15;
    Pref = 101325;
    k = 1.4;
    R = cp * (k-1)/k;

    st.id = id;
    st.description = description;
    st.T = T;
    st.P = P;
    st.h = cp * T;
    st.s = cp*log(T/Tref) - R*log(P/Pref);
    st.mdot = mdot;

    if abs(cp - 1005) < abs(cp - 1100)
        st.fluid = 'air';
    else
        st.fluid = 'exhaust_gas';
    end
end
