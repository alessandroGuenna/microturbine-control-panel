function K_choke = computeKchoke(p, T1)
%COMPUTEKCHOKE  Fixed turbine choke-flow constant of the machine.
%   K_choke = m_eg*sqrt(T3)/P3 at the nominal design point. Self-contained:
%   it computes ONLY what is needed for K_choke (P3 and the exhaust-gas flow
%   m_eg), NOT the full cycle. Evaluate ONCE and treat the result as a FIXED
%   HARDWARE CONSTANT (turbine throat).
%
%   K_choke = computeKchoke(p)        at the default design ambient T1 = 15 C.
%   K_choke = computeKchoke(p, T1)    at a different inlet temperature [K].
%
%   p is a struct with the nominal design data / physical constants:
%       p.P1, p.PR_des, p.T3_des, p.m_a_des          (design air flow at PR_des,n_des)
%       p.eta_c, p.eta_t, p.lambda, p.eta_comb, p.LHV, p.lambda_eg_filter,
%       p.cp_air, p.cp_eg, p.UAc0, p.UAh0, p.ma0, p.meg0
%
%   The recuperator is the shared 5-node engine (recupSteady5.m), so K_choke is
%   consistent with designPoint.m and the schedule without running either.

if nargin < 2 || isempty(T1)
    T1 = 15 + 273.15;                 % default nominal design ambient
end

P_amb = 101325;
k_air = 1.4;
N     = 5;
n_exp = (k_air-1)/k_air;

ma  = p.m_a_des;
T3  = p.T3_des;

% compressor outlet temperature (isentropic + efficiency) and turbine-inlet pressure
T2  = T1 * (1 + (p.PR_des^n_exp - 1)/p.eta_c);
P3  = p.PR_des * p.P1 * (1 - p.lambda*ma^2);

% iterate exhaust-gas flow: P4 <-> m_eg, closing through the CC energy balance
m_eg = ma;
for it = 1:200
    P4   = P_amb + p.lambda_eg_filter * m_eg^2;
    beta = P3 / P4;
    T4   = T3 - p.eta_t*T3*(1 - 1/beta^n_exp);

    T2pp = recupSteady5(ma, m_eg, T2, T4, p.UAc0, p.UAh0, p.ma0, p.meg0, ...
                        p.cp_air, p.cp_eg, N);

    m_f  = ma*(p.cp_eg*T3 - p.cp_air*T2pp) / (p.LHV*p.eta_comb - p.cp_eg*T3);

    m_eg_new = ma + m_f;
    if abs(m_eg_new - m_eg) < 1e-12
        m_eg = m_eg_new;
        break;
    end
    m_eg = m_eg_new;
end

K_choke = m_eg * sqrt(T3) / P3;
end
