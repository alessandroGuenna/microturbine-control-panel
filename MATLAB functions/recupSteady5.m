function [T2pp, T5, Tm] = recupSteady5(m_a, m_eg, T2, T4, UAc0, UAh0, ma0, meg0, cpa, cpe, N)
%RECUPSTEADY5  Steady state of the 5-node counterflow recuperator with metal.
%   Shared physics engine used by BOTH designPoint.m and computeSpeedSchedule.m
%   so the design point and the optimal-speed schedule describe the same plant.
%   Identical per-node effectiveness + Dittus-Boelter UA scaling as the Simulink
%   model (recCold / recHot charts). Node 1 = cold-inlet end, gas flows N -> 1.
%
%   Inputs
%       m_a, m_eg   cold- and hot-side mass flows            [kg/s]
%       T2, T4      cold-inlet (compressor out) / hot-inlet (turbine out) temps [K]
%       UAc0,UAh0   design-flow conductances                 [W/K]
%       ma0, meg0   reference (design) flows for UA scaling   [kg/s]
%       cpa, cpe    cold-/hot-side specific heats             [J/kgK]
%       N           number of nodes (5)
%   Outputs
%       T2pp        cold-side outlet temperature (-> CC inlet)   [K]
%       T5          hot-side  outlet temperature (-> HRB)        [K]
%       Tm          N-vector of metal node temperatures          [K]

Cc = m_a  * cpa;
Ch = m_eg * cpe;
UAc = UAc0 * (max(m_a ,1e-3)/ma0 )^0.8;      % h ~ mdot^0.8 (Dittus-Boelter)
UAh = UAh0 * (max(m_eg,1e-3)/meg0)^0.8;
ec = exp(-(UAc/N)/Cc);
eh = exp(-(UAh/N)/Ch);

% node residual r(Tm) = Qh(i) - Qc(i) is AFFINE in Tm -> solve the linear system
r0 = nodeRes(zeros(N,1), T2, T4, Cc, Ch, ec, eh, N);
A  = zeros(N);
for j = 1:N
    e = zeros(N,1); e(j) = 1;
    A(:,j) = nodeRes(e, T2, T4, Cc, Ch, ec, eh, N) - r0;
end
Tm = -A\r0;

% sweep the streams with the solved metal temps to get the outlets
Tc = T2;
for i = 1:N,     Tc = Tm(i) + (Tc - Tm(i))*ec; end
T2pp = Tc;
Th = T4;
for i = N:-1:1,  Th = Tm(i) + (Th - Tm(i))*eh; end
T5 = Th;
end

function rr = nodeRes(Tm, T2, T4, Cc, Ch, ec, eh, N)
Qc = zeros(N,1);  Qh = zeros(N,1);
t = T2;
for i = 1:N
    o = Tm(i) + (t - Tm(i))*ec;  Qc(i) = Cc*(o - t);  t = o;
end
t = T4;
for i = N:-1:1
    o = Tm(i) + (t - Tm(i))*eh;  Qh(i) = Ch*(t - o);  t = o;
end
rr = Qh - Qc;
end
