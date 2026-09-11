function r = solvePoint(nrad, PR, c, mapF)
%SOLVEPOINT  Steady-state operating point at shaft speed n [rad/s] and PR.
%   Shared plant-physics solver used by computeSpeedSchedule.m (the schedule
%   sweep) and scheduleIC.m (the flat-start initial conditions). Fixes K_choke,
%   derives T3 from the choke relation, and closes the CC energy balance through
%   the shared 5-node recuperator (recupSteady5.m).
%
%   c is the constants struct (as built in computeSpeedSchedule / stored in
%   sched.constants); mapF is the compressor-map interpolant flow = F(PR, n).
%
%   Returns struct r with fields: ok, ma, m_f, T3, betaT, P_el, eta_el.

r.ok = false;
r.P_el = NaN; r.eta_el = NaN; r.T3 = NaN; r.betaT = NaN; r.ma = NaN; r.m_f = NaN;

% --- compressor side (fully determined by n, PR) ---
ma = mapF(PR, nrad);
if ~isfinite(ma) || ma <= 0, return; end       % outside the map
T2  = c.T1*(1 + (PR^((c.k-1)/c.k) - 1)/c.eta_c);
P2p = PR*c.P1;
P3  = P2p*(1 - c.lambda*ma^2);
if P3 <= 0, return; end
Pcomp = ma*c.cp_air*(T2 - c.T1);

% --- solve the exhaust-gas mass flow M_eg (choke + CC + recuperator loop) ---
% residual: M_eg must equal ma + m_fuel needed to sustain the choke-set T3
res = @(Meg) (ma + fuelFor(Meg, ma, P3, T2, c)) - Meg;

lo = ma*(1+1e-5);  hi = ma*1.30;
flo = res(lo);  fhi = res(hi);
if ~(isfinite(flo) && isfinite(fhi)) , return; end
if sign(flo) == sign(fhi)
    % try to widen once
    hi = ma*1.6; fhi = res(hi);
    if sign(flo) == sign(fhi), return; end
end
try
    Meg = fzero(res, [lo hi], optimset('Display','off','TolX',1e-9));
catch
    return;
end
if ~isfinite(Meg) || Meg <= ma, return; end

% --- recover the converged state ---
T3 = (c.K_choke*P3/Meg)^2;
P4 = c.P_amb + c.lambda_eg_filter*Meg^2;
bt = P3/P4;
T4 = T3*(1 - c.eta_t*(1 - bt^(-(c.k-1)/c.k)));
m_f = Meg - ma;

Pturb = Meg*c.cp_eg*(T3 - T4);
P_el  = c.eta_mech*c.eta_gen*(Pturb - Pcomp);

r.ok    = true;
r.ma    = ma;
r.m_f   = m_f;
r.T3    = T3;
r.betaT = bt;
r.P_el  = P_el;
r.eta_el= P_el/(m_f*c.LHV);
end


% ============================================================================
%  Fuel needed at a trial exhaust-gas flow M_eg (choke-set T3 + CC balance)
% ============================================================================
function mf = fuelFor(Meg, ma, P3, T2, c)
T3 = (c.K_choke*P3/Meg)^2;                      % choked turbine sets T3
P4 = c.P_amb + c.lambda_eg_filter*Meg^2;        % exhaust back-pressure
bt = P3/P4;
T4 = T3*(1 - c.eta_t*(1 - bt^(-(c.k-1)/c.k)));  % turbine expansion
T2pp = recupSteady5(ma, Meg, T2, T4, c.UAc0, c.UAh0, c.ma0, c.meg0, ...
                    c.cp_air, c.cp_eg, c.Nrec);  % shared REC engine -> CC inlet temp
mf = ma*(c.cp_eg*T3 - c.cp_air*T2pp) / ...       % CC energy balance
     (c.LHV*c.eta_comb - c.cp_eg*T3);
end
