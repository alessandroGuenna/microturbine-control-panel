function [n_des, PR_des, T3_des, ic] = scheduleIC(sched, P0)
%SCHEDULEIC  Flat-start design / initial-condition point at electrical load P0.
%   Returns the operating point the closed loop settles to at load P0, so that
%   seeding the integrators from it produces no startup transient:
%       n_des  = n_ref(P0)   -- exactly the speed the Simulink governor holds
%       PR_des, T3_des       -- pressure ratio / turbine-inlet temperature such
%                               that the plant produces EXACTLY P0 at that speed
%
%   [n_des,PR_des,T3_des]    = scheduleIC(sched, P0)
%   [n_des,PR_des,T3_des,ic] = scheduleIC(sched, P0)   % ic = full solvePoint state
%
%   sched is the struct returned by computeSpeedSchedule.m (uses .mapF,
%   .constants, .P_el_W, .n_ref_rpm, .PR_ref). P0 is the electrical load [W].
%
%   Method: the governor fixes the speed (n_ref(P0)); the only remaining degree
%   of freedom is PR, solved so the plant power equals P0 (1-D root find on the
%   shared solvePoint). This inverts designPoint's (n,PR,T3)->power mapping.

c    = sched.constants;
mapF = sched.mapF;

% (1) governor speed at this load  (same smoothed lookup the Simulink block uses)
P0c   = min(max(P0, sched.P_el_W(1)), sched.P_el_W(end));   % clip like the lookup
n_des = interp1(sched.P_el_W, sched.n_ref_rpm, P0c, 'linear');
nrad  = n_des*2*pi/60;

% (2) pressure ratio so the plant produces exactly P0 at that fixed speed.
% Seed the search at the schedule's operating PR (feasible, near the root);
% power is monotonic in PR, so fzero converges without reaching the surge edge.
PR0 = interp1(sched.P_el_W, sched.PR_ref, P0c, 'linear');
f   = @(PR) powerErr(PR, nrad, P0, c, mapF);
try
    PR_des = fzero(f, PR0, optimset('Display','off','TolX',1e-10));
catch ME
    error('scheduleIC:solveFailed', ...
        'Could not solve PR for P0 = %.3f kW at n_ref = %.0f rpm (%s). Is P0 in the schedule band [%.2f, %.2f] kW?', ...
        P0/1e3, n_des, ME.message, sched.P_el_W(1)/1e3, sched.P_el_W(end)/1e3);
end
if ~isfinite(PR_des)
    error('scheduleIC:solveFailed', ...
        'No PR produces P0 = %.3f kW at n_ref = %.0f rpm.', P0/1e3, n_des);
end

% (3) recover the consistent steady state at (n_des, PR_des)
ic     = solvePoint(nrad, PR_des, c, mapF);
T3_des = ic.T3;
end

% ----------------------------------------------------------------------------
function e = powerErr(PR, nrad, P0, c, mapF)
r = solvePoint(nrad, PR, c, mapF);
if r.ok
    e = r.P_el - P0;
else
    e = 1e12;                 % infeasible (surge-side / off-map): "power too high"
end
end
