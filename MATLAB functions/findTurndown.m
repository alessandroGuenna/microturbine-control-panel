function [T1_C, Pmin_kW, fitPmin] = findTurndown(mdl, T1_list_C, K_choke, schedOpts)
%FINDTURNDOWN  Dynamic minimum-load (turndown) vs ambient temperature (parallel).
%   For each ambient in T1_list_C [deg C], scans a grid of constant electrical
%   loads and finds the lowest one the closed-loop model holds without running
%   away (the choke-driven turndown the static schedule cannot predict). Every
%   (ambient, load) probe is an independent simulation, so they are all run in
%   parallel with parsim. Returns the turndown per ambient and a linear fit
%   Pmin[kW] = f(T1[deg C]).
%
%   [T1_C, Pmin_kW, fitPmin] = findTurndown(mdl, T1_list_C, K_choke, schedOpts)
%
%     mdl        model name, e.g. 'microturbine_model_v2'
%     T1_list_C  ambient temperatures to test [deg C], e.g. -10:5:35
%     K_choke    fixed turbine constant (from computeKchoke)
%     schedOpts  the SAME computeSpeedSchedule options used for the run schedule
%                in data_v2, WITHOUT 'K_choke'/'T1' (map vectors, 'T3_min', ...)
%
%   Requires the Parallel Computing Toolbox. The shaft speed must be logged to
%   logsout under the name 'n_shaft'. Each probe seeds its own dp/sched/T1 via
%   Simulink.SimulationInput (no base-workspace sharing), and disables the
%   min-load clamp (P_el_min = 0) so loads below it can be probed.

    load_system(mdl);
    set_param(mdl,'FastRestart','off');            % parsim manages the workers itself
    T1_list_C = T1_list_C(:);

    % ---- build the full grid of probes as SimulationInput objects ----
    nLoads = 12;                                   % load resolution per ambient
    in  = Simulink.SimulationInput.empty;
    tgt = zeros(0,2);                              % [ambient index, load W] per probe
    for i = 1:numel(T1_list_C)
        T1    = T1_list_C(i) + 273.15;
        sched = computeSpeedSchedule('K_choke',K_choke,'T1',T1, schedOpts{:});
        loads = linspace(sched.P_el_W(1), 0.65*max(sched.P_el_W), nLoads);
        for P0 = loads
            in(end+1)    = buildProbe(mdl, T1, P0, sched);   %#ok<AGROW>
            tgt(end+1,:) = [i, P0];                          %#ok<AGROW>
        end
    end

    % ---- run every probe in parallel ----
    % parsim warns "one or more simulations completed with errors". Here that is
    % EXPECTED: sub-turndown probes run away and trip the speed assertion, which is
    % exactly how instability is detected. Silence that one summary warning (it is
    % the only warning thrown on the client) and print a clear note instead.
    wsave = warning('off','all');
    out   = parsim(in, 'ShowProgress','on', 'StopOnError','off', ...
                       'TransferBaseWorkspaceVariables','on');
    warning(wsave);

    nErr = nnz(arrayfun(@(s) ~isempty(s.ErrorMessage), out));
    fprintf(['\n[findTurndown] %d of %d probes ran away and errored -- this is ' ...
             'EXPECTED:\n   loads below the turndown trip the speed assertion; ' ...
             'that is how the unstable region is detected.\n\n'], nErr, numel(out));

    % ---- reduce: lowest stable load per ambient ----
    Pmin_kW = nan(size(T1_list_C));
    for k = 1:numel(out)
        if isStableOut(out(k))
            i = tgt(k,1);
            Pmin_kW(i) = min([Pmin_kW(i); tgt(k,2)/1e3]);   % track lowest stable load
        end
    end

    % report + fit
    for i = 1:numel(T1_list_C)
        if isnan(Pmin_kW(i))
            warning('findTurndown:noStablePoint', ...
                'No stable load found on the grid at T1=%+.1f C.', T1_list_C(i));
        else
            fprintf('T1=%+5.1f C  ->  turndown = %.1f kW\n', T1_list_C(i), Pmin_kW(i));
        end
    end

    T1_C = T1_list_C;
    ok   = ~isnan(Pmin_kW);
    fitPmin = polyfit(T1_C(ok), Pmin_kW(ok), 1);   % kW vs deg C  (mirror of fit_Pmax)
end


% ============================================================================
%  Build one probe's SimulationInput: seed dp/sched/T1 as per-sim variables
%  (self-contained, so parallel workers need no shared base workspace).
% ============================================================================
function in = buildProbe(mdl, T1, P0, sched)
    c = sched.constants;
    [n_des, PR_des, T3_des, ic] = scheduleIC(sched, P0);

    dp = designPoint(c.P1, T1, PR_des, c.eta_c, c.eta_t, ic.ma, c.lambda, ...
        c.K_choke, c.eta_comb, c.LHV, T3_des, c.lambda_eg_filter, ...
        c.cp_air, c.cp_eg, n_des, c.eta_mech, c.eta_gen, c.J, ...
        c.UAc0, c.UAh0, c.ma0, c.meg0);
    dp.turbine.K_choke  = c.K_choke;
    dp.recuperator.ma0  = c.ma0;   dp.recuperator.meg0 = c.meg0;
    dp.recuperator.UAc0 = c.UAc0;  dp.recuperator.UAh0 = c.UAh0;
    dp.recuperator.Cm   = c.Cm;

    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('dp',       dp);
    in = in.setVariable('sched',    sched);
    in = in.setVariable('T1',       T1);
    in = in.setVariable('P_el_min', 0);            % min-load clamp OFF during calibration
    in = in.setModelParameter('StopTime','12', 'EnablePacing','off');
end


% ============================================================================
%  Stability verdict from a parsim SimulationOutput.
% ============================================================================
function ok = isStableOut(so)
    if ~isempty(so.ErrorMessage)                   % assertion trip / solver failure
        ok = false;  return                        % -> ran away = unstable
    end
    ns = so.logsout.get('n_shaft').Values.Data;    % shaft speed [rpm]
    ok = all(ns > 48500) && all(ns < 71500) && (max(ns) - min(ns) < 3000);
end
