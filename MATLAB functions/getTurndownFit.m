function fitPmin = getTurndownFit(mdl, T1_list_C, K_choke, schedOpts, cacheFile)
%GETTURNDOWNFIT  Cached dynamic-turndown fit vs ambient temperature.
%   Loads the turndown fit from disk if the cache file exists; otherwise runs
%   the one-time closed-loop calibration (findTurndown), saves it, and returns
%   it. fitPmin gives the minimum stable electrical load [kW] as a function of
%   ambient temperature [deg C].
%
%   fitPmin = getTurndownFit(mdl, T1_list_C, K_choke, schedOpts)
%   fitPmin = getTurndownFit(mdl, T1_list_C, K_choke, schedOpts, cacheFile)
%
%     mdl        model name, e.g. 'microturbine_model_v2'
%     T1_list_C  ambient temperatures to calibrate at [deg C], e.g. -10:5:35
%     K_choke    fixed turbine constant (from computeKchoke)
%     schedOpts  the SAME computeSpeedSchedule options used for the run schedule
%                in data_v2, WITHOUT 'K_choke'/'T1' (map vectors, 'T3_min', ...)
%     cacheFile  .mat path (default: ../Allegati/turndownMap.mat)

    if nargin < 5 || isempty(cacheFile)
        cacheFile = fullfile(fileparts(mfilename('fullpath')), '..', ...
                             'Allegati', 'turndownMap.mat');
    end

    if isfile(cacheFile)
        S = load(cacheFile, 'fitPmin');
        fitPmin = S.fitPmin;
        fprintf('\nTurndown map: using cache %s\n', cacheFile);
        return
    end

    fprintf('\nTurndown map: not found -> recalculating (slow)...\n');
    [T1_C, Pmin_kW, fitPmin] = findTurndown(mdl, T1_list_C, K_choke, schedOpts); %#ok<ASGLU>
    save(cacheFile, 'fitPmin', 'T1_C', 'Pmin_kW');
    fprintf('\nTurndown map: saved to %s\n', cacheFile);
end
