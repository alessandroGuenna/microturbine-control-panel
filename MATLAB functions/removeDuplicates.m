function [beta_clean, n_clean, flow_clean] = removeDuplicates(beta, n, flow, tol_beta, tol_n)
% cleanScatteredData  Remove near-duplicate points from scattered lookup table data.
%
%   [beta_clean, n_clean, flow_clean] = cleanScatteredData(beta, n, flow)
%   [beta_clean, n_clean, flow_clean] = cleanScatteredData(beta, n, flow, tol_beta, tol_n)
%
%   Inputs:
%       beta     - Nx1 vector, coordinate 1
%       n        - Nx1 vector, coordinate 2
%       flow     - Nx1 vector, function values
%       tol_beta - (optional) absolute tolerance for beta differences, default 1e-10
%       tol_n    - (optional) absolute tolerance for n differences,    default 1e-3
%
%   Outputs:
%       beta_clean - cleaned coordinate 1 vector
%       n_clean    - cleaned coordinate 2 vector
%       flow_clean - cleaned function values
%       keep       - logical mask indicating which original points were kept
%
%   The function keeps the first occurrence of each point and discards all
%   later points that lie within the given tolerances in both coordinates.

    % Set default tolerances if not provided
    if nargin < 4, tol_beta = 1e-10; end
    if nargin < 5, tol_n    = 1e-3;  end

    % Validate input lengths
    if ~(length(beta)==length(n) && length(n)==length(flow))
        error('All input vectors must have the same length.');
    end

    N = length(beta);
    keep = true(N, 1);          % initially keep all points

    % Compare each point with all later points
    for i = 1:N
        if keep(i)
            for j = i+1:N
                if keep(j)
                    % Check if point j is within tolerances of point i
                    if abs(beta(i) - beta(j)) <= tol_beta && ...
                       abs(n(i)    - n(j))    <= tol_n
                        keep(j) = false;      % mark as duplicate
                    end
                end
            end
        end
    end

    % Apply mask to produce cleaned vectors
    beta_clean = beta(keep);
    n_clean    = n(keep);
    flow_clean = flow(keep);

    % Report results (optional, can be commented out)
    fprintf('\n--------------------------------------------------')
    fprintf('\nRemoving duplicates from compressor map.........')
    fprintf('\nOriginal points: %d\n', N);
    fprintf('Points removed : %d\n', N - sum(keep));
    fprintf('Remaining points: %d\n', sum(keep));
end