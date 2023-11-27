%% T_l = findLimitsInCubic(trajdata, upper, lower, varargin)
%
% Method: Get nodes as gridpoints and extrema.
%         Check if dist(node-limit) < epsilon.
%         If yes, check, if this condition is on consecutive nodes fulfilled. yes->segment, no-> touchpoint 
%
%
% Return 2-row table with information about limit contact:
%  First row: times of the events
%  Second row: which event occurs. 'lt' labels touch point, 'ls' start of segment at limit, 'le'
%    end of segment at limit
%
%   1: start of upper limit segment
%   2: end of upper limit segment
%   3: touch point of upper limit segment
%   4: start of lower limit segment
%   5: end of lower limit segment
%   6: touch point of lower limit segment
%
% ADDITIONAL ARGUMENT:
%   stateFilter:    (default=-1)    determine which states are used
%   epsilon:        (default=1e-3)  tolerance used
%   T:              (default=[])    provide extrema such that their computation is avoided (T must be the result of findExtremaInCubic).
function T_l = findLimitsInCubic(trajdata, upper, lower, varargin)
    [stateFilter, epsilon, T] = getFunctionArguments(varargin, 'stateFilter', -1, 'epsilon', 1e-3, 'T', []);
    
    upper = colvec(upper);
    lower = colvec(lower);
    
    coeffs = trajdata.coefficients;
    times = trajdata.times;
    lastEntry = trajdata.lastEntry;
    ndim = trajdata.n;
    
    if stateFilter == -1
        stateFilter = 1:ndim-1;
    elseif isempty(stateFilter)
        stateFilter = 1:ndim;
    end
    upper = upper(stateFilter);
    lower = lower(stateFilter);
    sf_bool = false(ndim,1); sf_bool(stateFilter) = true;
    lastEntry = lastEntry(sf_bool);
    coeffs = coeffs(repmat(sf_bool, (trajdata.nGridpoints-1), 1), :);
    ndim = length(upper);
    
    if isempty(T)
        T = findExtremaInCubic(trajdata, stateFilter);
    end
    T = uniquetol(cell2mat(T(:))', 1e-3);
    extrStates = evaluateDerivative(trajdata, T, 0);
    extrStates = extrStates(stateFilter,:);
        
    atNodes = [reshape(coeffs(:,1), ndim,[]), lastEntry];    
    ptsTable = mergeTimetables([T; extrStates], [[rowvec(times(:,1)), trajdata.tf]; atNodes], 'makeUnique', 2);
    
    isOnULimit = max(upper - ptsTable(2:end,:), 0) < epsilon;
    isOnLLimit = max(ptsTable(2:end,:) - lower, 0) < epsilon;
    
    T_l = findLimits_helper(ndim, isOnULimit, isOnLLimit, ptsTable(1, :));
end

