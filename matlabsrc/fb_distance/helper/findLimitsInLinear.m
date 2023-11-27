%% T_l = findLimitsInLinear(trajdata, upper, lower, varargin)
%
% Method: Check if dist(node-limit) < epsilon.
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
function T_l = findLimitsInLinear(trajdata, upper, lower, varargin)
    [stateFilter, epsilon] = getFunctionArguments(varargin, 'stateFilter', -1, 'epsilon', 1e-3);
    
    upper = colvec(upper);
    lower = colvec(lower);
    v = trajdata.coefficients;
    if stateFilter == -1
        stateFilter = 1:size(v,2)-1;
    elseif isempty(stateFilter)
        stateFilter = 1:size(v,2);
    end
    upper = upper(stateFilter);
    lower = lower(stateFilter);
    v = v(:,stateFilter);
    ndim = length(upper);
    
    isOnULimit = max(upper - v', 0) < epsilon;
    isOnLLimit = max(v' - lower, 0) < epsilon;
    
    T_l = findLimits_helper(ndim, isOnULimit, isOnLLimit, trajdata.times);
end

