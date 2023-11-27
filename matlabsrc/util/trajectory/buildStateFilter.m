%% stateFilter = buildStateFilter(stateFilter, nMax)
% Build a general state filter from the input.
% If
%   stateFilter is emtpy:       return 1:nMax
%   stateFilter is -1:          return 1:nMax-1
%   stateFilter is some vector: return stateFilter
%
function stateFilter = buildStateFilter(stateFilter, nMax)
    if isempty(stateFilter)
        stateFilter = 1:nMax;
    elseif isscalar(stateFilter) && stateFilter == -1
        stateFilter = 1:nMax-1;
    end
    if islogical(stateFilter)
        stateFilter = find(stateFilter);
    end
end