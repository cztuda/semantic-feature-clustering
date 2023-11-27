%% [tt, mappingIndex] = mergeTimetables(tt1, tt2, varargin)
% Merge two matrices in the format tt1=[times1; values1] and tt2=[times2; values2]
% where times1, times2 are vectors of size [1, n] and values1, values2 are matrices of size [m, n]
% and sort the resulting matrix of size [m+1, 2*n] such that the first row is ascending.
%
% INPUT:
%   tt1:    matrix of size [m+1, n]
%   tt2:    matrix of size [m+1, n]
% OUTPUT:
%   tt:     matrix of size [m+1, 2*n] such that tt(1,:) is sorted
% 
% ADDITIONAL ARGUMENTS:
%   makeUnique:     (default=0): 
%                     0 -> keep all values
%                    -1 -> make first row unique and check if the values are the same.
%                           If they are not, then throw an error
%                     1 -> make first row unique and check if the values are the same.
%                           If they are not, then prefer the value from the first table
%                     2 -> make first row unique and check if the values are the same.
%                           If they are not, then prefer the value from the second table
%   uniqueTol:      (default=1e-5): threshold for checking the time for uniqueness
%   diffTol:        (default=1e-5): threshold (norm distance) for checking values for uniqueness
%   ascending:      (default=true) set false to make first row descending
function [tt, mappingIndex] = mergeTimetables(tt1, tt2, varargin)
    [makeUniqueMode, isAscending, tnqTol, diffTol] = getFunctionArguments(varargin, 'makeUnique', 0, 'ascending', true, 'uniqueTol', 1e-5, 'diffTol', 1e-5);
    if isAscending
        direction = 'ascend';
    else
        direction = 'descend';
    end
    
    times = [tt1(1,:), tt2(1,:)];
    values = [tt1(2:end,:), tt2(2:end,:)];
    mappingIndex = [ones(1, size(tt1,2)), 2*ones(1,size(tt2,2))];
    
    if makeUniqueMode
        [C, IA, IC] = uniquetol(times, tnqTol);
        if length(C) < length(times)
            for i = 1:length(IA)
                sel = IC==IC(IA(i));
                if sum(sel) > 1 && max(normColumn(diff(values(:,sel),1,2))) > diffTol               
                    switch(makeUniqueMode)
                        case -1
                            error('mergeTimetables: The values for the same time are not the same.');
                        case 1
                            ind = find(mappingIndex(sel)==1, 1);
                            if ~isempty(ind)
                                IA(i) = ind;
                            end
                        case 2
                            ind = find(mappingIndex(sel)==2, 1);
                            if ~isempty(ind)
                                IA(i) = ind;
                            end
                    end
                end
            end
            times = C;
            values = values(:,IA);
            mappingIndex = mappingIndex(IA);
        end
    end
    
    I = secondOutValue(@sort, times, direction);
    tt = [times; values];
    tt = tt(:, I);
    if nargout > 1
        mappingIndex = mappingIndex(I);
    end
    
end