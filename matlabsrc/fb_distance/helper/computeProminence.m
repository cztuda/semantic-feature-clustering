%% P = computeProminence(T, isMaximum, data, relativeProminence)
% Compute the prominence values for maxima or minima of a given trajectory.
%
% INPUT:
%   T:          Cell of time values at which the maxima/minima occur. Length of the cell is equals
%                 the dimension of the trajectory, each cell entry list the times at which for the
%                 respective dimension maxima or minima occur
%   isMaximum:  Cell of bool arrays, lengths are the same as for T. True, if the respective time
%                 entry denotes a maximum
%   data:       trajectory data struct
%   relativeProminence:     (default=false) Set true to compute normalized prominence values.
%
% ADDITIONAL ARGUMENTS:
%   isMinimum:  cell of bool arrays, lengths are the same as for T. True, if the respective time
%                 entry denotes a minimum
%   subsel:     
%   noBVals:    enable a variant: do not use the boundary value if there is some minimum/maximum 
%                 value inside the interval 
function P = computeProminence(T, isMaximum, data, relativeProminence, varargin)
    [isMinimum, subsel, noBVals] = getFunctionArguments(varargin, 'isMinimum', {}, 'subsel', {}, 'avoidBoundaryValues', false);
    if isempty(isMinimum)
        isMinimum = cellfun(@(c)~c, isMaximum, 'UniformOutput', false);
    end
    if isempty(subsel)
        subsel = cellfun(@(c)1:length(c), T, 'UniformOutput', false);
    end
    n = length(T);
    yLeft = evaluateDerivative(data, data.t0, 0);%data.coefficients(1:n,1);
    yRight = evaluateDerivative(data, data.tf, 0); %sum(data.coefficients(end-data.n+1:end,:),2);
    P = cell(1,n);
    
    for i = 1:n
        t_extr = T{i};
        y_extr = evaluateDerivative(data, t_extr, 0);
        if n > 1
            y_extr = selectIndex(y_extr, i);
        end
        isMax = rowvec(isMaximum{i});
        isMin = rowvec(isMinimum{i});
        n_j = length(t_extr);
        seq = 1:n_j;
        subsel_i = subsel{i};
        n_si = length(subsel_i);
        prominence = zeros(1, n_si);
        for k = 1:n_si
            j = subsel_i(k);
            if isMax(j)
                filter = isMax & y_extr > y_extr(j);
                iL = find(filter & seq<j, 1, 'last');
                iR = find(filter & seq>j, 1, 'first');
                
                if isempty(iL), iL = 0; end
                tmpL = min(y_extr(isMin & seq > iL & seq < j));  
                if isempty(tmpL)
                    tmpL = yLeft(i);
                    isLeftBound = -inf;
                else
                    isLeftBound = 0;
                end
                
                if isempty(iR), iR = n_j+1; end
                tmpR = min(y_extr(isMin & seq > j & seq < iR));
                if isempty(tmpR)
                    tmpR = yRight(i);
                    isRightBound = -inf;
                else
                    isRightBound = 0;
                end
                
                if noBVals
                    refLvl = max(tmpL+isLeftBound, tmpR+isRightBound);
                    if isinf(refLvl)
                        refLvl = max(tmpL, tmpR);
                    end
                else
                    refLvl = max(tmpL, tmpR);
                end
                prominence(j) = y_extr(j) - refLvl;                
            elseif isMin(j)
                filter = isMin & y_extr < y_extr(j);
                iL = find(filter & seq<j, 1, 'last');
                iR = find(filter & seq>j, 1, 'first');
                
                if isempty(iL), iL = 0; end
                tmpL = max(y_extr(isMax & seq > iL & seq < j));                
                if isempty(tmpL)
                    tmpL = yLeft(i);
                    isLeftBound = inf;
                else
                    isLeftBound = 0;
                end
                
                if isempty(iR), iR = n_j+1; end
                tmpR = max(y_extr(isMax & seq > j & seq < iR));
                if isempty(tmpR)
                    tmpR = yRight(i);
                    isRightBound = inf;
                else
                    isRightBound = 0;
                end
                
                if noBVals
                    refLvl = min(tmpL+isLeftBound, tmpR+isRightBound);
                    if isinf(refLvl)
                        refLvl = min(tmpL, tmpR);
                    end
                else
                    refLvl = min(tmpL, tmpR);
                end
                prominence(j) = refLvl - y_extr(j);
            end
        end
        if nargin > 3 && relativeProminence
            d = max([y_extr(isMax), yLeft(i), yRight(i)]) - min([y_extr(isMin), yLeft(i), yRight(i)]);
            prominence = prominence ./ d;
        end
        P{i} = prominence;
    end
end