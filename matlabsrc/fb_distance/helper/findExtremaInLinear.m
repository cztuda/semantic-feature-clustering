%% [res, isMax, isMin] = findExtremaInLinear(data, filter, epsilon=1e-2, minIncrease)
%
% INPUT:
%   data:     data struct that describes the linear spline function
%   filter:   (default=[]) ignore some indices in the function
%   epsilon:  (optional, default=1e-2) small derivative values below this
%               threshold are filtered from the result
% OUTPUT:
%   res:      list of times: points on the trajectory
%   isMax:    all values res(isMax) are maxima, res(~isMax) are minima
% ADDITIONAL ARGUMENTS:
%   asLogicalIndex:
%             return result as vector of logicals instead of the list of
%              times. The length of the vector is data.nGridpoints
%   filterExtrema
%             (default=true) filter small extrema
%   sepDims   (default=false) set true to return a cell fun with results per dimension
%
% CAUTION: If sepDim=false, then sum(res) <= sum(isMax) + sum(isMin)
%
function [extrema, isMax, isMin] = findExtremaInLinear(data, varargin)
    [filter, epsilon, logicalResult, doExtremaFiltering, sepDims] = getFunctionArguments(varargin, {[], 1e-2}, 'asLogicalIndex', false, 'filterExtrema', true, 'sepDims', false);
    if isempty(filter)
        filter = true(data.n,1);
    end
    
    vec = data.times;
    dvec = vec(2:end)-vec(1:end-1);

    u = data.coefficients;
    if nargin >= 2 && ~isempty(filter)
        u = u(:, filter);
    end
    du = u(2:end,:)-u(1:end-1,:);


    % search for local extrema:
    dudvec = du./dvec;
    if doExtremaFiltering
        uSmooth = useSGolayFilter(vec, data.coefficients, 3, makeOdd(round(length(data.times)/10)));
        tmp = ischange(sign(dudvec));
        [extrema, isMax, isMin] = filterExtrema(tmp, vec, uSmooth, epsilon, sepDims);
    else
        [extrema, isMax, isMin] = getUnfilteredExtrema(dudvec, sepDims);
    end
    %extrema = [false; extrema; false];
        
    if ~logicalResult
        if sepDims
            extrema = cellfun(@(c)vec(c), extrema, 'UniformOutput', false);
        else
            extrema = vec(extrema);
        end
    else
        if sepDims
            extrema = cellfun(@(c)[c; false], extrema, 'UniformOutput', false);
        else
            extrema = [extrema; false];
        end
    end
end

function [extrema, isMax, isMin] = getUnfilteredExtrema(dudvec, sepDims)
    [n, ndim] = size(dudvec);
    potentialSignChange = dudvec(1:end-1,:).*dudvec(2:end,:) <= 0;
    if sepDims
        extrema = repmat({false(n,1)}, 1, ndim);
        isMax = cell(1,ndim);
        isMin = cell(1,ndim);
        ind = arrayfun(@(i)find(potentialSignChange(:,i))+1, 1:ndim, 'UniformOutput', false);
        
        for i = 1:ndim
            extrema{i}(ind{i})=true;
            extrema{i}(end)=false;
            
            row = find(extrema{i});
            isMax{i} = dudvec(row-1,i)>0|dudvec(row,i)<0;
            isMin{i} = dudvec(row-1,i)<0|dudvec(row,i)>0;
        end
    else
        extrema = false(n,1);
        ind = find(any(potentialSignChange,2))+1;
        
        extrema(ind)=true;
        extrema(end)=false;
        %extrema(ind(unique(findConsecutiveIntegers(ind)))) = true;

        row = find(extrema);
        col = sum(potentialSignChange(row-1,:).*(1:ndim),2);
        col = fixColIndex(col, ndim, potentialSignChange, row);
        isMax = arrayfun(@(r, c) dudvec(r-1,c)>0|dudvec(r,c)<0, row, col);
        isMin = arrayfun(@(r, c) dudvec(r-1,c)<0|dudvec(r,c)>0, row, col);
    end
end
function col = fixColIndex(col, ndim, potentialSignChange, row)
    toolarge = find(col>ndim);
    if any(toolarge)
        for i = 1:length(toolarge)
            col(toolarge(i)) = find(potentialSignChange(row(toolarge(i))-1,:),1);
        end
    end
end

function [isExtr, isMax, isMin] = filterExtrema(tmp, vec, uSmooth, epsilon, sepDims)
    tmpIndex = find(tmp);
    row = mod(tmpIndex-1, size(tmp,1))+1;
    col = (tmpIndex-row)/size(tmp,1) + 1;
    
    col = col(row <= size(tmp,1)-1);
    row = row(row <= size(tmp,1)-1);
    
    isMax = false(size(row,1),1);
    isMin = isMax;
    
    window = 100; % 3
    tolWindow = 60; % 2
    
    n = size(vec,1);
    for i = 1:length(col)
        if row(i) == 1
            continue % ignore the first point
        end
        I = max(1,row(i)-window):min(n, row(i)+window);
        params = polyfit(vec(I), uSmooth(I,col(i)), 2);
        if params(1) > epsilon % check if minimum ...
            t_extr = -0.5*params(2)/params(1);
            isMin(i) = t_extr < vec(min(n,row(i)+tolWindow)) & t_extr > vec(max(1,row(i)-tolWindow)); % max must be near the examined point
        elseif params(1) < -epsilon % ... or maximum
            t_extr = -0.5*params(2)/params(1);
            isMax(i) = t_extr < vec(min(n,row(i)+tolWindow)) & t_extr > vec(max(1,row(i)-tolWindow)); % max must be near the examined point
        end
    end
    isExtr = isMin | isMax;
    
    if sepDims
        ndim = size(tmp,2);
        isExtr = repmat({isExtr}, 1, ndim);
        isMin = repmat({isMin}, 1, ndim);
        isMax = repmat({isMax}, 1, ndim);
        for i = 1:ndim
            isMin{i}(col~=i) = false;
            isMax{i}(col~=i) = false;
            isExtr{i} = false(size(tmp,1),1);
            anyExtr = isMin{i}(col==i) | isMax{i}(col==i);
            isExtr{i}(row(col==i)) = anyExtr;
            isMin{i} = selectIndex(isMin{i}(col==i), anyExtr);
            isMax{i} = selectIndex(isMax{i}(col==i), anyExtr);
        end
    else
        isExtrTmp = isExtr;
        isExtr = false(size(tmp,1),1);
        [row, isExtrTmp, indices] = uniqueRows(row, isExtrTmp);
        isExtr(row) = isExtrTmp;
        isMin = selectIndex(isMin(indices), isExtrTmp);
        isMax = selectIndex(isMax(indices), isExtrTmp);
    end
end

function [urows, uIsExtr, indices] = uniqueRows(rows, isExtr)
    [urows, indices] = unique(rows);
    if length(urows) ~= length(rows)
        uIsExtr = false(size(urows));
        for i = 1:length(urows)
            sel = rows == urows(i);
            if any(isExtr(sel))
                iid = find(isExtr(sel),1);
                ids = find(sel);
                indices(i) = ids(iid);
                uIsExtr(i) = true;
            end
        end
    else
        uIsExtr = isExtr;
    end
end

function v = makeOdd(v)
    v = v + 1-mod(v,2);
end
