%% [T, isMax] = findExtremaInCubic(data, filter=[], varargin)
% Find extrema in piecewise cubic functions.
% INPUT:
%   data:       data struct that describes the cubic spline function
%   filter:     (default=[]) ignore some indices in the function
% OUTPUT:
%   T:          cell(1,n) of times where the extrema can be found for each index, n is the dimension
%                 of the cubic function.
%   isMax:      cell(1,n) with boolean value, if the respective extremum is a maximum or not.
%
% ADDITIONAL ARGUMENTS:
%   omitStart:  (default=false) do not list roots at start time
%   omitEnd:    (default=false) do not list roots at end time
%   useMatlab:  (default=false)
%   relProm:    (default=false) compute relative prominence (only available if useMatlab=false).
%                 Values are relative wrt range of values covered by the respective trajectory.
%
function [T, isMax, prominence] = findExtremaInCubic(data, varargin)
    [filter, omitStart, omitEnd, useMatlab, relProm] = getFunctionArguments(varargin, {[]}, 'omitStart', false, 'omitEnd', false, 'useMatlab', false, 'relProm', false);
    
    C = data.coefficients;
    n = data.n;
    coeffs = permute(reshape(C, n, [], 4), [1, 3, 2]);

    a = C(:,4);
    b = C(:,3);
    c = C(:,2);

    % compute roots of first derivative:
    x1 = (-b + sign(-b).*sqrt(b.*b - 3*a.*c))./(3*a);
    x2 = -2*b./(3*a) - x1; % Vieta formula
    sel = abs(imag(x1)) < 1e-2;
    isMultipleRoot = abs(x1-x2) < 1e-6;
    
    x1 = reshape(real(x1), n, []);
    x2 = reshape(real(x2), n, []);
    sel = reshape(sel, n, []);
    isMultipleRoot = reshape(isMultipleRoot, n, []);
    if ~isempty(filter)
        x1 = x1(filter,:);
        x2 = x2(filter,:);
        sel = sel(filter,:);
        isMultipleRoot = isMultipleRoot(filter, :);
        n = size(x1,1);
    end

    % select those real roots that are in [0, 1]
    sel2 = (0 <= x1 & x1 <= 1) & sel;
    sel3 = (0 <= x2 & x2 <= 1) & sel & ~isMultipleRoot;
    
    if useMatlab
        [T, isMax, prominence] = findPeaksWithMatlab(sel2, sel3, x1, x2, data);
        return;
    end
    
    % remove consecutive peaks: peaks that are closer than 1e-4 on the time scale
    T = cell(1,n);
    isMax = cell(1,n);
    linCoeffs = permute(coeffs(:,1,:), [1,3,2]);
    [allTtmp, allInd] = getRoots(sel2, sel3, x1, x2, data.times);
    for i = 1:n
        ind = unique(allInd{i});
        Ttmp = allTtmp{i};
        Ttmp2 = [];
        subsel = true(size(Ttmp));
    
        consInd = findConsecutiveIntegers(ind);
        isConsec = find(diff(consInd) > 0);
        for j = 1:length(isConsec)
            cind = consInd(:,isConsec(j));
            irange = ind(cind(1)):ind(cind(2));
            subsel(cind(1):cind(2)) = false;
            rangeX = data.times(irange,1);
            rangeY = reshape(coeffs(i,1,irange),1,[]);
            [linSw,~,linInd] = piecewiseLinearFit(rangeX, rangeY);
            
            if ~isempty(linSw)
                lindat = createLinearDataFromFit(rangeX, rangeY, linInd);
                extremaInLin = findExtremaInLinear(lindat, [], 1e-2);
                extremaInLin = extremaInLin(extremaInLin~= rangeX(1) & extremaInLin~= rangeX(end));
            else
                lc2 = linCoeffs(i,:);
                lc2(:, irange(2:end-1))=[];
                t2 = data.times(:,1);
                t2 = [t2(1:irange(1)); t2(irange(end):end)-t2(irange(end))+t2(irange(2))] ;

                extremaInLin = find(findExtremaInLinear(secondOutValue(@createPiecewiseLinear, t2, lc2), [], 1e-7, 'asLogicalIndex', true));
                extremaInLin = extremaInLin(extremaInLin >= irange(1)-1 & extremaInLin <= irange(1)+2);
                extremaInLin(extremaInLin > irange(1)) = extremaInLin(extremaInLin > irange(1)) + irange(end)-irange(1)-1;
                extremaInLin = data.times(extremaInLin, 1);
                %d = absoluteDifference(data, coeffs, Ttmp, irange, i)
            end
            Ttmp2 = [Ttmp2; extremaInLin(:)];%#ok<AGROW>
        end
        T{i} = uniquetol(sort([Ttmp(subsel); Ttmp2]), 1e-4);
        isMax{i} = selectIndex(evaluateDerivative(data, T{i}, 2) < 0, i);
    end
    
    [T, isMax] = removeStartAndEnd(T, isMax, 1e-4, data, omitStart, omitEnd);
    for i = 1:length(T)
        if isempty(T{i})
            T{i} = zeros(0,1);
        end
    end
    if nargout > 2
        prominence = computeProminence(T, isMax, data, relProm);
    end
end

function [T, isMax] = removeStartAndEnd(T, isMax, tol, data, omitStart, omitEnd)
    if omitStart
        t0 = data.times(1);
    else
        t0 = [];
    end
    if omitEnd
        tf = data.times(end);
    else
        tf = [];
    end
    
    for i = 1:length(T)
        t = T{i};
        e = true(size(t));
        if ~isempty(t0)
            e = e & ~(abs(t-t0) < tol);            
        end
        if ~isempty(tf)
            e = e & ~(abs(t-tf) < tol);
        end
        T{i} = t(e);
        isMax{i} = isMax{i}(e);
    end
end

function [ind2, ind3, Ttmp2, Ttmp3] = getCandidates(sel2, sel3, x1, x2, times)
    n = size(x1,1);
    ind2 = arrayfun(@(i)find(sel2(i,:)), 1:n, 'UniformOutput', false);
    ind3 = arrayfun(@(i)find(sel3(i,:)), 1:n, 'UniformOutput', false);
    Ttmp2 = arrayfun(@(i)times(ind2{i}, 1) + x1(i, ind2{i})'.*diff(times(ind2{i},:), 1, 2), 1:n, 'UniformOutput', false);
    Ttmp3 = arrayfun(@(i)times(ind3{i}, 1) + x2(i, ind3{i})'.*diff(times(ind3{i},:), 1, 2), 1:n, 'UniformOutput', false);
end

function [Ttmp, ind] = getRoots(sel2, sel3, x1, x2, times)
    n = size(x1,1);
    [ind2, ind3, Ttmp2, Ttmp3] = getCandidates(sel2, sel3, x1, x2, times);
    
    Ttmp = cell(1,n);
    ind = cell(1,n);
    for i = 1:n
        [Ttmp{i}, sInd] = sort([Ttmp2{i}; Ttmp3{i}]);
        indTmp = [ind2{i}(:); ind3{i}(:)];
        ind{i} = indTmp(sInd);
    end
end

function [T_e, Te_isMax, Te_prom] = findPeaksWithMatlab(sel2, sel3, x1, x2, data)
    times = data.times;
    
    n = size(x1,1);
    [~, ~, Ttmp2, Ttmp3] = getCandidates(sel2, sel3, x1, x2, times);
    
    T = uniquetol(sort([times(:,1); times(end); cell2mat(Ttmp2'); cell2mat(Ttmp3')]), 1e-4);
    T = T(2:end-1);
    V = cropLastIndex(evaluateDerivative(data, T, 0));
    
    T_e = cell(1,n);
    Te_isMax = cell(1,n);
    Te_prom = cell(1,n);
    for i = 1:n
        [~, locMax,~,promMax] = findpeaks(V(i,:), T);
        [~, locMin,~,promMin] = findpeaks(-V(i,:), T);
        [T_e{i}, I] = sort([locMax(:); locMin(:)]);
        tmp = [true(length(locMax),1); false(length(locMin),1)];
        Te_isMax{i} = tmp(I);
        tmp = [promMax(:); promMin(:)];
        Te_prom{i} = tmp(I);
    end
    %TODO: sort constant regions!
end

function tmp()
    T = [];
    for i = 1:n
        % ##### consider first roots #####
        ind = find(sel2(i,:));
        Ttmp = data.times(ind, 1) + x1(i, ind)'.*diff(data.times(ind,:), 1, 2);
        % filter points where also the second derivative is close to zero
      %%
        subsel = true(size(Ttmp));
        for j = 1:length(ind)
            tmp = evaluateDerivative(data, Ttmp(j), 2);
            subsel(j) = abs(tmp(i)) > epsilon;
        end
        T = [T; Ttmp(subsel)]; %#ok<AGROW>
        
        % ##### consider second roots #####
        ind = find(sel3(i,:));
        Ttmp = data.times(ind, 1) + x2(i, ind)'.*diff(data.times(ind,:), 1, 2);
        % filter points where also the second derivative is close to zero
        subsel = true(size(Ttmp));
        for j = 1:length(ind)
            tmp = evaluateDerivative(data, Ttmp(j), 2);
            subsel(j) = abs(tmp(i)) > epsilon;% filter points where also the second derivative is close to zero
        end
        T = [T; Ttmp(subsel)]; %#ok<AGROW>
    end
    
end

% function P = computeProminence(T, isMaximum, data, relativeProminence)
%     n = length(T);
%     yLeft = data.coefficients(1:n,1);
%     yRight = sum(data.coefficients(end-data.n+1:end,:),2);
%     P = cell(1,n);
%     
%     for i = 1:n
%         t_extr = T{i};
%         y_extr = selectIndex(evaluateDerivative(data, t_extr, 0), i);
%         isMax = isMaximum{i};
%         n_j = length(t_extr);
%         seq = 1:n_j;
%         prominence = zeros(1, n_j);
%         for j = 1:n_j
%             if isMax(j)
%                 filter = isMax & y_extr > y_extr(j);
%                 iL = find(filter & seq<j, 1, 'last');
%                 iR = find(filter & seq>j, 1, 'first');
%                 
%                 if isempty(iL), iL = 0; end
%                 tmpL = min(y_extr(~isMax & seq > iL & seq < j));  
%                 if isempty(tmpL)
%                     tmpL = yLeft(i);
%                 end
%                 
%                 if isempty(iR), iR = n_j+1; end
%                 tmpR = min(y_extr(~isMax & seq > j & seq < iR));
%                 if isempty(tmpR)
%                     tmpR = yRight(i);
%                 end
%                 
%                 refLvl = max(tmpL, tmpR);
%                 prominence(j) = y_extr(j) - refLvl;                
%             else
%                 filter = ~isMax & y_extr < y_extr(j);
%                 iL = find(filter & seq<j, 1, 'last');
%                 iR = find(filter & seq>j, 1, 'first');
%                 
%                 if isempty(iL), iL = 0; end
%                 tmpL = max(y_extr(isMax & seq > iL & seq < j));                
%                 if isempty(tmpL)
%                     tmpL = yLeft(i);
%                 end
%                 
%                 if isempty(iR), iR = n_j+1; end
%                 tmpR = max(y_extr(isMax & seq > j & seq < iR));
%                 if isempty(tmpR)
%                     tmpR = yRight(i);
%                 end
%                                 
%                 refLvl = min(tmpL, tmpR);
%                 prominence(j) = refLvl - y_extr(j);
%             end
%         end
%         if nargin > 3 && relativeProminence
%             d = max([y_extr(isMax), yLeft(i), yRight(i)]) - min([y_extr(~isMax), yLeft(i), yRight(i)]);
%             prominence = prominence ./ d;
%         end
%         P{i} = prominence;
%     end
% end


function lindat = createLinearDataFromFit(x, y, sInd)
    ind = [1, sInd, length(x)];
    lindat = struct('n', size(y,1), ...
        'isadj', 'F', ...
        'n2', 0, ...
        'type', 1, ...
        'nGridpoints', length(sInd)+2, ...
        't0', x(1), ...
        'tf', x(end), ...
        'times', x(ind), ...
        'coefficients', y(:,ind)');
end