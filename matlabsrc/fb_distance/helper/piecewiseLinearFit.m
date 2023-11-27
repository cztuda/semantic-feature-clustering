%% [s, R, sInd, p] = piecewiseLinearFit(x, y, n=[])
% OR [s, R, sInd, p] = piecewiseLinearFit(x, y)
% Fit a piecewise linear function (n segments) through the data (x,y).
% If n is empty or not given, then a good value for n is determined automatically.
% 
% INPUT:
%   x:      [1 x k] vector, may be unsorted
%   y:      [1 x k ]vector, may be unsorted, must not be a matrix
%   n:      either a scalar, gives the number of segments, or a vector, which gives the initial
%             switching indices
%           If n is omitted or empty, then a good value for n is determined automatically.
% OUTPUT:
%   s:      [1, l], vector of all switching times, l <= n
%   R:      sum of squared error of all data points
%   sInd:   index of switching time
%   p:      polynomial coefficients for each segment such that @(t)p(1,1)*t+p(1,2) is the linear
%             approximation of the first section
function [s, R, sInd, p] = piecewiseLinearFit(x, y, n)
    if nargin < 3 || isempty(n)
        [s, R, sInd, p] = adaptiveNPwLinearFit(x, y);
        return;
    end
    ux = sort(unique(x));
    N = length(ux);
    [s, n] = getStartS(n, N);
    
    isNotInLocalMin = true(1, n);
    cyc = [-ones(1, n+1); zeros(n-1, n+1)];
    it = 1; i = 1;
    
    R = allRegression(x, y, s);
    
    while any(isNotInLocalMin) && it < 1000           
        [s, R, isNotInLocalMin(i)] = searchIndexCorrection(x, y, R, ux, s, i);

        [isCycling, cyc] = checkIfCycling(cyc, s, it);
        if isCycling % avoid cycling between solutions
            break;
        end
        it = it+1;
        
        i = i+1;
        if i > n
            if any(isNotInLocalMin)
                i = 1;
                isNotInLocalMin(:) = true;
            else
                break;
            end
        end
    end
    
    sInd = s;
    s = ux(s);
    if nargout > 2
        [~, p] = allRegression(x, y, sInd);
    end
end


function [s, v, index, p] = adaptiveNPwLinearFit(T, Y)
    %% [s, v, index, p] = adaptiveNPwLinearFit(T, Y)
    % Automatically determine a good n
    v = [];
    p = {};
    s = {};
    index = {};
    for i = 0:5
        [s{i+1}, v(i+1), index{i+1}, p{i+1}] = piecewiseLinearFit(T, Y, i); %#ok<AGROW>
        changed = hasChanged(v);% ischange(v);
        if (any(changed) && ~changed(end)) || v(end) < 1e-4
            n = find(changed,1,'last');
            if isempty(n)
                n = length(v);
                changed = true;
            end
            index = index{n};
            p = p{n};
            s = s{n};
            v = v(n);
            break
        end
    end
    if ~any(changed)
        index = index{1};
        p = p{1};
        s = s{1};
        v = v(1);
    end
end

function ch = hasChanged(v)
    ch = ischange(v);
    if ~any(ch)
        v2 = v(1:end-1)./v(2:end);
        ch2 = ischange(v2);
        if any(ch2) && ~ch2(end)
            ch(find(ch2, 1)) = 1;
        end
    end
end

function [R, P] = allRegression(x, y, s) % 
    R = 0;
    P = zeros(length(s)+1,2);
    for i = 0:length(s)
        sel = getSel(length(x),s,i);
        [P(i+1,:), dv] = getPiece(x(sel), y(:,sel));
        R = R + sum(normColumn(dv.*dv));
    end
end


function [s, R, hasChanged] = searchIndexCorrection(x, y, R, ux, s, i)
    if i == 1
        iL = 1;
    else
        iL = s(i-1)+1;
    end
    if i == length(s)
        iR = length(ux);
    else
        iR = s(i+1)-1;
    end
    
    s2 = s;
    allRs = zeros(1, iR-iL+1);
    for ind = iL:iR
        s2(i) = ind;
        allRs(ind-iL+1) = allRegression(x, y, s2);
    end
    [Rmin, I] = min(allRs);
    if Rmin < R
        s(i) = I+iL-1;
        R = Rmin;
        hasChanged = true;
    else
        hasChanged = false;
    end
end


function [s, n] = getStartS(n, maxN)
    if length(n) > 1
        s = sort(unique(min(max(n, 1), maxN)), 'ascend');
        n = length(s);
    else
        n = min(n, maxN);
        s = selectIndex(round(linspace(0, maxN+1, n+2)), 2:n+1);
    end
end


%%
function [isCycling, cyc] = checkIfCycling(cyc, s, it)
    isCycling = false;
    cyc(:, max(1, mod(it, size(cyc,2)-1)+1)) = s;
    if any(nearestNeighbor(cyc) == 0 & cyc(1,:) >= 0)
        isCycling = true;
    end
end


%% sel = getSel(x, s, i)
% get data selection that is in the given interval
% i is in 0..length(s)
function sel = getSel(nx, s, i)
    if isempty(s)
        sel = 1:nx;
    else
        if i <= 0
           sel = 1:s(1);
        elseif i >= length(s)
            sel = s(end)+1:nx;
        else
            sel = s(i)+1:s(i+1);
        end
    end
end

function [p, dv] = getPiece(x, y)
    if isempty(x)
        p = [0, 0];
        dv = 0;
        return;
    elseif length(x) == 1
        p = [0, y];
        dv = 0;
        return;
    end
    p = ([colvec(x), ones(length(x),1)]\(colvec(y)))';
    dv = p(1)*colvec(x)+p(2) - colvec(y);
end