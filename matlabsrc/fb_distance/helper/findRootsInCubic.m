%% [roots, constZero] = findRootsInCubic(data, indices=[], threshold=1e-6)
%
% INPUT:
%   data:       data struct with information about the cubic function
%   indices:    for which indices to search for roots
%   threshold:  at which a value is considered equal to zero
%
% OUTPUT:
%   roots:      cell(1,n)  with arrays(1,n_i) with all times where roots occur for each index
%   constZero:  cell(1,n) with arrays(2,n_i) containing the start and end time of a constantly zero
%                 region for each index
%
% OPTIONAL ARGUMENTS:
%   omitStart:  (default=false) do not list roots at start time
%   omitEnd:    (default=false) do not list roots at end time
%   zero:       (default=0) which number is considered as root? Use this value to detect times at
%                 which the trajectory crosses values other than zero. Scalar or vector of size data.n 
function [roots, constZero] = findRootsInCubic(data, varargin)
    [indices, threshold, omitStart, omitEnd, zero] = getFunctionArguments(varargin, {[], 1e-6}, 'omitStart', false, 'omitEnd', false, 'zero', []);
    
    n = data.n;    
    times = data.times;
    t0 = times(1);
    tf = times(end);
    lastEntry = data.lastEntry;
    if ~isempty(indices)
        coeffs = permute(reshape(data.coefficients, n, [], 4), [1,3,2]);
        coeffs = coeffs(indices,:,:);
        lastEntry = lastEntry(indices);
        n = size(coeffs,1);
    else
        coeffs = permute(reshape(data.coefficients, n, [], 4), [1,3,2]);
    end
    if ~isempty(zero)
        if isscalar(zero)
            zero = repmat(zero, n, 1);
        elseif length(zero) ~= n
            zero = zero(indices);
        end
        coeffs(:,1,:) = coeffs(:,1,:) - colvec(zero);
        lastEntry = lastEntry - colvec(zero);
    end
    
    [zeros, index1] = zerosInBetween(times, coeffs, threshold);    
    [zeros2, index2] = zerosAtGripoints(coeffs, times, lastEntry, threshold);
    zeros = [zeros; zeros2];
    roots = arrayfun(@(i)zeros([index1;index2]==i), 1:n, 'UniformOutput', false);
        
    if nargout > 1
        [constZero, index] = findZeroRegion(coeffs, times, threshold);
        constZero = arrayfun(@(i)constZero(:,index==i), 1:n, 'UniformOutput', false);
    end
    
    roots = removeStartAndEnd(roots, 1e-5, t0, tf, omitStart, omitEnd);
end

function roots = removeStartAndEnd(roots, tol, t0, tf, omitStart, omitEnd)
    if ~omitStart
        t0 = [];
    end
    if ~omitEnd
        tf = [];
    end
    
    for i = 1:length(roots)
        t = roots{i};
        e = true(size(t));
        if ~isempty(t0)
            e = e & ~(abs(t-t0) < tol);            
        end
        if ~isempty(tf)
            e = e & ~(abs(t-tf) < tol);
        end
        roots{i} = t(e);
    end
end

function [zero, index] = zerosInBetween(times, coeffs, threshold)
    if nargin < 2
        threshold = 1e-6;
    end
    allRoots = reshape(map(@(j)map(@(i)roots(fliplr(coeffs(i,:, j))), 1:size(coeffs,1)), 1:size(coeffs,3)), 3, size(coeffs,1),[]);    
    allRoots = removeMultipleRoots(allRoots, threshold);
    
    zeros = threshold < allRoots & allRoots < double(imag(allRoots)==0)-threshold; % 0 < r < 1 and r real
    [~, index, iz] = ind2sub(size(allRoots), find(zeros));    
    zero = times(iz,1)+allRoots(zeros).*diff(times(iz,:), [], 2);
end

function allRoots = removeMultipleRoots(allRoots, threshold)
    %% allRoots = removeMultipleRoots(allRoots, threshold=1e-6)
    % Check if the same root appears multiple times for the same function and remove all but one
    % occurence.
    if nargin < 2
        threshold = 1e-6;
    end
    f = abs(allRoots(1,:,:)-allRoots(2,:,:)) < threshold;
    allRoots(1, f) = nan;
    f = abs(allRoots(2,:,:)-allRoots(3,:,:)) < threshold;
    allRoots(2, f) = nan;
    f = abs(allRoots(1,:,:)-allRoots(3,:,:)) < threshold;
    allRoots(1, f) = nan;
end

function [zero, index] = zerosAtGripoints(coeffs, times, lastEntry, threshold)
    if nargin < 2
        threshold = 1e-6;
    end
    n = size(coeffs,1);
    nT = size(coeffs,3);
    
    [index, iy] = ind2sub([n, nT], find(abs(reshape(coeffs(:,1,:), n, nT)) < threshold));    
    zero = times(iy,1);
    
    ix = find(abs(lastEntry) < threshold);
    index = [index; ix];
    zero = [zero; times(end,2)*ones(size(ix))];
end


function [zero, index] = findZeroRegion(coeffs, times, threshold)
    if nargin < 2
        threshold = 1e-6;
    end
    n = size(coeffs,1);
    nT = size(coeffs,3);
    
    isConst = reshape(all(coeffs(:,2:end,:)==0,2) & abs(coeffs(:,1,:)) < threshold, n, nT);
    [ix, iy] = ind2sub([n, nT], find(isConst));
    
    zero = zeros(2,0);
    index = [];
    
    if ~isempty(ix)
        v = unique(ix);
        for i = 1:length(v)
            vals = iy(ix==v(i));
            res = findConsecutiveIntegers(vals);
            startTimes = times(vals(res(1,:)), 1);
            endTimes = times(vals(res(2,:)), 2);
            
            zero = [zero, [startTimes; endTimes]]; %#ok<AGROW>
            index = [index, v(i)*ones(1, length(startTimes))];%#ok<AGROW>
        end
    end
end
