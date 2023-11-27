%% [roots, constZero] = findRootsInLinear(data, indices=[], varargin)
%
% OUTPUT:
%   roots:      cell(1,n)  with arrays(1,n_i) with all times where roots occur for each index
%   constZero:  cell(1,n) with arrays(2,n_i) containing the start and end time of a constantly zero
%                 region for each index
% ADDITIONAL ARGUMENTS:
%   zero:       (default=0) which number is considered as root? Use this value to detect times at
%                 which the trajectory crosses values other than zero. Scalar or vector of size data.n 
function [roots, constZero] = findRootsInLinear(data, varargin)
    [indices, zero] = getFunctionArguments(varargin, {[]}, 'zero', []);
        
    nT = length(data.times);
    n = data.n;
    roots = zeros(n, nT+1);
    constZero = zeros(2, nT+1, n);
    nums = zeros(2, n); % ZEROS; CONSTANTLY_ZERO
    last_was_constantly_zero = false(1,n);
    
    times = data.times;
    if ~isempty(indices)
        coeffs = data.coefficients(:,indices);
    else
        coeffs = data.coefficients;
    end
    if ~isempty(zero)
        if isscalar(zero)
            zero = repmat(zero, 1, n);
        end
        coeffs = coeffs - zero;
    end
    
    
    for i = 1:nT
        if i == nT
            atT1 = coeffs(i,:) == 0;
            enlargeConstZeroRegion   =  atT1 &  last_was_constantly_zero;
            zeroAtT1                 =  atT1 & ~last_was_constantly_zero;
        else
            atT1 = coeffs(i,:) == 0;
            atT2 = coeffs(i+1,:) == 0;

            % check for zeros
            enlargeConstZeroRegion   =  atT1 &  atT2 &  last_was_constantly_zero;
            createNewConstZeroRegion =  atT1 &  atT2 & ~last_was_constantly_zero;
            zeroAtT1                 =  atT1 & ~atT2 & ~last_was_constantly_zero;
            zeroInBetween = coeffs(i,:).*coeffs(i+1,:) < 0;
            last_was_constantly_zero =  atT1 &  atT2;
        end
        
        % - zero at t1
        nums(1,:) = nums(1,:)+zeroAtT1;
        if sum(zeroAtT1)>1
            tmpA = find(zeroAtT1);
            for k = 1:length(tmpA)
                roots(tmpA(k), nums(1,tmpA(k))) = times(i);
            end
        else
            roots(zeroAtT1, nums(1,zeroAtT1)) = times(i);        
        end
        if i == nT
            break;
        end
        
        % store zeros, consider the following cases:
        % - enlarge csr:
        if sum(enlargeConstZeroRegion)>1
            tmpA = find(zeroInBetween);
            for k = 1:length(tmpA)
                constZero(2, nums(2,tmpA(k)), tmpA(k)) = times(i+1);
            end
        elseif any(enlargeConstZeroRegion)
            constZero(2, nums(2,enlargeConstZeroRegion), enlargeConstZeroRegion) = times(i+1);
        end
                
        % - create new csr:
        nums(2,:) = nums(2,:)+createNewConstZeroRegion;
        if sum(createNewConstZeroRegion)>1
            tmpA = find(zeroInBetween);
            for k = 1:length(tmpA)
                constZero(:, nums(2,tmpA(k)), tmpA(k)) = [times(i); times(i+1)];
            end
        elseif any(createNewConstZeroRegion)
            constZero(:, nums(2,createNewConstZeroRegion), createNewConstZeroRegion) = [times(i); times(i+1)];
        end
                
        % - zero in between
        nums(1,:) = nums(1,:)+zeroInBetween;
        if sum(zeroInBetween)>1
            tmpA = find(zeroInBetween);
            for k = 1:length(tmpA)
                roots(tmpA(k), nums(1,tmpA(k))) = findRootInLinear(times(i), times(i+1), coeffs(i,tmpA(k))', coeffs(i+1,tmpA(k))');
            end
        else
            roots(zeroInBetween, nums(1,zeroInBetween)) = findRootInLinear(times(i), times(i+1), coeffs(i,zeroInBetween)', coeffs(i+1,zeroInBetween)');
        end
                       
    end
    
    roots = arrayfun(@(i) roots(i, 1:nums(1,i)), 1:n, 'UniformOutput', false);
    constZero = arrayfun(@(i) constZero(:, 1:nums(2,i), i), 1:n, 'UniformOutput', false);
end

function t0 = findRootInLinear(t1, t2, val1, val2)
    %% t0 = findRootInLinear(t1, t2, val1, val2)
    t0 = (t2-t1)/(val2-val1).*(-val1)+t1;
end
