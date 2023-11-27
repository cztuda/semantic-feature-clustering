%% T_l = findLimits_helper(ndim, isOnULimit, isOnLLimit, allTimes)
% Part of the two functions findLimitsInCubic and findLimitsInLinear. Do not use as standalone function. 
function T_l = findLimits_helper(ndim, isOnULimit, isOnLLimit, allTimes)
    T_l = cell(1, ndim);
    for i = 1:ndim        
            onULimit = find(isOnULimit(i,:));
            onLLimit = find(isOnLLimit(i,:));
            if ~isempty(onULimit)
                resU = findConsecutiveIntegers(onULimit);
                resU = onULimit(resU);
                if isvector(resU)
                    resU = colvec(resU);
                end

                isTouchU = diff(resU, 1, 1)==0;
                n1U = sum(isTouchU);
                n2U = sum(~isTouchU);
            else
                resU = zeros(2,0);
                isTouchU = [];
                n1U = 0;
                n2U = 0;
            end
            if ~isempty(onLLimit)
                resL = findConsecutiveIntegers(onLLimit);
                resL = onLLimit(resL);
                if isvector(resL)
                    resL = colvec(resL);
                end

                isTouchL = diff(resL, 1, 1)==0;
                n1L = sum(isTouchL);
                n2L = sum(~isTouchL);
            else
                resL = zeros(2,0);
                isTouchL = [];
                n1L = 0;
                n2L = 0;
            end
            inds = [resU(1, isTouchU), resL(1, isTouchL), resU(1, ~isTouchU), resL(1, ~isTouchL), resU(2, ~isTouchU), resL(2, ~isTouchL)];
            type = [3*ones(1, n1U), 6*ones(1, n1L), 1*ones(1, n2U), 4*ones(1, n2L), 2*ones(1, n2U), 5*ones(1, n2L)];

            times = allTimes(inds);
            [times, ISort] = sort(times);
            T_l{i} = [rowvec(times); type(ISort)];
    end
end