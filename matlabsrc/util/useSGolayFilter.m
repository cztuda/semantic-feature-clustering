%% [sigF, dsigF] = useSGolayFilter(t, sig, order, framewidth)
% 
% INPUT:
%   t:          [nx1], vector of times, is assumed to contain ascending values
%                 if the times do not form a regular grid, the values are resampled and interpolated
%   sig:        [nxm] signal of dimension m, may also be [(n+1)xm] if the times form a regular grid
%   order:      the S-G order
%   framewidth: the S-G framewidth
%
%
function [sigF, dsigF] = useSGolayFilter(t, sig, order, framewidth)
    if nargin < 3
        order = 3;
        framewidth = 39;
    end
    is_regular_time_grid = length(uniquetol(diff(t), 1e-3)) == 1;
    [t, sig] = fixDimensions(t, sig, is_regular_time_grid);
    framewidth = modifyFramewidth(framewidth, sig, order);
    if isempty(framewidth)
        sigF = sig;
        dsigF = diff(sig, 1, 1)./diff(t,1,1);
        dsigF = [dsigF; dsigF(end,:)];
        return;
    end
    if ~is_regular_time_grid
        dt = min(diff(t));
        t0 = t(1); tf = t(end);
        t_old = t;
        t = colvec(linspace(t0, tf, 2*ceil((tf-t0)/dt)));
        sig = resample(t_old, sig, t);
    end
    
    [b, g] = sgolay(order,framewidth);
    
    ybegin = b(end:-1:(framewidth+3)/2,:) * sig(framewidth:-1:1,:);
    yend = b((framewidth-1)/2:-1:1,:) * sig(end:-1:end-(framewidth-1),:);
    ycenter = cell2mat(arrayfun(@(i)conv(sig(:,i)',b((framewidth+1)/2,:),'valid'), 1:size(sig,2), 'UniformOutput', false)')';
    sigF = [ybegin; ycenter; yend];
    
    if nargout > 1
        dy = cell2mat(arrayfun(@(i)conv(sig(:,i), factorial(1)/(-(t(2)-t(1)))^1 * g(:,1+1), 'same'), 1:size(sig,2), 'UniformOutput', false));

        framewidth = modifyFramewidth(framewidth*2+1, sig, order);
        b = sgolay(order,framewidth);
        dybegin = b(end:-1:(framewidth+3)/2,:) * dy(framewidth:-1:1,:);
        dyend = b((framewidth-1)/2:-1:1,:) * dy(end:-1:end-(framewidth-1),:);
        dycenter = cell2mat(arrayfun(@(i)conv(dy(:,i)',b((framewidth+1)/2,:),'valid'), 1:size(dy,2), 'UniformOutput', false)')';
        dsigF = [dybegin; dycenter; dyend];
    end
    
    if ~is_regular_time_grid
        sigF = resample(t, sigF, t_old);
        if nargout > 1
            dsigF = resample(t, dsigF, t_old);
        end
    end
end

function framewidth = modifyFramewidth(framewidth, sig, order)
    framewidth_ub = floor((size(sig,1)-1)/2);
    framewidth_lb = order+1;
    if framewidth_ub < framewidth_lb
        framewidth = [];
        return;
    end
    framewidth = max(min(framewidth, framewidth_ub), framewidth_lb);
    if mod(framewidth,2) == 0
        if framewidth > framewidth_lb
            framewidth = framewidth-1;
        elseif framewidth < size(sig,1)-1
            framewidth = framewidth+1;
        else
            error('useSGolayFilter: Cannot choose an appropriate framewidth');
        end
    end
end

function [t, v] = fixDimensions(t, v, is_regular_time_grid)
    t = colvec(t);
    if size(v,1) ~= length(t)
        if size(v,1) == length(t)+1 && is_regular_time_grid && length(t) > 1
            t = [colvec(t); t(end) + t(2)-t(1)];
        else
            v = v';
        end
    end
    if size(v,1) ~= length(t)
        if size(v,1) == length(t)+1 && is_regular_time_grid && length(t) > 1
            t = [colvec(t); t(end) + t(2)-t(1)];
        else
            error('The dimension of time and signal must agree.');
        end
    end
end

function v_new = resample(t, v, t_new)
    pwf = struct('type', 1, 't0', t(1), 'n', size(v,2), 'tf', t(end), 'times', t, 'coefficients', v);
    v_new = evaluateDerivative(pwf, t_new, 0)';
end
