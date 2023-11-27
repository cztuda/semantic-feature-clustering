%% [chptsList, salience] = findChangepointsInTimeSeries(time, values, winLen=2000)
% Find points in a time series where fast changes occur.
% The time series is given as tuple of time and values.
%
% INPUT:
%   time:       [1xk] vector, the times that correspond to the respective values, k is the number of gridpoints 
%   values:     [nxk] matrix, the values that correspond to the respective time, n is the dimension of the time series 
%   winLen:     (optional) width of the window (as indices) that is used to look for changes
% 
% OUTPUT:
%   chptsList:
%   salience:
%
function [chptsList, salience] = findChangepointsInTimeSeries(time, values, varargin)
    [winLen] = getFunctionArguments(varargin, {2000});
    if mod(winLen,2)
        winLen = winLen-1;
    end
    
    if size(values,2) ~= length(time)
        values = values';
    end
    ndim = size(values,1);
    chpts = zeros(size(values));
    
    for i = 1:ndim
        chpts(i,:) = changepointDetection(values(i,:), winLen, @(d)sqrt(sum(d.*d)));
    end
    [~, trajdata] = createPiecewiseLinear(time, chpts);

    [T_e, Te_isMax, Te_isMin] = findExtremaInLinear(trajdata, [], 'filterExtrema', false, 'sepDims', true);
    Te_prom = computeProminence(T_e, Te_isMax, trajdata, true, 'isMinimum', Te_isMin);
    
    D = max(values, [], 2)-min(values, [], 2);
    chptsList = cell(1, ndim);
    salience = cell(1, ndim);
    for i = 1:ndim
        chpts_start = T_e{i}(Te_prom{i}>0.5 & Te_isMax{i}');
        chptI = zeros(1, length(chpts_start));
        salienceI = zeros(1, length(chpts_start));
        for j = 1:length(chpts_start)
            [ichpt, d] = getChptDataIdx(find(time==chpts_start(j),1), winLen, values(i,:));
            chptI(j) = time(ichpt);
            salienceI(j) = d/D(i);
        end
        chptsList{i} = chptI; 
        salience{i} = salienceI;
    end
end

function [i_mid, d, i_start, i_end] = getChptDataIdx(i_start, winLen, data)
    sd_start = max(i_start-winLen/2, 1);
    sd_end = min(i_start+winLen/2, length(data));
    
    s = piecewiseLinearFit(sd_start:sd_end, data(sd_start:sd_end), 2);
    i_start = s(1);
    i_end = s(2);
    d =  abs(data(sd_start)-data(sd_end));
    i_mid = round((i_end+i_start)/2);
end
