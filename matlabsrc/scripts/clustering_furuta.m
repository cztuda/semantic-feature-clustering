%% Perm0
measureTime = false;
doPlotAllClusters = false; 
reenableWarning = locallyDisableWarning('export_fig:exportgraphics');

path = fullfile(pwd(), '..', 'data', 'furuta');
outpath = createDir(pwd(), '..', 'data', 'furuta', 'output');
savename_trajs = 'Trajectories_perm0.mat';
savename_cluster = 'v0_clustering.mat';
method = 3;
[T3, D3, Z3, TCT, ind, problem, Trajs] = applyClustering(path, savename_trajs, savename_cluster, true, method, 0, 'measureTime', measureTime);
title('feature-based')
method = 0;
[T0, D0, Z0] = applyClustering(path, savename_trajs, savename_cluster, true, method, 0, 'measureTime', measureTime);
title('DTW')
save(fullfile(outpath, 'clustering_furuta_p0'), 'T3', 'D3', 'Z3', 'T0', 'D0', 'Z0', 'TCT', 'ind', 'problem', 'Trajs');

fprintf('feature-based:\n');
evaluateClustering(T3, ind);
fprintf('DTW:\n');
evaluateClustering(T0, ind);

close all;
plotAndSave(outpath, 'cluster_feature_furutaP0_dendr', Z3, true, ind, 3, 'logY', true, 'legend', true);
plotAndSave(outpath, 'cluster_DTW_furutaP0_dendr', Z0, true, ind, 3, 'logY', true);
if doPlotAllClusters
    plotAllClusters(Trajs, T3, outpath, 'cluster_feature_furutaP0_plot')
    % plotAllClusters(Trajs, T0, outpath, 'cluster_DTW_furutaP0_plot')
end

%% Perm1
measureTime = false;
doPlotAllClusters = false;
reenableWarning = locallyDisableWarning('export_fig:exportgraphics');
path = fullfile(pwd(), '..', 'data', 'furuta');
outpath = createDir(pwd(), '..', 'data', 'furuta', 'output');
savename_trajs = 'Trajectories_perm1.mat';
savename_cluster = 'v1_clustering.mat';
method = 3;
[T3, D3, Z3, TCT, ind, problem, Trajs] = applyClustering(path, savename_trajs, savename_cluster, true, method, 0, 'measureTime', measureTime);
title('feature-based')
method = 0;
[T0, D0, Z0] = applyClustering(path, savename_trajs, savename_cluster, true, method, 0, 'measureTime', measureTime);
title('DTW')
save(fullfile(outpath, 'clustering_furuta_p1'), 'T3', 'D3', 'Z3', 'T0', 'D0', 'Z0', 'TCT', 'ind', 'problem', 'Trajs');

fprintf('feature-based:\n');
evaluateClustering(T3, ind);
fprintf('DTW:\n');
evaluateClustering(T0, ind);

close all; 
plotAndSave(outpath, 'cluster_feature_furutaP1_dendr', Z3, true, ind, 4, 'logY', true, 'legend', true);
plotAndSave(outpath, 'cluster_DTW_furutaP1_dendr', Z0, true, ind, 7, 'logY', true);
if doPlotAllClusters
    plotAllClusters(Trajs, T3, outpath, 'cluster_feature_furutaP1_plot')
end

%% Perm2
measureTime = false;
doPlotAllClusters = false;
reenableWarning = locallyDisableWarning('export_fig:exportgraphics');
path = fullfile(pwd(), '..', 'data', 'furuta');
outpath = createDir(pwd(), '..', 'data', 'furuta', 'output');
savename_trajs = 'Trajectories_perm2.mat';
savename_cluster = 'v2_clustering.mat';
method = 3;
[T3, D3, Z3, TCT, ind, problem, Trajs] = applyClustering(path, savename_trajs, savename_cluster, true, method, 0, 'measureTime', measureTime);
title('feature-based')
method = 0;
[T0, D0, Z0] = applyClustering(path, savename_trajs, savename_cluster, true, method, 0, 'measureTime', measureTime);
title('DTW')
save(fullfile(outpath, 'clustering_furuta_p2'), 'T3', 'D3', 'Z3', 'T0', 'D0', 'Z0', 'TCT', 'ind', 'problem', 'Trajs');

fprintf('feature-based:\n');
evaluateClustering(T3, ind);
fprintf('DTW:\n');
evaluateClustering(T0, ind);

close all;
% plotAndSave(outpath, 'cluster_feature_furutaP2_dendr', Z3, true, ind, 4, 'logY', true);
plotAndSave(outpath, 'cluster_feature_furutaP2_dendr', Z3, true, ind, 3, 'logY', true, ...
    'userColoring', {{selectIndex(pycolors('tab10'), 4), selectIndex(pycolors('tab10'), 3)}}, 'legend', true);
plotAndSave(outpath, 'cluster_DTW_furutaP2_dendr', Z0, true, ind, 4, 'logY', true);
if doPlotAllClusters
    plotAllClusters(Trajs, T3, outpath, 'cluster_feature_furutaP2_plot')
    plotAllClusters(Trajs, T0, outpath, 'cluster_DTW_furutaP2_plot')
end


%% IN THE FOLLOWING: UTILITY FUNCTIONS

%% [T, D, TCT, ind, problem, Trajs] = applyClustering(path, save_t, save_c, doSort=true, method=3, binOffset=0, varargin)
% INPUT:
%   path: path to folder where the saves are
%   save_t: savename of trajcol and trajs 
%           OR {trajcol, Trajs} OR {problem, Trajs}
%   save_c: savename where ind describing the clustering is saved
%           OR ind
%
% ADDITIONAL ARGUMENTS:
%   TCT
%   D
%   save_times
function [T, D, Z, TCT, ind, problem, Trajs] = applyClustering(path, save_t, save_c, varargin)
    [doSort, method, binOffset, TCT, D, measureTime] = getFunctionArguments(varargin, {true, 3, 0}, 'TCT', [], 'D', [], 'measureTime', false);

    [problem, Trajs, ind] = getData(path, save_t, save_c);
    
    if doSort
        [Trajs, ind] = sortTrajectories(Trajs, ind);
    end
    nBin = length(unique(ind));
    
    if isempty(TCT)
        TCT = TrajCharTable.getDefaultTrajstructTCTHandle(problem, true, 'promFilter', 0.02);
    end
    args = {};
    if ~isempty(D)
        args = [args, 'D', D];
    end
    
    
    [T,~,~,D, Z] = clusterOcpSolutions(Trajs, 'method', method, 'plotDendro', true, 'how', 'nBins', 'howParam', nBin+binOffset, 'measureTime', measureTime, 'mArgs', {TCT}, args{:});

    if measureTime
        global measured_times_1 measured_times_2
        persistent counter;
        if isempty(counter)
            counter = 1;
        end
        if ~isfolder('/tmp/trajClustering')
            mkdir('/tmp/trajClustering');
        end
        save(['/tmp/trajClustering/measuredTimes_', num2str(counter, '%02i')], 'measured_times_1', 'measured_times_2');
    counter = counter + 1;
    end
end

function [problem, Trajs, ind] = getData(path, save_t, save_c)
    if ischar(save_t)
        s1 = load(fullfile(path, save_t), 'trajcol', 'Trajs');
        problem = s1.trajcol.problem;
        Trajs = s1.Trajs;
    else
        Trajs = save_t{2};
        if isstruct(save_t{1})
            problem = save_t{1}.problem;
        else
            problem = save_t{1};
        end
    end
    if ischar(save_c)
        s2 = load(fullfile(path, save_c), 'ind');
        ind = s2.ind;
    else
        ind = save_c;
    end
end

%% plotAndSave(outpath, filename, Z, allLeaves=true, groundTruth=[], lineColorTh=[], varargin)
% ADDITIONAL ARGUMENTS:
%   showNumbers:    (default=false) show numbers on the x-axis
%   logY:           (default=false) use logarithmic y-axis
%   legend:         (default=[]), true or cell array of legend names
function plotAndSave(outpath, filename, Z, varargin)
    [allLeaves, groundTruth, lineColorTh, shownumbers, logY, userColoring, lgnd] = getFunctionArguments(varargin, {true, [], []}, 'showNumbers', false, 'logY', false, 'userColoring', [], 'legend', []);

    plotDendrogram(Z, allLeaves, groundTruth, lineColorTh, 'showNumbers', shownumbers, 'logY', logY, 'userColoring', userColoring, 'legend', lgnd);
    
    if ~isempty(filename)
        export_fig(fullfile(outpath, filename), '-png', '-r600');
        savefig(fullfile(outpath, filename))
    end
end

function plotAllClusters(Trajs, ind, outfolder, filename, figOffset)
    nCluster = length(unique(ind));
    if nargin < 5
        figOffset = 100;
    end
    map(@(i)map(@(c)plotTrajectoryData(c, 'fig', figOffset+i, 'comparative', -1, 'splitState', true, 'objective', false, 'splitState', true), Trajs(ind==i)), 1:nCluster);
    
    for i = 1:nCluster
        figure(figOffset+i);
        title('');
        subplot(3,1,1);
        legend('off');
        subplot(3,1,2);
        legend('off');
        subplot(3,1,3);
        legend('off');
        export_fig(fullfile(outfolder, [filename, '_c', num2str(i)]), '-png', '-r600', '-white');
        savefig(fullfile(outfolder, [filename, '_c', num2str(i)]));
    end
end
