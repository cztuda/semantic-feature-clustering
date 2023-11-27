
L = load(fullfile(pwd(), '..', 'data', 'manutec', 'general.mat'), 'problem', 'Trajs');
outpath = createDir(pwd(), '..', 'data', 'manutec', 'output');
problem = L.problem;
Trajs = L.Trajs;
measureTime = true;

manualClustering = {sort([14,17,20,23,26,29,1,10,7,4]), sort([12,13,16,19,22,25]), sort([8, 11,27,30,2]), [21,28], 9, [5,6], [15, 24], 3, 18};
[Trajs_sorted, ind_sorted] = sortTrajectories(Trajs', manualClustering);

nCluster = length(unique(ind_sorted));
clusterLengths=map(@(i)length(find(ind_sorted == i)), 1:nCluster);
clusterStarts = cumsum([1, clusterLengths]);
ind = map(@(i)i*ones(clusterLengths(i),1), (1:length(clusterLengths))');

fprintf('cluster lengths:\n  %s\n', mat2str(clusterLengths));
fprintf('cluster starts:\n  %s\n', mat2str(clusterStarts));

% remove roots from the control features
TCT = TrajCharTable.getDefaultTrajstructTCTHandle(problem, true, 'promFilter', 0.02);
list = TCT.TCList(2);
list = list(2);
TCT = TCT.TCList(2, list);

if measureTime
    doClustering = @clusterOcpSolutions_measureTime;
else
    doClustering = @clusterOcpSolutions;
end

[T3,~,~,D3, Z3] = doClustering(Trajs_sorted, 'method', 3, 'plotDendro', false, 'how', 'nBins', 'howParam', nCluster, 'mArgs', {TCT});
plotAndSave(outpath, 'cluster_feature_manutec_dendr', Z3, true, ind, 11, 'logY', true, 'ncolors', 3, 'legend', true);

[T0,~,~,D0, Z0] = doClustering(Trajs_sorted, 'method', 0, 'plotDendro', false, 'how', 'nBins', 'howParam', nCluster, 'mArgs', {TCT});
% plotAndSave(outpath, 'cluster_DTW_manutec_dendr', Z0, true);
plotAndSave(outpath, 'cluster_DTW_manutec_dendr', Z0, true, ind, 10, 'logY', true, 'ncolors', 3);

fprintf('\nfeature-based-based:\n');
evaluateClustering(T3, ind_sorted);
fprintf('DTW-based:\n');
evaluateClustering(T0, ind_sorted);

save(fullfile(outpath, 'clustering_manutec_results'));

close all;
plotAllClusters(Trajs_sorted, ind_sorted, outpath, 'cluster_gt_manutec_plot', 100);
close all;
plotAllClusters(Trajs_sorted, T3, outpath, 'cluster_feature_manutec_plot', 200);
close all;
% plotAllClusters(Trajs_sorted, T0, outpath, 'cluster_DTW_manutec_plot', 300);
close all;


function plotAndSave(outpath, filename, Z, varargin)
    [allLeaves, groundTruth, lineColorTh, shownumbers, logY, ncolors, lgnd] = getFunctionArguments(varargin, {true, [], []}, 'showNumbers', false, 'logY', false, 'ncolors', 0, 'legend', []);

    plotDendrogram(Z, allLeaves, groundTruth, lineColorTh, 'showNumbers', shownumbers, 'logY', logY, 'nColors', ncolors, 'legend', lgnd);
    
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
        fig = figure(figOffset+i);
        title('');
        subplot(3,1,1);
        legend('off');
        subplot(3,1,2);
        legend('off');
        subplot(3,1,3);
        legend('off');
        fig.Color = [1,1,1];
        export_fig(fullfile(outfolder, [filename, '_c', num2str(i)]), '-png', '-r600', fig);
        savefig(fig, fullfile(outfolder, [filename, '_c', num2str(i)]));
    end
end