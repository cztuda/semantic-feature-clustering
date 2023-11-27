path = fullfile(pwd(), '..', 'data', 'time_evaluation', 'measurements');
outfolder = createDir(pwd(), '..', 'data', 'time_evaluation', 'output');

% sfolders = subfolders(path);
sfolders = {'Furuta', 'Manutec', 'Humanoid'};
nn = length(sfolders);

dtw_times = cell(1,nn);
svr_prepT = cell(1,nn);
svr_times = cell(1,nn);
svr_pareto = cell(1,nn);
svr_list = cell(1,nn);

for i = 1:nn
    [dtw_times{i}, svr_prepT{i}, svr_times{i}, svr_pareto{i}, svr_list{i}] = compile_data_structures(fullfile(path, sfolders{i}));
end

plot_times(svr_prepT, sfolders, 'SVRspell preparation');
plot_times(svr_times, sfolders, 'SVRspell-based');
plot_times(dtw_times, sfolders, 'DTW');

plot_all_times(svr_prepT, svr_times, dtw_times, sfolders, outfolder);
plotSvrDependency(svr_list{1}, [], outfolder);


%% main routine
function [dtw_times, svr_prepT, svr_times, svr_pareto, svr_list] = compile_data_structures(path)
nn = length(find(map(@(n)~isempty(regexp(n.name, '.*.mat', 'once')), dir(path))))/2;

dtw_times = cell(1,nn);
svr_prepT = cell(1,nn);
svr_times = cell(1,nn);
svr_pareto = cell(1,nn);

svr_list = cell(1,3);

ptr_dtw = 1; ptr_svrspell = 1;
for i = 1:(2*nn)
    tmp = load(fullfile(path, ['measuredTimes_', num2str(i, '%02i'), '.mat']));
    measured_times_1 = tmp.measured_times_1;
    measured_times_2 = tmp.measured_times_2;
    
    if isempty(measured_times_1) % dtw case
        dtw_times{ptr_dtw} = measured_times_2;
        ptr_dtw = ptr_dtw+1;
    else % svrspell case
        svr_prepT{ptr_svrspell} = measured_times_1;
        svr_times{ptr_svrspell} = measured_times_2{1};
        svr_pareto{ptr_svrspell} = measured_times_2{2};
        
        m = max(svr_pareto{ptr_svrspell}, [], 'all')+1;
        V = cell(m);
        for j = 1:length(measured_times_2{1})
            V{svr_pareto{ptr_svrspell}(1,j)+1, svr_pareto{ptr_svrspell}(2,j)+1} = [V{svr_pareto{ptr_svrspell}(1,j)+1, svr_pareto{ptr_svrspell}(2,j)+1}, svr_times{ptr_svrspell}(1,j)]; 
        end
        svr_list{ptr_svrspell} = V;
        ptr_svrspell = ptr_svrspell+1;
    end
end

end

%% ploting scripts:
function plotSvrDependency(svr_list, id, outfolder)
    if nargin == 1 || isempty(id)
        L = mergeListCells(svr_list{1}, svr_list{2});
        L = mergeListCells(L, svr_list{3});
    else
        L = svr_list{id};
    end

    Vd = cellfun(@(c)mean(c), L);

    fig = figure(); 
    fig.Color = [1,1,1];
    b = bar3(Vd);
    for k = 1:length(b)
        zdata = b(k).ZData;
        b(k).CData = zdata;
        b(k).FaceColor = 'interp';
    end
    
    function correctTicks()
        set(gca(), 'YTick', gca().YTick+1)
        set(gca(), 'XTick', gca().XTick+1)
        if gca().XTick(1) ~= 1, set(gca(), 'XTick', [1, gca().XTick]); end
        if gca().YTick(1) ~= 1, set(gca(), 'YTick', [1, gca().YTick]); end
        set(gca(), 'XTickLabel', arrayfun(@(u)num2str(u), str2double(gca().XTickLabel)-1, 'UniformOutput', false))
        set(gca(), 'YTickLabel', arrayfun(@(u)num2str(u), str2double(gca().YTickLabel)-1, 'UniformOutput', false))        
    end
    
    correctTicks();
    ylimvals = [find(arrayfun(@(i)any(~isnan(Vd(i,:))), 1:size(Vd,1) ), 1)-0.4, find(arrayfun(@(i)any(~isnan(Vd(i,:))), 1:size(Vd,1) ), 1, 'last')+1+0.4];
    xlimvals = [find(arrayfun(@(i)any(~isnan(Vd(:,i))), 1:size(Vd,2) ), 1)-0.4, find(arrayfun(@(i)any(~isnan(Vd(:,i))), 1:size(Vd,2) ), 1, 'last')+1+0.4];
    set(gca(), 'YLim', ylimvals);
    set(gca(), 'XLim', xlimvals);
    
%     xlabel('Num features 1');
%     ylabel('Num features 2');
    zlabel('Time');
    view(-140, 20);
    if nargin > 2 && ~isempty(outfolder)
        export_fig(fullfile(outfolder, 'svr_dependency'), '-png', '-svg', '-r600', '-q101', '-silent');
        savefig(fullfile(outfolder, 'svr_dependency'));
    end
    
    fig = figure(); 
    fig.Color = [1,1,1];    
    imagesc(Vd);
    set(gca(), 'YDir', 'normal')
    correctTicks();
    set(gca(), 'YLim', ylimvals-[0,1]);
    set(gca(), 'XLim', xlimvals-[0,1]);
    colormap(flipud(fblue));
    h = colorbar();
    set(gca().Children(1), 'AlphaData', 1-isnan(Vd));
    h.Label.String = 'Time [s]';
    xlabel('n\_features\_1');
    ylabel('n\_features\_2');
    if nargin > 2 && ~isempty(outfolder)
        export_fig(fullfile(outfolder, 'svr_dependency_flat'), '-png', '-svg', '-r600', '-q101', '-silent');
        savefig(fullfile(outfolder, 'svr_dependency_flat'));
    end
end

function plot_times(data, names, ttl)
    lbls = cellfun(@(name, data)ifelse(length(data)>1, arrayfun(@(i)sprintf('%s\\_%i', name, i), 1:length(data), 'UniformOutput', false), name), names, data, 'UniformOutput', false);
    lbls = [lbls{:}];
    
    d2 = cellfun(@(c)c', [data{:}], 'UniformOutput', false);
    nmax = max(cellfun(@length, d2));
    bpdata = map(@(c)[c; nan(nmax-length(c),1)], d2);
    
    fig = figure(); 
    fig.Color = [1,1,1]; 
    iosr.statistics.boxPlot(lbls, bpdata);
    
%     set(gca().XAxis, 'TickLabels', lbls);
    title(ttl);
    grid on;
    ylabel('time [s]');
    set(gca().YAxis, 'Scale', 'log');
end

function plot_all_times(svr_p, svr_t, dtw_t, sfolders, outfolder)
    lbls = cellfun(@(name, data)ifelse(length(data)>1, arrayfun(@(i)sprintf('%s\\_%i', name, i), 1:length(data), 'UniformOutput', false), name), sfolders, svr_p, 'UniformOutput', false);
    lbls = [lbls{:}];
    lbls = strrep(lbls, 'Humanoid', 'Hum. Mot. 1');
    
    svr_p = cellfun(@(c)c', [svr_p{:}], 'UniformOutput', false);
    svr_t = cellfun(@(c)c', [svr_t{:}], 'UniformOutput', false);
    dtw_t = cellfun(@(c)c', [dtw_t{:}], 'UniformOutput', false);
    
    nmax = max([cellfun(@length, svr_p), cellfun(@length, svr_t), cellfun(@length, dtw_t)]);
    
    bp_svr_p = map(@(c)[c; nan(nmax-length(c),1)], svr_p);
    bp_svr_t = map(@(c)[c; nan(nmax-length(c),1)], svr_t);
    bp_dtw_t = map(@(c)[c; nan(nmax-length(c),1)], dtw_t);
    bp_data = cat(3, bp_svr_p, bp_svr_t, bp_dtw_t);
    
    fig = figure(); 
    fig.Color = [1,1,1];
    colors = {[0.8500 0.3250 0.0980], [0.6350 0.0780 0.1840], [0 0.4470 0.7410]};
    
    iosr.statistics.boxPlot(lbls, bp_data, ...
        'groupLabels', {'feature-based [prep]','feature-based [SVRspell]','DTW [dist]'}, ...
        'theme', 'colorall', ...
        'boxcolor', colors, ...
        'symbolColor', colors, ...
        'xSeparator', true, ...
        'showScatter', false, ...
        'scatterAlpha', 0.2, ...
        'boxAlpha', 0.8, ...
        'symbolMarker', 'x', ...
        'showLegend', true, ...
        'method', 'R-5');
    set(gca().YAxis, 'Scale', 'log');
    correctLegend();
    xSepInd = find(arrayfun(@(c)isprop(c, 'XData') && length(c.XData) == 2 && c.XData(1) == c.XData(2) && c.YData(1)==0, fig.Children(2).Children));
    xSepDataBouns = ylim();
    for i = 1:length(xSepInd)
        fig.Children(2).Children(xSepInd(i)).YData = xSepDataBouns;
    end
    grid on;
    ylabel('Time [s]');
    
    if nargin > 4 && ~isempty(outfolder)
        export_fig(fullfile(outfolder, 'all_times'), '-png', '-svg', '-r600', '-q101', '-silent');
        savefig(fullfile(outfolder, 'all_times'));
    end
    
    ylim([0.0006, 0.35]);
    figPos = gcf().Position;
    figPos(4) = 360;
    set(gcf(), 'Position', figPos);
    if nargin > 4 && ~isempty(outfolder)
        export_fig(fullfile(outfolder, 'all_times_2'), '-png', '-svg', '-r600', '-q101', '-silent');
        savefig(fullfile(outfolder, 'all_times_2'));
    end
end

%% util

function correctLegend()
%     fig.Children(1).Location = 'northeast';
    set(gcf().Children(1), 'Location', 'none');
    legendpos = gcf().Children(1).Position;
    legendpos(1) = 0.23;
    legendpos(2) = 0.79;
    set(gcf().Children(1), 'Position', legendpos);
end

function L = mergeListCells(L1, L2)
    m = max(length(L1), length(L2));
    if length(L1) < m
        L = cell(m);
        L(1:length(L1), 1:length(L1)) = L1;
        L1 = L;
    end
    if length(L2) < m
        L = cell(m);
        L(1:length(L2), 1:length(L2)) = L2;
        L2 = L;
    end
    L = cellfun(@(c1, c2)[c1, c2], L1, L2, 'UniformOutput', false);    
end
