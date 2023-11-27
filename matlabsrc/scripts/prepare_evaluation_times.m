path = '/tmp/trajClustering';

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

plot_times(svr_prepT, 'SVRspell preparation');
plot_times(svr_times, 'SVRspell-based');
plot_times(dtw_times, 'DTW');

plotSvrDependency(svr_list);

%% ploting scripts:
function plotSvrDependency(svr_list, id)
    if nargin == 1
        L = mergeListCells(svr_list{1}, svr_list{2});
        L = mergeListCells(L, svr_list{3});
    else
        L = svr_list{id};
    end

    Vd = cellfun(@(c)mean(c), L);

    figure(); 
    b = bar3(Vd);
    for k = 1:length(b)
        zdata = b(k).ZData;
        b(k).CData = zdata;
        b(k).FaceColor = 'interp';
    end
    set(gca(), 'YTick', gca().YTick+1)
    set(gca(), 'XTick', gca().XTick+1)
    if gca().XTick(1) ~= 1, set(gca(), 'XTick', [1, gca().XTick]); end
    if gca().YTick(1) ~= 1, set(gca(), 'YTick', [1, gca().YTick]); end
    set(gca(), 'XTickLabel', arrayfun(@(u)num2str(u), str2double(gca().XTickLabel)-1, 'UniformOutput', false))
    set(gca(), 'YTickLabel', arrayfun(@(u)num2str(u), str2double(gca().YTickLabel)-1, 'UniformOutput', false))
    set(gca(), 'YLim', [find(arrayfun(@(i)any(~isnan(Vd(i,:))), 1:size(Vd,1) ), 1)-0.4, find(arrayfun(@(i)any(~isnan(Vd(i,:))), 1:size(Vd,1) ), 1, 'last')+1+0.4])
    set(gca(), 'XLim', [find(arrayfun(@(i)any(~isnan(Vd(:,i))), 1:size(Vd,2) ), 1)-0.4, find(arrayfun(@(i)any(~isnan(Vd(:,i))), 1:size(Vd,2) ), 1, 'last')+1+0.4])
    
    xlabel('Num features 1');
    ylabel('Num features 2');
    zlabel('Time ');
    view([-0.55;1.2;0.4]);
end

function plot_times(data, ttl)
    figure(); boxplot(cell2mat(data')');
    set(gca().XAxis, 'TickLabels', {'Furuta_1', 'Furuta_2', 'Furuta_3'});
    title(ttl)
    ylabel('time for distance measurement');
    set(gca().YAxis, 'Scale', 'log');
end


%% util

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
