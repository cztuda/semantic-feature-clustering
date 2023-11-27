%% makeDendrogramLogarithmic(fig=[])
% Set the scale of the y-axis of the dendrogram to logarithmic. This requires special treatment as
% the vertical lines in the dendrogram go to zero, which causes problems in a logarithmic scale. For
% this reason, the vertical lines are cutted at some value (1/2*min(XData)) and the y-limits are set
% to this value to make the dendrogram look properly with logarithmic scale.
%
function makeDendrogramLogarithmic(fig)
    if nargin > 0
        if isnumeric(fig)
            hf = figure(fig);
        else
            hf = fig;
            figure(fig.Number); % gain focus
        end
    else
        hf = gcf();
    end
    ha = gca();

    lines = ha.Children();
    lines = lines(map(@(v)~strcmp(v.Type, 'patch'), lines));
    txt = lines(map(@(v)strcmp(v.Type, 'text'), lines));
    lines = lines(map(@(v)strcmp(v.Type, 'line'), lines));
    
    values = map(@(L)L.YData, lines');
    ylimmin = min(values(values>0))/2;
    
    for i = 1:length(lines)
        set(lines(i), 'YData', map(@(v)max(v, ylimmin), lines(i).YData));
    end
    
    ylim([ylimmin, selectIndex(ylim(),2)]);
    set(ha.YAxis, 'Scale', 'log');
    
    for i = 1:length(txt)
        txt(i).Position(2) = ylimmin;
    end
end