%% colorcodeXLabel(groundTruth, varargin)
% Colorcode the ground truth in a dendrogram as colored boxes on the x-axis.
%
% ADDITIONAL ARGUMENTS:
%   showNumbers:    (default=true) show numbers on the x-axis
%   nColors:        (default=0) if >0, restrict the number of clusters that is colored
%   colors:         (default=[]) the colors to use, if emtpy, the python colors are used
%   legend:         (default=[]) true or cell array of labels
function colorcodeXLabel(groundTruth, varargin)
    [showNumbers, nColors, colors, lgndlabels] = getFunctionArguments(varargin, ...
        'showNumbers', true, 'nColors', 0, 'colors', [], 'legend', []);
    ax = gca();
    labels = ax.XTickLabel;
    labels = mat2cell(labels, ones(1, size(labels,1)), size(labels,2));
    numLabels = str2double(labels);
    ticks = gca().XTick;
    
    if isempty(colors)
        colors = pycolors('tab10');
    end
    classes = unique(groundTruth);
    if nColors > 0
        colors((nColors+1):end,:) = repmat([1,1,1], size(colors,1)-nColors, 1);
        classes = sortBySize(classes, groundTruth);
    end

    th = cell(1, length(classes));
    for i = 1:length(classes)
        classInd = find(groundTruth == classes(i));
        sel = findIn(numLabels, classInd);
        if showNumbers
            thetext = labels(sel);
        else
            thetext = '  ';
        end
        if ax.YLim(1) == 0
            th{i} = text(ticks(sel), 0*ones(1, sum(sel)), thetext, ...
                'BackgroundColor', colors(i,:), 'FontSize', ax.FontSize, ...
                'Horiz', 'center', 'Vert', 'top', 'Margin', 1);
        else
            th{i} = text(ticks(sel), (ax.YLim(1)-diff(ax.YLim)*0.01)*ones(1, sum(sel)), thetext, ...
                'BackgroundColor', colors(i,:), 'FontSize', ax.FontSize, ...
                'Horiz', 'center', 'Vert', 'top', 'Margin', 1);
        end
    end
    if ~isempty(lgndlabels)
        if length(lgndlabels)==1 && islogical(lgndlabels) && lgndlabels
            legendlabels = arrayfun(@(i)['Cluster ', num2str(i)], 1:length(classes), 'UniformOutput', false);
            sel = arrayfun(@(i)~all(colors(i,:)==[1,1,1]), 1:min(length(legendlabels),size(colors,1)));
            createLegend(legendlabels(sel), colors, classes);
        else
            createLegend(lgndlabels, colors, classes);
        end
    end
    set(ax,'xticklabel',{[]});
end

function createLegend(legendlabels, colors, classes)
    children = gca().Children;
    map(@(i)set(children(i), 'HandleVisibility', 'off'), 1:length(children));
    resetVisibility = onCleanup(@()map(@(i)set(children(i), 'HandleVisibility', 'on'), 1:length(children)));
    hold on;
    for i = 1:length(classes)
        if ~all(colors(i,:)==[1,1,1])
            patch(nan, nan, colors(i,:));
        end
    end
    lg = legend(legendlabels{:});
    set(lg, 'AutoUpdate', 'off');
    set(lg, 'Location', 'best');
end

function classes = sortBySize(classes, groundTruth)
    lens = arrayfun(@(id)sum(groundTruth==id), classes);
    classes = classes(secondOutValue(@sort, lens, 'descend'));
end

function selection = findIn(whereToFind, whatToFind)
    selection = false(size(whereToFind));
    for i = 1:length(whatToFind)
        selection = selection | whereToFind==whatToFind(i);
    end
end