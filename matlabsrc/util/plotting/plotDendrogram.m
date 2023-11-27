%% plotAndSave(Z, allLeaves=true, groundTruth=[], lineColorTh=[], varargin)
%
% INPUT:
%   Z:              values
%   allLeaves:      plot all leaves of the dendrogram
%   groundTruth:    vector of ground truth data (same numbers are the same cluster)
%   lineColorTh:    if set to integer to color clusters, it gives the maximum number of clusters that are colored 
% 
% ADDITIONAL ARGUMENTS:
%   showNumbers:    (default=false) show numbers on the x-axis
%   logY:           (default=false) use logarithmic y-axis
%   correctLineColors:  
%                   (default=true)
%   colors:         (default=[]) which colors to use, default are the python colors tab10
%   nColors:        (default=0) if >0, restrict the number of clusters that is colored
%   userColoring:   (default=[]), cell where the user can replace colors and select color of single lines 
%                     {{fromColor, toColor}, ...}
%                     fromColor (1x3 or 1x1) is either a color to be replaced or a row to be colored
%                     toColor (1x3) is the new color
%   legend:         (default=[]) true or cell array of labels
function plotDendrogram(Z, varargin)
    [allLeaves, groundTruth, lineColorTh, shownumbers, logY, correctLineColors, ncolors, colors, userColoring, lgnd] = getFunctionArguments(varargin, ...
        {true, [], []}, 'showNumbers', false, 'logY', false, 'correctLineColors', true, 'nColors', 0, 'colors', [], 'userColoring', [], 'legend', []);

    fig = figure(); fig.Color = [1,1,1];
    if ~isempty(lineColorTh)
        cutoff = median([Z(end-lineColorTh+1,3) Z(end-lineColorTh+2,3)]);
        if allLeaves
            dendrogram(Z, 10000, 'ColorThreshold', cutoff)
        else
            dendrogram(Z, 'ColorThreshold', cutoff);
        end
    else
        if allLeaves
            dendrogram(Z, 10000);
        else
            dendrogram(Z);
        end
    end
    if ~isempty(groundTruth)
        colorcodeXLabel(groundTruth, 'showNumbers', shownumbers, 'nColors', ncolors, 'colors', colors, 'legend', lgnd);
    elseif ~shownumbers
        xticklabels([]); 
    end
    yticklabels([]);
    if correctLineColors
        correctDendrogramColoring(userColoring);
    end
    if logY
        makeDendrogramLogarithmic();
    end
    if ~isempty(groundTruth) && ~shownumbers
        correctBoxSizes()
    end
    makeLinesThicker()
end


function correctBoxSizes()
    txt = gca().Children();
    txt = txt(map(@(v)strcmp(v.Type, 'text'), txt));
    hold on
    for i = 1:length(txt)
        c = txt(i).Position(1);
        ext = txt(i).Extent;
        x = [c-0.5, c-0.5, c+0.5, c+0.5, c-0.5];
        y = [ext(2), ext(2)+ext(4)];
        y = [y(1), y(2), y(2), y(1), y(1)];
        col = txt(i).BackgroundColor;
        patch(x, y, col, 'Clipping', 'off', 'EdgeAlpha', 0);
        delete(txt(i));
    end
end


function makeLinesThicker()
    lines = gca().Children();
    lines = lines(map(@(v)strcmp(v.Type, 'line'), lines));
    map(@(l)set(l, 'LineWidth', 1.3), lines);
end