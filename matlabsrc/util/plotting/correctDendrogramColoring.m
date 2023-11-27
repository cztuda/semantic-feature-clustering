%% correctDendrogramColoring(manualReplacement)
% Correct the diagram coloring to fit the ground truth color. Note that not all cases are
% implemented/covered, such that manual corrections may be necessary to achieve the desired result.
% Manual corrections can be done using the optional input argument 'manualReplacement'
%
% INPUT:
%   manualReplacement (default=[]) cell where the user can replace colors and select color of single 
%                        lines. The input format is
%                           {{fromColor, toColor}, ...}
%                        fromColor (1x3 or 1x1) is either a color to be replaced or a row to be colored
%                        toColor (1x3) is the new color
%
function correctDendrogramColoring(manualReplacement)
    if nargin < 1
        manualReplacement = [];
    end
    ha = gca();

    lines = ha.Children();
    lines = lines(map(@(v)~strcmp(v.Type, 'patch'), lines));
    txt = lines(map(@(v)strcmp(v.Type, 'text'), lines));
    lines = lines(map(@(v)strcmp(v.Type, 'line'), lines));
    
    boxColors = map(@(v)v.BackgroundColor, txt);
    gtColors = unique(boxColors, 'rows');
    if ~isempty(gtColors)
        gtColors = gtColors(~all(gtColors==[1,1,1],2),:);
    end
    gtIndices = arrayfun(@(gtk)find(map(@(i)all(boxColors(i,:)== gtColors(gtk,:)), 1:size(boxColors,1))), 1:size(gtColors,1), 'UniformOutput', false);
    gtXIndices = arrayfun(@(k) arrayfun(@(i)txt(i).Position(1), gtIndices{k}), 1:length(gtIndices), 'UniformOutput', false);
    gtBoxXIndices = arrayfun(@(i)txt(i).Position(1), 1:length(txt));
    
    lineColors = map(@(v)v.Color, lines);
    allLineColors = unique(lineColors, 'rows');
    xIndex2lineIndex = getLineAssignements(lines); % xi2li(k) gives the line index that is associated with the k-th x-axis value 
    
    
    bestFittingMatchPerGT = arrayfun(@(i)getBestColorPerGTC(gtXIndices{i}, xIndex2lineIndex, lines), 1:length(gtIndices), 'UniformOutput', false);
%     [bestFittingMatchPerGT, manualReplacement] = replaceMatches(bestFittingMatchPerGT, manualReplacement);
    if any(map(@(v)size(v,1), bestFittingMatchPerGT)> 1)
        % do next steps
        warning('correctDendrogramColoring: Case not yet implemented');
%     elseif checkForDoubles(bestFittingMatchPerGT)
%         warning('correctDendrogramColoring: Case not yet implemented');
    else
        sel = map(@(v)~isempty(v), bestFittingMatchPerGT);
        replaceColor(lines, cell2mat(colvec(bestFittingMatchPerGT(sel))), gtColors(sel,:));
        colorSingleObjectClusters(lines, boxColors, gtColors, gtBoxXIndices, xIndex2lineIndex);
        manualReplacement = replaceColorsAsGiven(lines, manualReplacement);
        replaceColorsByLine(lines, xIndex2lineIndex, manualReplacement);
    end
end

function detailedComparison1(BFM)
    nBestMatches = map(@(v)size(v,1), BFM);
    problemIndices = find(nBestMatches>1);
    for i = 1:length(problemIndices)
        isAnyNotInUse = false;
        for k = 1:size(BFM{problemIndices(i)},1)
            inUse = colorIsInUse(BFM, BFM{problemIndices(i)}(k,:), problemIndices(i));
            if any(inUse)
                
            else
                isAnyNotInUse = true;
            end
        end
    end
end

function isDoubles = checkForDoubles(bFMPGT)
    A = map(@(i)map2(@(c)ifelse(isempty(c)||isempty(bestFittingMatchPerGT{i}), false, all(bestFittingMatchPerGT{i}==c,2)), bestFittingMatchPerGT((1:length(bestFittingMatchPerGT))~=i)), (1:length(bestFittingMatchPerGT))');
    isDoubles = any(A, 'all');
end
function inUse = colorIsInUse(bFMPGT, color, excludeIndx)
    inUse = false(1, length(bFMPGT));
    for i = 1:length(bFMPGT)
        if i ~= excludeIndx && ~isempty(bFMPGT{i})
            inUse(i) = any(all(bFMPGT{i}==color,2));
        end
    end
end

function [bestColor, mx] = getBestColorPerGTC(gtIndices, xIndex2lineIndex, lines)
    clusterColor = map(@(li)lines(li).Color, colvec(xIndex2lineIndex(gtIndices)));
    [unque, cnt] = countUnique(clusterColor);
%     if ~(length(cnt) == 1 && cnt == 1 && length(gtIndices) == 1)
        cnt(all(unque==[0,0,0],2)) = 0;% filter black as this is not a full cluster
%     end
    mx = max(cnt);
    bestColor = unque(cnt==mx&cnt>0,:);
end

function xIndex2lineIndex = getLineAssignements(lines)
    bl = getBottomLine(lines);
    xIndex2lineIndex = zeros(1, getXMax(lines));
    
    for i = 1:length(lines)
        sel = (lines(i).XData-round(lines(i).XData))==0 & lines(i).YData==bl;
        if any(sel)
            x = lines(i).XData(sel);
            xIndex2lineIndex(x) = i;
        end
    end    
end

function colorSingleObjectClusters(lines, boxColors, gtColors, gtBoxXIndices, xIndex2lineIndex)
    cnt = arrayfun(@(i)sum(all(boxColors==gtColors(i,:),2)), 1:size(gtColors,1));
    for i = 1:length(cnt)
        if cnt(i) == 1
            xpos = gtBoxXIndices(find(all(boxColors==gtColors(i,:),2),1));
            li = xIndex2lineIndex(xpos);
            if any(diff(lines(li).XData)) && length(lines(li).XData) > 2
                % split line and color only the part that goes straight down
                data = [lines(li).XData; lines(li).YData];
                if data(1,1) == xpos
                    sel1 = 1:2;
                    sel2 = 2:size(data,2);
                elseif data(1,end) == xpos
                    sel1 = size(data,2)-1:size(data,2);
                    sel2 = 1:size(data,2)-1;
                else
                    continue; % this should not happen
                end
                reenableWarning = locallyDisableWarning('MATLAB:gui:array:InvalidArrayShape'); %#ok<NASGU>
                set(lines(li), 'XData', data(1,sel1))
                set(lines(li), 'YData', data(2,sel1))
                set(lines(li), 'Color', gtColors(i,:))
                line(data(1,sel2), data(2,sel2), 'Color', [0,0,0]);
            else
                set(lines(li), 'Color', gtColors(i,:));
            end
        end
    end
end

function xMax = getXMax(lines)
    values = map(@(L)L.XData, lines');
    xMax = max(values);
end

function bl = getBottomLine(lines)
    values = map(@(L)L.YData, lines');
    bl = min(values);
end

function [unque, cnt] = countUnique(vec)
    unque = unique(vec, 'rows');
    cnt = arrayfun(@(i)sum(all(vec==unque(i,:),2)), 1:size(unque,1));
end

function replaceColor(lines, fromColors, toColors)
    if size(fromColors,1) ~= size(toColors,1)
        error('Length of given color cells must be the same.');
    end
    indices = arrayfun(@(k) find(map(@(i)all(lines(i).Color==fromColors(k,:)), 1:length(lines))), 1:size(fromColors,1), 'UniformOutput', false);
    
    for k = 1:length(indices)
        for i = 1:length(indices{k})
            set(lines(indices{k}(i)), 'Color', toColors(k,:));
        end
    end
end

function [bestFittingMatchPerGT, manualReplacement] = replaceMatches(bestFittingMatchPerGT, manualReplacement)
    if ~isempty(manualReplacement)
        colorEntries = find(map(@(c)length(c{1})>1, manualReplacement));
        MR = manualReplacement(colorEntries);
        sel = true(size(MR));
        for i = 1:length(bestFittingMatchPerGT)
            if size(bestFittingMatchPerGT{i},1)==1
                res = map(@(c)all(bestFittingMatchPerGT{i}==c{1},2), MR);
                if any(res)
                    bestFittingMatchPerGT{i} = MR{find(res,1)}{2};
                    sel(find(res,1)) = false;
                end
            end
        end
        allSel = true(size(manualReplacement));
        allSel(colorEntries) = sel;
        manualReplacement = manualReplacement(allSel);
    end
end

function manualReplacement = replaceColorsAsGiven(lines, manualReplacement)
    sel = true(size(manualReplacement));
    for i = 1:length(manualReplacement)
        if length(manualReplacement{i}{1})>1
            replaceColor(lines, manualReplacement{i}{1}, manualReplacement{i}{2});
            sel(i) = false;
        end
    end
    manualReplacement = manualReplacement(sel);
end

function replaceColorsByLine(lines, xIndex2lineIndex, manualReplacement)
    for i = 1:length(manualReplacement)
        set(lines(xIndex2lineIndex(manualReplacement{i}{1})), 'Color', manualReplacement{i}{2});
    end
end
