%% plotMyPeaks(handle, x, y, isMaximum, roots)
% Plot markers at defined positions to show peaks
% Based on the internal Matlab function plotpkmarkers and findpeaks.plotSignalWithPeaks, but with 
% extended functionality. My function can be used 
% - for multiple lines and multiple axes
% - to denote also minimas
% - to denote roots with a '+'
% - to denote changepoints with a 'o'
%
function plotPeakAndRootMarker(handle, x, y, isMaximum, roots, lmts, yLmts, xChpt, yChpt)
    if isstruct(x)
        [x, y, isMaximum, roots, lmts, yLmts, xChpt, yChpt] = extractData(x);
    else
        if nargin < 4 || isempty(isMaximum)
            isMaximum = getDefaultEmpty(iscell(x)*length(x));
        end
        if nargin < 5 || isempty(roots)
            roots = getDefaultEmpty(iscell(x)*length(x));
        end
        if nargin < 6 || isempty(lmts)
            lmts = getDefaultEmpty(iscell(x)*length(x));
        end
        
        if nargin < 7 || isempty(yLmts)
            yLmts = getDefaultEmpty(iscell(x)*length(x));
        end          
        if nargin < 8 || isempty(xChpt)
            xChpt = getDefaultEmpty(iscell(x)*length(x));
        end        
        if nargin < 9 || isempty(yChpt)
            yChpt = getDefaultEmpty(iscell(x)*length(x));
        end  
    end
    if isempty(x)
        return;
    end
    
    if strcmp(get(handle, 'Type'), 'figure')
        axes = findall(handle, 'Type', 'axes');
        if length(axes) > 1
            arrayfun(@(i)plotPeakAndRootMarker(axes(i), x{i}, y{i}, isMaximum{i}, roots{i}, lmts{i}, yLmts{i}, xChpt{i}, yChpt{i}), 1:length(axes));
            return;
        elseif length(axes) == 1
            handle = axes;
        end
    end    
        
    if strcmp(get(handle, 'Type'), 'axes')
        % handle is an axes object
        handles = mat2cells(findall(handle, 'Type', 'line'));
        if length(handles) > 1 || iscell(x)
            hH = cell(size(handles));
            hL = hH;
            n = length(handles);
            if length(x) == 1 && iscell(x) && iscell(x{1}) && length(x{1}) == n
                x = x{1};
                y = y{1};
                isMaximum = isMaximum{1};
                roots = roots{1}; 
                lmts = lmts{1};
                yLmts = yLmts{1};
                xChpt = xChpt{1};
                yChpt = yChpt{1};
            end
%             if isempty(isMaximum)
%                 isMaximum = cellfun(@(c)true(1, length(c)), y, 'UniformOutput', false);
%             end
            for i = n:-1:1
                [hH{n-i+1}, hL{n-i+1}] = createMarkers(handles{n-i+1}, x{i}, y{i}, isMaximum{i}, roots{i}, lmts{i}, yLmts{i}, xChpt{i}, yChpt{i});
            end
            setZoomActions(hH, hL, y);
        elseif length(handles) == 1
            handle = handles{1};
        end
    end
    if strcmp(get(handle, 'Type'), 'line')
        % handle is line object
        [hH, hL] = createMarkers(handle, x, y, isMaximum, roots, lmts, yLmts, xChpt, yChpt);
        setZoomActions({hH}, {hL}, y);
        return;
    end
end


%% [x, y, isMaximum, roots, lmts, yLmts] = extractData(x)
% Extract the input arguments from the struct.
function [x, y, isMaximum, roots, lmts, yLmts, xChpt, yChpt] = extractData(S)
    if isfield(S, 'xExtr')
        x = S.xExtr;
    else
        x = [];
        y = [];
        isMaximum = [];
        roots = [];
        lmts = [];
        yLmts = [];
        xChpt = [];
        yChpt = [];
        return;
    end
    y = S.yExtr;
    if isfield(S, 'isMaximum')
        isMaximum = S.isMaximum;
    else
        isMaximum = getDefaultEmpty(iscell(x)*length(x));
    end
    if isfield(S, 'roots')
        roots = S.roots;
    else
        roots = getDefaultEmpty(iscell(x)*length(x));
    end
    if isfield(S, 'lmts')
        lmts = S.lmts;
        yLmts = S.yLmts;
    else
        lmts = getDefaultEmpty(iscell(x)*length(x));
        yLmts = lmts;
    end
    if isfield(S, 'xChpt')
        xChpt = S.xChpt;
        yChpt = S.yChpt;
    else
        xChpt = getDefaultEmpty(iscell(x)*length(x));
        yChpt = xChpt;
    end
end


%% setZoomActions(hLineMaximum, hLineMinimum, y)
function setZoomActions(hH, hL, y)
    if ~isempty(y)
        hFig = getFigH(hH);
        if isempty(hFig)
            return;
        end
        
        if ~isprop(hFig,'TransientSizeChangedAndZoomCallbacks')
            pi = addprop(hFig,'TransientSizeChangedAndZoomCallbacks');
            pi.Transient = true;
            hFig.TransientSizeChangedAndZoomCallbacks = struct();
        end
        hFig.TransientSizeChangedAndZoomCallbacks.(getAxisTag(hH, hL)) = {hH, hL};
        
        
        [cbSizeChanged, cbZoom] = createCallbacks();
        
        % offset markers on a zoom event.
        hZoom = zoom(hFig);        
        cbActionPostZoom = get(hZoom,'ActionPostCallback');
        if isempty(cbActionPostZoom)            
            set(hZoom, 'ActionPostCallback', cbZoom);
        end
        
        % offset markers on a SizeChange/SizeChanged event.
        if ~isprop(hFig,'TransientSizeChangedListener')
            pi = addprop(hFig,'TransientSizeChangedListener');
            pi.Transient = true;

            ll = event.listener(hFig, 'SizeChanged', cbSizeChanged);
            hFig.TransientSizeChangedListener = ll;
        end
        
        %Offset the markers now.
        cbZoom(hFig, struct('Axes', getAxisHandle(hH, hL)));
    end
end

function V = getDefaultEmpty(n)
    if n > 0
        V = repmat({[]}, 1, n);
    else
        V = [];
    end
end

function hFig = getFigH(hLine)
    if iscell(hLine)
        hFig = ancestor(firstNonempty(hLine), 'figure');
    else
        hFig = ancestor(hLine, 'figure');
    end
end

function [hH, hL] = createMarkers(hLine, x, y, isMaximum, roots, lmts, yLmts, xChpt, yChpt)
    hAxes = ancestor(hLine,'Axes');

    % use the color of the line
    color = get(hLine,'Color');
    
    if any(isMaximum)
        hH = line(x(isMaximum), y(isMaximum), 'Parent', hAxes, 'Marker', 'v', 'LineStyle', 'none', ...
            'Color', color, 'MarkerFaceColor', color, 'tag', 'PeakH');    
        setappdata(hH, 'YPos', y(isMaximum));
    else
        hH = [];
    end
    
    if any(~isMaximum)
        hL = line(x(~isMaximum), y(~isMaximum), 'Parent', hAxes, 'Marker', '^', 'LineStyle', 'none', ...
            'Color', color, 'MarkerFaceColor', color, 'tag', 'PeakL');
        setappdata(hL, 'YPos', y(~isMaximum));
    else
        hL = [];
    end
    
    if any(roots)
        line(roots, zeros(size(roots)), 'Parent', hAxes, 'Marker', '+', 'LineStyle', 'none', ...
            'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 10, 'tag', 'Root');
    end
    
    if any(lmts)
        line(lmts, yLmts, 'Parent', hAxes, 'Marker', '*', 'LineStyle', 'none', ...
            'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 10, 'tag', 'Limit');
    end
    
    if any(xChpt)
        line(xChpt, yChpt, 'Parent', hAxes, 'Marker', 'o', 'LineStyle', 'none', 'MarkerEdgeColor', color, ...
            'Color', color, 'MarkerFaceColor', 'none', 'MarkerSize', 7, 'tag', 'Changepoint');
    end
end

function [cbSizeChanged, cbZoom] = createCallbacks()
    cbZoom = @(obj, evt) offsetPeakMarkerCell(obj.TransientSizeChangedAndZoomCallbacks, evt.Axes);
    cbSizeChanged = @(obj, ~) sizeChangedCallback(obj);
end

function sizeChangedCallback(hfig)
    s = hfig.TransientSizeChangedAndZoomCallbacks;
    names = fieldnames(s);
    for i = 1:length(names)
        offsetPeakMarkerCell(s, findobj(hfig, 'Tag', names{i}));
    end
end

function offsetPeakMarkerCell(cbStruct, hAxes)
    % Fetch the data needed
    [hH, hL] = cbStruct.(hAxes.Tag){:};
    axesPos = getpixelposition(hAxes);
    yLim = get(hAxes,'YLim');
    
    if ~isempty(hH)
        for i = 1:length(hH)
            if isempty(hH{i})
                continue;
            end
            y = getappdata(hH{i}, 'YPos');

            % bump the line y data by the marker size
            yMarkerOffset = get(hH{i}, 'MarkerSize');
            yOffset = yMarkerOffset * diff(yLim) ./ axesPos(4);
            yOld = get(hH{i}, 'YData');
            yNew = y + yOffset;
            if ~isequaln(yOld, yNew)
                set(hH{i}, 'YData', yNew);
            end
        end
    end
    if ~isempty(hL)
        for i = 1:length(hL)
            if isempty(hL{i})
                continue;
            end
            y = getappdata(hL{i}, 'YPos');

            % bump the line y data by the marker size
            yMarkerOffset = get(hL{i}, 'MarkerSize');
            yOffset = yMarkerOffset * diff(yLim) ./ axesPos(4);
            yOld = get(hL{i}, 'YData');
            yNew = y - yOffset;
            if ~isequaln(yOld, yNew)
                set(hL{i}, 'YData', yNew);
            end
        end
    end
end

function axis = getAxisHandle(hH, hL)
    h = tryGetNonempty(hH, hL);
    if isempty(h)
        return;
    end
    if iscell(h)
        axis = ancestor(firstNonempty(hH), 'axes');
    else
        axis = ancestor(hH, 'axes');
    end
end

function tag = getAxisTag(hH, hL)
    axis = getAxisHandle(hH, hL);
    
    if isempty(axis.Tag)
        axis.Tag = ['Axis_', num2str(randi(1e8))];
    end
    
    tag = axis.Tag;
end


function h = tryGetNonempty(hH, hL)
    if isempty(hH)
        h = hL;
    else
        h = hH;
    end
end
