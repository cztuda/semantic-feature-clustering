% fig = plotTrajectoryData(trajectory, varargin) 
% Create the plots for the given trajectory data
% EXAMPLES:
%   >> plotTrajectoryData(universalFunctionTrajectory);
% 
%   >> plotTrajectoryData(optTrajStruct);
%
%   >> plotTrajectoryData(trajcol, i);
%
%
% INPUT:
%   trajectory:          universalFunction object
%                       OR struct with .state, .control
%                       OR TrajectoryCollector object/struct (this requires a second argument index)
%   index (optional):    only required if a trajectory collector object is given, then this is the index of the trajectory
%                       OR label 'control' or 'state' if only a universal function object is given
% OUTPUT:
%   fig:                  figure handle
% OPTIONAL ARGUMENTS
%   fig            : []         - figure handle
%   stateIndices   : []         - an array of state indices
%   controlIndices : []         - an array of control indices
%   objective      : true
%   timegrid       : []         - can be an array of times or a step dt
%   sepControl     : true       - plot control in a separate subplot
%   sepObj         : true       - plot objective in a separate subplot
%   splitState     : false      - plot position and velocity in separate plots
%   plotCurrentGrid: false      - [unused]
%   print          : ''
%   printOnly      : false
%   description    : []         - struct with fields: .title, .xlabel, .ylabel
%   plotAdj        : false      - [unused] plot the adjugate variables
%   comparative    : false      - add the data to some existing trajectory plot
%   xline          : []         - draw vertical line in all plots
%   tfminus        : 0          - subtract some value from tfminus
function fig = plotTrajectoryData(trajcol, varargin)
    if isstruct(trajcol) && isfield(trajcol, 'dircol') % if is TrajectoryCollector struct
        index = varargin{1};
        state = trajcol.statefunctions{index};
        control = trajcol.controlfunctions{index};
        trajcol.state = state;
        trajcol.control = control;
        varargin = varargin(2:end);
        clear state control index;
        tf = trajcol.state.getUpperBound();
    elseif isa(trajcol, 'universalFunction')
        if ~isempty(varargin) && strcmp(varargin{1}, 'control')
            state = [];
            control = trajcol;
            tf = control.getUpperBound();
        else
            state = trajcol;
            control = [];
            tf = state.getUpperBound();
        end
        trajcol = struct('state', state, 'control', control);
        if ~isempty(varargin) && ( strcmp(varargin{1}, 'control') || strcmp(varargin{1}, 'state'))
            varargin = varargin(2:end);
        end
        clear state control;
    else
        if iscell(trajcol.state) % handle multi-result struct from stackedDircol_interface
            fig = cell(size(trajcol.state));
            tmp = trajcol;
            if ~any(strcmp(varargin, 'comparative'))
                varargin = [varargin, 'comparative', -1]; % default is all plots in one
            end
            for i = 1:length(trajcol.state)
                tmp.state = trajcol.state{i};
                tmp.cost = trajcol.cost(i);
                tmp.dataX = trajcol.dataX{i};
                tmp.dataAdj = trajcol.dataAdj{i};
                
                if iscell(fig)
                    fig{i} = plotTrajectoryData(tmp, varargin{:});
                    if any(strcmp(varargin, 'comparative'))
                        fig = fig{i};
                        varargin = [varargin, 'fig', fig.Number]; %#ok<AGROW> appended only once
                    end
                else
                    fig = plotTrajectoryData(tmp, varargin{:});
                end
            end
            return;
        end
        tf = trajcol.state.getUpperBound();
    end
    trajectory = trajcol;
    [fig, stateIndices, controlIndices, plotObjective, timegrid, separateControl, ...
      separateObjective, splitState, plotCurrentGrid, print, printOnly, description, plotAdj, comparative, x_line, tfMinus] = ...
        getFunctionArguments(varargin, 'fig', [], 'stateIndices', EMPTY_DEFAULT_ARGUMENT, 'controlIndices', EMPTY_DEFAULT_ARGUMENT, 'objective', true, 'timegrid', [], ...
        'sepControl', true, 'sepObj', true, 'splitState', false, 'plotCurrentGrid', false, 'print', '', 'printOnly', false, ...
        'description', [], 'plotAdj', false, 'comparative', 0, 'xline', [], 'tfminus', 0);
    tf = tf-tfMinus;
        
    if isempty(timegrid)
        timegrid = linspace(0, tf, 500);
    end
    if length(timegrid) == 1
        timegrid = 0:timegrid:tf;
    end
    if iscolumn(timegrid)
        time = transpose(timegrid);
    end
    separateObjective = separateObjective & plotObjective;
    state = [];
    objective = [];
    if ~isempty(trajectory.state)
        state = trajectory.state.evaluate(timegrid);
        if plotObjective
            objective = state(end,:);
        end
        if ~isa(stateIndices, 'EMPTY_DEFAULT_ARGUMENT')
            state = state(stateIndices,:);
        else
            state = state(1:end-1,:);
            stateIndices = 1:size(state,1);
        end
        if splitState
            n2 = size(state,1);
            if n2==1
                error('Number of state variables must be strictly larger than 1 when splitting.');
            end
            if mod(n2,2)
                warning('Number of state variables is uneven. Ignoring the last state when splitting.');
                state2 = state(ceil(n2/2):end,:);
                state = state(1:floor(n2/2),:);
                stateIndices2 = stateIndices(ceil(n2/2):end);
                stateIndices = stateIndices(1:floor(n2/2));
            else
                state2 = state(n2/2+1:end,:);
                state = state(1:n2/2,:);
                stateIndices2 = stateIndices(n2/2+1:end);
                stateIndices = stateIndices(1:n2/2);
            end
        end
    end
    control = [];
    if ~isempty(trajectory.control)
        control = trajectory.control.evaluate(timegrid);
        if ~isa(controlIndices, 'EMPTY_DEFAULT_ARGUMENT')
            control = control(controlIndices,:);
        else
            controlIndices = 1:size(control,1);
        end
    end
    if ~plotObjective
        objective = [];
    end
    if isempty(fig)
        fig = figure('visible', ~printOnly);
        fig.Color = [1,1,1];
        comparative = 0;
    else
        if isOctave()
            fig = figure(fig, 'visible', ~printOnly);
        else
            fig = figure(fig);
            set(fig, 'visible',  ~printOnly, 'Color', [1,1,1]);
        end
        if isempty(fig.Children)
            comparative = 0;
        end
    end
    if isempty(control)
        separateControl = false;
    end
    
    [titl, xlab, ylab] = getDescription(fig, description, tf, trajectory, comparative);
    if comparative
        tf = max(tf, selectIndex(xlim(),2));
    end
    if separateControl || separateObjective || splitState
        array = state;
        n = 1 + separateControl + separateObjective + splitState;
        if separateControl && ~isempty(control)
            subplot(n, 1, 2+splitState); hold on; grid on;
            doPlot(fig, n, timegrid, control, comparative);
            if ~comparative
                xlabel(xlab); ylabel('control');
                leg = createLegend([], controlIndices, false);
                legend(leg, 'location', 'northeastoutside', 'AutoUpdate', 'off');
            end
            xlim([0, tf]);
            if ~isempty(x_line)
                xline(x_line);
            end
        else
            array = [array; control];
%             controlIndices = [];
        end
        if separateObjective
            subplot(n, 1, n); hold on; grid on;
            if plotObjective
                doPlot(fig, n-1, timegrid, objective, comparative);
            end
            if ~comparative
                xlabel(xlab); ylabel('objective');
                leg = createLegend([], [], ~isempty(objective));
                legend(leg, 'location', 'northeastoutside', 'AutoUpdate', 'off');
            end
            if ~isempty(x_line)
                xline(x_line);
            end
            xlim([0, tf]);
        else
            array = [array; objective];
            objective = [];
        end
        if splitState
            subplot(n, 1, 2); hold on; grid on;
            doPlot(fig, 2, timegrid, state2, comparative);
            if ~comparative
                xlabel(xlab); ylabel('velocity');
                leg = createLegend(stateIndices2, [], false);
                legend(leg, 'location', 'northeastoutside', 'AutoUpdate', 'off');
            end
            if ~isempty(x_line)
                xline(x_line);
            end
            xlim([0, tf]);
        end
        subplot(n, 1, 1); hold on; grid on;
        doPlot(fig, 1, timegrid, array, comparative);
        if separateControl
            controlIndices = [];
        end
        title(modifyTitle(titl)); 
        if ~comparative
            xlabel(xlab); ylabel(createStateYLabel(separateControl, separateObjective, splitState, plotObjective, controlIndices));
            leg = createLegend(stateIndices, controlIndices, ~isempty(objective) & ~separateObjective);
            legend(leg, 'location', 'northeastoutside', 'AutoUpdate', 'off');
        end
        if ~isempty(x_line)
            xline(x_line);
        end
%         alignSubplots(fig);
        xlim([0, tf]);
    else
        array = [state; control; objective];
        doPlot(fig, 1, timegrid, array, comparative);
        hold on;
        title(modifyTitle(titl)); 
        if ~comparative
            leg = createLegend(stateIndices, controlIndices, ~isempty(objective));
            legend(leg, 'location', 'northeastoutside', 'AutoUpdate', 'off');
            xlabel(xlab); ylabel(ylab);
        end
        if ~isempty(x_line)
            xline(x_line);
        end
        xlim([0, tf]);
    end
    
    if ~isempty(print)
        print(fig, print);
    end
    if printOnly
        close(fig);
    end
end



  %% create legend:
function leg = createLegend(stateIndices, controlIndices, doObjective)
    s = {};
    for i = 1:length(stateIndices)
        s = [s, ['X_{', num2str(stateIndices(i)), '}'] ]; %#ok<AGROW>
    end
    if ~isa(controlIndices, 'EMPTY_DEFAULT_ARGUMENT')
        for i = 1:length(controlIndices)
            s = [s, ['U_{', num2str(controlIndices(i)), '}'] ]; %#ok<AGROW>
        end
    end
    if ~isempty(doObjective) && doObjective
        s = [s, 'obj'];
    end
    leg = s;
end
  

  
%% create title
function [title, xlabel, ylabel] = getDescription(fig, description, tf, trajcol, comparative)
    % title:
    if comparative
        childIdx = find(cellfun(@(c)~isempty(c.String), arrayfun(@(i)fig.Children(i).Title, 1:length(fig.Children), 'UniformOutput', false)), 1);
        if ~isempty(childIdx)
            sOld = fig.Children(childIdx).Title.String;
        else
            sOld = '';
        end
    end
    s = [];
    if isfield(trajcol, 'problem')
        s = [s, trajcol.problem.getName(), ': '];
        s = [s, 'T_f = ', num2str(trajcol.problem.getEndTime()), ' sek,'];
        stmp = vec2str(trajcol.problem.getInitialValues());
        s = [s, ['x_0 = ', stmp]];
        if comparative
            s = [sOld, '\newline', s];
        end
    else
        if comparative
            s = ['Comparative t', sOld(2:end), ' (', num2str(tf), ')'];
        else
            s = ['Trajectory plot', [', tf=', num2str(tf)]];
        end
    end
    title = s;
    
    % xlabel:
    if isempty(description) && isfield(description, 'xlabel')
        xlabel = description.xlabel;
    else
        xlabel = 'time';
    end
    
    % ylabel:
    if isempty(description) && isfield(description, 'ylabel')
        ylabel = description.ylabel;
    else
        ylabel = 'value';
    end
end

function label = createStateYLabel(separateControl, separateObjective, splitState, plotObjective, controlIndices)
    if splitState
        label = {'position'};
    else
        label = {'state'};
    end
    isobj = ~separateObjective && plotObjective;
    isctrl = ~separateControl && ~isempty(controlIndices);
    
    if isctrl
        label = [label, {'control'}];
        if splitState
            if isobj
                label{end-1} = [label{end-1}, ' and'];
            else
                label{end} = ['and ', label{end}];
            end
        else
            label{end-1} = [label{end-1}, ' and'];
        end
    end
    if isobj
        label{end} = [label{end}, ' and'];
        label = [label, {'objective'}];
    end
end

function doPlot(fig, axInd, timegrid, array, comparative)
    if comparative
        if isnumeric(fig)
            fig = figure(fig);
        end
        children = fig.Children(arrayfun(@(i)strcmp(fig.Children(i).Type, 'axes'), 1:length(fig.Children)));
        ax = children(axInd);
        if ~isempty(ax) && ~isempty(ax.Legend)
            ax.Legend.AutoUpdate = 'off';
        end
        nC = length(ax.Children);
        switch comparative
            case 1
                plot(timegrid, array, '--');
            case 2
                plot(timegrid, array, ':');
            case 3
                plot(timegrid, array, '-.');
            otherwise
                plot(timegrid, array);
        end
        copyColors(ax.Children, nC);
    else
        plot(timegrid, array);
    end
end

function copyColors(entities, nBelow)
    n = length(entities);
    nby2 = min(nBelow, n-nBelow);
    for i = 1:nby2
        entities(n-nBelow-i+1).Color = entities(n-i+1).Color;
    end
end

function ttl = modifyTitle(ttl)
    ind = regexp(ttl, '[^_]_[^_]');
    for i = length(ind):-1:1
        ttl = [ttl(1:ind(i)), '\', ttl((ind(i)+1):end)];
    end
end

%% not necessary in Matlab and not reliably working in Octave?
% function alignSubplots(fig)
%     axes = get(fig, 'children');
%     sel = ~strcmp('legend', get(axes, 'tag')) & ~strcmp('colorbar', get(axes, 'tag'));
%     axes = axes(sel);
%     positions = get(axes, 'position');
%     if iscell(positions)
%         positions = cell2mat(positions);
%     end
%     m = min(positions(:,3));
%     positions(:,3) = m;
%     for i = 1:length(axes)
%         set(axes(i), 'position', positions(i,:));
%     end
%     %
%     axes = get(fig, 'children');
%     axes = axes(~sel);
%     positions = get(axes, 'position');
%     if iscell(positions)
%         positions = cell2mat(positions);
%     end
%     m = min(positions(:,1));
%     positions(:,1) = m;
%     for i = 1:length(axes)
%         set(axes(i), 'position', positions(i,:));
%     end
% end
