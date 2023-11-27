%% [T, similarIndices, coph, D, Z] = clusterOcpSolutions(opts, varargin)
% Identify groups of similar trajectories.
%
% OUTPUT:
%   T:              cluster label (numeric) for each given element
%   similarIndices: find the position of the elements in the same cluster as the first element
%   coph:           the cophenet value.
%   D:              distance matrix
%   Z:              linkage data
% ADDITIONAL ARGUMENTS:
%   how:            (default='nBins'), either 'nBins' or 'cutoff'
%   howParam:       parameter for how. Default is 5 for 'nBins', and 0.8 for 'cutoff'
%   stateInds:      which state indices are considered for comparison of two trajectories 
%                     default=[], which is (1:end-1), set 0 for no state indices
%   controlInds:    which control indices are considered for comparison of two trajectories 
%                     default=[], which is (1:end), set 0 for no control indices.   
%   method:         (default=0) select the method how the similarity score is computed.
%                     0: use DTW 
%                     1: [removed!] 
%                     2: [removed!] 
%                     3: use TrajCharTable and svrSpell, requires handle as first argument in mArgs
%   mArgs:          (default={}) method arguments, must be a cell
%   weighted:       (default=true) use promScore and TDScore if methods is 1 or 2
%   plotDendro:     (default=false) plot the dendrogram in a new figure
%   D:              use a precomputed distance matrix and avoid recomputation
%   measureTime:    save time measurements in the global variables measured_times_1 and measured_times_2
function [T, similarIndices, coph, D, Z] = clusterOcpSolutions(opts, varargin)
    [how, howParam, stateInds, controlInds, method, isWeighted, doPlot, D, mArgs, measureTime] = ...
        getFunctionArguments(varargin, 'how', 'nBins', 'howParam', [], 'stateInds', [], 'controlInds', [], 'method', 0, 'weighted', true, 'plotDendro', false, 'D', [], 'mArgs', {}, 'measureTime', false);
    global measured_times_1 measured_times_2
    measured_times_1 = [];
    measured_times_2 = [];
    if isempty(howParam)
        if strcmp(how, 'nBins')
            howParam = 5;
        elseif strcmp(how, 'cutoff')
            howParam = 0.8;
        else
            error('Argument''how'' must be ''nBins'' or ''cutoff''.');
        end
    end
    num = length(opts);
    
    if num < 2
        warning('clusterOcpSolutions: At least two trajectories must be given for clustering.')
        return;
    end
    if isempty(stateInds)
        stateInds = 1:selectIndex(opts{1}.state.getDim(),2)-1;
    end
    if isscalar(stateInds) && stateInds == 0
        stateInds = [];
    end
    if isempty(controlInds)
        controlInds = 1:selectIndex(opts{1}.control.getDim(),2);
    end
    if isscalar(controlInds) && controlInds == 0
        controlInds = [];
    end
    
    switch method
        case 0
            % get the time grids for all 
            grids = cell2mat(arrayfun(@(i)linspace(0, opts{i}.state.getUpperBound, 1000)', 1:length(opts), 'UniformOutput', false))';
        %     distfun = @(x,Y)arrayfun(@(y)dtw(opts{x}.control.evaluate(grids(x,:)), opts{y}.control.evaluate(grids(y,:))), Y);
            if ~measureTime
                distfun = @(x,Y)arrayfun(@(y)dtw(...
                    [selectIndex(opts{x}.state.evaluate(grids(x,:)), stateInds); ...
                        selectIndex(opts{x}.control.evaluate(grids(x,:)), controlInds, 0)], ...
                    [selectIndex(opts{y}.state.evaluate(grids(y,:)), stateInds); ...
                        selectIndex(opts{y}.control.evaluate(grids(y,:)), controlInds, 0)]), Y);
            else
                measureTimeDistfunDTW([], [], opts, grids, stateInds, controlInds);
                distfun = @(x,Y)measureTimeDistfunDTW(x, Y);
                getTimes = @()measureTimeDistfunDTW();
            end
        case 1
            warning('This method (method=1 in clusterOcpSolutions) is REMOVED!')
        case 2
            warning('This method (method=2 in clusterOcpSolutions) is REMOVED!')
        case 3
            if isempty(D)
                h = mArgs{1};
                h = h.doNormalizeTime(isWeighted);
                tables = cell(1, length(opts));
                if measureTime
                    measured_times_1 = zeros(1, length(opts));
                end
                if length(h.getAllCategories) == 2
                    for i = 1:length(opts)
                        tic_label = tic;
                        [T, S, V] = h.getIndexwiseTable({opts{i}.dataX, opts{i}.dataU});
                        if measureTime, measured_times_1(i) = toc(tic_label); end
                        tables{i} = struct('Time', {T}, 'Symbol', {S}, 'Value', {V});
                    end
                else
                    for i = 1:length(opts)
                        tic_label = tic;
                        [T, S, V] = h.getIndexwiseTable(opts{i}.dataX);
                        if measureTime, measured_times_1(i) = toc(tic_label); end
                        tables{i} = struct('Time', T, 'Symbol', S, 'Value', V);
                    end
                end
            end
            if ~measureTime
                distfun = @(x,Y)( arrayfun(@(y)svrSpellOnTables(h, tables{x}, tables{y}), Y)  );
            else
                measureTimeDistfunSvrspell([], [], h, tables);
                distfun = @(x,Y)measureTimeDistfunSvrspell(x, Y);
                getTimes = @()measureTimeDistfunSvrspell();
            end
        otherwise
            error('clusterOCPSolutions: Given method index is not assigned to a method.');
    end

%     T = clusterdata((1:4)', 'Criterion', 'distance', 'Cutoff', cutoff, 'Distance', @(x,Y)distfun(x,Y));
        
    if isempty(D)
        D = pdist((1:num)', @(x,Y)distfun(x,Y));
    else
        D = squareform(D);
    end
    Z = linkage(D, 'single');
    
    if strcmp(how, 'cutoff')
        T = cluster(Z, 'Cutoff', howParam);
        similarIndices = find(T==T(1));
    elseif strcmp(how, 'nBins')
        if length(howParam) == 1
            T = cluster(Z, 'MaxClust', howParam);
            similarIndices = find(T==T(1));
        else
            T = map(@(nb)cluster(Z, 'MaxClust', nb), howParam);
            similarIndices = arrayfun(@(i) find(T(:,i)==T(1,i)), 1:size(T,2), 'UniformOutput', false);
        end
    else
        error('Argument''how'' must be ''nBins'' or ''cutoff''.');
    end
    
    if measureTime
        measured_times_2 = getTimes();
    end

    if doPlot
        figure(); dendrogram(Z, length(opts));
    end    
    if nargout > 3
        coph = cophenet(Z, D);
    end
    
    
    %
%     ranking = getDistanceRanking(distfun, num);
%     similarIndices = getBestDistanceRankingValuesForI(ranking, 1, 0.25);
    

    if isvector(D) && nargout > 4
        D = squareform(D);
    end
end

function retval = measureTimeDistfunSvrspell(x, Y, h, tables)
    %% retval = measureTimeDistfunSvrspell(x, Y, h, tables)
    % call with four inputs: initialize
    % call with two inputs: measure time
    % call with no inputs: get measured times
    %
    %
    %
    persistent p_tables p_h p_time p_counter p_properties p_accessMode
    switch(nargin)
        case 0
            retval = {p_time(1:(p_counter-1)), p_properties(:,1:(p_counter-1))};
        case 2
            retval = zeros(1, length(Y));
            for i = 1:length(Y)
                ticlabel = tic;
                retval(i) = svrSpellOnTables(p_h, p_tables{x}, p_tables{Y(i)});
                p_time(p_counter) = toc(ticlabel);
                if ~p_accessMode
                    if length(p_tables{1}) > 1
                        p_accessMode = 3;
                    elseif iscell(p_tables{1}.Time) && iscell(p_tables{1}.Time{1})
                        p_accessMode = 2;
                    else
                        p_accessMode = 1;
                    end
                end
                switch(p_accessMode)
                    case 1
                        p_properties(1, p_counter) = max(cellfun(@(c)length(c), p_tables{x}.Time));
                        p_properties(2, p_counter) = max(cellfun(@(c)length(c), p_tables{Y(i)}.Time));
                    case 2
                        prop = [];
                        prop(1,:) = map(@(j) map(@(c)length(c), p_tables{x}.Time{j}), 1:length(p_tables{x}.Time));
                        prop(2,:) = map(@(j) map(@(c)length(c), p_tables{Y(i)}.Time{j}), 1:length(p_tables{Y(i)}.Time));
                        prop = getParetoFrontInMatrix(1./prop);
                        p_properties(:, p_counter) = 1./prop{1};
    %                     p_properties(1, p_counter) = max(arrayfun( @(j) max(cellfun(@(c)length(c), p_tables{x}.Time{j})), 1:length(p_tables{x}.Time)));
    %                     p_properties(2, p_counter) = max(arrayfun( @(j) max(cellfun(@(c)length(c), p_tables{Y(i)}.Time{j})), 1:length(p_tables{Y(i)}.Time)));
                    case 3
                        p_properties(1, p_counter) = max(cellfun(@(c)length(c), {p_tables{x}.Time}));
                        p_properties(2, p_counter) = max(cellfun(@(c)length(c), {p_tables{Y(i)}.Time}));                        
                end
                p_counter = p_counter + 1;
            end            
        case 4
            retval = [];
            p_tables = tables;
            p_h = h;
            p_counter = 1;
            p_time = zeros(1,100000);
            p_properties = zeros(2, 100000);
            p_accessMode = 0;
        otherwise
            error('Unexpected number of arguments.');
    end
end
function retval = measureTimeDistfunDTW(x, Y, opts, grids, stateInds, controlInds)
    persistent p_time p_counter p_opts p_grids p_stateInds p_controlInds;
    switch(nargin)
        case 0
            retval = p_time(1:(p_counter-1));
        case 2            
            retval = zeros(1, length(Y));
            for i = 1:length(Y)
                ticlabel = tic;
                retval(i) = dtw(...
                [selectIndex(p_opts{x}.state.evaluate(p_grids(x,:)), p_stateInds); ...
                    selectIndex(p_opts{x}.control.evaluate(p_grids(x,:)), p_controlInds, 0)], ...
                [selectIndex(p_opts{Y(i)}.state.evaluate(p_grids(Y(i),:)), p_stateInds); ...
                    selectIndex(p_opts{Y(i)}.control.evaluate(p_grids(Y(i),:)), p_controlInds, 0)]);
                p_time(p_counter) = toc(ticlabel);
                p_counter = p_counter + 1;
            end            
        case 6
            retval = [];
            p_opts = opts;
            p_grids = grids;
            p_stateInds = stateInds;
            p_controlInds = controlInds;
            p_counter = 1;
            p_time = zeros(1,100000);
        otherwise
            error('Unexpected number of arguments.');
    end
end

% get the indices for the i-the element, where the distance is among the best ones
function ind = getBestDistanceRankingValuesForI(distRanking, I, threshold)
    if ~exist('threshold', 'var')
        threshold = 0.25;
    end
    n = length(distRanking);
    nn = n*(n-1)/2;
    
    if exist('I', 'var')
        ind = find(distRanking(:,I) <= threshold*nn);
    else
        ind = arrayfun(@(I)find(distRanking(:,I) <= threshold*nn), 1:n, 'UniformOutput', false);
    end
end


% rank the entries in the distance matrix, starting from the lowest value
function retval = getDistanceRanking(distfun, num)
    dist = cell2mat(arrayfun(@(i)distfun(i,1:num)', 1:num, 'UniformOutput', false));

    trilI = find(tril(dist));
    
    [~, sind] = sort(dist(trilI));
    
    tmp = zeros(length(sind),1);
    tmp(sind) = 1:length(sind);
    
    retval = zeros(num);
    retval(trilI) = tmp;
    
    retval = retval+retval';
end
