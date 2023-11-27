h_gatherJoints = @gatherJoints;
h_plotBundle = @plotBundle;
h_gatherJointsOverParticipants = @gatherJointsOverParticipants;
h_prepareTCT = @prepareTCT;
h_prepareFunStruct = @prepareFunStruct;
if ~exist('Normalized', 'var'), Normalized = []; end
outfolder = createDir(pwd(), '..', 'data', 'human_motion', 'output');
measureTime = false;

% Normalized = perform_clustering_humanMotion_dataset(Normalized, 1, [], [9,9], false);

Normalized = perform_clustering_humanMotion_dataset(Normalized, 0, outfolder, [9,4], false, 'measureTime', measureTime);
perform_clustering_humanMotion_dataset(Normalized, 1, outfolder, [9,9], false, 'measureTime', measureTime);
% perform_clustering_humanMotion_dataset(Normalized, 2, outfolder);

% perform_clustering_humanMotion_dataset(Normalized, 0, [], [9,4], false, 'measureTime', true);


%% perform_clustering_humanMotion_dataset(Normalized, scenario, outfolder, binOffset=[0,0], plotTestset=true, varargin)
%
function [Normalized, Z0, Z3, T0, T3, groundTruth, TCT, testfuns] = ...
                perform_clustering_humanMotion_dataset(Normalized, scenario, outfolder, varargin)
    [binOffset, plotTestset, measureTime] = getFunctionArguments(varargin, {[0,0], true}, 'measureTime', false);
    infolder = fullfile(pwd(), '..', 'data', 'human_motion');
    if ~exist('Normalized', 'var') || isempty(Normalized)
        tmp = load(fullfile(infolder, 'joint_angle_data'), 'Normalized');
        Normalized = tmp.Normalized;
        % Streaming = tmp.Streaming;
    end
    if measureTime
        doClustering = @clusterOcpSolutions_measureTime;
    else
        doClustering = @clusterOcpSolutions;
    end

    isExperimental = nargin < 3 || isempty(outfolder);
    switch(scenario)
        case 0
            % WORKS WELL:
            fieldSel = {};
            fieldSel = [fieldSel, {{'AB01', 'Run', 's1x8', 'i0'}}]; % 60
            fieldSel = [fieldSel, {{'AB01', 'Stair', 's1', 'in35'}}]; % 5 
            fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'i10'}}];
            fieldSel = [fieldSel, {{'AB01', 'SitStand', 'ss', 'stand2sit'}}]; % 14            
        case 1
            % ON PAR:
            fieldSel = {};
            fieldSel = [fieldSel, {{'AB01', 'Run', 's1x8', 'i0'}}]; % 60
            fieldSel = [fieldSel, {{'AB01', 'Stair', 's1', 'i20'}}]; % 5 
            fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'i0'}}];
            fieldSel = [fieldSel, {{'AB01', 'SitStand', 'ss', 'stand2sit'}}]; % 14            
        case 2
            % WORKS NOT SO WELL:
            fieldSel = {};
            fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'i0'}}];
            fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'in5'}}];
            fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'i5'}}];            
        otherwise
            fieldSel = {};
            fieldSel = [fieldSel, {{'AB01', 'Run', 's1x8', 'i0'}}]; % 60
            fieldSel = [fieldSel, {{'AB01', 'Stair', 's1', 'i35'}}]; % 5 
            fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'i10'}}];
            fieldSel = [fieldSel, {{'AB01', 'SitStand', 'ss', 'stand2sit'}}]; % 14  
%     fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'in10'}}];
%     fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'i10'}}];
    
%     fieldSel = [fieldSel, {{'AB01', 'Run', 's1x8', 'i0'}}]; % 60
%     fieldSel = [fieldSel, {{'AB01', 'Stair', 's1', 'i20'}}]; % 5 
%     fieldSel = [fieldSel, {{'AB01', 'Walk', 's0x8', 'i0'}}];
%     fieldSel = [fieldSel, {{'AB01', 'SitStand', 'ss', 'stand2sit'}}]; % 14
%     fieldSel = [fieldSel, {{'AB01', 'Run', 's2x0', 'i0'}}]; % 67
%     fieldSel = [fieldSel, {{'AB01', 'Stair', 's1', 'i35'}}]; % 5
%     fieldSel = [fieldSel, {{'AB01', 'Stair', 's1', 'i20'}}]; % 5
    % fieldSel = [fieldSel, {{'AB01', 'SitStand', 'ss', 'sit2stand'}}]; % 14
    
            warning('Using experimental scenario');
            isExperimental = true;
    end
    
    times = linspace(0,1,150);
    nBins = length(fieldSel);
    stateSel = 1;
    testset = cellfun(@(c)gatherJointsOverParticipants(Normalized, c, 1, stateSel), fieldSel, 'UniformOutput', false);
    testset = cat(3, testset{:});
    testfuns = arrayfun(@(i)prepareFunStruct(testset(:,:,i), times), 1:size(testset,3), 'UniformOutput', false);
    groundTruth = map(@(i)i*ones(1,length(fieldnames(Normalized))), 1:nBins);

    promFilter = 0.2; 
    TCT = prepareTCT(promFilter);
    nStates = 5*length(stateSel);
    
    [T0,~,~,~, Z0] = doClustering(testfuns, 'method', 0, 'plotDendro', false, 'stateInds', 1:nStates, 'controlInds', 0, 'how', 'nBins', 'howParam', nBins+binOffset(1));
    [T3, ~, ~, ~, Z3] = doClustering(testfuns, 'method', 3, 'plotDendro', false, 'stateInds', 1:nStates, 'controlInds', 0, 'how', 'nBins', 'howParam', nBins+binOffset(2), 'mArgs', {TCT});
    
    if ~isExperimental
        close all;
    end
    
    lgnd = arrayfun(@(i)sprintf('(%i) %s-%s-%s', i, fieldSel{i}{2}, fieldSel{i}{3}, fieldSel{i}{4}), 1:4, 'UniformOutput', false);
    plotDendrogram(Z0, true, groundTruth, nBins+binOffset(1), 'showNumbers', isExperimental, 'logY', true, 'legend', lgnd); 
    set(gcf().Children(1), 'Location', 'northwest')
    if isExperimental
        title('DTW');
    else
        fname = sprintf('dsNature_set%i_dtw_dendr', scenario);
        export_fig(fullfile(outfolder, fname), '-png', '-svg', '-r600', '-q101', '-silent');
        savefig(fullfile(outfolder, fname));
    end
    
    plotDendrogram(Z3, true, groundTruth, nBins+binOffset(2), 'showNumbers', isExperimental, 'logY', true); 
    if isExperimental
        title('svrSpell-based');
    else
        fname = sprintf('dsNature_set%i_svrsp_dendr', scenario);
        export_fig(fullfile(outfolder, fname), '-png', '-svg', '-r600', '-q101', '-silent');
        savefig(fullfile(outfolder, fname));        
    end

    if ~ishandle(101) && plotTestset
        for i = 1:(length(testfuns)/10)
            cellfun(@(c)plotTrajectoryData(c, 'sepObj', false, 'fig', 100+i, 'comparative', -1, ...
            'controlIndices', []), testfuns((i-1)*10+(1:10)), 'UniformOutput', false);
            if isExperimental
                title(sprintf('Cluster %02i: %s', i-1, strjoin(fieldSel{i}, ' - ')));
            else
                title('');
%                 legend('Location', 'best')
                legend('off');
                fname = sprintf('dsNature_set%i_cluster%02i_%s', scenario, i, strjoin(fieldSel{i}(2:end), '-'));
                export_fig(fullfile(outfolder, fname), '-png', '-svg', '-r600', '-q101', '-silent');
                savefig(fullfile(outfolder, fname));                
            end
        end
    end
    
end


%% utility

function alltrajs = gatherJointsOverParticipants(Normalized, fss, n, stateSel)
    participants = fieldnames(Normalized);
    if nargin < 4
        stateSel = 1;
    end
    if nargin < 3
        n = 1;
    end
    alltrajs = [];
    for p = 1:length(participants)
        substr = selectFrom(Normalized, fss, participants{p});
        
        sub = fieldnames(substr);
        angles = cell(1, length(sub));
        for i = 1:length(sub)
           angles{i} = substr.(sub{i});
           angles{i} = angles{i}(:,stateSel,:);
        end
        allAngles = cat(2, angles{:});
        alltrajs = cat(3, alltrajs, allAngles(:,:,1:n));
    end
end

function allAngles = gatherJoints(Normalized, f, stateSel)
    if nargin < 3
        stateSel = 1;
    end
    substruct = selectFrom(Normalized, f);
    sub = fieldnames(substruct);
    angles = cell(1, length(sub));
    for i = 1:length(sub)
       angles{i} = substruct.(sub{i});
       angles{i} = angles{i}(:,stateSel,:);
    end
    allAngles = cat(2, angles{:});
end

function substruct = selectFrom(N, f, part)
    if nargin < 3        
        if isstruct(f)
            part = f.participant;
        elseif iscell(f)
            part = f{1};
        end
    end
    N = N.(part);
    if isstruct(f)
        substruct = N.(f.task).(f.tSpec).(f.sel).jointAngles;
    elseif iscell(f)
        substruct = N.(f{2}).(f{3}).(f{4}).jointAngles;
    end
end

function plotBundle(t, v)
    fig = figure(); plot(t, v(:,:,1));
    dim = size(v,2);
    fig.Color = [1,1,1];
    grid on; hold on;
    colors = colororder(fig);
    for i = 2:size(v,3)
        for j = 1:dim
            plot(t, v(:,j,i), 'Color', colors(j,:)); 
        end
    end
end

function funStruct = prepareFunStruct(values, varargin)
    [time, asCubic] = getFunctionArguments(varargin, ...
        {[], false});

    if isempty(time)
        time = 0:(1/500):((1/500)*(length(values)-1));
    end

    fPos = values;
    if asCubic
        [dataX,funX] = createCubicInterpolation(time, fPos);
    else
        [funX, dataX] = createPiecewiseLinear(time, fPos);
    end
    [funU, dataU] = createPiecewiseLinear(0,0);
    funStruct = struct('state', funX, 'control', funU, 'dataX', dataX, 'dataU', dataU);
end

function TCT = prepareTCT(promFilter, forPlotting)    
    TCT = TrajCharTable.getDefaultStateOnlyTCTHandle([], false, ...
        'withRoots', false, ...
        'promFilter', 0.2, ...
        'stateFilter', []);
%     TCT = TCT.addTrajChar(TC_Changepoints([], 0.04, []));
%     TCT = TCT.addTrajChar(TC_Passpoints([], [], 'endpointIsRef', false));
    
    if nargin > 1 && forPlotting
        TCT = TCT.doNormalizeTime(false);
    end
    if ~isempty(promFilter)
        TCT = TCT.TCList(1,1, TCT.TCList(1,1).promFilter(promFilter));
    end
end
