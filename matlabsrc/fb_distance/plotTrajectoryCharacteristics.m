%% plotTrajectoryCharacteristics(Traj, T, fig, doSplit)
% 
%
function plotTrajectoryCharacteristics(Traj, T, fig, doSplit)
    if isa(T, 'TrajCharTable')
        T = T.doNormalizeTime(false);
        T = T.getTable({Traj.dataX, Traj.dataU});
    end

    if nargin < 4 || isempty(doSplit)
        doSplit = true;
    end

    useObjective = Traj.dataX.n == max(map(@(c)length(c), T(:,1)));
    if nargin < 3 || isempty(fig)
        fig = plotTrajectoryData(Traj, 'splitState', doSplit, 'sepObj', false, 'objective', useObjective);
    end
    n = length(findall(fig, 'Type', 'axes'));
    
    nx = Traj.dataX.n-1+useObjective;
    if isempty(Traj.dataU)
        nu = 0;
    else
        nu = Traj.dataU.n;
    end
    if size(T,1)==1 && size(T,2) > 1
        if ~isempty(T{1}{end, 1}) % compatibility!
            T = map2(@(c)c([1,2,3,5,4],:), T, 'UniformOutput', false);
        end
        
        S = extractInformation(T{1}, nx, Traj.state);
        uS = extractInformation(T{2}, nu, Traj.control);
        
        plotPeakAndRootMarker(fig, createCells(S, doSplit, n, uS));
    else
        if ~isempty(T{end, 1}) % compatibility!
            T = T([1,2,3,5,4],:);
        end
    
        S = extractInformation(T, Traj.dataX.n-1+useObjective, Traj.state);
        
        plotPeakAndRootMarker(fig, createCells(S, doSplit, n));
    end
end

function S = extractInformation(T, n, fun)
    S = struct('xExtr', [], 'yExtr', [], 'isMaximum', [], 'roots', [], 'lmts', [], 'yLmts', []);
    % find extrema:
    S.xExtr = arrayfun(@(i)cell2mat(T(1, map(@(c)~isempty(c)&&c==i, T(3,:)) & map(@(c)~isempty(c)&&(c(1)=='^'||c(1)=='v'), T(2,:)) )), 1:n, 'UniformOutput', false);
    S.yExtr = arrayfun(@(i)selectIndex(fun.evaluate(S.xExtr{i}), i, 0), 1:n, 'UniformOutput', false);
    % check, if is a maximum or minimum
    S.isMaximum = arrayfun(@(i)cell2mat(T(2, map(@(c)~isempty(c)&&c==i, T(3,:)) & map(@(c)~isempty(c)&&(c(1)=='^'||c(1)=='v'), T(2,:)) ))=='^', 1:n, 'UniformOutput', false);
    % find roots that are marked as '0' in the table:
    roots = arrayfun(@(i)cell2mat(T(1,(map(@(j)~isempty(T{3,j})&&T{3,j}==i&&T{2,j}(1)=='0', 1:size(T,2))))), 1:n, 'UniformOutput', false);
    % find roots given by sign changes, where other states have a min/max:
    roots2 = arrayfun(@(i)cell2mat(T(1,(map(@(j)~isempty(T{3,j})&&(T{2,j}(1)=='^'||T{2,j}(1)=='v')&&(T{4,j-1}(i)-T{4,j}(i))~=0, 1:size(T,2))))), 1:n, 'UniformOutput', false);
    % merge root lists:
    S.roots = cellfun(@(c1,c2)sort([c1,c2]), roots, roots2, 'UniformOutput', false);
    % find limits:
    S.lmts = arrayfun(@(i)cell2mat(T(1,(map(@(j)~isempty(T{3,j})&&T{3,j}==i&&T{2,j}(1)=='L', 1:size(T,2))))), 1:n, 'UniformOutput', false);
    S.yLmts = arrayfun(@(i)selectIndex(fun.evaluate(S.lmts{i}), i, 0), 1:n, 'UniformOutput', false);
    % find changepoints:
    S.xChpt = arrayfun(@(i)cell2mat(T(1, map(@(c)~isempty(c)&&c==i, T(3,:)) & map(@(c)~isempty(c)&&c(1)=='l', T(2,:)) )), 1:n, 'UniformOutput', false);
    S.yChpt = arrayfun(@(i)selectIndex(fun.evaluate(S.xChpt{i}), i, 0), 1:n, 'UniformOutput', false);
end

function S = createCells(S, doSplit, n, S2)
    f = fields(S);
    for i = 1:length(f)
        V = S.(f{i});
        C = cell(1,n);
        if doSplit
            nV = size(V,2);
            sel1 = 1:floor(nV/2);
            sel2 = sel1(end)+1:nV;
            C{1,1} = V(:,sel1);
            C{1,2} = V(:,sel2);
        else
            C{1} = V;
        end
        if nargin > 3
            C{1,3-(~doSplit)} = S2.(f{i});
        end
        S.(f{i}) = C;
    end
end

