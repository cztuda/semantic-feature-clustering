%% [T_sorted, I_sorted] = sortTrajectories(Trajs, ind)
% Sort the given cell array of trajectories such that the indices describing a clusterin appear in
% order.
% E.g.: ind=[1,2,2,1,3,4,3,3,1,1,2,1], then
%        T_sorted = Trajs([1,4,9,10,12, 2,3,11, 5,7,8, 6])
%
% ind can also be given as cell array, where each cell lists all indices in a cluster. The above ind
% would be given as ind={[1,4,9,10,12], [2,3,11], [5,7,8], [6]}.
%
function [T_sorted, I_sorted] = sortTrajectories(Trajs, ind)
    if iscell(ind)
        itmp = nan(length(Trajs), 1);
        for i = 1:length(ind)
            itmp(ind {i}) = i;
        end
        itmp(isnan(itmp)) = i+1;
        ind = itmp;
    end
    
    uind = unique(ind);
    sums = map(@(i)sum(ind==i), uind)';
    
    I_sorted = cell2mat(arrayfun(@(j, num)num*ones(1, j), sums, 1:length(uind), 'UniformOutput', false))';
    T_sorted = map(@(i)Trajs(ind==i), uind)';
    T_sorted = [T_sorted{:}];
end