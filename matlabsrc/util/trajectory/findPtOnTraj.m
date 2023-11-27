%% t = findPtOnTraj(traj, pt, tf, t0=0, subselection=[])
% Find the time of the first occurrence of some point pt on a trajectory traj with start time 0 and end time tf.
% If more than one occurrence exist, the first occurrence is returned
% If the point is not exactly on the trajectory, the time at which the trajectory has minimal distance to this point is returned.
% INPUT:
%   traj: the trajectory, some universal function object, must depend on a single scalar value
%   pt:   some point on or close to the trajectory
%   tf:   custom final time of the trajectory (if empty, trajectory's final time is used)
%   t0:   custom start time of the trajectory (if empty, trajectory's start time is used)
%   subselection:   logical vector that enables/disables entries in the trajectory vector, default=[]
% OUTPUT:
%   t:    the time, such that || pt - traj.evaluate(t) ||_2 is minimal
%         if t0 != 0, then t0 is added to t!  
%   mindist:
%         the minimal distance from the given point to the trajectory (0,
%         if it is exactly on the trajectory)
% ADDITIONAL ARGUMENTS:
%   searchSteps:    (default=[1e-2, 1e-4]) the fineness of the grid in the refinement steps. More
%                     than two steps may be given. The first value is the sensitivity of the search,
%                     the last value gives the accuracy of the solution returned.
%   fixedDistance:  (default=0) if > 0, then the first occurence is searched where the given point 
%                     the distance fixedDistance to the trajectory
function [t, mindist] = findPtOnTraj(traj, pt, tf, varargin)
    [t0, subselection, searchSteps, ptDist] = getFunctionArguments(varargin, {[], []}, 'searchSteps', [1e-2, 1e-4], 'fixedDistance', 0);
    if nargin < 3 || isempty(tf)
        tf = traj.getUpperBound();
    end
    if isempty(t0)
        t0 = traj.getLowerBound();
    end
    searchSteps = sort(abs(searchSteps), 'descend'); % make sure that steps are positive and sorted
    
    I = zeros(size(searchSteps));
    Offset = zeros(size(searchSteps));
    a = t0; b = tf;
    for i = 1:length(searchSteps)
        step = searchSteps(i);
        grid = a:step:b;
        pts = traj.evaluate(grid);
        if ~isempty(subselection)
            pts = pts(subselection,:);
        else
            pts = cropLastIndex(pts);
        end
        if ~isempty(subselection)
            pt_tmp = repmat(pt(subselection), 1, size(pts, 2)) - pts;
        else
            pt_tmp = repmat(pt, 1, size(pts, 2)) - pts;
        end
        [mindist, ind] = min(abs(normColumn(pt_tmp) - ptDist)); % min dist on grid
        if ind > 1
            a = grid(ind-1); % change left interval side
            Offset(i) = 1; % interval starts one index before
        end
        if ind < length(grid)
            b = grid(ind+1); % change right interval side
        end
        I(i) = ind;
    end
    
    % determine time:
    mindist = mindist + ptDist;
    t = t0 + sum(searchSteps .* (I-Offset-1));
    if Offset(end) % the last offset must not be used
        t = t + searchSteps(end);
    end
end
