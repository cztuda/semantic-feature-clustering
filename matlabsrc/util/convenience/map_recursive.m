%% v = map_recursive(fun, A, B)
% Input B is optional. A and B are assumed to have the same cell structure.
% fun is applied to the non-cell elements in A and B;
%
% Example:
%   >> map_recursive(@plus, {{[1,2]}, {[3,4]}}, {{[8,5]}, {[3,1]}})
%   ans = 
%     {{[9,7]}, {[6,5]}}
function v = map_recursive(fun, A, B)
    if iscell(A)
        if nargin > 2
            v = arrayfun(@(i)map_recursive(fun, A{i}, B{i}), 1:length(A), 'UniformOutput', false);
        else
            v = cellfun(@(c)map_recursive(fun, c), A, 'UniformOutput', false);
        end
    else
        if nargin > 2
            v = fun(A, B);
        else
            v = fun(A);
        end
    end
end