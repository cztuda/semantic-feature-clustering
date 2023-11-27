%% [front, frontIndex] = getParetoFrontInMatrix(A)
% Consider a discrete multi-objective minimization problem, where f1(x), f2(x), ..., fk(x) are 
% objectives to be minimized and x1, x2, ..., xn the discrete values.
% INPUT:
%       [ f1(x1),  f1(x2),  ...,  f1(xn) ]
%   A = [ f2(x1),  f2(x2),  ...,  f2(xn) ]
%       [ ...   ,  ...   ,  ...,  ...    ]
%       [ fk(x1),  fk(x2),  ...,  fk(xn) ]
% OUTPUT:
%   front:      cell of all columns in A that form a pareto front
%   frontIndex: indices of all columns in A that form a pareto front. 
%                 It is an array, if the columns in A are unique, else a cell.
%
function [front, frontIndex] = getParetoFrontInMatrix(A)
    if size(A,2) == 0
        front = [];
        frontIndex = [];
        return;
    end
    [B, idx] = makeUnique(A);
    
    front = {B(:,1)};
    frontIndex = 1;
    for i = 2:size(B,2)
        dominates = cellfun(@(a)all(B(:,i) <= a), front);
        if any(dominates) % dominates elts in pareto front
            % so remove these elts and add it to the front
            front(dominates) = [];
            frontIndex(dominates) = [];   %#ok<AGROW>
            front = [front, {B(:,i)}];    %#ok<AGROW>
            frontIndex = [frontIndex, i]; %#ok<AGROW>
        elseif ~any(cellfun(@(a)all(B(:,i) >= a), front)) % not dominated by any elt in pareto front
            % so add it to the front
            front = [front, {B(:,i)}];    %#ok<AGROW>
            frontIndex = [frontIndex, i]; %#ok<AGROW>
        end
    end
    
    if size(B,2) < size(A,2) && nargout > 1
        frontIndex = arrayfun(@(i)find(idx==i), frontIndex, 'UniformOutput', false);
    end
end

function [A, Ai] = makeUnique(A)
    [A, ~, Ai] = unique(A', 'rows');
    A = A';
    Ai = Ai';
end
