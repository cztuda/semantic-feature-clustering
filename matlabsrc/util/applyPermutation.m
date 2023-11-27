%
%
%
function sequence = applyPermutation(sequence, permutation, dim)
    if nargin < 3
        dim = 2;
    end
    if iscell(permutation)
        permutation = cycToTuple(permutation, size(sequence, dim));
    else
        if size(permutation,1) == 2
            permutation = permutation(2,:);
        end
        if length(permutation) ~= size(sequence, dim)
            error('Sequence and permutation must have the same length.');
        end
    end
        
    if dim==1
        sequence = sequence(permutation,:);
    elseif dim==2
        sequence = sequence(:,permutation);
    else
        error('Dimension dim must be 1 or 2.')
    end    
end


function permT = cycToTuple(permC, n)
    permT = 1:n;
    for i = 1:length(permC)
        p = permC{i};
        permT(p(1:end-1)) = p(2:end);
        permT(p(end)) = p(1);
    end
end