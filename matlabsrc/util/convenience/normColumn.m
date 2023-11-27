%% n = normColumn(A, p)
% implementation of the norm of all column vectors in the matrix A.
% This is equivalent to the Octave command
%   norm(A, [], 'columns');
function n = normColumn(A, p)
    nn = size(A,2);
    if nargin > 1
        n = zeros(1, nn);
        for i = 1:nn
            n(i) = norm(A(:,i), p);
        end
    else
        if nn==1
            n = norm(A);
        else
            n = sqrt(sum(A.*A,1));
        end
    end
end