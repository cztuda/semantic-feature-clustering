%% [D, I] = nearestNeighbor(A)
% returns for each column-vector in A the shortest distance to any other
% vector in A
% INPUT:
%   A:  [mxn]-matrix: n vectors of dimension m
% OUTPUT:
%   D   n-vector the shortest distance for each vector
%   I   n-vector indices of the nearest neighbor
function [D, I] = nearestNeighbor(A)
    n = size(A, 2);
    I = zeros(1,n);
    D = I;
    
    for i = 1:n
        A2 = bsxfun(@minus, A(:,i), A);
        m = sqrt(sum(A2.*A2, 1));
        [m, index] = min(m(1:n ~= i));
        D(i) = m;
        if index >= i
            I(i) = index+1;
        else
            I(i) = index;
        end
    end
    
    
end


function [D, I] = tmp(A)
    n = size(A, 2);
       
    
    for i = 1:(n-1)
        A2 = bsxfun(@minus, A(:,i), A);
        m = sqrt(sum(A2.*A2,1));
%        [m, index] = min(m(1:n ~= i));
        [m, index] = min(m((i+1):n));
        D(i) = m;
        I(i) = index+i;
    end
end


function A = tmp2(A, threshold)
    n = size(A, 2);
    i = 1;
    while i < n
        A2 = bsxfun(@minus, A(:,i), A);
        m = sqrt(sum(A2.*A2,1));
        A(:,(i+1):n) = A(:, m((i+1):n) > threshold);
        n = size(A, 2);
    end
end


function m = euclSquared(v)
    m = sum(v.*v, 1);
end
