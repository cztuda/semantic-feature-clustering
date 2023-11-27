%% function M = cropLastIndex(M)
% remove the last row from the given column
% INPUT:
%   M:      [n+1 x m] matrix
% OUTPUT:
%   M:      [n x m] matrix
function M = cropLastIndex(M)
    M = M(1:end-1,:);
end