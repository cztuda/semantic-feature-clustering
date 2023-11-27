%% indicesBack = sortBackIndices(indices)
% Given some sorting indices (e.g. [~, ind] = sort(A)), this method finds
% some indices I such that the original order can be restored (i.e., such
% that for A2 = A(ind), A = A2(I) holds)
% 
% INPUT:
%   
function indicesBack = sortBackIndices(indices)
    indicesBack = 1:length(indices);
    indicesBack(indices) = indicesBack;
end