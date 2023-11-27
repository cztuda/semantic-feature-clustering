%% res = findConsecutiveIntegers(array)
% Find start and end of consecutive integers in the given array.
%
% INPUT:
%   array:  [1,n] array of integers
% OUTPUT:
%   res:    [2,m] array. First row gives start index of consecutive integers, last row gives the end
%             index of consecutive elements
%   
% EXAMPLE:
%   >> A = [1, 4, 5, 6, 9, 11, 12, 15];
%   >> findConsecutiveIntegers(A)
%   ans =
%        1     2     5     6     8
%        1     4     5     7     8
function res = findConsecutiveIntegers(array)
    t = diff(array(:)')==1;
    y = [t, false];
    x = xor(y, [false,t]);
    ii = cumsum(~(x|y) + y.*x);
    res = map(@(i)[find(ii==i,1);find(ii==i,1,'last')], 1:ii(end));
end