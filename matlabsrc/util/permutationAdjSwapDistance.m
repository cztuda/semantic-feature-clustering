%% len = permutationAdjSwapDistance(A)
% Compute the adjacent swap distance (see below) of the given permutation (in one-line notation!).
%
% INPUT:
%   A:      a permutation in one-line notation (1,n) array of unique integers 1 to n.
%
% OUTPUT:
%   len:    the adjacent swap distance for permutation A
%
%
% NOTES:
% 
% Permutation notations:
%   two-line:     (1  2  3  4  5)
%                 (2  5  4  3  1)
%   one-line:
%                 (2  5  4  3  1)
%   cyclic
%                 (1  2  5)(3  4)
% The notations above all define the same permutation.
%
% What is the adjacent swap distance:
% A transposition is a permutation that swaps to elements, so (a b) in cyclic form.
% An adjacent transposition is a permutation of the form (a a+1) (in cycle notation).
% Every permutation is a product of adjacent transpositions [1]. We call the minimum number of 
% adjacent transpositions to form this permutation the "length of the permutation" [2] or "adjacent
% swap distance" [4].
% In a permutation pi, two indices i and j (with i < j) are inverted if pi_i > pi_j. The number of
% inversions in a permutation gives the adjacent swap distance [4].
% The problem of finding a sequence of adjacent transpositions to sort a permutation is solved by
% the bubble sort algorithm [3].
%
% [1] https://www.ocf.berkeley.edu/~rohanjoshi/2019/01/04/odd-and-even-permutations/
% [2] https://math.stackexchange.com/questions/1309287/the-number-of-adjacent-transpositions
% [3] Zuylen, Bieron, Schalekamp, Yu: "An Upper Bound on the Number of Circular Transpositions to Sort a Permutation" (2014)
%       https://arxiv.org/pdf/1402.4867.pdf
% [4] Chitturi, Sudborough, Voit, Feng: "Adjacent Swaps on Strings", Computing and Combinatorics (2008) 
%       https://link.springer.com/content/pdf/10.1007/978-3-540-69733-6.pdf on pages 299-308
% 
function len = permutationAdjSwapDistance(A)
    len = sum(triu(map(@(a)A(:)'<a, A(:))), 'all');
end