%% [score, xy, xx, yy] = svrspell(X, Y, softMatchingMatrix=[], charWeights=[], fun_lengthW=[], fun_gapW=[], tX=[], tY=[])
%
% Similarity measure (metric) for two sequences.  
%
% INPUT:
%   X:                  sequence 1, must consist of integer entries >= 1
%   Y:                  sequence 2, must consist of integer entries >= 1
%   softMatchingMatrix: quadratic, symmetric, positive-semidefinite matrix, number of rows/cols must
%                         be at least the number of unique characters in X and Y.
%                       Entries denote the matching of two characters.
%   charWeights:        positive values, internally: softMatchMat = diag(charWeights);
%   fun_lengthW:        @(k, mue_k)...
%   fun_gapW:           
%   tX:                 running-lengths for X
%   tY:                 running-lengths for Y
%   
% OUTPUT:
%   score = sqrt(xx + yy - 2*xy)
%
function [score, xy, xx, yy] = svrspell(X, Y, varargin)
    [softMatchingMatrix, charWeights, fun_lengthW, fun_gapW, tX, tY, vX, vY] = ...
        getFunctionArguments(varargin, {[], [], [], [], [], [], [], []});
    
    if ~isempty(fun_lengthW)
        warning('svrspell: Use of length weighting function is buggy: grid-algorithm and trail-algorithm do not give the same result.')
    end
    if any(X<1) || any(Y<1)
        error('All values in X or Y must not be smaller than 1.');
    end
    
    if ~isempty(fun_lengthW)
        fun_lengthW = getByteStreamFromArray(fun_lengthW);
    end
    if ~isempty(fun_gapW) && ~isstruct(fun_gapW)
        fun_gapW = getByteStreamFromArray(fun_gapW);
    end
    
    [score, xy, xx, yy] = svrspell_mex(X-1, Y-1, softMatchingMatrix, charWeights, fun_lengthW, fun_gapW, tX, tY, vX, vY);
end
