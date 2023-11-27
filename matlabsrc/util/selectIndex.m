%% A = selectIndex(A, index, columnIndex=0)
%
% Call:
% (1) A = selectIndex(A, index)
% Subselect the given indices of the given vector:
% If A is a matrix, its rows are selected as given by index.
%
% EXAMPLES:
% 
%  >> A = eye(3);
%  >> disp( selectIndex(A, [1,3]) )
%        1     0     0
%        0     0     1
%
%
%  >> A = [6;5;4;3;2;1];
%  >> disp( selectIndex(A, [2, 6]) )
%        5
%        1
% 
%
% Call:
% (2) A = selectIndex(A, rowIndex, columnIndex)
% Subselect the given indices in the given vector or matrix.
% Use 0 as a shortcut for all indices.
%
% EXAMPLES:
% 
%  >> A = eye(3);
%  >> disp( selectIndex(A, [1,3], 3) )
%        0
%        1
%
%
%  >> A = eye(3);
%  >> disp( selectIndex(A, 0, [1,3]) )
%        1     0
%        0     0
%        0     1
%
%
function A = selectIndex(A, index, columnIndex)
    if ~ismatrix(A)
        error('selectIndex: More than two dimensions are not supported.');
    end
    if isempty(A)
        if (size(A,1) == 0 && ~isempty(index)) || (size(A,2) == 0 && nargin > 2 && ~isempty(columnIndex))
            return;
        end
    end
    if nargin == 2
        if any(size(A) == 1)
            A = A(index);
        else
            A = A(index,:);
        end
    else
        if ~islogical(index) && all(index==0)
            index = 1:size(A,1);
        end
        if all(columnIndex==0)
            columnIndex = 1:size(A,2);
        end
        
        A = A(index,columnIndex);
    end
end

