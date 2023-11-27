%% v = firstNonempty(A)
% Find and return the first nonempty element in the array or cell A
function v = firstNonempty(A)
    e = map(@(c)~isempty(c), A);
    ind = find(e, 1);
    
    if isempty(ind)
        v = [];
    else    
        if iscell(A)
            v = A{ind};
        else
            v = A(ind);
        end
    end
end