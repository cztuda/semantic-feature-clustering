%% C = mat2cells(M)
% Convert the given matrix into a cell. Each entry is a single cell
% element.
function C = mat2cells(M)
    C = mat2cell(M, ones(1, size(M,1)), ones(1, size(M,2)));
end