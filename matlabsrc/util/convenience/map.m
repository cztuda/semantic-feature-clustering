%% res = map(handle, arg)
% res = cell2mat(cellfun(handle, arg, 'UniformOutput', false));
%   or
% res = cell2mat(arrayfun(handle, arg, 'UniformOutput', false));
function res = map(handle, arg)
    if iscell(arg)
        res = cellfun(handle, arg, 'UniformOutput', false);
    else
        res = arrayfun(handle, arg, 'UniformOutput', false);
    end
    try
        res = cell2mat(res);
    catch ME
        if ~strcmp(ME.identifier, 'MATLAB:cell2mat:UnsupportedCellContent')
            rethrow(ME);
        end
    end
end