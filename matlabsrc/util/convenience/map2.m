%% res = map(handle, arg1, ..., ['UniformOutput', true/false], ['SecureCall', defaultValue)
% Execute the given handle on each element of the given array or cell. If
% an error occurs on execution, the given default argument is used instead
% of the handle output.
%
% res = cell2mat(cellfun(handle, arg1, ..., 'UniformOutput', false));
%   or
% res = cell2mat(arrayfun(handle, arg1, ..., 'UniformOutput', false));
%
% OPTIONAL ARGUMENTS:
%   UniformOutput:      (default=true) return an array if possible
%   SecureCall:         if this is set, then the handle calls are protected
%                         with a try-catch and the defaultValue is used as 
%                         return value in the catch block 
function res = map2(handle, varargin)
    [args, unifOut, default] = getOptionalArguments(varargin);
    if ~isa(default, 'EMPTY_DEFAULT_ARGUMENT')
        h = @(v)secure_handle(handle, default, v);
    else
        h = handle;
    end

    if isempty(args)
        res = [];
        return;
    end
    if iscell(args{1})
        res = cellfun(h, args{:}, 'UniformOutput', false);
    else
        res = arrayfun(h, args{:}, 'UniformOutput', false);
    end

    if unifOut
        try
            res = cell2mat(res);
        catch ME
            if ~strcmp(ME.identifier, 'MATLAB:cell2mat:UnsupportedCellContent') && ~strcmp(ME.identifier, 'MATLAB:cell2mat:MixedDataTypes')
                rethrow(ME);
            end
        end
    end
end

function retval = secure_handle(handle, default, arg)
    try
        retval = handle(arg);
    catch
        retval = default;
    end
end

function [args, unifOut, secureCall] = getOptionalArguments(args)
    unifOut = true;
    secureCall = EMPTY_DEFAULT_ARGUMENT;
    if any(cellfun(@ischar, args))
        iSel = find(cellfun(@ischar, args));
        markDel = false(size(args));
        i = find(strcmp(args(iSel), 'UniformOutput'), 1, 'last');
        if ~isempty(i) && length(args) >= iSel(i)+1
            unifOut = args{iSel(i)+1};
            markDel([iSel(strcmp(args(iSel), 'UniformOutput')), iSel(strcmp(args(iSel), 'UniformOutput'))+1]) = true;
        end
        i = find(strcmp(args(iSel), 'SecureCall'), 1, 'last');
        if ~isempty(i) && length(args) >= iSel(i)+1
            secureCall = args{iSel(i)+1};
            markDel([iSel(strcmp(args(iSel), 'SecureCall')), iSel(strcmp(args(iSel), 'SecureCall'))+1]) = true;
        end
        args(markDel)=[];
    end
end

