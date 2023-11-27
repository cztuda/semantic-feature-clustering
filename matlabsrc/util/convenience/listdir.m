%% dlst = listdir(foldername, varargin) 
%   Return content of the given directory as cell of strings
%
% INPUT:
%   foldername: name of the folder to list
% OUTPUT:       
%   dlst:       the content of the folder as cell array
% ADDITIONAL ARGUMENTS:
%   files:      (default=true), list files
%   folders:    (default=true), list folders
%   dots:       (default=false), if true, also list '.' and '..'
%   filter:     (default=[]), a handle @(c)... that returns true if c is an acceptable name or a
%                 string representing a regular expression
function dlst = listdir(foldername, varargin)
    [getFiles, getFolders, getSelfParent, filter] = getFunctionArguments(varargin, ...
        'files', true, 'folders', true, 'dots', false, 'filter', []);
    
    dlst = {dir(foldername).name};
    sel = true(1,length(dlst));
    if ~getSelfParent
        sel = sel & rowvec(map(@(c)~(strcmp(c, '.')||strcmp(c, '..')), dlst));
    end
    if ~getFiles
        sel = sel & rowvec(~isfile(fullfile(foldername, dlst)));
    end
    if ~getFolders
        sel = sel & rowvec(~isfolder(fullfile(foldername, dlst)));
    end
    dlst = dlst(sel);
    if ~isempty(filter)
        if ischar(filter)
            filter = @(c)~isempty(regexp(c, filter, 'once'));
        end
        dlst = dlst(map(filter, dlst));
    end
end