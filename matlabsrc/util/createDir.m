%% path = createDir(varargin)
% Return the given path (similar to fullfile) and create the folder if not existent.
%
function path = createDir(varargin)
    path = fullfile(varargin{:});
    if ~isfolder(path)
        mkdir(path);
    end
end