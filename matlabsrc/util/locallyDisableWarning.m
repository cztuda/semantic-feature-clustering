%% cleanupObj = locallyDisableWarning(wrnIdentifier1, wrnIdentifier2, ...)
% Temporarily disable one or more warning(s). The warnings are re-enabled
% as soon as the returned object cleanupObj is deleted.
function cleanupObj = locallyDisableWarning(varargin)
    oldStates = cellfun(@(c)warning('query', c).state, varargin, 'UniformOutput', false);
    cellfun(@(c)warning('off', c), varargin);
    cleanupObj = onCleanup(@()cellfun(@(w, os) warning(os, w), varargin, oldStates));
end