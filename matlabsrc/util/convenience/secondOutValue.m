%% secondOutValue(FUN, varargin)
% Return the second output of the given function FUN. The additional
% arguments in varargin are used as arguments for the function FUN.
function out = secondOutValue(FUN, varargin)
    [~, out] = FUN(varargin{:});
end
