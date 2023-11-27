%% val = ifelse(truefalse, iftrue, iffalse)
% Return iftrue, if truefalse is true, else returns iffalse
% This function allows true-false-statements in arrayfun or cellfun.
% Example of use:
%   >> cellfun(@(c)ifelse(isempty(c), 0, c), {1, 2, [], 3})
%   ans = [1,2,0,3]
%
% If truefalse is a char array, then the arguments iftrue and iffalse are expected to be also char
% arrays. The expressions iftrue and iffalse are evaluated in the calling workspace only if required.
%
%
% INPUT:
%   truefalse:  true or false
%   iftrue:     some value
%   iffalse:    some other value
% OUTPUT:
%   val:        the value iftrue or iffalse
function val = ifelse(truefalse, iftrue, iffalse)
    if ischar(truefalse)
        if evalin('caller', truefalse)
            val = evalin('caller', iftrue);
        else
            val = evalin('caller', iffalse);
        end
    else
        if numel(truefalse) == 1
            if truefalse
                val = iftrue;
            else
                val = iffalse;
            end
        else
            val = zeros(size(truefalse));
            if numel(iftrue) == 1
                val(truefalse) = iftrue;
            else
                val(truefalse) = iftrue(truefalse);
            end
            if numel(iffalse) == 1
                val(~truefalse) = iffalse;
            else
                val(~truefalse) = iffalse(~truefalse);
            end
        end
    end
end