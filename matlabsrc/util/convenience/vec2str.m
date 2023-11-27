
%% s = vec2str(vec)
% Transform a given vector in some string representation that is easy to
% read (and can be used e.g. in plots).
% INPUT:
%   vec:    some numeric vector
%   sel:    (optional) subselection of entries
% OUTPUT
%   s:      some string representation of this vector
function s = vec2str(vec, sel)
    if nargin == 1
        s = '[';
        for i = 1:length(vec)-1
            s = strcat(s, num2str(vec(i)), ', ');
        end
        if ~isempty(vec)
            s = strcat(s, num2str(vec(end)));
        end
        s = strcat(s, ']');
    else
        s = '[';
        for i = 1:length(vec)-1
            s = strcat(s, num2str_2(vec(i), sel(i)), ', ');
        end
        if ~isempty(vec)
            s = strcat(s, num2str_2(vec(end), sel(end)));
        end
        s = strcat(s, ']');
    end
end

function s = num2str_2(num , sel_i)
    if sel_i
        s = num2str(num);
    else
        s = '_';
    end
end