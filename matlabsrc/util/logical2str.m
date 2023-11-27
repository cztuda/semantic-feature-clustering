%% s = logical2str(b, category)
% Represent a logical with a string.
% 
% INPUT:
%   b:
%   category:   either be a string denoting a predefined category or a 2-dim cell array {'iftrue', 'iffalse'}
%               Predefined categories are:
%                truefalse: {'true', 'false'}   <- (default)
%                yesno:     {'yes', 'no'}
%                tf:        {'y', 'n'}
%                janein:    {'ja', 'nein'}
% OUTPUT:
%   s:          string representation of the logical
%
function s = logical2str(b, category)
    if length(b) > 1
        if nargin > 1
            s = map2(@(c)logical2str(c, category), b, 'UniformOutput', false);
        else
            s = map2(@(c)logical2str(c), b, 'UniformOutput', false);
        end
        return;
    end
    if isempty(b)
        s = ''; 
        return; 
    end
    
    if nargin == 1
        category = {'true', 'false'};
    elseif ischar(category)
        category = getCategory(category);
    end
    
    s = category{2};
    if b
        s = category{1};
    end
end

function cat = getCategory(cat)
    switch(cat)
        case 'truefalse'
            cat = {'true', 'false'};
        case 'yesno'
            cat = {'yes', 'no'};
        case 'tf'
            cat = {'t', 'f'};
        case 'janein'
            cat = {'ja', 'nein'};
        otherwise
            error('logical2str: Unknown category.');
    end
end