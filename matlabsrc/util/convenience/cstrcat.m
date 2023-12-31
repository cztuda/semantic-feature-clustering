% Copyright (C) 1994-2017 John W. Eaton
%
% This file is part of Octave.
%
% Octave is free software; you can redistribute it and/or modify it
% under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 3 of the License, or (at
% your option) any later version.
%
% Octave is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
% General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with Octave; see the file COPYING.  If not, see
% <http://www.gnu.org/licenses/>.

% -*- texinfo -*-
% @deftypefn {} {} cstrcat (@var{s1}, @var{s2}, @dots{})
% Return a string containing all the arguments concatenated horizontally
% with trailing white space preserved.
%
% For example:
%
% @example
% @group
% cstrcat ("ab   ", "cd")
%       @result{} "ab   cd"
% @end group
% @end example
%
% @example
% @group
% s = [ "ab"; "cde" ];
% cstrcat (s, s, s)
%       @result{} "ab ab ab "
%          "cdecdecde"
% @end group
% @end example
% @seealso{strcat, char, strvcat}
% @end deftypefn

% Author: jwe
function st = cstrcat (varargin)

  if (nargin == 0)
    % Special because if varargin is empty, iscellstr still returns
    % true but then "[varargin{:}]" would be of class double.
    st = "";
  elseif (iscellstr (varargin))
    st = [varargin{:}];
  else
    error ("cstrcat: arguments must be character strings");
  end

end