%% [state, control] = default_statecontrolfilter(problem, time, state, control)
% Filter the state and control variables such that they can be used in the
% learner. If no state/control values are given, the number of state and
% control variables are returned.
% EXAMPLE
%
% >> [nx, nu] = furuta_statecontrolfilter(problem)
%       returns nx=4 and nu=1
%
% >> [X, U] = furuta_statecontrolfilter(problem, 0, [0.2; 0.1; 0.0; 0.3; 3.2], [-20]);
%       returns X=[0.2;0.1;0.0;0.3] and U=[3.2]
%
% INPUT:
%   problem:    the problem description object
%   state:      [5xn] a vector of state vectors
%   control:    [1xn] a vector of control values
%   time:       [1xn] a vector of time values
% OUTPUT:
%   state:      the transformed state vector
%   control:    the transformed control vector
% OPTIONAL ARGUMENTS:
%   invert:     (default=false) if true, the filtering is undone (missing
%                 information filled with zeros)
function [state, control] = default_statecontrolfilter(problem, varargin)
    [state, control, ~, ~] = getFunctionArguments(varargin, {[], [], []}, 'invert', false);
    
    if nargin == 2
        control = [];
    end
    if nargin == 1
        state = problem.getNumberOfStateVariables();
        control = problem.getNumberOfControlVariables();
        return;
    end

end