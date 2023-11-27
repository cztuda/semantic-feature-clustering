

classdef problem_interface < handle
  properties(SetAccess = private, Hidden = true)
    objectHandle;
    callOnCleanup;
  end
  properties(SetAccess=private, GetAccess=public)
    typeLabel;
  end
  properties(Access=?TrajectoryCollector_FilterWriter)
      scfilter = @default_statecontrolfilter;
  end
  methods(Access=public,Static)
      function obj = uapendulum()
          obj = problem_interface('uapendulumproblem');
      end
      function obj = furutaPendulum()
          obj = problem_interface('furutapendulumproblem');
      end
      function obj = furutaSkewPendulum()
          obj = problem_interface('furutapendulum_skewproblem');
      end
      function obj = manutec()
          obj = problem_interface('manutecproblem');
      end
      function obj = frankaEmikaPanda()
          obj = problem_interface('frankaemikapandaproblem');
          slb = obj.getStateVariableLowerBound();
          sub = obj.getStateVariableUpperBound();
          s0 = 0.5*(slb+sub);
          c0 = zeros(obj.getNumberOfControlVariables, 1);
          obj.setInitialValues(s0, c0);
          obj.mjSetSimState(s0);
          obj.setFinalValues(s0, c0);
          
          obj.setEndTime(1.0, 0.1, 5.0);
          
          obj.setControlVariableLowerBound(0.01*obj.getControlVariableLowerBound());
          obj.setControlVariableUpperBound(0.01*obj.getControlVariableUpperBound());
      end
      function obj = mujocoFurutaPendulum()
          obj = problem_interface('mjfurutapendulumproblem');          
          pold = problem_interface.furutaSkewPendulum();
          
          obj.setStateVariableLowerBound(pold.getStateVariableLowerBound());
          obj.setStateVariableUpperBound(pold.getStateVariableUpperBound());
          obj.setControlVariableLowerBound(-0.2);
          obj.setControlVariableUpperBound(0.2);
            
          obj.setStartTime(pold.getStartTime());
          [tf, tfl, tfu] = pold.getEndTime();
          obj.setEndTime(tf, tfl, tfu);
            
          obj.setInitialValues([0;pi;0;0], 0);
          obj.setFinalValues([0;0;0;0], 0);
      end
      function obj = mujocoHumanoid()
          obj = problem_interface('mujocohumanoidproblem');
      end
  end
  methods      
        %% Constructor - Create a new C++ class instance 
        function this = problem_interface(type)
          if ischar(type)
              switch(type) % some 'shortcuts' for problem names:
                  case 'PENDULUM'
                      type = 'uapendulumproblem';
                  case 'FURUTA'
                      type = 'furutapendulumproblem';
                  case 'MANUTEC'
                      type = 'manutecproblem';
                  case 'PANDA'
                      type = 'frankaemikapandaproblem';
                  case 'HUMANOID'
                      type = 'mujocohumanoidproblem';
              end
            [this.objectHandle, this.typeLabel] = problem_interface_mex('new', type);
            this.callOnCleanup = onCleanup(@()this.delete());
          elseif isa(type, 'uint64')
              this.objectHandle = type;
              this.typeLabel = 'UNKNOWN';
              this.callOnCleanup = onCleanup(@()this.delete());
          elseif isstruct(type) && isfield(type, 'data') && isfield(type, 'typeLabel')
              if type.data(1)~='('
                this.objectHandle = problem_interface_mex('unserialize', type.data, 0);
              else
                  this.objectHandle = [];
              end
              if isempty(this.objectHandle)
                  error('problem_interface:loadobj: Failed to unserialize instance.');
              end
              this.typeLabel = type.typeLabel;
              if isfield(this, 'scfilter')
                  this.scfilter = type.scfilter;
              end
              this.callOnCleanup = onCleanup(@()this.delete());
              
          end
        end
        
        function obj2 = copy(obj)
%             [handle2, label2] = problem_interface_mex('copy', obj.objectHandle, obj.typeLabel);
%             obj2 = problem_interface(handle2);
%             obj2.typeLabel = label2;
            obj2 = problem_interface(obj.typeLabel);
            obj2.scfilter = obj.scfilter;
            
            obj2.setConstants(obj.getConstants());
            obj2.setControlIsActiveAtEnd(obj.getControlIsActiveAtEnd());
            obj2.setControlIsActiveAtStart(obj.getControlIsActiveAtStart());
            obj2.setControlIsAngle(obj.getControlIsAngle());
            obj2.setControlVariableLowerBound(obj.getControlVariableLowerBound());
            obj2.setControlVariableUpperBound(obj.getControlVariableUpperBound());
%             obj2.setDynamicMode(obj.getDynamicMode());
            [tmp1, tmp2, tmp3] = obj.getEndTime();
            obj2.setEndTime(tmp1, tmp2, tmp3);
            [tmp1, tmp2] = obj.getFinalValues();
            obj2.setFinalValues(tmp1, tmp2);
            [tmp1, tmp2] = obj.getInitialValues();
            obj2.setInitialValues(tmp1, tmp2);
            obj2.setName(obj.getName());
            obj2.setParameters(obj.getParameters());
            obj2.setStateIsActiveAtEnd(obj.getStateIsActiveAtEnd());
            obj2.setStateIsActiveAtStart(obj.getStateIsActiveAtStart());
            obj2.setStateIsAngle(obj.getStateIsAngle());
            obj2.setStateVariableLowerBound(obj.getStateVariableLowerBound());
            obj2.setStateVariableUpperBound(obj.getStateVariableUpperBound());
            
        end
        
        function nstate = getNumberOfStateVariables(this)
          nstate = problem_interface_mex('getNumberOfStateVariables', this.objectHandle);
        end
        
        function ncontrol = getNumberOfControlVariables(this)
          ncontrol = problem_interface_mex('getNumberOfControlVariables', this.objectHandle);
        end
        
        function name = getName(this)
          name = problem_interface_mex('getProblemName', this.objectHandle);
        end
        
        function setName(this, name)
        %% setName(this, name)
          problem_interface_mex('setProblemName', this.objectHandle, name);
        end
        
        function setInitialGuessWithMergedLagrange(this, varargin)
        %% setInitialGuessWithMergedLagrange(this, stateguess=[], controlguess=[], validX=0)
            [stateguess, controlguess, validX] = getFunctionArguments(varargin, {[], [], 0});
            setInitialGuess(this, stateguess, controlguess, [], validX, true);
        end
        
        
        function success = setInitialValues(this, state, control)
        %% success = setInitialValues(this, state, control)
            success = problem_interface_mex('setInitialValues', this.objectHandle, state, control);
        end
        
        function [state, control] = getInitialValues(this)
        %% [state, control] = getInitialValues(this)
            if nargout > 1
                [state, control] = problem_interface_mex('getInitialValues', this.objectHandle);
            else
                state = problem_interface_mex('getInitialValues', this.objectHandle);
                control = [];
            end
        end
        
        function success = setFinalValues(this, state, control)
        %% success = setFinalValues(this, state, control)
            success = problem_interface_mex('setFinalValues', this.objectHandle, state, control);
        end
        
        function [state, control] = getFinalValues(this)
        %% [state, control] = getFinalValues(this)
            if nargout > 1
                [state, control] = problem_interface_mex('getFinalValues', this.objectHandle);
            else
                state = problem_interface_mex('getFinalValues', this.objectHandle);
                control = [];
            end
        end
        
        function stateLB = getStateVariableLowerBound(this)
            stateLB = problem_interface_mex('getStateVariableLowerBound', this.objectHandle);
        end
        
        function stateUB = getStateVariableUpperBound(this)
            stateUB = problem_interface_mex('getStateVariableUpperBound', this.objectHandle);
        end
        
        function controlLB = getControlVariableLowerBound(this)
            controlLB = problem_interface_mex('getControlVariableLowerBound', this.objectHandle);
        end
        
        function controlUB = getControlVariableUpperBound(this)
            controlUB = problem_interface_mex('getControlVariableUpperBound', this.objectHandle);
        end
        
        
        function setStateVariableLowerBound(this, stateLB)
            problem_interface_mex('setStateVariableLowerBound', this.objectHandle, stateLB);
        end
        
        function setStateVariableUpperBound(this, stateUB)
            problem_interface_mex('setStateVariableUpperBound', this.objectHandle, stateUB);
        end
        
        function setControlVariableLowerBound(this, controlLB)
            problem_interface_mex('setControlVariableLowerBound', this.objectHandle, controlLB);
        end
        
        function setControlVariableUpperBound(this, controlUB)
            problem_interface_mex('setControlVariableUpperBound', this.objectHandle, controlUB);
        end
        
        function setStateIsAngle(this, isAngle)
            problem_interface_mex('setStateIsUnconstrainedAngle', this.objectHandle, isAngle);
        end
        
        function isAngle = getStateIsAngle(this)
            isAngle = problem_interface_mex('getStateIsUnconstrainedAngle', this.objectHandle);
        end
        
        function setControlIsAngle(this, isAngle)
            problem_interface_mex('setControlIsUnconstrainedAngle', this.objectHandle, isAngle);
        end
        
        function isAngle = getControlIsAngle(this)
            isAngle = problem_interface_mex('getControlIsUnconstrainedAngle', this.objectHandle);
        end
        
        function setStartTime(this, time)
        %% setStartTime(this, time)
            problem_interface_mex('setStartTime', this.objectHandle, time);
        end
        
        function setEndTime(this, time, lowerB, upperB)
            %% setEndTime(this, time, lowerB, upperB)
            if nargin < 3
                lowerB = time;
            end
            if nargin < 4
                upperB = time;
            end
            problem_interface_mex('setEndTime', this.objectHandle, time, lowerB, upperB);
        end
        
        function time = getStartTime(this)
        %% time = getStartTime(this)
          time = problem_interface_mex('getStartTime', this.objectHandle);
        end
        
        function [time, lowerB, upperB] = getEndTime(this)
        %% [time, lowerB, upperB] = getEndTime(this)
          res = problem_interface_mex('getEndTime', this.objectHandle);
          if numel(res) == 3
            time = res(1);
            lowerB = res(2);
            upperB = res(3);
          end
        end
        
        function paramarray = getParameters(this)
          paramarray = problem_interface_mex('getParameters', this.objectHandle);
        end
        
        function param = getParameter(this, index)
          param = problem_interface_mex('getParameter', this.objectHandle, index);
        end
        
        function constarray = getConstants(this)
          constarray = problem_interface_mex('getConstants', this.objectHandle);
        end
        
        function const = getConstant(this, index)
          const = problem_interface_mex('getConstant', this.objectHandle, index);
        end
        
        function ds = evaluateDynamics(this, state, control, varargin)
        %% ds = evaluateDynamics(this, state, control, time=0)
          if nargin >= 4
              time = varargin{1};
          else
              time = 0;
          end
          ds = problem_interface_mex('evaluateDynamics', this.objectHandle, state, control, time);
        end
        
        function [J_x, J_u, J_p] = evaluateJacobians(this, state, control, varargin)
        %% [J_x, J_u, J_t] = evaluateJacobians(this, state, control, time=0)
          if nargin >= 4
              time = varargin{1};
          else
              time = 0;
          end
          switch(nargout)
              case 0
                  return;
              case 1
                 J_x = problem_interface_mex('evaluateJacobians', this.objectHandle, state, control, time);
              case 2
                 [J_x, J_u] = problem_interface_mex('evaluateJacobians', this.objectHandle, state, control, time);
              otherwise
                 [J_x, J_u, J_p] = problem_interface_mex('evaluateJacobians', this.objectHandle, state, control, time);              
          end
          
          %transform output from vectors to the actual Jacobian matrices:
          nx = this.getNumberOfStateVariables();
          nu = this.getNumberOfControlVariables();
          np = length(this.getConstants());
          if(size(J_x,2) > 1)
              J_x = permute(reshape(J_x, nx, nx, []), [2, 1, 3]);
              if(exist('J_u', 'var'))
                  J_u = permute(reshape(J_u, nu, nx, []), [2, 1, 3]);
              end
              if(exist('J_p', 'var'))
                  J_p = permute(reshape(J_p, np, nx, []), [2, 1, 3]);
              end  
          else
              J_x = reshape(J_x, nx, [])';
              if(exist('J_u', 'var'))
                  J_u = reshape(J_u, nu, [])'; 
              end
              if(exist('J_p', 'var')) 
                  J_p = reshape(J_p, np, [])';
              end  
          end
        end
        
        function lagr = evaluateLagrangeIntegrand(this, state, control, varargin)
        %% lagr = evaluateLagrangeIntegrand(this, state, control, time=0)
          if nargin >= 4
              time = varargin{1};
          else
              time = 0;
          end
          lagr = problem_interface_mex('evaluateLagrangeTerm', this.objectHandle, state, control, time);
        end
        
        function mayer = evaluateMayerTerm(this, stateL, controlL, stateR, controlR, tf)
        %% mayer = evaluateMayerTerm(this, stateL, controlL, stateR, controlR, tf)
          mayer = problem_interface_mex('evaluateMayerTerm', this.objectHandle, stateL, controlL, stateR, controlR, tf);
        end
        
        function makeStatesConsistent(this)
            problem_interface_mex('makeStatesConsistent', this.objectHandle);
        end
        
        function mode = getDynamicMode(this)
            mode = problem_interface_mex('getDynamicMode', this.objectHandle);
        end
        
        function succ = setDynamicMode(this, mode)
            problem_interface_mex('setDynamicMode', this.objectHandle, mode);
        end
        
        function description = getDynamicModeDescription(this)
            description = problem_interface_mex('getDynamicModeDescription', this.objectHandle);
        end
        
        function success = setParameter(this, index, value)
        %% success = setParameter(this, index, value)
          if numel(index) ~= numel(value)
            error("Problem_interface:setParameter: The number of indices and of given parameters must be the same");
          end
          succ = false(1, numel(index));
          for i = 1:length(index)
            succ(i) = problem_interface_mex('setParameter', this.objectHandle, index(i), value(i));
          end
          success = all(succ);
        end
        
        function succ = setParameters(this, values)
            succ = this.setParameter(mat2cells((1:length(values))-1), values);
        end
                
        function b = isParameter(this, name)
            b = problem_interface_mex('isParameter', this.objectHandle, name);
        end
        
        function printParameterDescription(this)
            out = problem_interface_mex('getParameterDescription', this.objectHandle);
            fprintf(out);
        end
        function out = getParameterDescription(this, index)
            if nargin > 1
                out = problem_interface_mex('getParameterDescription', this.objectHandle, index);
            else
                out = problem_interface_mex('getParameterDescription', this.objectHandle);
            end
        end        
        
        
        function success = setConstant(this, index, value)
        %% success = setConstant(this, index, value)
          if ischar(index)
              index = {index};
          end
          if iscell(index)
              succ = false(1, numel(index));
              for i = 1:length(index)
                succ(i) = problem_interface_mex('setConstant', this.objectHandle, index{i}, value(i));
              end
          else
              if numel(index) ~= numel(value)
                error("Problem_interface:setConstant: The number of indices and of given constants must be the same");
              end
              succ = false(1, numel(index));
              for i = 1:length(index)
                succ(i) = problem_interface_mex('setConstant', this.objectHandle, index(i), value(i));
              end
          end
          success = all(succ);
        end
        
        function b = isConstant(this, name)
            b = problem_interface_mex('isConstant', this.objectHandle, name);
        end
        
        function succ = setConstants(this, values)
            succ = this.setConstant(mat2cells((1:length(values))-1), values);
        end
        
        function printConstantDescription(this)
          out = problem_interface_mex('getConstantDescription', this.objectHandle);
          fprintf(out);
        end
        function out = getConstantDescription(this, index)
            if nargin > 1
                out = problem_interface_mex('getConstantDescription', this.objectHandle, index);
            else
                out = problem_interface_mex('getConstantDescription', this.objectHandle);
            end
        end  
        
        function isactive = getStateIsActiveAtStart(this)
          isactive = problem_interface_mex('getStateIsFixedAtInitialTime', this.objectHandle);
        end
        function setStateIsActiveAtStart(this, isactive)
          problem_interface_mex('setStateIsFixedAtInitialTime', this.objectHandle, isactive);
        end
        
        function isactive = getStateIsActiveAtEnd(this)
          isactive = problem_interface_mex('getStateIsFixedAtFinalTime', this.objectHandle);
        end
        function setStateIsActiveAtEnd(this, isactive)
          problem_interface_mex('setStateIsFixedAtFinalTime', this.objectHandle, isactive);
        end
        
        function isactive = getControlIsActiveAtStart(this)
          isactive = problem_interface_mex('getControlIsFixedAtInitialTime', this.objectHandle);
        end
        function setControlIsActiveAtStart(this, isactive)
          problem_interface_mex('setControlIsFixedAtInitialTime', this.objectHandle, isactive);
        end
        
        function isactive = getControlIsActiveAtEnd(this)
          isactive = problem_interface_mex('getControlIsFixedAtFinalTime', this.objectHandle);
        end
        function setControlIsActiveAtEnd(this, isactive)
          problem_interface_mex('setControlIsFixedAtFinalTime', this.objectHandle, isactive);
        end
        
        function [state, control] = statecontrolfilter(obj, varargin)
            [state, control] = obj.scfilter(obj, varargin{:});
        end
        
        function id = getObjectHandle(obj)
            id = obj.objectHandle;
        end
        
        function obj = minus(obj1, obj2)
            if ~strcmp(obj1.typeLabel, obj2.typeLabel)
                error('The type of the objects must be the same.');
            end
            obj = problem_interface(obj1.typeLabel);
            
            c1 = obj1.getConstants;
            c2 = obj2.getConstants;
            obj.setConstants(c1-c2);
            
            obj.setStartTime(obj1.getStartTime()-obj2.getStartTime());
            [tf1,tfl1,tfu1] = obj1.getEndTime();
            [tf2,tfl2,tfu2] = obj2.getEndTime();
            obj.setEndTime(tf1-tf2, tfl1-tfl2, tfu1-tfu2);
            
            obj.setStateVariableLowerBound(obj1.getStateVariableLowerBound()-obj2.getStateVariableLowerBound());
            obj.setStateVariableUpperBound(obj1.getStateVariableUpperBound()-obj2.getStateVariableUpperBound());
            obj.setControlVariableLowerBound(obj1.getControlVariableLowerBound()-obj2.getControlVariableLowerBound());
            obj.setControlVariableUpperBound(obj1.getControlVariableUpperBound()-obj2.getControlVariableUpperBound());
            
            obj.setStateIsActiveAtStart(double(~xor(obj1.getStateIsActiveAtStart(), obj2.getStateIsActiveAtStart())));
            obj.setStateIsActiveAtEnd(double(~xor(obj1.getStateIsActiveAtEnd(), obj2.getStateIsActiveAtEnd())));
            obj.setControlIsActiveAtStart(double(~xor(obj1.getControlIsActiveAtStart(), obj2.getControlIsActiveAtStart())));
            obj.setControlIsActiveAtEnd(double(~xor(obj1.getControlIsActiveAtEnd(), obj2.getControlIsActiveAtEnd())));
            
            [is1, ic1] = obj1.getInitialValues();
            [is2, ic2] = obj2.getInitialValues();
            [fs1, fc1] = obj1.getFinalValues();
            [fs2, fc2] = obj2.getFinalValues();
            obj.setInitialValues(is1-is2, ic1-ic2);
            obj.setFinalValues(fs1-fs2, fc1-fc2);
        end
        
        % custom save-function
        % get an object and return a struct
        function s = saveobj(obj)
            s.typeLabel = obj.typeLabel;
            s.data = problem_interface_mex('serialize', obj.objectHandle);
            s.scfilter = obj.scfilter;
        end

      function disp(obj)
          fprintf('  %s: <a href="matlab: helpPopup(''problem_interface'')"> problem_interface  </a> with properties:\n', obj.getName());
          fprintf('    n_S=%d, n_C=%d\n', obj.getNumberOfStateVariables, obj.getNumberOfControlVariables);
          constants = problem_interface_mex('getConstantDescription', obj.objectHandle);
          tmp1 = strsplit(constants, newline);
          tmp1 = [repmat({'    '}, 1, length(tmp1)-1); tmp1(1:end-1); repmat({newline}, 1, length(tmp1)-1)];
          if ~isempty(tmp1)
            fprintf(cell2mat(tmp1(:)'));
          end
          tmp1 = obj.getStartTime();
          [tmp4, tmp5, tmp6] = obj.getEndTime();
          fprintf('\n    t_0 = %d,\tt_f = %d in [%d, %d]\n\n', tmp1, tmp4, tmp5, tmp6);
          [tmp1, tmp2] = obj.getInitialValues();
          tmp3 = obj.getStateIsActiveAtStart();
          tmp4 = obj.getControlIsActiveAtStart();
          fprintf('    S_start = %s,\tC_start = %s\n', vec2str(tmp1, tmp3), vec2str(tmp2, tmp4));
          [tmp1, tmp2] = obj.getFinalValues();
          tmp3 = obj.getStateIsActiveAtEnd();
          tmp4 = obj.getControlIsActiveAtEnd();
          fprintf('    S_end   = %s,\tC_end   = %s\n\n', vec2str(tmp1, tmp3), vec2str(tmp2, tmp4));
          fprintf('    %s <= S <= %s,\tangular?= %s\n', vec2str(obj.getStateVariableLowerBound), vec2str(obj.getStateVariableUpperBound), vec2str(obj.getStateIsAngle));
          fprintf('    %s <= C <= %s,\tangular?= %s\n', vec2str(obj.getControlVariableLowerBound), vec2str(obj.getControlVariableUpperBound), vec2str(obj.getControlIsAngle));
      end
      
      %% %%%%%  mujoco functions  %%%%% %%
      
      function [dx, xnew] = mjSimulateStep(obj, arg1, arg2, arg3, arg4)
          %% [xnew, dx] = mjSimulateStep(obj, varargin)
          % [...] = mjSimulateStep(u)
          % [...] = mjSimulateStep(x, u)
          % [...] = mjSimulateStep(u, t)
          % [...] = mjSimulateStep(x, u, t)
          %
          % Perform a single simulation step. State x and time t, if omitted, correspond to the models
          % current state and time. State and control must be given in the full mjDimensions, i.e.,
          % nq+nv+na for the state and nv+nu+nb for the control.
          %
          % Instead of vectors, matrices can be given (with a time vector), then all the states and
          % times are evaluated all at once.
          %
          res = cell(1, nargout);
          switch(nargin)
              case 2
                  [res{:}] = problem_interface_mex('mujoco_simulateStep', obj.objectHandle, arg1);
              case 3
                  [res{:}] = problem_interface_mex('mujoco_simulateStep', obj.objectHandle, arg1, arg2);
              case 4
                  [res{:}] = problem_interface_mex('mujoco_simulateStep', obj.objectHandle, arg1, arg2, arg3);
              case 5
                  [res{:}] = problem_interface_mex('mujoco_simulateStep', obj.objectHandle, arg1, arg2, arg3, arg4);
          end
          if nargout > 0
              if nargout > 1
                  dx = res{2};
              end
              xnew = res{1};
          end
      end
      function [X, T] = mjSimulate(obj, ctrl, x0)
          t0 = ctrl.getLowerBound();
          t_old = obj.mjGetTime();
          obj.mjSetTime(t0);
          if nargin > 2 && ~isempty(x0)
            obj.mjSetSimState(x0);  
          else
              x0 = obj.mjGetSimState();
          end
          cleanT0 = onCleanup(@()obj.mjSetTime(t_old));
          cleanX0 = onCleanup(@()obj.mjSetSimState(x0));
          
          tf = ctrl.getUpperBound();
          dt = obj.mjGetOptions.timestep;
          X = [x0, zeros(length(x0), floor(ctrl.getUpperBound()/(obj.mjGetOptions().timestep)))];
          T = t0:dt:tf;
          i = 2;
          while obj.mjGetTime() < tf
              u = ctrl.evaluate(obj.mjGetTime());
              [~,X(:,i)] = obj.mjSimulateStep(u);
              i=i+1;
          end
      end
      function ret = mjGetSimState(obj)
          ret = problem_interface_mex('mujoco_getCurrentState', obj.objectHandle);
      end
      function mjSetSimState(obj, val)
          problem_interface_mex('mujoco_setCurrentState', obj.objectHandle, val);
      end
      function ret = mjGetSimVelocity(obj)
          ret = problem_interface_mex('mujoco_getCurrentVelocity', obj.objectHandle);
      end
      function time = mjGetTime(obj)
          time = problem_interface_mex('mujoco_getCurrentTime', obj.objectHandle);
      end
      function mjSetTime(obj, time)
          problem_interface_mex('mujoco_setCurrentTime', obj.objectHandle, time);
      end
      function [nq, nv, na, nu, nb] = mjGetDimensions(obj)
          [nq, nv, na, nu, nb] = problem_interface_mex('mujoco_getDimensions', obj.objectHandle);
      end
      function options = mjGetOptions(obj)
          opt = problem_interface_mex('mujoco_getOptionData', obj.objectHandle);
          options = struct( ...
              'timestep', opt(0+1), ...
              'apirate', opt(1+1), ...
              'impratio', opt(2+1), ...
              'tolerance', opt(3+1), ...
              'noslip_tolerance', opt(4+1), ...
              'mpr_tolerance', opt(5+1), ...
              'gravity_0', opt(6+1), ...
              'gravity_1', opt(7+1), ...
              'gravity_2', opt(8+1), ...
              'wind_0', opt(9+1), ...
              'wind_1', opt(10+1), ...
              'wind_2', opt(11+1), ...
              'magnetic_0', opt(12+1), ...
              'magnetic_1', opt(13+1), ...
              'magnetic_2', opt(14+1), ...
              'density', opt(15+1), ...
              'viscosity', opt(16+1), ...
              'integrator', opt(17+1), ...
              'collision', opt(18+1), ...
              'cone', opt(19+1), ...
              'jacobian', opt(20+1), ...
              'solver', opt(21+1), ...
              'iterations', opt(22+1), ...
              'noslip_iterations', opt(23+1), ...
              'mpr_iterations', opt(24+1));
      end
      function mjSetOptions(obj, options)
          opt = zeros(25);
              opt(1) = options.timestep;
              opt(2) = options.apirate;
              opt(3) = options.impratio;
              opt(4) = options.tolerance;
              opt(5) = options.noslip_tolerance;
              opt(6) = options.mpr_tolerance;
              opt(7) = options.gravity_0;
              opt(8) = options.gravity_1;
              opt(9) = options.gravity_2;
              opt(10) = options.wind_0;
              opt(11) = options.wind_1;
              opt(12) = options.wind_2;
              opt(13) = options.magnetic_0;
              opt(14) = options.magnetic_1;
              opt(15) = options.magnetic_2;
              opt(16) = options.density;
              opt(17) = options.viscosity;
              opt(18) = options.integrator;
              opt(19) = options.collision;
              opt(20) = options.cone;
              opt(21) = options.jacobian;
              opt(22) = options.solver;
              opt(23) = options.iterations;
              opt(24) = options.noslip_iterations;
              opt(25) = options.mpr_iterations;
          problem_interface_mex('mujoco_setOptionData', obj.objectHandle, opt);
      end
      function dx = mjForward(obj, x, u, t)
          dx = problem_interface_mex('mujoco_forward', obj.objectHandle, x, u, t);
      end
      function mjPrintDescription(obj)
          fprintf(problem_interface_mex('mujoco_getDescription', obj.objectHandle));
      end
%       function mjScreenshot(obj, filename, camProps)
%           if nargin < 2 || isempty(filename)
%               filename = ['screenshot_', obj.getName()];
%           end
%           if nargin < 3
%               camProps = obj.mjGetDefaultCamera();
%           end
%           cp = [camProps.azimuth, camProps.distance, camProps.elevation, camProps.lookat(1), camProps.lookat(2), camProps.lookat(3)];
%           problem_interface_mex('mujoco_screenshot', obj.objectHandle, filename, cp);
%       end
%       function mjShowSim(obj, ctrlfun, camProps)
%           if nargin < 2 || isempty(ctrlfun)
%               ctrlfun = createPiecewiseLinear([0, 10], zeros(obj.getNumberOfControlVariables(), 2));
%           end
%           if nargin < 3
%               camProps = obj.mjGetDefaultCamera();
%           end
%           cp = [camProps.azimuth, camProps.distance, camProps.elevation, camProps.lookat(1), camProps.lookat(2), camProps.lookat(3)];
%           problem_interface_mex('mujoco_showsim', obj.objectHandle, ctrlfun, cp);
%       end
%       function mjRecordSim(obj, savename, ctrlfun, camProps)
%           
%       end
%       function mjRecordTraj(obj, savename, statefun, camProps)
%           
%       end
      function s = mjGetDefaultCamera(obj)
          v = problem_interface_mex('mujoco_defaultCamprops', obj.objectHandle);
          s = struct('azimuth', v(1), ...
                    'distance', v(2), ...
                    'elevation', v(3), ...
                    'lookat', v(4:6));
        if strcmp(obj.getName(), 'MjFurutaPendulum')
            s.distance = 0.55;
            s.azimuth = 115;
            s.elevation = -20;
            s.lookat(3) = s.lookat(3) - 0.1;
        end
      end
      
     %% mjVisualize(obj, savename, ctrl/state, cam=[])
     % 
     % Required variables in the save file, depending on the mode:
     % 
     %          | screensht | showSimul | recordSim | recordTrj |
     % ---------+-----------+-----------+-----------+-----------+
     % savename |     +     |           |     +     |     +     |
     %     ctrl |           |     +     |     +     |           |
     %    state |           |           |           |     +     |
     %      cam |    (+)    |    (+)    |    (+)    |    (+)    |
     % 
     function mjVisualize(obj, varargin)
         ocp = obj.saveobj().data;%#ok<NASGU>
         saveargs = {'ocp'};
         if nargin == 2
             if ischar(varargin{1})
                 savename = varargin{1};%#ok<NASGU>
                 saveargs = [saveargs, 'savename'];
             elseif isa(varargin{1}, 'universalFunction')
                 ctrl = varargin{1}.saveobj().data;%#ok<NASGU>
                 saveargs = [saveargs, 'ctrl'];
             end
         elseif nargin == 3
             if ischar(varargin{1})
                 savename = varargin{1};%#ok<NASGU>
                 saveargs = [saveargs, 'savename'];
             elseif isa(varargin{1}, 'universalFunction')
                 ctrl = varargin{1}.saveobj().data;%#ok<NASGU>
                 saveargs = [saveargs, 'ctrl'];
             end
             if isstruct(varargin{2})
                 cam = varargin{2};%#ok<NASGU>
                 saveargs = [saveargs, 'cam'];
             elseif isa(varargin{2}, 'universalFunction')
                 if obj.getNumberOfControlVariables() == selectIndex(varargin{2}.getDim(), 2)
                     ctrl = varargin{2}.saveobj().data;%#ok<NASGU>
                     saveargs = [saveargs, 'ctrl'];
                 else
                     state = varargin{2}.saveobj().data;%#ok<NASGU>
                     saveargs = [saveargs, 'state'];
                 end
             end
         elseif nargin == 4
             savename = varargin{1};%#ok<NASGU>
             saveargs = [saveargs, 'savename'];
             if obj.getNumberOfControlVariables() == selectIndex(varargin{2}.getDim(), 2)
                 ctrl = varargin{2}.saveobj().data;%#ok<NASGU>
                 saveargs = [saveargs, 'ctrl'];
             else
                 state = varargin{2}.saveobj().data;%#ok<NASGU>
                 saveargs = [saveargs, 'state'];
             end
             cam = varargin{3}; %#ok<NASGU>
             saveargs = [saveargs, 'cam'];
         else
             error('Unexpected number of input arguments.');
         end
         
         save('data/tmp_mjVisualize', saveargs{:});
         [~,~] = system('../cppsrc/build/bin/mujoco_printer data/tmp_mjVisualize.mat');
     end
  end

  methods(Access = private)            
    %% Destructor - Destroy the C++ class instance
    function delete(this)
        if ~isempty(this.objectHandle)
            problem_interface_mex('delete', this.objectHandle);
        end
    end
  end
  
  methods(Static)
      function obj = loadobj(s)
          if isstruct(s)
              obj = problem_interface(s);
          elseif isa(s, 'problem_interface')
              obj = s;
          end
      end
  end
  
end
