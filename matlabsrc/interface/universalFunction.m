

classdef universalFunction < handle
  properties(Access = private, Hidden = true)
    instanceHandle;
    state % 0=undefined, -2=cpp-ND-ND-handle, -1=cpp-ND-Real-handle, 1=matlab-handle
    callOnCleanup;
  end
  properties(Access = private)
    functionHandle;
    n_input;
    n_output;
    upperBound;
    lowerBound;
  end
  methods(Static)
      function obj = CppFun(arg)
         obj = universalFunction();
          if nargin < 1
              return;
          end
          if ischar(arg)
              try
                obj.readCppFunFromFile(arg);
              catch
                  try
                    obj = universalFunction.loadobj(arg);
                  catch
                     error('Unable to create universal function with given input string.');
                  end
              end
          elseif isstruct(arg)
              obj = obj.createCppFun(arg);
          end          
      end
  end
  methods
    function obj = universalFunction(varargin)
        obj.instanceHandle = uint64(0);
        obj.functionHandle = 0;
        obj.upperBound = -Inf;
        obj.lowerBound = Inf;
        obj.state = 0;
        obj.callOnCleanup = onCleanup(@()obj.delete());
        if nargin == 1
            obj = obj.copyConstructor(varargin{1});
        elseif nargin > 1
            error('UniversalFunction:UniversalFunction: Invalid number of input arguments.');
        end
    end
    
    function b_u = getUpperBound(obj)
      b_u = obj.upperBound;
    end
    function b_l = getLowerBound(obj)
        b_l = obj.lowerBound;
    end
    
    function obj = delete_cpp(obj)
        if obj.state == -1
          universalFunction_mex('delete_piecewise', obj.instanceHandle);
        elseif obj.state == -2
          universalFunction_mex('delete_general', obj.instanceHandle);
        else
            return;
        end
        obj.instanceHandle = 0;
        obj.state = 0;
        obj.upperBound = -Inf;
        obj.lowerBound = Inf;
    end
    
    function obj = delete_matlab(obj)
      if obj.state == 1
        obj.functionHandle = 0;
        obj.state = 0;
        obj.upperBound = -Inf;
        obj.lowerBound = Inf;
      end
    end
    
    % setMatlabFun(obj, handle, insize, outsize=[], upperB=Inf, lowerB=-Inf)
    function obj = setMatlabFun(obj, handle, insize, varargin)
      switch(nargin)
          case 4
              outsize = varargin{1};
              upperB = Inf;
              lowerB = -Inf;
          case 5
              outsize = varargin{1};
              upperB = varargin{2};
              lowerB = -Inf;
          case 6
              outsize = varargin{1};
              upperB = varargin{2};
              lowerB = min(varargin{3}, upperB); % avoid invalid intervals
          otherwise
              outsize = [];
              upperB = Inf;
              lowerB = -Inf;
      end
      
      if obj.isCppFun()
        obj = obj.delete_cpp();
      end
      if obj.isMatlabFun()
        obj = obj.delete_matlab();
      end
      obj.functionHandle = handle;
      obj.state = 1;
      obj.n_input = insize;
      if isempty(outsize)
        obj.n_output = length( handle(zeros(insize,1)) );
      else
        obj.n_output = outsize;
      end
      obj.upperBound = upperB;
      obj.lowerBound = lowerB;
    end
    
    function h = getMatlabHandle(obj)
      if obj.isMatlabFun()
        h = obj.functionHandle;
      else
        h = [];
      end
    end
    
    function obj = setUpperBound(obj, ub)
        if obj.isMatlabFun()
            if isnumeric && length(ub) == obj.n_output && ub >= obj.lowerBound
                obj.upperBound = ub;
            end
        else
            error('universalFunction:setUpperBound: Invalid operation. Only valid if universal function wraps a matlab handle.');
        end
    end
    function obj = setLowerBound(obj, lb)
        if obj.isMatlabFun()
            if isnumeric && length(lb) == obj.n_output && lb <= obj.upperBound
                obj.lowerBound = lb;
            end
        else
            error('universalFunction:setLowerBound: Invalid operation. Only valid if universal function wraps a matlab handle.');
        end
    end
    
    % obj = setCppFun(obj, ptrID, cppType, upperB=Inf)
    % cppType: -1=piecewise, -2=general
    function obj = setCppFun(obj, ptrID, cppType, varargin) 
        if nargin > 3
            upperB = varargin{1};
        else
            upperB = Inf;
        end
      if (cppType ~= -1) && (cppType ~= -2)
        error('universalFunction:setCppFun: Unknown cppType');
      end
      if obj.isCppFun()
        obj = obj.delete_cpp();
      end
      if obj.isMatlabFun()
        obj = obj.delete_matlab();
      end
      obj.instanceHandle = ptrID;
      obj.state = cppType;
      obj.lowerBound = -Inf;
      if cppType == -1
        [obj.lowerBound, ub] = universalFunction_mex('get_piecewise_interval', ptrID);
        upperB = min(ub, upperB);
      end
      obj.upperBound = upperB;
    end
    
    % create a piecewise cpp function R -> R^n 
    % column vector of n input arguments that subdivide the input space
    % [n x m] matrix of result values, where each row is the result of the respective input
    % set type to 'linear' (default) or 'cubic' to define the interpolation method
    function obj = createCppFun(obj, data)
        switch(data.type)
            case 1
                ptrID = universalFunction_mex('generateLinear', data.times, data.coefficients);
                ub = data.times(end);
            case 3
                vec = obj.data2vec(data);
                ptrID = universalFunction_mex('generateCubicFromData', vec, data.n);
                ub = data.tf;
            otherwise
                error('Unknown type.');
        end
        if ptrID ~= 0
            obj = obj.setCppFun(ptrID, -1, ub);
        end
    end
    
    function obj = readCppFunFromFile(obj, filename)
      ptrID = universalFunction_mex('readFunctionFromFile', filename);
      if ~isnan(ptrID)
        [~, ub] = universalFunction_mex('get_piecewise_interval', ptrID);
        obj = obj.setCppFun(ptrID, -1, ub);
      else
        error('universalFunction:readCppFunFromFile: An error occured. Could not import function from file.\n');
      end
    end
        
    function bool = isMatlabFun(obj)
      bool = (obj.state == 1);
    end
    
    function bool = isCppFun(obj)
      bool = (obj.state < 0);
    end
    
    function result = evaluate(obj, value)
      if isempty(value)
          result = [];
          return;
      end
      switch obj.state
          case -1
            result = universalFunction_mex('eval_piecewise', obj.instanceHandle, value);
            return;
          case -2
            result = universalFunction_mex('eval_general', obj.instanceHandle, value);
            return;
           case 1
             if all(all(value >= obj.lowerBound)) && all(all(value <= obj.upperBound))
                result = obj.functionHandle(value);
             end
             return;
        otherwise 
          error('No handle assigned.');
      end    
    end
        
    function [result, t] = evaluateLinspace(obj, n)
        t0 = obj.getLowerBound();
        tf = obj.getUpperBound();
        if isinf(t0), t0 = 0; end
        if isinf(tf), if t0 < tf, tf = 1; else, tf = 0; end, end
        
        if tf-t0 ~= 0
            t = linspace(t0, tf, n);
            result = obj.evaluate(t);
        else
            result = [];
            t = [];
        end
    end
    
    function result = evaluateEnd(obj)
      result = obj.evaluate(obj.upperBound);
    end
    
    function result = evaluateDerivative(obj, value, varargin)
      if nargin < 3
          degree = 1;
      else
          degree = varargin{1};
      end
      if isempty(value)
          result = [];
          return;
      end
      if obj.state == -1
        result = universalFunction_mex('eval_derivative_piecewise', obj.instanceHandle, value, degree);
      else
        error('This function type does not support the computation of derivatives.');
      end
    end
    
    function obj_copy = copyWithCroppedOutputDimension(obj)
        if obj.n_output <= 1
            warning('Output dimension must be at least two to be cropped. Nothing has done');
            obj_copy = [];
            return;
        end
        if obj.state > 0
            obj_copy = universalFunction().copyConstructor(obj);
            handle = obj_copy.functionHandle;
            obj_copy.functionHandle = @(x)cropLastIndex(handle(x));
            obj_copy.n_output = obj_copy.n_output-1;
        elseif obj.state < 0
            switch(obj.state)
                case -1
                    handle_copy = universalFunction_mex('copyWithCroppedOutput_piecewise', obj.instanceHandle);
                case -2
                    handle_copy = universalFunction_mex('copyWithCroppedOutput_general', obj.instanceHandle);
                otherwise
                    error('universalFunction:copyWithCroppedOutputDimension: Unexpected function state.');
            end
            obj_copy = universalFunction().copyConstructor(obj, handle_copy);
        end
    end
    
    function obj_copy = copyWithDomainMapping(obj, t0, tf)
        if obj.state > 0
            obj_copy = universalFunction().copyConstructor(obj);
            handle = obj_copy.functionHandle;
            lb = obj.lowerBound;
            ub = obj.upperBound;
            if nargin < 3
                tf = ub;
            end
            obj_copy.functionHandle = @(x)handle(lb + (x-t0)*(ub-lb)/(tf-t0));
            obj_copy.lowerBound = t0;
            obj_copy.upperBound = tf;
        elseif obj.state < 0
            switch(obj.state)
                case -1
                    if nargin < 3
                        handle_copy = universalFunction_mex('copyWithDomainMapping_piecewise', obj.instanceHandle, t0, obj.upperBound);
                    else
                        handle_copy = universalFunction_mex('copyWithDomainMapping_piecewise', obj.instanceHandle, t0, tf);
                    end
                case -2
                    error('universalFunction:copyWithDomainMapping: Only piecewise functions are supported.');
                otherwise
                    error('universalFunction:copyWithDomainMapping: Unexpected function state.');
            end
            obj_copy = universalFunction().copyConstructor(obj, handle_copy);
            obj_copy.lowerBound = universalFunction_mex('getT0', obj_copy.instanceHandle);
            obj_copy.upperBound = universalFunction_mex('getTf', obj_copy.instanceHandle);
        end
    end
    
    function obj_copy = copyWithDomainCut(obj, t0, tf)
        if obj.state > 0
            obj_copy = universalFunction().copyConstructor(obj);
            obj_copy.lowerBound = t0;
            if nargin < 3
                obj_copy.upperBound = tf;
            end
        elseif obj.state < 0
            switch(obj.state)
                case -1
                    if nargin < 3
                        handle_copy = universalFunction_mex('copyWithDomainCut_piecewise', obj.instanceHandle, t0, obj.upperBound);
                    else
                        handle_copy = universalFunction_mex('copyWithDomainCut_piecewise', obj.instanceHandle, t0, tf);
                    end
                case -2
                    error('universalFunction:copyWithDomainCut: Only piecewise functions are supported.');
                otherwise
                    error('universalFunction:copyWithDomainCut: Unexpected function state.');
            end
            obj_copy = universalFunction().copyConstructor(obj, handle_copy);
            obj_copy.lowerBound = universalFunction_mex('getT0', obj_copy.instanceHandle);
            obj_copy.upperBound = universalFunction_mex('getTf', obj_copy.instanceHandle);
        end
    end
    
    function obj_copy = copyWithDomainShift(obj, offset)
        if obj.state > 0
            obj_copy = universalFunction().copyConstructor(obj);
            handle = obj_copy.functionHandle;
            obj_copy.functionHandle = @(x)handle(x)+offset;
            obj_copy.lowerBound = obj_copy.lowerBound + offset;
            obj_copy.upperBound = obj_copy.upperBound + offset;
        elseif obj.state < 0
            switch(obj.state)
                case -1
                    handle_copy = universalFunction_mex('copyWithDomainShift_piecewise', obj.instanceHandle, offset);
                case -2
                    error('universalFunction:copyWithDomainShift: Only piecewise functions are supported.');
                otherwise
                    error('universalFunction:copyWithDomainShift: Unexpected function state.');
            end
            obj_copy = universalFunction().copyConstructor(obj, handle_copy);
            obj_copy.lowerBound = obj_copy.lowerBound + offset;
            obj_copy.upperBound = obj_copy.upperBound + offset;
        end
    end
    
    function obj_copy = copyWithDomainCutAndShift(obj, t0)
        obj_copy = obj.copyWithDomainCut(t0).copyWithDomainShift(-t0);
    end
    
    function dim = getDim(obj, index)
        if obj.state < 0
            dim = [universalFunction_mex('NX', obj.instanceHandle), ...
                universalFunction_mex('NY', obj.instanceHandle)];
        elseif obj.state > 0
            dim = [obj.n_input, obj.n_output];
        else
            dim = [];
        end
        if nargin > 1
            if ~isempty(dim) && index==1
                dim = dim(1);
            elseif ~isempty(dim) && index==2
                dim = dim(2);
            end
        end
    end
    
    function h = getObjectHandle(obj)
        if obj.isCppFun
            h = obj.instanceHandle;
        else
            h = 0;
        end
    end
    
    function disp(obj)
        fprintf('  <a href="matlab: helpPopup(''universal_function'')">universal_function</a> with properties:\n');
        if obj.state < 0
            type = 'cpp-fun';
            in = obj.getDim();
            out = in(2);
            in = in(1);
        elseif obj.state > 0
            type = 'matlab';
            in = obj.n_input;
            out = obj.n_output;
        else
            type = 'undefined';
            in = 0;
            out = 0;
        end
        fprintf('    function type:     = %s\n', type);
        fprintf('    dimensions:        = %2i -> %2i\n', in, out);
        fprintf('    definition range:  = [%6.3f, %6.3f]\n', obj.lowerBound, obj.upperBound);
        
    end
    
    
    function s = saveobj(obj)
        if strcmp(warning('query', 'MATLAB:structOnObject').state, 'on')
            warning('off', 'MATLAB:structOnObject');
            cleanupObj = onCleanup(@()obj.warningCleanup());
        end
        s = struct(obj);
        s.state = obj.state;
        s.n_input = obj.n_input;
        s.n_output = obj.n_output;
        s.ub = obj.upperBound;
        s.lb = obj.lowerBound;
        s = rmfield(s,'callOnCleanup');
        if obj.state > 0
            s.fh = obj.functionHandle;
        else
            s.fh = 0;
        end
        if obj.state < 0
            s.data = universalFunction_mex('serialize', obj.instanceHandle);
        else
            s.data = 0;
        end
    end
    
  end  

  methods(Access = private)
    function delete(obj)
%        fprintf('deleted universalFunction\n');
      obj.delete_cpp(); % check isCppFun is done in delete_cpp, so no need to do it here
      % call delete_matlab is not (yet) necessary
    end
    
    function obj = copyConstructor(obj, univFun, handleID)
      if isa(univFun, 'universalFunction')
        cleanupObj = []; %#ok<NASGU>
        if strcmp(warning('query', 'MATLAB:structOnObject').state, 'on')
            warning('off', 'MATLAB:structOnObject');
            cleanupObj = onCleanup(@()obj.warningCleanup());
        end

        ufs = struct(univFun);
        if univFun.state < 0 % if this wraps a cpp-handle
            if nargin > 2
                newIH = handleID;
            else
                switch univFun.state
                    case -1
                        newIH = universalFunction_mex('copy_piecewise', ufs.instanceHandle, []);
                    case -2
                        newIH = universalFunction_mex('copy_general', ufs.instanceHandle, []);
                    otherwise
                        error('universalFunction:copy: Unexpected function state.')
                end
            end
            obj.instanceHandle = newIH;
        else % if this wraps a matlab handle, just copy:
            obj.instanceHandle = ufs.instanceHandle;
        end
        obj.state = ufs.state;
        obj.upperBound = ufs.upperBound;
        obj.lowerBound = ufs.lowerBound;
        obj.functionHandle = ufs.functionHandle;
        obj.n_input = ufs.n_input;
        obj.n_output = ufs.n_output;
      elseif isa(univFun, 'uint64')
          obj.instanceHandle = univFun;
          [obj.lowerBound, obj.upperBound] = universalFunction_mex('getDefinitionInterval', obj.instanceHandle);
          obj.n_input = universalFunction_mex('NX', obj.instanceHandle);
          obj.n_output = universalFunction_mex('NY', obj.instanceHandle);
          obj.state = -1 - double(obj.n_input > 1);
          obj.functionHandle = 0;
      else
        error('UniversalFunction: [Copy constructor]: Given argument is not a universal function.');
      end      
    end
  end
  
  methods(Static)
      function datastruct = readDataFromFile(filename)
          % read data:
          [n, isadj, type, nswitch, nGridpoints, t0, tf, names, times, coefficients, parameter] = universalFunction_mex('readDataFromFile', filename);
          % store data in struct:
          datastruct.n = n;
          if isadj >= 65
              datastruct.isadj = char(isadj);
          else
              datastruct.isadj = 'F';
          end
          datastruct.n2 = nswitch;
          datastruct.type = type;
          datastruct.nGridpoints = nGridpoints;
          datastruct.t0 = t0;
          datastruct.tf = tf;
          
          tmp = strtrim(strsplit(names, '|'));
          datastruct.names = tmp(~cellfun(@isempty, tmp));
          
          switch(type)
              case 1
                  datastruct.times = times;
                  coefficients = reshape(coefficients, n, [])';
                  datastruct.coefficients = coefficients;
              case 3
                  datastruct.times = reshape(times(1:end-1), 2, [])';
                  datastruct.lastEntry = coefficients((end-n+1):end);
                  datastruct.coefficients = reshape(coefficients(1:end-n), 4, [])';
              otherwise
                  datastruct.times = [];
                  datastruct.coefficients = [];
          end
          datastruct.parameter = parameter;
      end
    
      function obj = loadobj(s)
          if isstruct(s)
			  obj = universalFunction();
              obj.state = s.state;
              obj.functionHandle = s.fh;
              obj.n_input = s.n_input;
              obj.n_output = s.n_output;
              obj.upperBound = s.ub;
              if isfield(s, 'lb')
                obj.lowerBound = s.lb;
              else
                  obj.lowerBound = 0;
              end
              if s.state < 0
                  try
                      handle = universalFunction_mex('unserialize', s.data, 0);
                  catch
                      handle = 0;
                  end
              else
                  handle = 0;
              end
              obj.instanceHandle = handle;
              if obj.instanceHandle == 0
                  obj.state = 0;
              end
          elseif ischar(s)
              obj = universalFunction(universalFunction_mex('unserialize', s, 0));
          else
              obj = s;
          end
      end
  end
  
  methods(Access = private, Static)
    function vec = data2vec(data)
      vec = [data.times'; reshape(data.coefficients', data.n*4, [])];
      vec = [vec(:); data.times(end); data.lastEntry];
    end  
    function warningCleanup()
        warning('on', 'MATLAB:structOnObject');
    end
  end
  
end
