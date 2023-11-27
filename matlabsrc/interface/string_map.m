%% string_map
% Lookup table for (double-) values with char-arrays as keys.
% Brings the std::map<std::string, double> C++-class to Matlab
%
classdef string_map < handle
  properties(SetAccess = private, Hidden = true)
    objectHandle;
    callOnCleanup;
  end
  
  methods      
        %% Constructor - Create a new C++ class instance 
        function this = string_map(varargin)
            
            if nargin == 0 
                this.objectHandle = stringmap_mex('new');
                this.callOnCleanup = onCleanup(@()this.delete());
                
            elseif nargin == 2
                this.objectHandle = stringmap_mex('new', varargin{1}, varargin{2});
                this.callOnCleanup = onCleanup(@()this.delete());
                
            elseif nargin == 1 && isa(varargin{1}, 'uint64')
                this.objectHandle = varargin{1};
                this.callOnCleanup = onCleanup(@()this.delete());
                
            elseif nargin == 1 && isstruct(varargin{1})
                s = varargin{1};
                this.objectHandle = stringmap_mex('new', s.keys, s.values);
                this.callOnCleanup = onCleanup(@()this.delete());
                
            else
                error('Unexpected intput.');
            end
        end
        
        function obj2 = copy(obj)
            obj2 = string_map(stringmap_mex('copy', obj.objectHandle));            
        end
                
        function obj = insert(obj, keys, values)
            stringmap_mex('insert', obj.objectHandle, keys, values);
        end
        
        function obj = remove(obj, key)
            stringmap_mex('remove', obj.objectHandle, key);
        end
        
        function values = lookup(obj, keys)
            values = stringmap_mex('lookup', obj.objectHandle, keys);
        end
        
        function keys = allKeys(obj)
            keys = stringmap_mex('all_keys', obj.objectHandle);
        end
        
        
        function id = getObjectHandle(obj)
            id = obj.objectHandle;
        end
        
        % custom save-function
        % get an object and return a struct
        function s = saveobj(obj)
            keys = obj.allKeys();
            values = obj.lookup(keys);
            s = struct('keys', {keys}, 'values', values);
        end

      function disp(obj)
          keys = obj.allKeys();
          values = obj.lookup(keys);
          fprintf('string_map object\n')
          keyL = max(map(@(c)length(c), keys));
          for i = 1:length(keys)
              if rem(values(i),1)> 0
                  fprintf(['  %', num2str(keyL), 's -> %5.3f\n'], keys{i}, values(i))
              else
                  fprintf(['  %', num2str(keyL), 's -> %5i\n'], keys{i}, values(i))
              end
          end
      end
      
  end  

  methods(Access = private)            
    %% Destructor - Destroy the C++ class instance
    function delete(this)
        if ~isempty(this.objectHandle)
            stringmap_mex('delete', this.objectHandle);
        end
    end
  end
  
  methods(Static)
      function obj = loadobj(s)
          if isstruct(s)
              obj = string_map(s);
          elseif isa(s, 'string_map')
              obj = s;
          end
      end
  end
  
end
