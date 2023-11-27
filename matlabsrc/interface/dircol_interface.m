
classdef dircol_interface < handle
  properties(SetAccess = private, Hidden = true)
    instanceHandle;
  end
      
  methods
      function obj = dircol_interface(varargin)
        instanceHandle = [];
      end
    
    function disp(obj)
        fprintf('Dummy dircol object, code removed.\n');
    end
        
    function s = saveobj(obj)
        s = '';
    end
  end  

  
  methods(Static)
      function obj = loadobj(s)
          obj = dircol_interface(s);
      end
            
  end
end