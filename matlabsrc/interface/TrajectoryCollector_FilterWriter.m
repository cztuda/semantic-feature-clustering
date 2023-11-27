classdef TrajectoryCollector_FilterWriter
    properties(Access=private)
        handle;
    end
    
    methods(Access=public)
        function obj = TrajectoryCollector_FilterWriter(handle)
            obj.handle = handle;
        end
        function problem = writeFilterHandle(obj, problem)
            problem.scfilter = obj.handle;
        end
    end
end