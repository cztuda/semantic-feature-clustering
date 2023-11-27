
classdef TC_Empty < TrajectoryCharacteristic
    properties(Access=private)
        stateFilter
        n
    end
    
    methods(Access=public)
        function obj = TC_Empty(trajdata, varargin)
            %% obj = TC_Empty(trajdata, stateFilter=-1)
            % Constructor.
            % INPUT:
            %   trajdata:    universalFunction data struct, may be empty to create a handle
            obj@TrajectoryCharacteristic(trajdata, {}, {});
            obj.stateFilter = getFunctionArguments(varargin, {-1});
            if ~isempty(trajdata)
                obj.n = length(buildStateFilter(obj.stateFilter, trajdata.n));
            else
                obj.n = [];
            end
        end
        
        function [time_list, event_table] = getEventTable(~, ~)
            %% [time_list, event_table] = getEventTable(obj, trajdata)
            % Get the table with the extrema.
            % INPUT:
            %    trajdata:       (optional) get event table for this trajdata. If not set, the trajdata
            %                      given in the construction of this object is used, this argument is
            %                      required if this is a handle
            % OUTPUT:
            %    time_list:      first line of the table
            %    event_table:    the lines specified in obj.getRowIndices
            time_list = [];
            event_table = cell(5,0);
        end
        
        function [time_list, symbol_list, value_list] = getIndexwiseEventTable(obj, trajdata)
            if isempty(obj.n)
                if nargin > 1
                    obj.n = length(buildStateFilter(obj.stateFilter, trajdata.n));
                else
                    error('TC_Empty::getEventTable: A TC handle requires trajdata as second input.');
                end
            end
            time_list = repmat({}, 1, obj.n);
            symbol_list = repmat({}, 1, obj.n);
            value_list = repmat({[]}, 1, obj.n);
        end
        
        function obj = useAdditionalInformation(obj, varargin)
        end
        
        function rowInds = getRowIndices(~)
            rowInds = [];
        end
        
        function n = dim(~)
            n = 0;
        end
        
        function ish = isHandle(obj)
            ish = isempty(obj.t0tf);
        end
        
        function obj = update(obj, trajdata)
            obj.n = length(buildStateFilter(obj.stateFilter, trajdata.n));
        end
        
        function disp(obj)
            isH = '';
            if obj.isHandle()
                isH = '(handle)';
            end
            fprintf('TrajectoryCharacteristics: Empty %s\n', isH);
            disp@TrajectoryCharacteristic(obj);
        end
    end
end