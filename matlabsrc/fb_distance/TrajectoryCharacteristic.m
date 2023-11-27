

classdef(Abstract) TrajectoryCharacteristic
    properties(Access=protected)
        symbols
        symbols_explanation
        t0tf
    end
    
    methods(Access=public, Abstract)
        %% getEventTable
        % Return the event table with a description of the trajectory.
        % INPUT:
        %   trajdata:     optional input, required for handles
        % OUTPUT:
        %   time_list:    list of times at which events occur
        %   event_table:  more information about the events listed in time_list. This is a cell
        %                   where the rows correspond with the row_indices (obj::getRowIndices)that 
        %                   indicate at the respective rows in TrajCharTable. The event_table does 
        %                   not contain the times at which the events occur.
        %   
        [time_list, event_table] = getEventTable(obj, trajdata);
        
        %% [time_list, event_table] = getIndexwiseEventTable(obj, trajdata)
        % Like getEventTable, but as separate cell for each index
        [time_list, symbol_list, value_list] = getIndexwiseEventTable(obj, trajdata);
        
        %% getRowIndices
        % OUTPUT:
        %   row_indices:  indicate to which of the rows in TrajCharTable the rows in the event
        %                   table (obj::getEventTable) belong to.
        rowInds = getRowIndices(obj);
        
        %% dim
        % INPUT:
        % Get the dimension of the trajectory. All TCs of the same category should have the same
        % dimension. The dimension gives the maximum number in the third TrajCharTable row.
        % If this is a handle, 0 is returned.
        n = dim(obj);
        
        %% useAdditionalInformation
        % Add some TC specific information that has to be used in the table.
        obj = useAdditionalInformation(obj, varargin);
                
        %% isHandle
        ish = isHandle(obj);
    end
    
    methods(Access=public)
        function obj = TrajectoryCharacteristic(trajdata, symbols, symbols_explanation)
            obj.symbols = symbols;
            obj.symbols_explanation = symbols_explanation;
            if ~isempty(trajdata)
                obj.t0tf = [trajdata.t0, trajdata.tf];
            else
                obj.t0tf = [];
            end
        end
                
        %% obj = update(obj, trajdata)
        % Update the trajectory object with the given trajectory data, 
        % e.g., because new trajdata has to be considered or because some properties of the
        % TrajectoryCharacteristic object have changed.
        function obj = update(obj, trajdata)
            obj.t0tf = [trajdata.t0, trajdata.tf];
        end
        
        
%         function [time_list, event_list] = getSingleLineEventTable(obj)
%             [time_list, event_list] = getEventTable(obj);
%             [time_list, event_list] = eventTableToSingleLine(obj, time_list, event_list);
%         end
        
        function S = getSymbol(obj, i)
            if nargin > 1
                S = obj.symbols{i};
            else
                S = obj.symbols;
            end
        end
        function S = getSymbolExplanation(obj, i)
            if nargin > 1
                S = obj.symbols_explanation{i};
            else
                S = obj.symbols_explanation;
            end
        end
        function t0AndTf = getTimeRange(obj, trajdata)
            %% getTimeRange
            % INPUT:
            %   trajdata:     optional input, required for handles
            % get the minimum and maximum time [t0, tf] of the trajectory data as two-element array
            if nargin > 1
                t0AndTf = [trajdata.t0, trajdata.tf];
            elseif ~obj.isHandle()
                t0AndTf = obj.t0tf;
            else
                error('TrajectoryCharacteristic::getTimeRange: A TC handle requires trajdata as second input.');
            end
        end
        
        function disp(obj)
            sym = obj.getSymbol();
            fprintf('  symbols: %s\n', ['''', strjoin(sym, ''', '''), '''']);
            if ~isempty(obj.t0tf)
                fprintf('  [t0, tf] = [%5.2f, %5.2f]\n', obj.t0tf(1), obj.t0tf(2));
            end
        end
    end
    
    methods(Access=protected)
%         function [time_list, event_list] = eventTableToSingleLine(obj, time_list, event_list)
%             
%         end
    end
end