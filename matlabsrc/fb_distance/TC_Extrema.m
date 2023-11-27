
classdef TC_Extrema < TrajectoryCharacteristic
   properties(Access=private)
       table
       prominences
       isMax
       theStateFilter
       prominenceFilter
   end
   
   methods(Access=public)
       function obj = TC_Extrema(trajdata, varargin)
           %% obj = TC_Extrema(trajdata, stateFilter=-1, varargin)
           % Constructor.
           % INPUT:
           %   trajdata:    universalFunction data struct, may be empty to create a handle
           %   stateFilter: (optional) which entries in the state vector should be used.
           %                  -1: (default) crop the last index
           %                  []: use all entries
           %                  vector(int): use the specified entries 
           % ADDITIONAL ARGUMENTS:
           %   promFilter:  (default=0.02)
           obj@TrajectoryCharacteristic(trajdata, {'^', 'v'}, {'maximum', 'minimum'});
           
           [obj.theStateFilter, obj.prominenceFilter] = getFunctionArguments(varargin, {-1}, 'promFilter', 0.02);
           
           if ~isempty(trajdata)
                obj = obj.registerTrajdata(trajdata);
            else
                obj.table = [];
                obj.prominences = [];
                obj.isMax = [];
                obj.t0tf = [];
            end
       end
       
       function obj = update(obj, trajdata)
           obj = update@TrajectoryCharacteristic(obj, trajdata);
           obj = obj.registerTrajdata(trajdata);
       end
       
       function [time_list, event_table] = getEventTable(obj, trajdata)
           %% [time_list, event_table] = getEventTable(obj, trajdata)
           % Get the table with the extrema.
           % INPUT:
           %    trajdata:       (optional) get event table for this trajdata. If not set, the trajdata
           %                      given in the construction of this object is used, this argument is
           %                      required if this is a handle
           % OUTPUT:
           %    time_list:      first line of the table
           %    event_table:    the lines specified in obj.getRowIndices
           if nargin > 1
                obj = obj.registerTrajdata(trajdata);
            elseif obj.isHandle()
                error('TC_Roots::getEventTable: A TC handle requires trajdata as second input.');
           end
           
           time_list = map(@(c)rowvec(c), obj.table);
    
           type = repmat('v', 1, length(time_list));
           type(map(@(c)rowvec(c), obj.isMax)) = '^';

           table_indices = map(@(i)i*ones(1, length(obj.table{i})), 1:length(obj.table));

           event_table = [mat2cells(type); mat2cells(table_indices); mat2cells(cell2mat(obj.prominences))];
           
           [time_list, I] = sort(time_list);
           event_table = event_table(:, I);           
       end
       
       function [time_list, symbol_list, value_list] = getIndexwiseEventTable(obj, trajdata)
           if nargin > 1
                obj = obj.registerTrajdata(trajdata);
            elseif obj.isHandle()
                error('TC_Roots::getIndexwiseEventTable: A TC handle requires trajdata as second input.');
           end
           
           time_list = obj.table;           
           symbol_list = cellfun(@(c)arrayfun(@(s)ifelse(s, '^', 'v'), c, 'UniformOutput', false), obj.isMax, 'UniformOutput',false); 
           value_list = obj.prominences;
       end
       
       function obj = useAdditionalInformation(obj, varargin)
       end
       
       function table = getRawTable(obj, trajdata)
           if nargin > 1
                obj = obj.registerTrajdata(trajdata);
                table = obj.table;
            elseif ~obj.isHandle()
                table = obj.table;
            else
                error('TC_Roots::getRawTable: A TC handle requires trajdata as second input.');
           end
       end
       
       function rowInds = getRowIndices(~)
           rowInds = [2,3,4];
       end
       
       function n = dim(obj)
            n = length(obj.table);
       end
       
       function ish = isHandle(obj)
           ish = isempty(obj.table);
       end
       
       function out = promFilter(obj, in)
           if nargin < 2
               % getter
               out = obj.prominenceFilter;
           else
               % setter
               obj.prominenceFilter = in;
               out = obj;
           end
       end
       
       function out = stateFilter(obj, in)
           if nargin < 2
               % getter
               out = obj.stateFilter;
           else
               % setter
               obj.stateFilter = in;
               out = obj;
           end
       end
       
       function disp(obj)
           isH = '';
           if obj.isHandle()
               isH = '(handle)';
           end
           fprintf('TrajectoryCharacteristics: Extrema %s\n', isH);
           sf = 'vec';
           if isempty(obj.theStateFilter), sf = '[]'; elseif isscalar(obj.theStateFilter) && obj.theStateFilter==-1, sf = '-1'; end
           fprintf('  State filter: %s\n', sf);
           fprintf('  Prominence filter: %5.2f\n', obj.prominenceFilter);
           disp@TrajectoryCharacteristic(obj);
       end
   end
   
   methods(Access=private)
       function obj = registerTrajdata(obj, trajdata)
           obj.t0tf = [trajdata.t0, trajdata.tf];
           sFilter = buildStateFilter(obj.theStateFilter, trajdata.n);
           
           if trajdata.type == 1
               [T_e, Te_isMax, Te_isMin] = findExtremaInLinear(trajdata, sFilter, 'filterExtrema', false, 'sepDims', true);
               Te_prom = computeProminence(T_e, Te_isMax, trajdata, true, 'isMinimum', Te_isMin);
               [obj.table, obj.prominences, obj.isMax] = obj.filterProminence(T_e, Te_prom, Te_isMax, obj.prominenceFilter);
               
           elseif trajdata.type == 3
               [T_e, Te_isMax, Te_prom] = findExtremaInCubic(trajdata, sFilter, 'omitStart', true, 'omitEnd', true, 'relProm', true);
               [obj.table, obj.prominences, obj.isMax] = obj.filterProminence(T_e, Te_prom, Te_isMax, obj.prominenceFilter);
               
           else
               error('TC_Roots: Trajdata must be linear or cubic.');
           end
       end
   end
   
   methods(Access=private, Static)
       function [T_e, Te_prom, Te_isMax] = filterProminence(T_e, Te_prom, Te_isMax, promFilter)
           if promFilter>0
               for i = 1:length(Te_prom)
                   sel = Te_prom{i}>=promFilter;
                   T_e{i} = T_e{i}(sel);
                   Te_isMax{i} = Te_isMax{i}(sel);
                   Te_prom{i} = Te_prom{i}(sel);
               end
           end
       end
   end
    
end