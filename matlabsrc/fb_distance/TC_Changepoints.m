
classdef TC_Changepoints < TrajectoryCharacteristic
   properties(Access=private)
       table
       stateFilter
       windowSize
   end
   
   methods(Access=public)
       function obj = TC_Changepoints(trajdata, varargin)
           %% obj = TC_Changepoints(trajdata, windowSize=0.04, stateFilter=-1)
           % Constructor.
           % INPUT:
           %   trajdata:    universalFunction data struct, may be empty to create a handle
           %    windowSize: size of sliding window in which for fast changes is searched. Can be an
           %                  absolute value (number of frames) or a relative value (<1, share of
           %                  the number of all frames).
           %   stateFilter: (optional) which entries in the state vector should be used.
           %                  -1: (default) crop the last index
           %                  []: use all entries
           %                  vector(int): use the specified entries 
           %  ADDITIONAL ARGUMENTS:
           obj@TrajectoryCharacteristic(trajdata, {'l'}, {'changepoint'});
           [obj.windowSize, obj.stateFilter] = getFunctionArguments(varargin, {0.04, -1});
                      
           if ~isempty(trajdata)
               obj = obj.registerTrajdata(trajdata);
           else
               obj.table = [];
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
                error('TC_Changepoints::getEventTable: A TC handle requires trajdata as second input.');
           end
           
           time_list = map2(@(c)[c{1,:}], obj.table, 'SecureCall', []);
           if ~isempty(time_list)
               table_indices = map(@(i)i*ones(1, size(obj.table{i}, 2)), 1:length(obj.table));

               type = map2(@(c)c(2,:), obj.table, 'SecureCall', []);
               type = [type{:}];
               
               table_values = map2(@(c)[c{3,:}], obj.table, 'SecureCall', []);
               
               event_table = [type; mat2cells(table_indices); mat2cells(table_values)];

               [time_list, I] = sort(time_list);
               event_table = event_table(:, I);  
           else
               event_table = cell(3,0);
           end
       end
       
       function [time_list, symbol_list, value_list] = getIndexwiseEventTable(obj, trajdata)
           if nargin > 1
                obj = obj.registerTrajdata(trajdata);
            elseif obj.isHandle()
                error('TC_Changepoints::getIndexwiseEventTable: A TC handle requires trajdata as second input.');
           end
           
           time_list = map2(@(c)cell2mat(c(1,:)), obj.table, 'SecureCall', [], 'UniformOutput', false);
           symbol_list = map2(@(c)c(2,:), obj.table, 'SecureCall', [], 'UniformOutput', false);
           value_list = map2(@(c)cell2mat(c(3,:)), obj.table, 'SecureCall', [], 'UniformOutput', false);
       end
       
       function rowInds = getRowIndices(~)
           rowInds = [2,3,4];
       end
       
       function n = dim(obj)
           n = length(obj.table);
       end
       
       function obj = useAdditionalInformation(obj, varargin)
       end
       
       function ish = isHandle(obj)
           ish = isempty(obj.table);
       end
       
       function disp(obj)
           isH = '';
           if obj.isHandle()
               isH = '(handle)';
           end
           fprintf('TrajectoryCharacteristics: Changepoints %s\n', isH);
           if obj.windowSize < 1
               fprintf('  WindowSize: %5.4f\n', obj.windowSize);
           else
               fprintf('  WindowSize: %5i\n', obj.windowSize);
           end
           sf = 'vec';
           if isempty(obj.stateFilter), sf = '[]'; elseif isscalar(obj.stateFilter) && obj.stateFilter==-1, sf = '-1'; end
           fprintf('  State filter: %s\n', sf);
           disp@TrajectoryCharacteristic(obj);
       end
   end
    
   methods(Access=private)
       function obj = registerTrajdata(obj, trajdata)
           obj.t0tf = [trajdata.t0, trajdata.tf];
           if obj.windowSize < 1
               obj.windowSize = round(obj.windowSize*trajdata.nGridpoints);
           end           
           
           if trajdata.type == 1
               [chpts, salience] = findChangepointsInTimeSeries(trajdata.times, trajdata.coefficients, obj.windowSize);
               T = cellfun(@(tme, sal)obj.mapTable(tme, sal), chpts, salience, 'UniformOutput', false);
           elseif trajdata.type == 3
               values = [reshape(trajdata.coefficients(:,1), trajdata.n, []), trajdata.lastEntry];
               times = [trajdata.times(:,1); trajdata. times(end)];
               [chpts, salience] = findChangepointsInTimeSeries(times, values, obj.windowSize);
               T = cellfun(@(tme, sal)obj.mapTable(tme, sal), chpts, salience, 'UniformOutput', false);
           else
               error('TC_Changepoints: Trajdata must be linear or cubic.');
           end
           obj.table = T;           
       end
       
       function T = mapTable(obj, time, salience)           
           T = cell(3, length(time));
           if isempty(T)
               return;
           end
           
           T(1,:) = mat2cells(rowvec(time));
           T(2,:) = {obj.getSymbol(1)};
           T(3,:) = mat2cells(rowvec(salience));         
       end   
   end
       
end