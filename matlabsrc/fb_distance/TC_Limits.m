
classdef TC_Limits < TrajectoryCharacteristic
   properties(Access=private)
       bounds % [2 x n]
       table
       stateFilter
       extrema
   end
   
   methods(Access=public)
       function obj = TC_Limits(trajdata, bounds, varargin)
           %% obj = TC_Limits(trajdata, bounds, varargin)
           % Constructor.
           % INPUT:
           %   trajdata:    universalFunction data struct, may be empty to create a handle
           %   bounds:      [lower; upper]
           %   stateFilter: (optional) which entries in the state vector should be used.
           %                  -1: (default) crop the last index
           %                  []: use all entries
           %                  vector(int): use the specified entries 
           %  ADDITIONAL ARGUMENTS:
           obj@TrajectoryCharacteristic(trajdata, {'L{', 'L}', 'L'}, {'limit_start', 'limit_end', 'limit_touch'});
           obj.stateFilter = getFunctionArguments(varargin, {-1});
           if size(bounds,2)==2 && size(bounds,1) ~= 2
               bounds = bounds';
           end           
           obj.bounds = bounds;
           
           if ~isempty(trajdata)
               obj = obj.registerTrajdata(trajdata);
           else
               obj.table = [];
           end
           obj.extrema = [];
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
           
           time_list = map2(@(c)[c{1,:}], obj.table, 'SecureCall', []);
           if ~isempty(time_list)
               table_indices = map(@(i)i*ones(1, size(obj.table{i}, 2)), 1:length(obj.table));

               type = map2(@(c)c(2,:), obj.table, 'SecureCall', []);
               type = [type{:}];

               event_table = [type; mat2cells(table_indices)];

               [time_list, I] = sort(time_list);
               event_table = event_table(:, I);  
           else
               event_table = cell(2,0);
           end
       end
       
       function [time_list, symbol_list, value_list] = getIndexwiseEventTable(obj, trajdata)
           if nargin > 1
                obj = obj.registerTrajdata(trajdata);
            elseif obj.isHandle()
                error('TC_Roots::getIndexwiseEventTable: A TC handle requires trajdata as second input.');
           end
           
           time_list = map2(@(c)cell2mat(c(1,:)), obj.table, 'SecureCall', [], 'UniformOutput', false);
           symbol_list = map2(@(c)c(2,:), obj.table, 'SecureCall', [], 'UniformOutput', false);
           value_list = map2(@(c)ones(1, size(c,2)), obj.table, 'SecureCall', [], 'UniformOutput', false);
       end
       
       function rowInds = getRowIndices(~)
           rowInds = [2,3];
       end
       
       function n = dim(obj)
           n = length(obj.table);
       end
       
       function obj = useAdditionalInformation(obj, varargin)
           if nargin > 1
                if isempty(varargin{1})
                    obj.extrema = [];
                elseif isa(varargin{1}, 'TC_Extrema')
                    if obj.isHandle()
                        obj.extrema = varargin{1};
                    else
                        obj.extrema = varargin{1}.getRawTable();
                    end
                end
            end
       end
       
       function ish = isHandle(obj)
           ish = isempty(obj.table);
       end
       
       function disp(obj)
           isH = '';
           if obj.isHandle()
               isH = '(handle)';
           end
           fprintf('TrajectoryCharacteristics: Limits %s\n', isH);
           fprintf('  Dimension: %2i\n', size(obj.bounds,2));
           fprintf('  knowns extrema: %s\n', logical2str(~isempty(obj.extrema), 'yesno'));
           sf = 'vec';
           if isempty(obj.stateFilter), sf = '[]'; elseif isscalar(obj.stateFilter) && obj.stateFilter==-1, sf = '-1'; end
           fprintf('  State filter: %s\n', sf);
           disp@TrajectoryCharacteristic(obj);
       end
   end
    
   methods(Access=private)
       function obj = registerTrajdata(obj, trajdata)
           obj.t0tf = [trajdata.t0, trajdata.tf];
           if isa(obj.extrema, 'TC_Extrema')
                obj.extrema = obj.extrema.getRawTable(trajdata);
            end
           
           bds = obj.bounds;
           if trajdata.type == 1
               lmts = findLimitsInLinear(trajdata, bds(2,:), bds(1,:), 'stateFilter', obj.stateFilter);
               T = cellfun(@(c)TC_Limits.mapTable(c), lmts, 'UniformOutput', false);
           elseif trajdata.type == 3
               lmts = findLimitsInCubic(trajdata, bds(2,:), bds(1,:), 'stateFilter', obj.stateFilter);
               T = cellfun(@(c)TC_Limits.mapTable(c), lmts, 'UniformOutput', false);
           else
               error('TC_Limits: Trajdata must be linear or cubic.');
           end
           obj.table = T;           
       end
       
   end
   
   methods(Access=private, Static)
       function T = mapTable(A)
           T = cell(size(A));
           if isempty(A)
               return;
           end
           T(1,:) = mat2cells(A(1,:));
           
           I = A(2,:);
           T(2, I==1|I==4) = {'L{'};
           T(2, I==2|I==5) = {'L}'};
           T(2, I==3|I==6) = {'L'};           
       end       
   end
    
end