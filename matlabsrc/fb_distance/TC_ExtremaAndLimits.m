
classdef TC_ExtremaAndLimits < TrajectoryCharacteristic
   properties(Access=private)
       table
       prominences
       types
       bounds % [2 x n]
       theStateFilter
       prominenceFilter
       enabledExtrema
       enabledLimits
       rmExtremaForLimits % can be 0:(do not remove extrema), 1:(remove extrema close to L{ and L}) or 2:(remove everything between L{ and L})
       mergeLimits
   end
   
   methods(Access=public)
       function obj = TC_ExtremaAndLimits(trajdata, bounds, varargin)
           %% obj = TC_ExtremaAndLimits(trajdata, bounds, stateFilter=-1, varargin)
           % Constructor.
           % INPUT:
           %   trajdata:    universalFunction data struct, may be empty to create a handle
           %   bounds:      [lower; upper]
           %   stateFilter: (optional) which entries in the state vector should be used.
           %                  -1: (default) crop the last index
           %                  []: use all entries
           %                  vector(int): use the specified entries 
           % ADDITIONAL ARGUMENTS:
           %   promFilter:  (default=0.0) filter all extrema or limits with prominence below the given threshold 
           %   rmExtremaForLimits: 
           %                (default=2) can be 0:(do not remove extrema), 1:(remove extrema close to Lx{ and Lx}) or 2:(remove everything between Lx{ and Lx})
           %   mergeLimits  (default=true) if false, Lx{ and Lx} are used, if true, they are merged into a single symbol Lx 
           %   compExtrema: (default=true) use this to completely disable the computation of extrema
           %   compLimits:  (default=true) use this to completely disable the computation of limits
           %
           %
           obj@TrajectoryCharacteristic(trajdata, {'^', 'v', 'Lu{', 'Lu}', 'Lu', 'Ll{', 'Ll}', 'Ll'}, ...
               {'maximum', 'minimum', 'upper_limit_start', 'upper_limit_end', 'upper_limit_touch', 'lower_limit_start', 'lower_limit_end', 'lower_limit_touch'});
           
           [obj.theStateFilter, obj.prominenceFilter, obj.rmExtremaForLimits, obj.mergeLimits, obj.enabledExtrema, obj.enabledLimits] = ...
               getFunctionArguments(varargin, {-1}, 'promFilter', 0.0, 'rmExtremaForLimits', 2, 'mergeLimits', true, 'compExtrema', true, 'compLimits', true);
           if size(bounds,2)==2 && size(bounds,1) ~= 2
               bounds = bounds';
           end           
           obj.bounds = bounds;
           
           if ~isempty(trajdata)
                obj = obj.registerTrajdata(trajdata);
            else
                obj.table = [];
                obj.prominences = [];
                obj.types = [];
                obj.t0tf = [];
            end
       end
       
       function obj = update(obj, trajdata)
           obj = update@TrajectoryCharacteristic(obj, trajdata);
           obj = obj.registerTrajdata(trajdata);
       end
       
       function S = getSymbol(obj, i)
           sel = obj.selectEnabledSymbols();
           all_symbols = obj.symbols(sel);
           
           if nargin > 1
               S = all_symbols{i};
           else
               S = all_symbols;
           end
       end
       
       function S = getSymbolExplanation(obj, i)
           sel = obj.selectEnabledSymbols();
           all_expl = obj.symbols_explanation(sel);
           
           if nargin > 1
               S = all_expl{i};
           else
               S = all_expl;
           end
       end
        
       function [time_list, event_table] = getEventTable(obj, trajdata)
           %% [time_list, event_table] = getEventTable(obj, trajdata)
           % Get the table with the extrema and limits.
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
                error('TC_ExtremaAndLimits::getEventTable: A TC handle requires trajdata as second input.');
           end
           
           time_list = map(@(c)rowvec(c), obj.table);
           if ~isempty(time_list)
               type = [obj.types{:}];
               proms = [obj.prominences{:}];
               table_indices = map(@(i)i*ones(1, length(obj.table{i})), 1:length(obj.table));

               event_table = [type; mat2cells(table_indices); mat2cells(proms)];
               
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
                error('TC_ExtremaAndLimits::getIndexwiseEventTable: A TC handle requires trajdata as second input.');
           end
           
           time_list = obj.table;
           symbol_list = obj.types;
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
                error('TC_ExtremaAndLimits::getRawTable: A TC handle requires trajdata as second input.');
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
          
       function out = isExtremaEnabled(obj, in)
           if nargin < 2
               % getter
               out = obj.enabledExtrema;
           else
               % setter
               obj.enabledExtrema = in;
               out = obj;
           end
       end
       
       function out = isLimitEnabled(obj, in)
           if nargin < 2
               % getter
               out = obj.enabledLimits;
           else
               % setter
               obj.enabledLimits = in;
               out = obj;
           end
       end
       
        function argout = doRemoveExtremaForLimits(obj, argin)
            %% argout = doRemoveExtremaForLimits(obj, argin)
            % Getter or setter, depending on use:
            % getter: argout = obj.doRemoveExtremaForLimits();
            % setter: obj = obj.doRemoveExtremaForLimits(argin);
            if nargin > 1
                if ~isscalar(argin)
                    error('TrajCharTable::doRemoveExtremaForLimits: Given argument must be scalar.');
                end
                obj.rmExtremaForLimits = argin;
                argout = obj;
            else
                argout = obj.rmExtremaForLimits;
            end
        end
        
        function argout = doMergeLimits(obj, argin)
            %% argout = doMergeLimits(obj, argin)
            % Getter or setter, depending on use:
            % getter: argout = obj.doMergeLimits();
            % setter: obj = obj.doMergeLimits(argin);
            if nargin > 1
                if ~isscalar(argin)
                    error('TrajCharTable::doRemoveExtremaForLimits: Given argument must be scalar.');
                end
                obj.mergeLimits = argin;
                argout = obj;
            else
                argout = obj.mergeLimits;
            end
        end
        
       function disp(obj)
           isH = '';
           if obj.isHandle()
               isH = '(handle)';
           end
           fprintf('TrajectoryCharacteristics: Extrema and Limits %s\n', isH);
           sf = vec2str(obj.theStateFilter);
           if isempty(obj.theStateFilter), sf = '[]'; elseif isscalar(obj.theStateFilter) && obj.theStateFilter==-1, sf = '-1'; end
           fprintf('  lower bound: %s\n', vec2str(obj.bounds(1,:)));
           fprintf('  upper bound: %s\n', vec2str(obj.bounds(2,:)));
           fprintf('  State filter: %s\n', sf);
           fprintf('  Prominence filter: %5.2f\n', obj.prominenceFilter);
           fprintf('  Merge limits: %s\n', logical2str(obj.mergeLimits, 'yesno'));
           fprintf('  Remove extrema for limits: %s\n', ifelse(obj.rmExtremaForLimits==1, 'at symbols', ifelse(obj.rmExtremaForLimits==2, 'between symbols', 'no')));
           fprintf('  Limits enabled: %s\n', logical2str(obj.enabledLimits, 'yesno'));
           fprintf('  Extrema enabled: %s\n', logical2str(obj.enabledExtrema, 'yesno'));
           disp@TrajectoryCharacteristic(obj);
       end
   end
   
   methods(Access=private)
       function obj = registerTrajdata(obj, trajdata)
           obj.t0tf = [trajdata.t0, trajdata.tf];
           sFilter = buildStateFilter(obj.theStateFilter, trajdata.n);
           bds = obj.bounds;
           
           if trajdata.type == 1
               if obj.enabledExtrema
                   [T_e, ~, Te_isMin] = findExtremaInLinear(trajdata, sFilter, 'filterExtrema', false, 'sepDims', true);
               else
                   T_e = repmat({[]}, 1, length(sFilter));
                   Te_isMin = T_e;
               end
               if obj.enabledLimits
                   T_l = findLimitsInLinear(trajdata, bds(2,:), bds(1,:), 'stateFilter', obj.theStateFilter);
               else
                   T_l = repmat({zeros(2,0)}, 1, length(sFilter));
               end
               
           elseif trajdata.type == 3
               if obj.enabledExtrema
                   [T_e, Te_isMax] = findExtremaInCubic(trajdata, sFilter, 'omitStart', true, 'omitEnd', true, 'relProm', true);
                   Te_isMin = cellfun(@(c)~c, Te_isMax, 'UniformOutput', false);
               else
                   T_e = repmat({[]}, 1, length(sFilter));
                   Te_isMin = T_e;
               end
               if obj.enabledLimits
                   T_l = findLimitsInCubic(trajdata, bds(2,:), bds(1,:), 'stateFilter', obj.theStateFilter, 'T', T_e);
               else
                   T_l = repmat({zeros(2,0)}, 1, length(sFilter));
               end
           else
               error('TC_ExtremaAndLimits: Trajdata must be linear or cubic.');
           end
           [times, Te_symbol] = TC_ExtremaAndLimits.merge(T_e, Te_isMin, T_l);
           if obj.enabledLimits && obj.enabledExtrema
               [times, Te_symbol] = useLimitInformationForExtrema(obj, times, Te_symbol);
           end
           
           Te_isMax = cellfun(@(c)strcmp(c, '^')|startsWith(c, 'Lu'), Te_symbol, 'UniformOutput', false);
           Te_isMin = cellfun(@(c)strcmp(c, 'v')|startsWith(c, 'Ll'), Te_symbol, 'UniformOutput', false);
           [times, Te_isMax, Te_isMin, Te_symbol] = TC_ExtremaAndLimits.sortAllInCell(times, Te_isMax, Te_isMin, Te_symbol);
           Te_prom = computeProminence(times, Te_isMax, trajdata, true, 'isMinimum', Te_isMin);
           [Table, Prominences, Types] = obj.filterProminence(times, Te_prom, Te_symbol, obj.prominenceFilter);
           
           [obj.table, obj.types, obj.prominences] = TC_ExtremaAndLimits.sortAllInCell(Table, Types, Prominences);
       end
              
       function [times, types] = useLimitInformationForExtrema(obj, times, types)
           if obj.rmExtremaForLimits==2
               [times, types] = TC_ExtremaAndLimits.clearEltsBetweenIndexwiseLimits(times, types, 1e-3, obj.mergeLimits);
           else
               if obj.rmExtremaForLimits==1
                   [times, types] = TC_ExtremaAndLimits.filterIndexwiseTable(times, types, 1e-3);
               end
               if obj.mergeLimits
                   [times, types] = TC_ExtremaAndLimits.mergeIndexwiseLimits(times, types);
               end
           end
       end
       
       function sel = selectEnabledSymbols(obj)
           sel = true(size(obj.symbols));
           if ~obj.enabledExtrema
               sel(1:2) = false;
           end
           if ~obj.enabledLimits
               sel(3:end) = false;
           end
           if obj.mergeLimits
               sel([3,4,6,7]) = false;
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
       
       
       function [T_e, types] = merge(T_e, Te_isMin, T_l)
           types = cell(size(T_e));
           for i = 1:length(T_e)
               tp = repmat({'^'}, 1, length(T_e{i}));
               tp(Te_isMin{i}) = {'v'};
               T_e{i} = [rowvec(T_e{i}), T_l{i}(1,:)];
               types{i} = [tp, TC_ExtremaAndLimits.number2symbol(T_l{i}(2,:))];               
           end
       end
       
       function [times, symbols] = clearEltsBetweenIndexwiseLimits(times, symbols, threshold, mergeLimits)
            for i = 1:length(times)
                allTimes = times{i};
                pairs = find(map(@(c)c(1)=='L'&length(c)>2, symbols{i}));
                toBeRemoved = false(size(times{i}));
                for j = 1:2:length(pairs)
                    toBeRemoved = toBeRemoved | (allTimes > allTimes(pairs(j)) - threshold & allTimes < allTimes(pairs(j+1)) + threshold);
                end

                if mergeLimits
                    lmts = pairs(1:2:end);
                    symbols{i}(lmts) = cellfun(@(c)c(1:2), symbols{i}(lmts), 'UniformOutput', false);  
                    times{i}(lmts) = sum(reshape(times{i}(pairs), 2, []), 1)/2;
                    toBeRemoved(pairs(1:2:end)) = false;
                else
                    toBeRemoved(pairs) = false;
                end

                times{i}(toBeRemoved) = [];
                symbols{i}(toBeRemoved) = [];
            end
       end
        
       function [times, symbols] = mergeIndexwiseLimits(times, symbols)
           for i = 1:length(times)
               allLimits = find(map(@(c)c(1)=='L'&length(c)>2, symbols{i}));
               lmts = allLimits(1:2:end);
               symbols{i}(lmts) = cellfun(@(c)c(1:2), symbols{i}(lmts), 'UniformOutput', false);
               times{i}(lmts) = sum(reshape(times{i}(allLimits), 2, []), 1)/2;

               lmts = allLimits(2:2:end);
               times{i}(lmts) = [];
               symbols{i}(lmts) = [];
           end
       end
       
       function [times, symbols] = filterIndexwiseTable(times, symbols, threshold)
            for i = 1:length(times)
                T = times{i};
                S = symbols{i};
                
                [C, IA, IC] = uniquetol(T, threshold);
                if length(C) < length(T)
                    indicesAppearingMoreOften = find(map(@(k)sum(IC==k), IC(IA))>1);
                    keepEntry = true(1, length(T));
                    
                    for j = 1:length(indicesAppearingMoreOften)
                        multipleIndex = find(IC==indicesAppearingMoreOften(j));
                        s = S(multipleIndex);
                        selL = (startsWith(s, 'L'));
                        if any(selL)
                            keepEntry(multipleIndex(~selL)) = false;
                        end
                    end
                    
                    times{i} = times{i}(keepEntry);
                    symbols{i} = symbols{i}(keepEntry);
                end
            end
       end
        
       function T = number2symbol(A)
           T = cell(size(A));
           T(A==1) = {'Lu{'};
           T(A==2) = {'Lu}'};
           T(A==3) = {'Lu'};
           T(A==4) = {'Ll{'};
           T(A==5) = {'Ll}'};
           T(A==6) = {'Ll'};
       end
       
       function [Times, Arg1, Arg2, Arg3] = sortAllInCell(Times, Arg1, Arg2, Arg3)
           for i = 1:length(Times)
               [Times{i}, I] = sort(Times{i});
               Arg1{i} = Arg1{i}(I);
               if nargin > 2
                   Arg2{i} = Arg2{i}(I);
               end
               if nargin > 3
                   Arg3{i} = Arg3{i}(I);
               end
           end
       end
   end
    
end