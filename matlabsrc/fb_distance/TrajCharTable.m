

classdef TrajCharTable
    %%
    %
    %
    % Handle type: Add TrajectoryCharacteristics handles and evaluate them for given trajectory data
    %
    % Rows where information can be stored:
    % row 1: (!) time stamp
    % row 2: (!) symbol
    % row 3: (!) index
    % row 4: salience
    % (row 5: signs)
    % The first three rows are required, the fifth row is reserved for TC_Roots
    properties(Access=private)
        listOfTrajectoryCharacteristics
        normalizeTime
        rmRelatedRootsForExtrema % this is only relevant for tables, not for indexwiseTables
        handleType % 0:undefined, 1:noHandle, -1:handle
    end
    
    methods(Access=public, Static)
        function [Table, TCT_handle] = getTrajstructTable(trajstruct, TCT_handle)
            %% [Table, TCT_handle] = getTrajstructTable(trajstruct, TCT_handle)
            % Get trajectory characteristics table from ordinary trajectory struct.
            % Provide a TCT handle as second input or use the default trajectory TCT handle.
            if nargin < 2
                TCT_handle = TrajCharTable.getDefaultTrajstructTCTHandle();
            end
            Table = TCT_handle.getTable({trajstruct.dataX, trajstruct.dataU});
        end
        
        function TCT_handle = getDefaultTrajstructTCTHandle(problem, varargin)
            %% TCT_handle = getDefaultTrajstructTCTHandle(problem, adaptZero)
            % Get the default handle for problems. Uses TC_Roots, TC_Extrema and, if problem
            % interface is given to provide the box constraints, TC_limits for the state and control 
            % trajectory. If adaptZero is true and problem interface is given, the final values are 
            % used to shift the zero in TC_Roots.
            % ADDITIONAL ARGUMENTS:
            %   adaptZero:      (default=false) if problem interface is given, the final values are 
            %                       used to shift the zero in TC_Roots.
            %   promFilter:     (default=0.0) prominence filter can be a scalar or [pf1, pf2]
            %   stateFilter:    (default=-1)
            %   controlFilter:  (default=[])
            %   stateWithRoots: (default=false) enable roots as feature for states
            [adaptZero, promFilter, sFilter, cFilter, sWithRoots] = getFunctionArguments(varargin, {}, ...
                'adaptZero', false, 'promFilter', 0.0, 'stateFilter', -1, 'controlFilter', [], 'stateWithRoots', false);
            if ~adaptZero || isempty(problem)
                zero = [];
                adaptZero = false;
            else
                zero = secondOutValue(@()problem.getFinalValues());
                zero(~problem.getControlIsActiveAtEnd()) = 0;
                if all(zero==0), zero = []; end
            end
            if nargin > 0
                bU = [problem.getControlVariableLowerBound(), problem.getControlVariableUpperBound()];
            else
                problem = [];
            end
            TCT_handle = TrajCharTable.getDefaultStateOnlyTCTHandle(problem, false, 'adaptZero', adaptZero, 'stateFilter', sFilter, 'promFilter', promFilter(1), 'withRoots', sWithRoots);
            
            TC1u = TC_Roots([], cFilter, 'inclDeriv', false, 'zero', zero);
            if length(promFilter) > 1
                promFilter = promFilter(2);
            end
            if nargin > 0
                TC2u = TC_ExtremaAndLimits([], bU, cFilter, 'promFilter', promFilter);
            else
                TC2u = TC_Extrema([], cFilter, 'promFilter', promFilter);
            end
            
                        
            TCT_handle = TCT_handle.addTrajChar(TC1u, 2);
            TCT_handle = TCT_handle.addTrajChar(TC2u, 2);
        end
        
        function TCT_handle = getDefaultStateOnlyTCTHandle(problem, withDummy, varargin)
            %% TCT_handle = getDefaultStateOnlyTCTHandle(problem, withDummy=false, adaptZero=problem.finalValues)
            % Like getDefaultTrajstructTCTHandle, but restricted to the state variable.
            % If withDummy=true, then TC_Empty is set for the control TCT handle.
            % ADDITIONAL ARGUMENTS:
            %   adaptZero:      (default=false) if problem interface is given, the final values are 
            %                       used to shift the zero in TC_Roots.
            %   promFilter:     (default=0.0) prominence filter can be a scalar or [pf1, pf2]
            %   stateFilter:    (default=-1)
            %   withRoots:      (default=false) enable roots as feature
            [adaptZero, promFilter, sFilter, wRoots] = getFunctionArguments(varargin, {}, ...
                'adaptZero', false, 'promFilter', 0.0, 'stateFilter', -1, 'withRoots', false);
            if ~adaptZero || isempty(problem)
                adaptZero = [];
            else
                adaptZero = problem.getFinalValues();
                adaptZero(~problem.getStateIsActiveAtEnd()) = 0;
                if all(adaptZero==0), adaptZero = []; end
            end
            if nargin < 2
                withDummy = false;
            end
            TCT_handle = TrajCharTable();
            
            if wRoots
                TC1x = TC_Roots([], sFilter, 'zero', adaptZero);
            end
            if nargin > 0 && ~isempty(problem)
                bX = [problem.getStateVariableLowerBound(), problem.getStateVariableUpperBound()];
                TC2x = TC_ExtremaAndLimits([], bX, sFilter, 'promFilter', promFilter);
            else
                TC2x = TC_Extrema([], sFilter, 'promFilter', promFilter);
            end
            
            if wRoots
                TCT_handle = TCT_handle.addTrajChar(TC1x);
            end
            TCT_handle = TCT_handle.addTrajChar(TC2x);

            if withDummy
                TCT_handle = TCT_handle.addTrajChar(TC_Empty([]), 2);
            end
        end
        
        function TCT_handle = getDefaultControlOnlyTCTHandle(problem, withDummy, varargin)
            %% TCT_handle = getDefaultControlOnlyTCTHandle(problem, withDummy=false, adaptZero=[])
            % Like getDefaultTrajstructTCTHandle, but restricted to the state variable.
            % If withDummy=true, then TC_Empty is set for the state TCT handle.
            % ADDITIONAL ARGUMENTS:
            %   adaptZero:      (default=false) if problem interface is given, the final values are 
            %                       used to shift the zero in TC_Roots.
            %   promFilter:     (default=0.0) prominence filter can be a scalar or [pf1, pf2]
            %   stateFilter:    (default=[])
            [adaptZero, promFilter, sFilter] = getFunctionArguments(varargin, {}, ...
                'adaptZero', false, 'promFilter', 0.0, 'stateFilter', []);
            if nargin < 2
                withDummy = false;
            end
            if isempty(problem) || ~adaptZero
                adaptZero = [];
            else
                adaptZero = secondOutValue(@()problem.getFinalValues);
                adaptZero(~problem.getControlIsActiveAtEnd()) = 0;
                if all(adaptZero==0), adaptZero = []; end
            end
            TCT_handle = TrajCharTable();
            if withDummy
                cat = 2;
                TCT_handle = TCT_handle.addTrajChar(TC_Empty([]), 1);
            else
                cat = 1;
            end
%             TCT_handle = TCT_handle.addTrajChar(TC_Roots([], sFilter, 'inclDeriv', false, 'zero', adaptZero), cat);
            
            if nargin > 0 && ~isempty(problem)
                bU = [problem.getControlVariableLowerBound(), problem.getControlVariableUpperBound()];
                TCT_handle = TCT_handle.addTrajChar(TC_ExtremaAndLimits([], bU, sFilter, 'promFilter', promFilter), cat);
            else
                TCT_handle = TCT_handle.addTrajChar(TC_Extrema([], sFilter, 'promFilter', promFilter), cat);
            end
        end
    end
    
    methods(Access=public)
        function obj = TrajCharTable(varargin) 
            %% obj = TrajCharTable(normalizeTime=true, rmRelatedRootsForExtrema=false)
            % Constructor.
            obj.listOfTrajectoryCharacteristics = {{}};
            obj.handleType = 0;
            [obj.normalizeTime, obj.rmRelatedRootsForExtrema] = getFunctionArguments(varargin, ...
                {true, false});
        end
        
        function ish = isHandle(obj)
            ish = obj.handleType < 0;
        end
        
        function obj = addTrajChar(obj, trajChar, varargin)
            %% obj = addTrajChar(obj, trajChar, varargin)
            % Add new TrajectoryCharacteristic object.
            [ctgry] = getFunctionArguments(varargin, {[]});
            if isempty(ctgry)
                ctgry = 1;
            elseif isinf(ctgry)
                ctgry = max(obj.getAllCategories())+1;
            end
            
            if isa(trajChar, 'TrajectoryCharacteristic')
                % check handle compatibility
                if ((~trajChar.isHandle())*2-1)*obj.handleType < 0
                    error('Either all TCs must be handles or they all must be not.');
                end
                if ~obj.handleType
                    obj.handleType = (~trajChar.isHandle())*2-1;
                end
                
                if length(obj.listOfTrajectoryCharacteristics) < ctgry
                    obj.listOfTrajectoryCharacteristics = [obj.listOfTrajectoryCharacteristics, ...
                        repmat({{}}, 1, ctgry-length(obj.listOfTrajectoryCharacteristics))];
                end
                
                % check that symbols are different
                allS = obj.getAllSymbols(ctgry);
                if any(map2(@(c)strcmp(allS, c), trajChar.getSymbol()))
                    error('AddTrajChar: Characteristic with such symbols already exists');
                end
                
                %check that number of variables for this category is correct
                if any(obj.getAllCategories()==ctgry) && obj.handleType == 1
                    cDim = obj.listOfTrajectoryCharacteristics{1}{1}.dim();
                    if cDim ~= trajChar.dim()
                        error('AddTrajChar: The given characteristic has unexpected dimension.');
                    end
                end
                
                obj.listOfTrajectoryCharacteristics{ctgry} = [obj.listOfTrajectoryCharacteristics{ctgry}, {trajChar}];
                obj = updateAdditionalInformation(obj, ctgry, length(obj.listOfTrajectoryCharacteristics{ctgry}));
            else
                error('TrajCharTable::addTrajChar: Input must be a TrajectoryCharacteristic');
            end
        end
        
        function argout = doNormalizeTime(obj, argin)
            %% argout = doNormalizeTime(obj, argin)
            % Getter or setter, depending on use:
            % getter: argout = obj.doNormalizeTime();
            % setter: obj = obj.doNormalizeTime(argin);
            if nargin > 1
                if ~isscalar(argin)
                    error('TrajCharTable::doNormalizeTime: Given argument must be scalar.');
                end
                obj.normalizeTime = argin;
                argout = obj;
            else
                argout = obj.normalizeTime;
            end
        end
        
        function argout = doRemoveRelatedRootsForExtrema(obj, argin)
            %% argout = doRemoveRelatedRootsForExtrema(obj, argin)
            % Getter or setter, depending on use:
            % getter: argout = obj.doRemoveRelatedRootsForExtrema();
            % setter: obj = obj.doRemoveRelatedRootsForExtrema(argin);
            % 
            % argin must be 0, 1 or 2:
            %   0 - do not remove extrema
            %   1 - remove extrema close to L{ and L}
            %   2 - remove everything between L{ and L}
            if nargin > 1
                if ~isscalar(argin)
                    error('TrajCharTable::doRemoveRelatedRootsForExtrema: Given argument must be scalar.');
                end
                obj.rmRelatedRootsForExtrema = argin;
                argout = obj;
            else
                argout = obj.rmRelatedRootsForExtrema;
            end
        end
                        
        % get a cell array of all symbols of all registered TCs
        function L = getAllSymbols(obj, category)
            L = {};
            for i = 1:length(obj.listOfTrajectoryCharacteristics{category})
                L = [L, rowvec(obj.listOfTrajectoryCharacteristics{category}{i}.getSymbol())]; %#ok<AGROW>
            end
            L = unique(L);
        end
        
        % get a cell array of all categories that group the currently registered TCs
        function L = getAllCategories(obj)
            L = find(map(@(c)~isempty(c), obj.listOfTrajectoryCharacteristics));
        end
        
        
        function Tables = getTable(obj, trajdata)
            %% Tables = getTable(obj, trajdata)
            % Get trajectory characteristics table from trajectory data or cell of trajectory data
            % structs, if handle has more than one category.
            if nargin > 1
                if length(trajdata) > 1
                    getETable = @(c, idx)c.getEventTable(trajdata{idx});
                    getData = @(idx)trajdata{idx};
                else
                    getETable = @(c, ~)c.getEventTable(trajdata);
                    getData = @(~)trajdata;
                end
            elseif obj.handleType > 0
                getETable = @(c, ~)c.getEventTable();
            else
                error('TrajCharTable::getTable: The TC handles require trajdata as second input.');
            end
            
            fullL = obj.listOfTrajectoryCharacteristics;
            Tables = cell(1, length(fullL));
            
            for ctgry = 1:length(fullL)
                L = fullL{ctgry};
                if ~isempty(L)
                    Table = cell(5, 200); % pre-initialize cell array
                    n=1;
                    rootIndex = find(map(@(c)isa(c, 'TC_Roots'), L), 1);
                    tcIndex = zeros(1, 200);
                    for i = 1:length(L)
                        [times, table] = getETable(L{i}, ctgry);
                        len = length(times);
                        Table(1, n:n+len-1) = mat2cells(rowvec(times));
                        Table(L{i}.getRowIndices(), n:n+len-1) = table;
                        tcIndex(n:n+len-1) = i*ones(1, len);
                        n = n+len;
                    end
                    Table = obj.sortAndCheckForDublicates(Table(:, 1:n-1), tcIndex(1:n-1), 1e-3, rootIndex, obj.rmRelatedRootsForExtrema);
                    I = secondOutValue(@sort, cell2mat(Table(1,:)));
                    Table = Table(:,I);

                    Table = obj.fillSignsInTable(Table, getData(ctgry).n, getData(ctgry).t0);
                    
                    if obj.normalizeTime
                        if nargin > 1
                            TBounds = map(@(c)c.getTimeRange(getData(ctgry))', L);
                        else
                            TBounds = map(@(c)c.getTimeRange()', L);
                        end
                        t0 = min(TBounds(1,:));
                        tf = max(TBounds(2,:));
                        Table(1,:) = mat2cells((cell2mat(Table(1,:))-t0)/(tf-t0));
                    end
                else
                    Table = cell(5,0);
                end
                Tables{ctgry} = Table;
            end
            if length(Tables)==1
                Tables = Tables{1};
            end
        end
        
        function [Times, Symbols, Prominences] = getIndexwiseTable(obj, trajdata)
            if nargin > 1
                if length(trajdata) > 1
                    getETable = @(c, idx)c.getIndexwiseEventTable(trajdata{idx});
                    getData = @(idx)trajdata{idx};
                else
                    getETable = @(c, ~)c.getIndexwiseEventTable(trajdata);
                    getData = @(~)trajdata;
                end
            elseif obj.handleType > 0
                getETable = @(c, ~)c.getIndexwiseEventTable();
            else
                error('TrajCharTable::getTable: The TC handles require trajdata as second input.');
            end
            
            fullL = obj.listOfTrajectoryCharacteristics;
            Times = cell(1, length(fullL));
            Symbols = cell(1, length(fullL));
            Prominences = cell(1, length(fullL));
            
            for ctgry = 1:length(fullL)
                L = fullL{ctgry};
                if ~isempty(L)
                    [times, symbols, prominences] = getETable(L{1}, ctgry);
                    
                    for i = 2:length(L)
                        [tim, sym, prom] = getETable(L{i}, ctgry);
                        times = cellfun(@(a, b)[rowvec(a), rowvec(b)], times, tim, 'UniformOutput', false);
                        symbols = cellfun(@(a, b)[rowvec(a), rowvec(b)], symbols, sym, 'UniformOutput', false);
                        prominences = cellfun(@(a, b)[rowvec(a), rowvec(b)], prominences, prom, 'UniformOutput', false);
                    end
%                     if obj.rmExtremaForLimits==1
%                         [times, symbols, prominences] = TrajCharTable.filterIndexwiseTable(times, symbols, prominences, 1e-3);
%                     end
                    
                    if obj.normalizeTime
                        if nargin > 1
                            TBounds = map(@(c)c.getTimeRange(getData(ctgry))', L);
                        else
                            TBounds = map(@(c)c.getTimeRange()', L);
                        end
                        t0 = min(TBounds(1,:));
                        tf = max(TBounds(2,:));
                        for i = 1:length(times)
                            times{i} = (times{i}-t0)/(tf-t0);
                        end
                    end
                    
                    [Times{ctgry}, Symbols{ctgry}, Prominences{ctgry}] = TrajCharTable.sortIndexwiseTable(times, symbols, prominences);
                else
                    Times{ctgry} = [];
                    Symbols{ctgry} = {};
                    Prominences{ctgry} = [];
                end
            end
            if length(Times)==1
                Times = Times{1};
                Symbols = Symbols{1};
                Prominences = Prominences{1};
            end
        end
        
        function out = TCList(obj, category, varargin)
            if nargin > 2 && isnumeric(varargin{1}) && isscalar(varargin{1})
                idx = varargin{1};
                offset = 1;
            else
                idx = 0;
                offset = 0;
            end
            if nargin == 2+offset
                % getter:
                if idx == 0
                    out = obj.listOfTrajectoryCharacteristics{category};
                else
                    out = obj.listOfTrajectoryCharacteristics{category}{idx};
                end
            else
                % setter:
                if idx == 0
                    obj.listOfTrajectoryCharacteristics{category} = varargin{1+offset};
                else
                    obj.listOfTrajectoryCharacteristics{category}{idx} = varargin{1+offset};
                end
                out = obj;
            end
        end
        
        function disp(obj)
            isH = '';
            if obj.handleType < 0
                isH = '(handle)';
            end
            fprintf('TrajCharTable %s\n', isH);
            fprintf('Normalize time: %s\n', logical2str(obj.normalizeTime, 'yesno'));
            fprintf('Remove related roots for extrema: %s\n', logical2str(obj.rmRelatedRootsForExtrema, 'yesno'));
            categories = obj.getAllCategories();
            for i = 1:length(categories)
                fprintf('-- Category %i --\n', categories(i));
                L = obj.listOfTrajectoryCharacteristics{categories(i)};
                for j = 1:length(L)
                    L{j}.disp();
                end                
            end
            fprintf('\n');
        end
    end
    
    methods(Access=private)
        function obj = updateAdditionalInformation(obj, ctgry, newI)
%             for ctgry = 1:length(obj.listOfTrajectoryCharacteristics)
                L = obj.listOfTrajectoryCharacteristics{ctgry};
                i_root = find(map(@(c)isa(c, 'TC_Roots'), L),1);
                i_extr = find(map(@(c)isa(c, 'TC_Extrema'), L),1);
                i_limt = find(map(@(c)isa(c, 'TC_Limits'), L),1);
                if ~isempty(i_extr) && ~isempty(i_root) && (i_extr == newI || i_root == newI)
                    L{i_root} = L{i_root}.useAdditionalInformation(L{i_extr});
                end
                if ~isempty(i_limt) && ~isempty(i_extr) && (i_extr == newI || i_limt == newI)
                    L{i_limt} = L{i_limt}.useAdditionalInformation(L{i_extr});
                end
                obj.listOfTrajectoryCharacteristics{ctgry} = L;
%             end
        end
    end
    
    methods(Access=private, Static)
        function Table = sortAndCheckForDublicates(Table, tcIndex, threshold, rootIndex, rmRtsForExtr)
            [C, IA, IC] = uniquetol(cell2mat(Table(1,:)), threshold);
            if length(C) < size(Table,2) && rmRtsForExtr % if some times appear more than once
                n = size(Table,2);
                keepTableColumn = true(1, n);
                tcIndex = rowvec(tcIndex);
                isRoot = any(map(@(ri)tcIndex==ri, colvec(rootIndex)), 1);
                
                indicesAppearingMoreOften = find(map(@(k)sum(IC==k), IC(IA))>1);
                for i = 1:length(indicesAppearingMoreOften) 
                    ic = indicesAppearingMoreOften(i);
                    % keep roots in each category implicitely
                    if rmRtsForExtr
                        sel = isRoot & (IC==ic)' & ~map(@(c)isempty(c), Table(2,:));
                        if any(sel)
                            rootIdx = find(sel,1);
    %                         Table(5,IC==ic) = Table(5, rootIdx);
                            keepTableColumn(rootIdx) = false;
                        end
                    end
%                     % remove extrema when there is a limit
%                     if rmExtrForLmts==1
%                         sel = [false, map(@(c)c(1)=='L', Table(2,2:end))] & rowvec(IC==ic);
%                         if any(sel)
%                             keepTableColumn( ...
%                                 selectIndex(find(IC==ic), (strcmp(Table(2, IC==ic),'v')) | (strcmp(Table(2, IC==ic),'^'))) ...
%                                 ) = false;                        
%                         end
%                     end
                end
                Table = Table(:, keepTableColumn);
            end
            
            I = secondOutValue(@sort, cell2mat(Table(1,:)));
            Table = Table(:,I);
        end
                
%         function Table = clearEltsBetweenLimits(Table, threshold)
%             allLimits = [false, map(@(c)c(1)=='L'&length(c)>1, Table(2,2:end))];
%             allTimes = [Table{1, :}];
%             allIndices = [0, Table{3, :}];
%             uniqueIndices = unique(allIndices(allLimits));
%             toBeRemoved = false(size(allTimes));
%             for i = 1:length(uniqueIndices)
%                 pairs = find(allLimits & allIndices == uniqueIndices(i));
%                 candidates = false(size(toBeRemoved));
%                 for j = 1:2:length(pairs)
%                     candidates = candidates | (allTimes > allTimes(pairs(j)) - threshold & allTimes < allTimes(pairs(j+1)) + threshold);
%                 end
%                 candidates = candidates & allIndices == uniqueIndices(i);
%                 candidates(pairs) = false;
%                 toBeRemoved = toBeRemoved | candidates;
%             end
%             Table = Table(:, ~toBeRemoved);
%         end
%         function [Times, Symbols, Prominences] = clearEltsBetweenIndexwiseLimits(Times, Symbols, Prominences, threshold)
%             for ctgry = 1:length(Times)
%                 times = Times{ctgry};
%                 symbols = Symbols{ctgry};
%                 prominences = Prominences{ctgry};
%                 for i = 1:length(times)
%                     allTimes = times{i};
%                     pairs = find(map(@(c)c(1)=='L'&length(c)>1, symbols{i}));
%                     toBeRemoved = false(size(times{i}));
%                     for j = 1:2:length(pairs)
%                         toBeRemoved = toBeRemoved | (allTimes > allTimes(pairs(j)) - threshold & allTimes < allTimes(pairs(j+1)) + threshold);
%                     end
%                     toBeRemoved(pairs) = false;
% 
%                     times{i}(toBeRemoved) = [];
%                     prominences{i}(toBeRemoved) = [];
%                     symbols{i}(toBeRemoved) = [];
%                 end
%                 Times{ctgry} = times;
%                 Symbols{ctgry} = symbols;
%                 Prominences{ctgry} = prominences;                
%             end
%         end
        
%         function Table = mergeTableLimits(Table)
%             allLimits = [false, map(@(c)c(1)=='L'&length(c)>1, Table(2,2:end))];
%             allTimes = [Table{1, :}];
%             allIndices = [0, Table{3, :}];
%             uniqueIndices = unique(allIndices(allLimits));
%             toBeRemoved = false(size(allTimes));
%             for i = 1:length(uniqueIndices)
%                 pairs = find(allLimits & allIndices == uniqueIndices(i));
%                 Table(1, pairs(1:2:end)) = mat2cells(sum(reshape(allTimes(pairs), 2, []), 1)/2);
%                 Table(2, pairs(1:2:end)) = {'L'};
%                 toBeRemoved(pairs(2:2:end)) = true;
%             end
%             Table = Table(:, ~toBeRemoved);
%         end
%         function [Times, Symbols, Prominences] = mergeIndexwiseLimits(Times, Symbols, Prominences)
%             for ctgry = 1:length(Times)
%                 times = Times{ctgry};
%                 symbols = Symbols{ctgry};
%                 prominences = Prominences{ctgry};
%                 for i = 1:length(times)
%                     allLimits = find(map(@(c)c(1)=='L'&length(c)>1, symbols{i}));
%                     lmts = allLimits(1:2:end);
%                     symbols{i}(lmts) = {'L'};
%                     times{i}(lmts) = sum(reshape(times{i}(allLimits), 2, []), 1)/2;
% 
%                     lmts = allLimits(2:2:end);
%                     times{i}(lmts) = [];
%                     prominences{i}(lmts) = [];
%                     symbols{i}(lmts) = [];
%                 end
%                 Times{ctgry} = times;
%                 Symbols{ctgry} = symbols;
%                 Prominences{ctgry} = prominences;
%             end
%         end
        
        
%         function [times, symbols, values] = filterIndexwiseTable(times, symbols, values, threshold)
%             for i = 1:length(times)
%                 T = times{i};
%                 
%                 [C, IA, IC] = uniquetol(T, threshold);
%                 if length(C) < length(T)
%                     indicesAppearingMoreOften = find(map(@(k)sum(IC==k), IC(IA))>1);
%                     extrAndLims = map(@(c)ifelse(c(1)=='L', 1, ifelse(c(1)=='^' | c(1)=='v', 2, 0)), symbols{i});
%                     keepEntry = true(1, length(T));
%                     
%                     for j = 1:length(indicesAppearingMoreOften) 
%                         ic = indicesAppearingMoreOften(j);
%                         sel = IC==ic;
%                         if any(extrAndLims(sel)==1) && any(extrAndLims(sel)==2)
%                             keepEntry(sel & extrAndLims'==2) = false;
%                         end
%                     end
%                     
%                     times{i} = times{i}(keepEntry);
%                     symbols{i} = symbols{i}(keepEntry);
%                     values{i} = values{i}(keepEntry);
%                 end
%             end
%         end
                
        function [times, symbols, values] = sortIndexwiseTable(times, symbols, values)
            for i = 1:length(times)
                [times{i}, I] = sort(times{i});
                symbols{i} = symbols{i}(I);
                values{i} = values{i}(I);
            end
        end
        
        function Table = fillSignsInTable(Table, n, t0)
            % Table = fillSignsInTable(Table, n, t0)
            % Normally, only Table is required. But if sign information is missing because TC_Roots
            % is not included, then n and t0 are used to build dummy sign information.
            if isempty(Table)
                return;
            end
            v = Table{5,1};
            if isempty(v)
                v = zeros(n, 1);
                Table = [{t0; []; []; 0; v}, Table];
            end
            for i = 2:size(Table,2)
                if isempty(Table{5,i})
                    Table{5,i} = v;
                else
                    v = Table{5,i};
                end
            end
        end
    end
end