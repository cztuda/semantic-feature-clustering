
classdef TC_Roots < TrajectoryCharacteristic
    properties(Access=private)
        trajdata
        extrema
        table
        iq
        idq
        rootHasDirectionalIndicator
        stateFilter
        inclDeriv
        zero
        useRootProminence
    end
    
    methods(Access=public)
        function obj = TC_Roots(trajdata, varargin)
            %% obj = TC_Roots(trajdata, stateFilter=-1, varargin)
            % Constructor.
            % INPUT:
            %   trajdata:    universalFunction data struct, may be empty to create a handle
            %   stateFilter: (optional) which entries in the state vector should be used.
            %                  -1: (default) crop the last index
            %                  []: use all entries
            %                  vector(int): use the specified entries 
            % ADDITIONAL ARGUMENTS:
            %   inclDeriv:  (default=true): assume that the given state vector is in the form [q; dq]
            %   signInd:    (default=false): add directional indicator to the root symbol
            %   zero:       (default=0): which number is considered as root? Use this value to detect times at
            %                 which the trajectory crosses values other than zero. Scalar or vector of size data.n 
            %   rootProm:   (default=false): set true to use function steepness in root as prominence 
            obj@TrajectoryCharacteristic(trajdata, {'0'}, {'root'});
            
            [obj.stateFilter, obj.inclDeriv, obj.rootHasDirectionalIndicator, obj.zero, obj.useRootProminence] = ...
                getFunctionArguments(varargin, {-1}, 'inclDeriv', true, 'signInd', false, 'zero', [], 'rootProm', true);
            
            if ~isempty(trajdata)
                obj.extrema = [];
                obj = obj.registerTrajdata(trajdata);
            else
                obj.trajdata = [];
                obj.table = [];
                obj.iq = [];
                obj.idq = [];
                obj.t0tf = [];
                obj.extrema = [];
            end
                        
            if obj.rootHasDirectionalIndicator
                obj.symbols = {'0+', '0-'};
                obj.symbols_explanation = {'root increasing', 'root decreasing'};
            end
            
        end
        
       function obj = update(obj, trajdata)
           obj = update@TrajectoryCharacteristic(obj, trajdata);
           obj = obj.registerTrajdata(trajdata);
       end
        
        function [time_list, event_table] = getEventTable(obj, trajdata)
            %% [time_list, event_table] = getEventTable(obj, trajdata)
            % Get the table with the roots.
            % INPUT:
            %    trajdata:       (optional) get event table for this trajdata. If not set, the trajdata
            %                      given in the construction of this object is used, this argument is
            %                      required if this is a handle
            % OUTPUT:
            %    time_list:      first line of the table
            %    event_table:    the lines specified in obj.getRowIndices
            if nargin > 1
                obj = obj.registerTrajdata(trajdata);
            elseif isempty(obj.trajdata)
                error('TC_Roots::getEventTable: A TC handle requires trajdata as second input.');
            end
            if ~isempty(obj.extrema) && ~isempty(obj.idq) % remove extremas for indices in idq where the zero in TC_Roots has changed.
                sf = buildStateFilter(obj.stateFilter, trajdata.n);
                obj.extrema(obj.iq(selectIndex(obj.getZero(obj.zero, length(obj.extrema), sf), obj.idq)~=0)) = {[]};
            end
            
            [signTable, time_list, indices] = obj.getSignTable(obj.trajdata, obj.table, obj.extrema, obj.iq, obj.idq);
            event_table = mat2cell(signTable, size(signTable,1), ones(1, size(signTable,2)));
            event_table = obj.createFullTable(event_table, time_list, indices);
        end
        
        function [time_list, symbol_list, value_list] = getIndexwiseEventTable(obj, trajdata)
            if nargin > 1
                obj = obj.registerTrajdata(trajdata);
            elseif obj.isHandle()
                error('TC_Roots::getIndexwiseEventTable: A TC handle requires trajdata as second input.');
            end
            if ~isempty(obj.extrema) && ~isempty(obj.idq) % remove extremas for indices in idq where the zero in TC_Roots has changed.
                sf = buildStateFilter(obj.stateFilter, trajdata.n);
                obj.extrema(obj.iq(selectIndex(obj.getZero(obj.zero, length(obj.extrema), sf), obj.idq)~=0)) = {[]};
            end
            
            
            time_list = TC_Roots.getRootsCell(obj.trajdata, obj.table, obj.extrema, obj.iq, obj.idq);
            symbol_list = cellfun(@(c)repmat(obj.symbols(1), 1, length(c)), time_list, 'UniformOutput', false);
            if obj.useRootProminence
                value_list = arrayfun(@(i)obj.computeRootProminence(time_list{i}, i), 1:length(time_list), 'UniformOutput', false);
            else
                value_list = cellfun(@(c)ones(1, length(c)), time_list, 'UniformOutput', false);
            end
        end
        
        function rowInds = getRowIndices(obj)
            if obj.useRootProminence
                rowInds = [2, 3, 4, 5];
            else
                rowInds = [2, 3, 5];
            end
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
            ish = isempty(obj.trajdata);
        end
        
        function out = doUseRootProminence(obj, in)
           if nargin < 2
               % getter
               out = obj.useRootProminence;
           else
               % setter
               obj.useRootProminence = in;
               out = obj;
           end
        end
       
        function out = zeros(obj, in)
           if nargin < 2
               % getter
               out = obj.zero;
           else
               % setter
               obj.zero = in;
               out = obj;
           end
        end
        
        function out = includeDerivativeInformation(obj, in)
           if nargin < 2
               % getter
               out = obj.inclDeriv;
           else
               % setter
               obj.inclDeriv = in;
               out = obj;
           end
        end
        
        function out = directionalRootIndicator(obj, in)
           if nargin < 2
               % getter
               out = obj.rootHasDirectionalIndicator;
           else
               % setter
               obj.rootHasDirectionalIndicator = in;
               out = obj;
           end
        end
        
        function disp(obj)
            isH = '';
            if obj.isHandle()
                isH = '(handle)';
            end
            fprintf('TrajectoryCharacteristics: Roots %s\n', isH);
            fprintf('  knowns extrema: %s\n', logical2str(~isempty(obj.extrema), 'yesno'));
            fprintf('  directional indicators: %s\n', logical2str(obj.rootHasDirectionalIndicator, 'yesno'));
            fprintf('  state includes derivatives: %s\n', logical2str(obj.inclDeriv, 'yesno'));
            fprintf('  use steepness as prominence: %s\n', logical2str(obj.useRootProminence, 'yesno'));
            shiftedRoots = 'no';
            if ~isempty(obj.zero)
                shiftedRoots = vec2str(rowvec(obj.zero));
            end
            fprintf('  shifted roots: %s\n', shiftedRoots);
            sf = 'vec';
            if isempty(obj.stateFilter), sf = '[]'; elseif isscalar(obj.stateFilter) && obj.stateFilter==-1, sf = '-1'; end
            fprintf('  State filter: %s\n', sf);
            
            if ~obj.isHandle()
                fprintf('  iq : %s\n', vec2str(obj.iq));
                fprintf('  idq: %s\n', vec2str(obj.idq));
            end
            disp@TrajectoryCharacteristic(obj);
        end
    end
    
    methods(Access=protected)
        function obj = registerTrajdata(obj, trajdata)
            obj.t0tf = [trajdata.t0, trajdata.tf];
            if isa(obj.extrema, 'TC_Extrema')
                obj.extrema = obj.extrema.getRawTable(trajdata);
            end
            
            sFilter = buildStateFilter(obj.stateFilter, trajdata.n);
            [obj.iq, obj.idq] = obj.getIndices(trajdata.n, obj.inclDeriv);
            trajdata = obj.subtractZeroFromTrajdata(trajdata, obj.zero, sFilter);
            
            obj.trajdata = trajdata;
            
            if trajdata.type == 1
                T = findRootsInLinear(trajdata, sFilter);
            elseif trajdata.type == 3
                T = findRootsInCubic(trajdata, sFilter, 1e-6, 'omitStart', true, 'omitEnd', true);
            else
                error('TC_Roots: Trajdata must be linear or cubic.');
            end
            obj.table = T;
        end
        
        function sf = getStateFilter(obj)
            sf = obj.stateFilter;
        end
        
        function Table = createFullTable(obj, event_table, time_list, idx)
            if length(event_table) > 1
                indices = (diff(cell2mat(event_table), 1, 2) )/2;            
                if obj.rootHasDirectionalIndicator
                    sgn = sum(indices,1);
                    sgnCell = repmat({'-'}, 1, length(sgn));
                    sgnCell(sgn>=0) = {'+'};
                    symbols = strcat(obj.symbols(1), sgnCell);
                else
                    symbols = repmat(obj.symbols(1), 1, length(event_table));
                    symbols{1} = [];
                end            
                indices = [{[]}, mat2cells(idx)];
            else
                indices = {[]};
                symbols = {[]};
            end
            if obj.useRootProminence
                prom = mat2cells([0, obj.computeRootProminence(time_list(2:end), cell2mat(indices))]);
                Table = [symbols; indices; prom; event_table];
            else
                Table = [symbols; indices; event_table];
            end
        end
        
        function prominence = computeRootProminence(obj, time_list, indices)
            prominence = evaluateDerivative(obj.trajdata, time_list, 1);
            if isscalar(indices)
                prominence = prominence(indices, :);
            else
                prominence = map(@(i)prominence(indices(i), i), 1:length(time_list));
            end
            prominence = 2/pi*abs(atan(prominence));
        end
    end
    
    methods(Access=private, Static)
        function [iq, idq] = getIndices(n, inclDeriv)
            if inclDeriv
                n = round(n);
                if mod(n,2) == 1
                    n = n-1;
                end
                iq = 1:(n/2);
                idq = (n/2)+1:n;
            else
                iq = 1:n;
                idq = [];
            end
        end
                
        function [roots, signs] = getSigns(roots, extrema, trajdata, index)
            roots = [0; roots(:); trajdata.times(end)];
            if ~isempty(extrema)
                roots = [roots; extrema];
            end
            roots = uniquetol(roots, 1e-3);
            if nargout > 1
                ntest = 12;
                checkpts = map(@(i)selectIndex(linspace(roots(i), roots(i+1), ntest+2), 2:ntest+1), 1:length(roots)-1);
                checkvals = evaluateDerivative(trajdata, checkpts, 0);
                checkvals = reshape(checkvals(index,:), ntest, [])';

                signs = nan(size(checkvals,1),1);
                signs(all(checkvals > 0, 2)) = 1;
                signs(all(checkvals < 0, 2)) = -1;
                signs(all(abs(checkvals) < 1e-8, 2)) = 0;
                if any(isnan(signs))
                    s = sum(checkvals(isnan(signs),:) > 0, 2);
                    sg = zeros(size(s));
                    sg(s >= 2*ntest/3) = 1;
                    sg(sum(checkvals(isnan(signs),:) < 0, 2) >= 2*ntest/3) = -1;
                    signs(isnan(signs)) = sg;
                end
            end
            roots = roots(2:end-1);
        end
        
        function [table, times, inds] = getSignTable(trajdata, T_r, T_e, iq, idq)
            [roots, signs] = TC_Roots.getRootsCell(trajdata, T_r, T_e, iq, idq);
            
            table = nan(length(signs), length(cell2mat(roots'))+1);
            [times, indices] = sort(cell2mat(roots'));
            times = [0;times];
            sorted = mat2cell(sortBackIndices(indices)', cellfun(@(c)length(c)-1,signs));
            for i = 1:length(signs)
                table(i, [1; sorted{i}+1]) = signs{i};
            end
            v = table(:,1);
            for i = 2:length(times)
                sel = isnan(table(:,i));
                table(sel, i) = v(sel);
                v = table(:,i);
            end
            inds = selectIndex(map(@(i)i*ones(1, length(roots{i})), 1:length(roots)), indices);
        end
        
        function [roots, signs] = getRootsCell(trajdata, T_r, T_e, iq, idq)
            roots = cell(size(T_r));
            signs = cell(size(T_r));
            if isempty(T_e)
                T_e = cell(size(T_r));
            end
            for j = 1:length(iq)
                i = iq(j);
                if nargout > 1
                    [roots{i}, signs{i}] = TC_Roots.getSigns(T_r{i}, [], trajdata, i);
                else
                    roots{i} = TC_Roots.getSigns(T_r{i}, [], trajdata, i);
                end
            end
            for j = 1:length(idq)
                i = idq(j);
                if nargout > 1
                    [roots{i}, signs{i}] = TC_Roots.getSigns(T_r{i}, T_e{iq(j)}, trajdata, i);
                else
                    roots{i} = TC_Roots.getSigns(T_r{i}, T_e{iq(j)}, trajdata, i);
                end
            end
        end
        
        function trajdata = subtractZeroFromTrajdata(trajdata, zero, sfilter)
            if ~isempty(zero)
                zero = TC_Roots.getZero(zero, trajdata.n, sfilter);
                
                if trajdata.type == 1
                    trajdata.coefficients = trajdata.coefficients - rowvec(zero);
                elseif trajdata.type == 3
                    trajdata.coefficients(:,1) = trajdata.coefficients(:,1) - ...
                        repmat(colvec(zero), trajdata.nGridpoints-1, 1);
                    trajdata.lastEntry = trajdata.lastEntry - colvec(zero);
                end
            end            
        end
        function zero = getZero(z, n, sfilter)
            if isscalar(z)
                zero = repmat(z, 1, n);
            elseif isempty(z)
                zero = zeros(1, n);
            elseif length(z) ~= n
                zero = zeros(1, n);
                zero(sfilter) = z;
            else
                zero = z;
            end
        end
    end
    
end