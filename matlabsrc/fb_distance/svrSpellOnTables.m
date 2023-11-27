

function Score = svrSpellOnTables(TCT, Traj1, Traj2, varargin)
    [enableTimes, enableValues] = getFunctionArguments(varargin, 'enableTimes', true, 'enableValues', true);
    if isfield(Traj1, 'dataX')
        [T1, S1, V1] = TCT.getIndexwiseTable({Traj1.dataX, Traj1.dataU});
    else
        if length(Traj1) > 1
            T1 = {Traj1.Time};
            S1 = {Traj1.Symbol};
            V1 = {Traj1.Value};
        else
            T1 = Traj1.Time;
            S1 = Traj1.Symbol;
            V1 = Traj1.Value;            
        end
    end
    if isfield(Traj2, 'dataU')
        [T2, S2, V2] = TCT.getIndexwiseTable({Traj2.dataX, Traj2.dataU});
    else
        if length(Traj1) > 1
            T2 = {Traj2.Time};
            S2 = {Traj2.Symbol};
            V2 = {Traj2.Value};
        else
            T2 = Traj2.Time;
            S2 = Traj2.Symbol;
            V2 = Traj2.Value;
        end
    end
    
    nCat = length(TCT.getAllCategories());
    Score = zeros(nCat,1);
    tfac = 1;%10;
    
    for i = 1:nCat
        symL = TCT.getAllSymbols(i);
        SM = getDefaultSoftMatchingMatrix(symL);
        symMap = string_map(symL, 1:length(symL));
        if nCat == 1
            t1 = T1;
            t2 = T2;
            s1 = S1;
            s2 = S2;
            rl1 = V1;
            rl2 = V2;
            v1 = V1;
            v2 = V2;
        else
            t1 = T1{i};
            t2 = T2{i};
            s1 = S1{i};
            s2 = S2{i};
            rl1 = V1{i};
            rl2 = V2{i};
            v1 = V1{i};
            v2 = V2{i};
        end
        
        dim = length(t1);
        score = zeros(dim,1);
        for j = 1:dim
            val1 = v1{j};%*0.2+0.9;
            val2 = v2{j};%*0.2+0.9;
            
%             score(j) = svrspell(symMap.lookup(s1{j}), symMap.lookup(s2{j}), SM, [], @(k, mue)1/(1+100*k*k)*mue, [], rl1{j}, rl2{j});
%             score(j) = svrspell(symMap.lookup(s1{j}), symMap.lookup(s2{j}), SM, [], [], struct('x', tfac*t1{j}, 'y', tfac*t2{j}), rl1{j}, rl2{j});
%             score(j) = svrspell(symMap.lookup(s1{j}), symMap.lookup(s2{j}), SM, [], [], [], rl1{j}, rl2{j});
%             score(j) = svrspell(symMap.lookup(s1{j}), symMap.lookup(s2{j}), SM, [], [], [], [], [], v1{j}, v2{j});

            score(j) = svrspell(symMap.lookup(s1{j}), symMap.lookup(s2{j}), SM, [], [], struct('x', tfac*t1{j}, 'y', tfac*t2{j}), [], [], val1, val2);
%             score(j) = svrspell(symMap.lookup(s1{j}), symMap.lookup(s2{j}), SM, [], [], [], [], [], val1, val2);
        end
        
        if any(isnan(score))
            Score(i) = nan;
        else
            Score(i) = norm(score);
        end
    end
    
    if any(isnan(Score))
        Score = nan;
    else
        Score = norm(Score);
    end
end


function SM = getDefaultSoftMatchingMatrix(s)
    SM = 0.5*eye(length(s));
    if isempty(SM)
        return;
    end
    
    i_max = strcmp(s, '^');
    i_min = strcmp(s, 'v');
    
    L_srt = strcmp(s, 'L{');
    L_end = strcmp(s, 'L}');
    L_pnt = strcmp(s, 'L');
    L_upp = strcmp(s, 'Lu');
    L_low = strcmp(s, 'Ll');
    
    if any(L_srt|L_end)
        SM(i_max|i_min, L_srt) = 0.4;
        SM(i_max|i_min, L_end) = 0.4;
        SM(i_min|i_max, L_pnt) = 0.4;        
    elseif any(L_upp|L_low)
        SM(i_max, L_upp) = 0.7;
        SM(i_min, L_low) = 0.7;
    else
        SM(i_min|i_max, L_pnt) = 0.6;        
    end
    
    SM=SM+SM';
end