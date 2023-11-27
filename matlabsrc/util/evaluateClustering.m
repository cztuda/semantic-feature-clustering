function evaluateClustering(cluster, groundTruth)
    classes = unique(groundTruth);
    fprintf('label | N     | n     | nIns  | nOuts \n');
    
    icUsed = [];
    for i = 1:length(classes)
        sel = groundTruth == classes(i);
        inC = cluster(sel);
        outC = cluster(~sel);
        [~, I] = sort(map(@(i)sum(inC==i), 1:length(classes)), 'descend');
        iC = I(find(~map(@(i)any(icUsed==i), I),1));
        icUsed = [icUsed, iC]; %#ok<AGROW>
        
%         T = getMap(cluster, groundTruth);
%         shall = sum(T,2);
%         [nIns, iC] = max(T,[],2);
%         nOut = map(@(i)sum(T(i, 1:5~=I(i))), (1:4)');
        
        nIns = sum(inC==iC);
        nOut = sum(outC==iC);
        shall = sum(groundTruth==i);
        is = sum(cluster==iC);
        fprintf(' %4i | %5i | %5i | %5i | %5i \n', iC, shall, is, nIns, nOut);
    end
end

function T = getMap(c1, c2)
    uc1 = unique(c1);
    uc2 = unique(c2);
    T = map(@(i)map(@(j)length(intersect(find(c1==i), find(c2==j))), colvec(uc2)), rowvec(uc1));
    
end