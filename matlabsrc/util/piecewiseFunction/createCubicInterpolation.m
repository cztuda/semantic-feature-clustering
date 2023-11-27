%% [cinterp, data] = createCubicInterpolation(t, Y, dY=[], asUniversalFunction)
% Compute the cubic interpolation for the given values Y at times t. t and
% Y must have the same length, Y can have arbitrary many rows.
% INPUT:
%   t:  [1xk]   the times t_i at which the function y(t_i) = Y(:,i) is known
%   Y:  [nxk]   the function values Y
%   dY: [nxk] or @(y, t) or []
%               the derivatives of Y at the times t_i. This input may be
%               empty, in this case, the derivatives are ignored when the
%               interpolation is computed. Otherwise, it must be either a
%               matrix of the same size as Y or a function handle that
%               allows the computation of the derivative with the signature
%               ([nx1], [1x1]) -> [nx1]
%   asUniversalFunction
%               [default=true], if true, the return a universal function
%               object, else only return the data struct
% OUTPUT:
%   res     either a universal function object or a struct describing the
%            cubic interpolation
function [res, res2] = createCubicInterpolation(t, Y, dY, asUniversalFunction)
    if nargin < 3 || isempty(dY)
        dY = [];
    else
        if isa(dY, 'function_handle')
            dyfun = dY;
            dY = zeros(size(Y));
            for i = 1:size(Y,2)
                dY(:,i) = dyfun(Y(:,i), t(i));
            end
        else
            dY = matchSizes(Y, dY);
        end
    end
    if nargin < 4
        asUniversalFunction = true;
    end
    if size(t,1) ~= 1
        if size(t,2) ~= 1
            error('createCubicInterpolation: One dimension of t must be 1.');
        else
            t = t';
        end
    end
    if size(Y,1)==length(t) && size(Y,2) ~= length(t)
        Y = Y';
    end
    if size(Y,2) ~= length(t)
        error('createCubicInterpolation: The length of Y and t must match in one dimension.');
    end 
    if isempty(dY) % only the values are given, do a cubic spline interpolation with Hermite boundary condition 
        dt = diff(t);
        dy = diff(Y, 1, 2);
        M = getMoments(dt, dy)';
        M1 = M(:,1:end-1);
        M2 = M(:,2:end);

        % poly = a + b*x + c*x^2 + d*x^3    
        a = Y(:,1:end-1);
        b = dy./dt - ((2*M1+M2)/6) .* dt;
        c = M1/2;
        d = (M2-M1)./(6*dt);


        b = b.*dt;
        c = c.*dt.^2;
        d = d.*dt.^3;
        
        coefficients = buildCoefficientMatrix(a, b, c, d);
    else % tangents at intermediate points are given, use them to construct the spline (as done by DIRCOL) 
        dY = matchSizes(Y, dY);
        
        dt = diff(t);
        y1 = Y(:, 1:end-1);
        dy1 = dY(:, 1:end-1);
        dy2 = dY(:, 2:end);
        h1 = -diff(Y, 1, 2);
        h2 = dy2 .* dt;
        
        a = y1;
        b = dy1 .* dt;
        c = -(3*h1 + 2*b + h2);
        d = 2*h1 + b + h2;
        
        coefficients = buildCoefficientMatrix(a, b, c, d);
    end

    data.n = size(Y, 1);
    data.isadj = 'F';
    data.n2 = 0;
    data.type = 3;
    data.nGridpoints = size(Y,2);
    data.t0 = t(1);
    data.tf = t(end);
    data.names = cell(1, data.n);
    data.names(:) = {'state '};
    data.names = arrayfun(@(i)cstrcat(data.names{i}, num2str(i)), 1:length(data.names), 'UniformOutput', false)';
    data.times = [t(1:end-1)', t(2:end)'];
    data.lastEntry = Y(:,end);
    data.parameter = zeros(0, 1);
    data.coefficients = coefficients;
    if nargout > 1
        res = data;
        res2 = universalFunction().createCppFun(data);
    else
        if asUniversalFunction
            res = universalFunction();
            res.createCppFun(data);
        else
            res = data;
        end
    end
end

function dY = matchSizes(Y, dY)
    if ~all(size(Y)==size(dY))
        dY = dY';
        if ~all(size(Y)==size(dY))
            error('createCubicInterpolation: The size of Y and dY must match.');
        end
    end
end

function M = getMoments(dt, dy)
    % natural boundary condition:
    mue0 = 1;
    muen = 1;
    lambda0 = 0;
    lambdan = 0;
    b0 = zeros(1, size(dy,1));
    bn = b0;
    
    hi2 = dt/6;
    n = length(dt)+1;
    
    D = spdiags([mue0, (dt(1:end-1)+dt(2:end))/3, muen]', 0, n, n) + ...
        spdiags([0, lambda0, hi2(2:end)]', 1, n, n) + ...
        spdiags([hi2(1:end-1), lambdan, 0]', -1, n, n);
    
    b = [b0; (dy(:,2:end)./dt(2:end)-dy(:,1:end-1)./dt(1:end-1))' ; bn];
%     n = length(b);
%     B = [[mue0, (dt(1:end-1)+dt(2:end))/3, muen]', [0, lambda0, hi2(2:end)]', [hi2(1:end-1), lambdan, 0]'];
%     D = spdiags(B, [0, 1, -1], n, n);
    

    M = D\b;
end

function M = buildCoefficientMatrix(a, b, c, d)
    M = [a(:), b(:), c(:), d(:)];

end
