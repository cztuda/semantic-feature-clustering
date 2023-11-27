%% [fun, data] = createPiecewiseLinear(time, value)
% create piecewise linear function (universal function object) from a time
% vector and a matrix of function values.
function [fun, data] = createPiecewiseLinear(time, value)
    if size(value,1) ~= length(time)
        value = value';
    end
    if size(value,1) ~= length(time)
        error('createPiecewiseLinear: Length of time and value must match in one dimension.');
    end
    [time, I] = sort(time);
    value = value(I,:);

    data.coefficients = value;
    data.times = colvec(time);
    data.tf = time(end);
    data.t0 = time(1);
    
    data.n = size(value,2);
    data.nGridpoints = size(value,1);
    data.type = 1;
    data.isadj = 'F';
    data.n2 = 0;
    data.parameter = [];
    data.names = arrayfun(@(i)sprintf('dim %d', i), 1:data.n, 'UniformOutput', false);
    
    fun = universalFunction();
    fun = fun.createCppFun(data);
end