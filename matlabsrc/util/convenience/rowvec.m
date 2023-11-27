%% x = rowvec(x)
% Return a row vector, if the given input is a vector
function x = rowvec(x)
  if iscolumn(x)
    x = x';
  end
end