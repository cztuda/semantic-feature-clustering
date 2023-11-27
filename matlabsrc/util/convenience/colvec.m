%% x = colvec(x)
% Return a column vector, if the given input is a vector
function x = colvec(x)
  if isrow(x)
    x = x';
  end
end