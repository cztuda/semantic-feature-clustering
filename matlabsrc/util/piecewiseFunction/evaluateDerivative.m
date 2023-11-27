%% res = evaluateDerivative(data, t, order, righthandsidelimit=true)
% 
% INPUT:
%   data:   struct describing the piecewise function with the following fields:.n
%             .t0
%             .tf
%             .n
%             .type
%             .times
%             .coefficients
%   t:      vector of times where to evaluate
%   order:  which derivative to evaluate (1 for first order derivative, 2 for second order etc.)
%             set 0 to evaluate the function
%   righthandsidelimit:
%           (default=true): if true, use the rhs limit at points that are not differentiable
%
% OUTPUT:
%   res:    [mxn] = evaluation points at times t, where n=length(t) and m is the dimension
%   
function res = evaluateDerivative(data, t, order, varargin)
    if length(t) > 1
        res = map(@(tt)evaluateDerivative(data, tt, order, varargin{:}), t(:)');
        return;
    end
    if nargin >= 4
        righthandsidelimit = varargin{1};
    else
        righthandsidelimit = true;
    end
  n = data.n;
  if isempty(t)
      res = zeros(n,0);
      return;
  end
  if t < data.t0 || t > data.tf
      res = [];
      return;
  end
  
  switch(data.type)
      
      case 3 
          %% THE CUBIC CASE:
          pos = find (data.times (:, 1) <= t & data.times (:, 2) >= t);
          if length(pos) > 1 % t=t0 or t=tf, then length(pos)=1 holds and both limits are evaluated "normally" (and thus are the same value)
            if righthandsidelimit
              pos = pos(end);
              T = data.times(pos,:);
              dT = T(2)-T(1);
              factors = [1 1 2 6];
              res = factors(order+1)*data.coefficients( (n*(pos-1)+1) : (pos*n), order+1 ) / (dT^order);
              return;
            else
              pos = pos(1);
            end
          end

          C = data.coefficients( (n*(pos-1)+1) : (pos*n), : );

          T = data.times(pos,:);
          dT = T(2)-T(1);
          T = (t-T(1))/dT;

          switch(order)
            case 0
              factors = [1 1 1 1];
              T = [1, T, T.^[2 3]];
            case 1
              factors = [0 1 2 3];
              T = [0, 1, T, T*T];
            case 2
              factors = [0 0 2 6];
              T = [0, 0, 1, T];
            case 3
              factors = [0 0 0 6];
              T = [0, 0, 0, 1];
          end
          res = C * (T .* factors)' / dT^order;
  
      case 1
          %% THE LINEAR CASE:
          if order > 1
              res = zeros(n, 1);
          elseif order == 0
              pos = find(t >= data.times, 1, 'last');
              if pos == length(data.times)
                  res = data.coefficients(end,:)';
                  return;
              end
              t_bar = (t-data.times(pos))/(data.times(pos+1)-data.times(pos));
              res = (t_bar*data.coefficients(pos+1,:) + (1-t_bar)*data.coefficients(pos,:))';
          else % order == 1
              pos = find(t >= data.times, 1, 'last');
              
              if data.times(pos) == t && pos < length(data.times) && pos > 1
                  res = (data.coefficients(pos+1,:)-data.coefficients(pos,:))'./(data.times(pos+1)-data.times(pos));
                  res2 = (data.coefficients(pos,:)-data.coefficients(pos-1,:))'./(data.times(pos)-data.times(pos-1));
                  res = (res + res2)/2;
              else
                  if pos == length(data.times)
                      pos = pos-1;
                  end
                  res = (data.coefficients(pos+1,:)-data.coefficients(pos,:))'./(data.times(pos+1)-data.times(pos));
              end
              
          end
        
      otherwise
          error('Can only evaluate cubic or linear functions.');
  end
end