function LN = adjustLikelihood(L)
%ADJUSTLIKELIHOOD Adjust the likelihood with std dev and mean.
%   LN = adjustLikelihood(L)
%
%   L the likelihood (m,1)
%   LN the likelihood normalized (m,1)

m = mean(L); % Calcul mean
s = std(L); % Calcul std dev
LN = zeros(size(L));
for i=1:length(L)
   if L(i) > m + s
       LN(i) = (L(i)-s)/m;
   else
      LN(i) = 1;
   end
end