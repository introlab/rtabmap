function P = generatePrediction(NewPlacePrediction, LoopClosurePrediction, m)
% GENERATEPREDICTION Generate a prediction matrix P(m+1,m+1) for the Bayes
% filter
%   P = generatePrediction(NewPlacePrediction, LoopClosurePrediction, m)
%
%   P(1,:) is the prediction for the "no loop closure event" using the
%   NoLoopClosurePrediction. NewPlacePrediction is a number between 0
%   and 1.
%
%   P(2:end, :) is the predictions for each m "loop closure event" using the
%   LoopClosurePrediction pattern. Format LoopClosurePrediction:
%   [virtualPlace LoopCLosure neighbor-1 neighbor+1 neighbor-2
%   neighbor+2...] 
%
%   Example:
%       predictionLC = [0.1 0.19 0.24 0.24 0.1 0.1 0.01 0.01];
%       virtualPlacePrior = 0.8;
%       m = 10; %We have 10 places
%       P = generatePrediction(virtualPlacePrior, predictionLC, m);
P = zeros(m+1,m+1);
if NewPlacePrediction<0 || NewPlacePrediction>1
    error(['NoLoopClosurePrediction=' num2str(NewPlacePrediction) ' > 1 or < 0!']);
end
if sum(LoopClosurePrediction) > 1
    error(['sum(LoopClosurePrediction)=' num2str(sum(LoopClosurePrediction)) ' > 1 or < 0!']);
elseif(sum(LoopClosurePrediction) == 1)
    warning(['sum(LoopClosurePrediction)=' num2str(sum(LoopClosurePrediction)) ' == 1, all probabilities will be zero for non-neighbors']);
end
P(1,:) = [NewPlacePrediction ones(1,m)*(1-NewPlacePrediction)/(m)];
predictionLC = LoopClosurePrediction;
for i=2:m+1
    y = zeros(1,m+1);

    loopClosureId = i;

    % The first must be the virtual place
    y(1) = predictionLC(1);

    % Set all others to a small value
    y(2:length(y)) = (1-sum(predictionLC))/m;

    % Set high values (gaussians curves) to loop closure neighbors
    probAdded = 0;
    % LoopID
    y(i) = y(i) + predictionLC(2); %0.175
    probAdded = probAdded + predictionLC(2);

    % look up backward for each neighbors
    n = loopClosureId;
    for k=3:2:length(predictionLC)
        n = n-1;
        if n > 1
            y(n) = y(n) + predictionLC(k);
            probAdded = probAdded + predictionLC(k);
        else
            break;
        end
    end

   % look up forward for each neighbors
    n = loopClosureId;
    for k=4:2:length(predictionLC)
        n = n+1;
        if n <= length(y)
            y(n) = y(n) + predictionLC(k);
            probAdded = probAdded + predictionLC(k);
        else
            break;
        end
    end

    % add values not set (they are forgotten) to the loop id
    totalLCProb = sum(predictionLC(2:length(predictionLC)));
    if(probAdded >= 0 && probAdded < totalLCProb)
        y(i) =  y(i) + totalLCProb - probAdded;
    elseif(probAdded > totalLCProb+0.001)
        error(['probAdded=' num2str(probAdded) ' > ' num2str(totalLCProb) ' ?']);
    end

    P(i,:) = y;
    sum(P(i,:));
    if sum(P(i,:)) < 1-0.001 || sum(P(i,:)) > 1+0.001
        error(['sum of the resulting pdf is not one (' num2str(sum(P(i,:))) ')!']);
    end
end