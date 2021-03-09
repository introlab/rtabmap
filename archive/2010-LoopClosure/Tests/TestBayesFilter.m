close all
clear all

% Test AvpdCore::BayesFilter::computePosterior()
prior = 1;
predictionNP = 0.9;
predictionLC = [0.1 0.24 0.18 0.18 0.1 0.1 0.04 0.04 0.01 0.01];
likelihood = [];
nIter = 10;

computePosteriorResult = zeros(nIter);
for i=1:nIter
    likelihood = [likelihood; 1];
    prediction = generatePrediction(predictionNP, predictionLC, length(likelihood)-1)
    if i==1
        prior = 1;
    %elseif i>2
    %    prior = [prior;prior(end)]; %use the same probability as the previous neighbor
    else
        prior = [prior;0];
    end
    prior = likelihood .* (prediction' * prior);
    prior = prior/sum(prior); %Normalize
    computePosteriorResult(1:length(prior),i) = prior;
end

%Adjust results (don't use float to compare)
computePosteriorResult = floor(computePosteriorResult*1000)';
disp('computePosteriorResult=');
disp(computePosteriorResult);
r = sum(sum((computePosteriorResult - [1000,0,0,0,0,0,0,0,0,0;900,99,0,0,0,0,0,0,0,0;820,117,62,0,0,0,0,0,0,0;756,111,82,50,0,0,0,0,0,0;704,103,84,67,40,0,0,0,0,0;663,96,82,69,54,32,0,0,0,0;631,90,79,69,58,44,26,0,0,0;604,84,76,68,58,48,36,21,0,0;583,79,73,66,58,49,40,30,17,0;567,74,69,64,58,50,41,33,25,14;]) ~= 0)); 
if r ~= 0
    error('computePosteriorResult is not valid!')
end

