close all
clear all

% Test AvpdCore::BayesFilter::computePosterior()
prior = 1;
predictionNP = 0.9;
predictionLC = [0 0.22 0.19 0.25 0.04 0.1 0.02 0.04 0.01 0.01];
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
r = sum(sum((computePosteriorResult - [1000,0,0,0,0,0,0,0,0,0;900,99,0,0,0,0,0,0,0,0;810,113,75,0,0,0,0,0,0,0;729,109,96,64,0,0,0,0,0,0;656,100,98,87,56,0,0,0,0,0;590,93,95,93,78,49,0,0,0,0;531,86,90,92,85,69,42,0,0,0;478,80,86,90,86,77,62,37,0,0;430,75,81,87,86,80,70,55,33,0;387,69,77,83,84,80,73,63,49,29;]) ~= 0)); 
if r ~= 0
    error('computePosteriorResult is not valid!')
end

