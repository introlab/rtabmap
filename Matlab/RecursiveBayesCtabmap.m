%%%%%%%%%%%%%%%%%%%%%%%%
%In this example, we show how the resursive Bayes filtering works in RTAB-Map
%(Already Visited Place Detector). We load real data from an experiment on
%the 090206-3 dataset (in bin/data/090206-3). This example doesn't
%show the "forgotten" skill of the real algorithm implemented in RTAB-Map, so
%it doesn't show how probabilities are managed when a signature is
%forgotten. The links (parent, loop) between signatures are not shown.
%
%You can use the same script for your data. Just have the same format than
%the one used in the data text files. An other way is to use the RTAB-Map Gui,
%it provides an option to dump the working memory (Edit->Dump memory)
%directly. 
%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all

%%%%%%%%%%%%
%User inputs
%%%%%%%%%%%%
stepByStep = 0; %If we want a pause after each iteration (1) otherwise batch mode (0)
plotPriorAfterEachIter = 1;
loopThreshold = 0.3;
predictionNP = 0.8; % prediction for "New place event"
predictionLC = [0.1 0.175 0.1 0.275 0.05 0.15 0.025 0.025]; % prediction pattern for "Loop closure event"
%%%%%%%%%%%%

% load data
signRef = dlmread('DumpMemorySign.txt', ' ', 1, 0);

%ignore the virtual place, it will be generated afer each iteration
signRef = signRef(2:end,:); %preallocation
nIter = size(signRef,1);

% Initialize the prior
prior = cell(nIter,1); %preallocation
% Initialize the memory, contains only a virtual place null
memory = zeros(nIter+1,size(signRef,2)); %preallocation
memory(1,1) = -1; %id virtual place
dictionary = [];

timeUpdateDictionary = zeros(nIter,1); %seconds
timeUpdateCommonSignature = zeros(nIter,1); %seconds
timeGetLikelihood = zeros(nIter,1); %seconds
timeAdjustLikelihood = zeros(nIter,1); %seconds
timeGeneratePrediction = zeros(nIter,1); %seconds
timeUpdatePrior = zeros(nIter,1); %seconds

% Recursive Bayes estimation
for iter=1:nIter
    % Signature creation
    newSign = signRef(iter,:); %[id wordIds...]
    t = cputime;
    dictionary = updateDictionary(dictionary, newSign);
    timeUpdateDictionary(iter) = cputime - t;
    t = cputime;
    
    % add to memory
    memory(iter+1, :) = newSign;
    
    % Update virtual place
    [memory(1,:) dictionary] = updateCommonSignature(memory(1:iter+1,:), dictionary);
    timeUpdateCommonSignature(iter) = cputime - t;
    t = cputime;

    % Compute the likelihood
    likelihood = computeLikelihood(memory(iter+1,:), memory(1:iter+1,:), dictionary);
    %ignore the last (current signature)
    likelihood = likelihood(1:end-1);
    timeGetLikelihood(iter) = cputime - t;
    t = cputime;
    
    % normalize the likelihood with std dev and mean
    likelihoodNormalized = adjustLikelihood(likelihood);
    timeAdjustLikelihood(iter) = cputime - t;
    t = cputime;

    % generate prediction
    prediction = generatePrediction(predictionNP, predictionLC, length(likelihood)-1);
    timeGeneratePrediction(iter) = cputime - t;
    t = cputime;

    % update the prior (recursive Bayes estimation equation)
    if iter == 1
        prior{iter} = 1; % 100% probability to be in a new place
    else
        prior{iter} = [prior{iter-1};0];
    end
    prior{iter} = likelihoodNormalized .* (prediction' * prior{iter});
    prior{iter} = prior{iter}/sum(prior{iter}); %Normalize
    timeUpdatePrior(iter) = cputime - t;
    t = cputime;
      
    if plotPriorAfterEachIter || iter == nIter
        figure(1)
        hold off
        x = -1;
        if length(prior{iter}) > 1
            x = [x 1:length(prior{iter})-1];
        end
        plot(x, prior{iter});
        axis([x(1) x(end)+6 0 1])
        hold on
        plot(x, ones(size(x))*loopThreshold, 'r')
        title(['Prior pdf (iteration ' num2str(iter) '/' num2str(nIter) ')'])
        legend('pdf', 'loop closure threshold')
    end

    clc
    disp(['(iteration ' num2str(iter) '/' num2str(nIter) ')'])
    
    if stepByStep ~= 0 && iter ~= nIter
        disp('Press any key to continue...')
        pause
    end
end

figure(2)
plot([timeUpdateDictionary timeUpdateCommonSignature timeGetLikelihood timeAdjustLikelihood timeGeneratePrediction timeUpdatePrior])
title('Timings (seconds)')
legend('timeUpdateDictionary', 'timeUpdateCommonSignature', 'timeGetLikelihood', 'timeAdjustLikelihood', 'timeGeneratePrediction', 'timeUpdatePrior')