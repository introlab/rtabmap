close all
clear all

% Test AvpdCore::BayesFilter::adjustLikelihood()
likelihood = [0 0 0 0];
adjustLikelihoodResult1 = adjustLikelihood(likelihood)
likelihood = [0.3 0.4 0.2 0.9];
theMean = mean(likelihood)
theSdtDev = std(likelihood)
adjustLikelihoodResult2 = floor(adjustLikelihood(likelihood)*1000)