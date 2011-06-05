%%%%%%%%%%%%%%%%%%
%Example of the recursive Bayes filtering
%In this example, the memory is fixed (each new signatures aren't added to
%the memory after each iteration). The likelihood is also predefined to see
%the effect of the filter.
%%%%%%%%%%%%%%%%%%
close all
clear all

%%%%%%%%%%%%
%User inputs
%%%%%%%%%%%%
stepByStep = 0; %If we want a pause after each iteration (1) otherwise batch mode (0)
usePrefinedGaussians = 0;
loopThreshold = 0.3;
%%%%%%%%%%%%

%Based on the very simple example on discrete recursive Bayes filter found 
%in the green book p. 543 
%(Russel and Norvig : Artificial Intelligence - A Modern Approach)

m = 11; %image counts in the working memory (virtual place included)
nIter = 10; %iterations
virtualPlacePrior = 0.8;

% likelihood = rand(nIter,m);
likelihood = [0.537812527177546,0.973823826601782,0.679517492334616,0.511334385523834,0.133172784517682,0.616295123827850,0.333032553637148,0.0931269898117809,0.834956800914038,0.789846368465099,0.966949104425550,0.646502710583242;0.134060600345818,0.764640454269780,0.704121891203037,0.914142330394849,0.295008954572581,0.669364170554407,0.413416552284401,0.319027136248953,0.325143400597060,0.913520615552384,0.208804360594363,0.128169077499018;0.540947224212142,0.243683930209342,0.460884056746050,0.0919339388425310,0.166627082935154,0.0372015327510951,0.414348152562633,0.886961614706448,0.367639299837922,0.533254359985603,0.520476252773875,0.0813205028568441;0.857368192475905,1,2,8,2,1,0.414348152562633,0.413416552284401,0.794839466004007,0.804076444490502,0.225546129397670,0.659227054221110;0.198017363329941,0.137854510225166,1,2,8,2,1,0.414348152562633,0.0993087162421777,0.562660356611037,0.567197873830803,0.0273988510815916;0.155609101290637,0.629811667025549,0.0762044335388465,1,2,8,2,1,0.414348152562633,0.750876451961769,0.998163526074098,0.985179800828888;0.0613777678138677,0.857014692659853,0.444624971046257,0.785375111038118,1,2,8,2,1,0.414348152562633,0.131865336731199,0.539330947126751;0.661073988932246,0.899798333270360,0.165706296551124,0.602400257965727,0.218271272437479,1,2,8,2,1,0.414348152562633,0.373834916709097;0.0186026419313314,0.348368402231864,0.398748973477181,0.465908353449366,0.706077628393318,0.522221302087482,1,2,8,2,1,0.414348152562633;0.291102936564625,0.486310193286963,0.920584670910461,0.298131346189007,0.0390136924156214,0.567617714955188,0.901802906347484,1,2,8,2,1;];
likelihoodNormalized = ones(nIter,m+1);

%Initialize the prediction matrix
%first row is for the virtual place (loop closure)
%next rows are for each already visited place
prediction = zeros(m+1,m+1);
%Gaussians...
if(usePrefinedGaussians == 0)
    %A-Create prediction with two gaussians    
    x = 1:1:m;
    for i=1:m
        y1 = gaussmf(x, [0.8 i-1]);
        y2 = gaussmf(x, [0.8 i+1]);
        y = y1+y2;
        y = (y / sum(y));
        prediction(i+1,:) = [0.1 y];
        %normalize
        prediction(i+1,:) = prediction(i+1,:) / sum(prediction(i+1,:));
    end
    prediction(1,:) = [virtualPlacePrior ones(1,m)*0.1/(m)];
    
    %Example
    x=1:1:9;
    y=gaussmf(x,[0.8 4])+gaussmf(x,[0.8 6]);
    y=y/sum(y);
    y=[0.1 y];
    y=y/sum(y);
    figure(1);
    plot([-1 x],y)
    title(['Example of the prediction (two gaussians) for the loop closure event at 5' char(10) '(we suppose that it''s more probable to be in the next/previous place) after a loop closure.' char(10) 'Id -1 means a new place.'])
    
else
    %B-Create prediction using predefined values (representing approximativaly 
    %a sum of gaussian curves)
    %(like implemented in c++, AvpdCore::BayesFilter::generatePrediction())
    %Format of predictionLC = {virtualPlace LoopCLosure neighbor-1 neighbor+1 neighbor-2 neighbor+2...}
    predictionLC = [0.1 0.175 0.1 0.275 0.05 0.15 0.025 0.025]; %Forward probabilities
%     predictionLC = [0.1 0.19 0.24 0.24 0.1 0.1 0.01 0.01]; %equal backward/forward
    prediction = generatePrediction(virtualPlacePrior, predictionLC, m);
    
    %Example
    x=1:1:9;
    smallValue = (1-sum(predictionLC)) / length(x-5);
    y = [predictionLC(1) smallValue predictionLC(7) predictionLC(5) predictionLC(3) predictionLC(2) predictionLC(4) predictionLC(6) predictionLC(8) smallValue];
    y=y/sum(y);
    figure(1);
    plot([-1 x],y)
    title(['Example of the prediction (predefined pdf) for the loop closure event at 5' char(10) '(we suppose that it''s more probable to be in the next and/or previous place) after a loop closure.' char(10) 'Id -1 means a new place.'])
end

%Initialize the prior with "no loop closure event"
prior = zeros(nIter+1, m+1);
prior(1,:) = prediction(1,:);

figure(2)
subplot(211)
imagesc(prediction)
title('prediction matrix (m+1 x m+1 where m is the number of images + 1 virtual place)')
subplot(212)
imagesc(prediction')
title('prediction^T')

statusLoop = zeros(1,nIter);

loopClosureId = 0;
for iter=1:nIter      
    %reset loopClosureId
    loopClosureId = 0;
    
    %Adjust likelihood
    likelihoodNormalized(iter,:) = adjustLikelihood(likelihood(iter,:));
    
    %posterior = n * likelihood * (prediction x prior)
    % Note the transposed prediction'
    prior(iter+1,:) = likelihoodNormalized(iter,:) .* (prediction' * prior(iter,:)')';
    
    %normalize
    prior(iter+1,:) = prior(iter+1,:) / sum(prior(iter+1,:));
        
    % Loop closure ?
    maxLoopProb = 0;
    for i=2:m+1
        if prior(iter+1,i) > maxLoopProb
            if i >= 3 && i <= m
                if prior(iter+1,i-1)+prior(iter+1,i)+prior(iter+1,i+1) > loopThreshold
                    loopClosureId = i-1;
                    maxLoopProb = prior(iter+1,i);
                end
            elseif i == 2
                if prior(iter+1,i) + prior(iter+1,i+1) > loopThreshold
                    loopClosureId = i-1;
                    maxLoopProb = prior(iter+1,i);
                end
            else
                if prior(iter+1,i-1) + prior(iter+1,i) > loopThreshold
                    loopClosureId = i-1;
                    maxLoopProb = prior(iter+1,i);
                end
            end
        end
    end

    statusLoop(iter) = loopClosureId;

    figure(3)
    plot(statusLoop(1:iter), 'o');
    title(['Loop closure (image id)(iteration ' num2str(iter) ')'])
    xlabel('Iteration')
    ylabel('Loop closure id')
    
    figure(4)
    subplot(411)
    plot(prior(iter,:)');
    title(['prior (iteration ' num2str(iter) ')'])
    subplot(412)
    plot((prediction' * prior(iter,:)')');
    title('prediction^T x prior')
    subplot(413)
    plot(likelihoodNormalized(iter,:));
    title('likelihoodNormalized')
    subplot(414)
    plot(prior(iter+1,:));
    title('posterior = n * likelihood * (prediction^T x prior)')
    
    if(stepByStep ~=0)
        disp('Press any key to continue...')
        pause
    end
end

figure(5)
subplot(3,1,1)
plot(likelihood');
title('likelihood')
subplot(3,1,2)
plot(likelihoodNormalized');
title('likelihoodNormalized')
subplot(3,1,3)
plot(prior');
title('posterior')

%% We simulate here when the neighbors of id=4 don't exist...
% On loop closure, the pdf is {1 ... 1 2 3 5 3 1...} where the
% first 3 is the center... (its a forward probability)
% No loop closure, the pdf is {10 1 1 1 1 1 1....}
figure(6)
tmp=[10,1,1,1,1,1,1,1,1,1,1;
    1,6,5,4,0,0,0,0,0,0,0;
    1,3,3,9,0,0,0,0,0,0,0;
    1,1,2,12,0,0,0,0,0,0,0;
    1,0,0,0,15,0,0,0,0,0,0;
    1,0,0,0,0,6,5,3,1,0,0;
    1,0,0,0,0,3,3,5,3,1,0;
    1,0,0,0,0,1,2,3,5,3,1;];
subplot(211)
imagesc(tmp)
title('Example of the effect of forgotten neighbors (around 5) on a prediction matrix')
subplot(212)
imagesc(tmp')
title('prediction^T')
