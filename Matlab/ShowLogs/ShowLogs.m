
%---------------------------------------------------------
% MatLab script.
% This shows some informations logged by the application.
% This script may work directly with octave.
%---------------------------------------------------------
% Just put along LogF.txt and LogI.txt files generated 
% (look in the application working directory).
% The files must have the same number of lines.
%
% Dependency : importfile.m
%---------------------------------------------------------

%--------------------
% Parameters
%--------------------

close all
clear all

Prefix = '.';
%Prefix = './Results';

DataSet = '';
%DataSet = 'NewCollege';
%DataSet = 'CityCentre';
%DataSet = 'Lip6Indoor';
%DataSet = 'Lip6Outdoor';
%DataSet = 'Lip6Outdoor_1Hz';
%DataSet = 'UdeS_1Hz';

% The Ground Truth is a squared bmp file (size must match the log files 
% length) where white dots mean loop closures.
% Grey dots mean 'loop closures to ignore', this happens when the rehearsal
% doesn't match consecutive images together.
PrefixGT = '.';
%PrefixGT = './GT';
GroundTruthFile = [PrefixGT '/' '090206-3_GT.bmp'];
%GroundTruthFile = [PrefixGT '/' DataSet '.bmp'];


%---------------------------------------------------------

display(' ');
display('Loading log files...');
importfile([Prefix '/' DataSet '/' 'LogF.txt']);
% COLUMN HEADERS : 
% 1 totalTime
% 2 timeMemoryUpdate,
% 3 timeReactivations,
% 4 timeLikelihoodCalculation,
% 5 timePosteriorCalculation,
% 6 timeHypothesesCreation,
% 7 timeHypothesesValidation,
% 8 timeRealTimeLimitReachedProcess,
% 9 timeStatsCreation
% 10 highestHypothesisValue
% 11 vpLikelihood
% 12 maxLikelihood
% 13 sumLikelihoods
% 14 mean likelihood
% 15 stddev likelihood

importfile([Prefix '/' DataSet '/' 'LogI.txt']);
% COLUMN HEADERS : 
% 1 lcHypothesisId,
% 2 mostLikelihoodId,
% 3 signaturesRemoved,
% 4 hessianThr,
% 5 wordsNewSign,
% 6 dictionarySize,
% 7 this->getSTMem().size(),
% 8 rejectLoopReason,
% 9 processMemoryUsed,
% 10 databaseMemoryUsed
% 11 signaturesReactivated
% 12 lcHypothesisReactivated
% 13 refUniqueWordsCount
% 14 reactivateId
% 15 non nulls count

startAt = 1;
% endAt = 1175;
endAt = length(LogF(:,1));

figure
H1 = plot(LogF(startAt:endAt,1)*1000);
hold on
% H2 = plot(1:length(LogF(startAt:endAt,1)), ones(length(LogF(startAt:endAt,1)),1).*mean(LogF(startAt:endAt,1))*1000, 'r-')
%title('Total process time / Location')
ylabel('Time (ms)')
xlabel('Location indexes')
meanTime = mean(LogF(startAt:endAt,1))*1000
%plot([1 length(LogF(:,1))], [800 800], 'r')
%plot([1 length(LogF(:,1))], [1000 1000], 'k')


% -------------------------
% Time details
figure
subplot(7,1,1)
plot(LogF(startAt:endAt,2)*1000)
title('timeMemoryUpdate (ms)')

subplot(7,1,2)
plot(LogF(startAt:endAt,3)*1000)
title('timeReactivations (ms)')

subplot(7,1,3)
plot(LogF(startAt:endAt,4)*1000)
title('timeLikelihoodCalculation (ms)')

subplot(7,1,4)
plot(LogF(startAt:endAt,5)*1000)
title('timePosteriorCalculation (ms)')

subplot(7,1,5)
plot(LogF(startAt:endAt,6)*1000)
title('timeHypothesesCreation (ms)')

subplot(7,1,6)
plot(LogF(startAt:endAt,7)*1000)
title('timeHypothesesValidation (ms)')

subplot(7,1,7)
plot(LogF(startAt:endAt,8)*1000)
title('timeStatsCreation (ms)')

% -------------------------
figure
plot([LogF(startAt:endAt,2) sum(LogF(startAt:endAt,2:3),2) sum(LogF(startAt:endAt,2:4),2) sum(LogF(startAt:endAt,2:5),2) sum(LogF(startAt:endAt,2:6),2) sum(LogF(startAt:endAt,2:7),2) sum(LogF(startAt:endAt,2:8),2)]);
legend('timeMemoryUpdate', 'timeReactivations', 'timeLikelihoodCalculation', 'timePosteriorCalculation', 'timeHypothesesCreation', 'timeHypothesesValidation', 'timeRealTimeLimitReachedProcess', 'timeStatsCreation')
title('Process time details')

figure
subplot(211)
plot(LogF(startAt:endAt,3));
title('Reactivation time')
subplot(212)
plot(LogI(:,11),'.')

% -------------------------
figure
subplot(211)
plot(LogI(startAt:endAt, 6));
title('dictionary size')

subplot(212)
plot([LogI(startAt:endAt, 9)/1000000 LogI(startAt:endAt, 10)/1000000]);
title('Memory usage (in MB)')
legend('Process', 'Database')
% -------------------------

figure
% subplot(211)
H1 = plot(LogI(startAt:endAt,7));
% hold on
% H2 = plot(1:length(LogI(startAt:endAt,7)), ones(length(LogI(startAt:endAt,7)),1).*mean(LogI(startAt:endAt,7)), 'r--')
%title('Working memory size')
meanWM = mean(LogI(startAt:endAt,7))
ylabel('WM size (locations)')
xlabel('Location indexes')
% set(H1,'color',[0.3 0.3 0.3])
% set(H2,'color',[0 0 0])
% subplot(212)
% plot(LogI(startAt:endAt,6));
meanDict = mean(LogI(startAt:endAt,6))
% ylabel('Dictionary size')
% xlabel('Location indexes')

meanWordsPerSign = mean(LogI(startAt:endAt,5))


% -------------------------
% Detected/Accepted/Rejected loop closures

% from VerifyEpipolarGeometry.h
% UNDEFINED, 10
% ACCEPTED, 11
% NO_HYPOTHESIS, 12
% MEMORY_IS_NULL, 13
% NOT_ENOUGH_MATCHING_PAIRS, 14
% EPIPOLAR_CONSTRAINT_FAILED, 15
% NULL_MATCHING_SURF_SIGNATURES 16

figure;
subplot(311)
plot(LogF(:,10), '.')
title('Highest posterior + lc accepted and rejected')
hold on
%rejected hypotheses
y = LogF(:,10);
x = 1:length(y);
y(LogI(startAt:endAt, 8) >= 10 & LogI(startAt:endAt, 8) <= 11) = [];
x(LogI(startAt:endAt, 8) >= 10 & LogI(startAt:endAt, 8) <= 11) = [];
plot(x,y, 'r.')
%rejected (by ratio) hypotheses
y = LogF(:,10);
x = 1:length(y);
y(LogI(startAt:endAt, 8) ~= 3) = [];
x(LogI(startAt:endAt, 8) ~= 3) = [];
plot(x,y, 'b.')
%Accepted hypotheses
y = LogF(:,10);
x = 1:length(y);
y(LogI(startAt:endAt, 8) < 10 | LogI(startAt:endAt, 8) > 11) = [];
x(LogI(startAt:endAt, 8) < 10 | LogI(startAt:endAt, 8) > 11) = [];
plot(x,y, 'g.')
subplot(312)
plot(LogI(:,2), '.')
title('Id corresponding to highest posterior + lc accepted and rejected')
hold on
%rejected hypotheses
y = LogI(:,2);
x = 1:length(y);
y(LogI(startAt:endAt, 8) >= 10 & LogI(startAt:endAt, 8) <= 11) = [];
x(LogI(startAt:endAt, 8) >= 10 & LogI(startAt:endAt, 8) <= 11) = [];
plot(x,y, 'r.')
%rejected (by ratio) hypotheses
y = LogI(:,2);
x = 1:length(y);
y(LogI(startAt:endAt, 8) ~= 3) = [];
x(LogI(startAt:endAt, 8) ~= 3) = [];
plot(x,y, 'b.')
%Accepted hypotheses
y = LogI(:,2);
x = 1:length(y);
y(LogI(startAt:endAt, 8) < 10 | LogI(startAt:endAt, 8) > 11) = [];
x(LogI(startAt:endAt, 8) < 10 | LogI(startAt:endAt, 8) > 11) = [];
plot(x,y, 'g.')
subplot(313)
plot(LogI(:,5),'.')
title('wordsNewSign')
hold on
%rejected hypotheses
y = LogI(:,5);
x = 1:length(y);
y(LogI(startAt:endAt, 8) >= 10 & LogI(startAt:endAt, 8) <= 11) = [];
x(LogI(startAt:endAt, 8) >= 10 & LogI(startAt:endAt, 8) <= 11) = [];
plot(x,y, 'r.')
%rejected (by ratio) hypotheses
y = LogI(:,5);
x = 1:length(y);
y(LogI(startAt:endAt, 8) ~= 3) = [];
x(LogI(startAt:endAt, 8) ~= 3) = [];
plot(x,y, 'b.')
%Accepted hypotheses
y = LogI(:,5);
x = 1:length(y);
y(LogI(startAt:endAt, 8) < 10 | LogI(startAt:endAt, 8) > 11) = [];
x(LogI(startAt:endAt, 8) < 10 | LogI(startAt:endAt, 8) > 11) = [];
plot(x,y, 'g.')
% %matched sign words
% y = LogI(:,2);
% x = 1:length(y);
% mask = zeros(1,length(y));
% y(LogI(startAt:endAt, 8) ~= 11) = [];
% for i=1:length(y)
%     mask(y(i)) = 1;
% end
% y = LogI(:,5);
% y(~mask) = [];
% x(~mask) = [];
% plot(x,y, 'c.')
% %matched sign words for rejected
% y = LogI(:,2);
% x = 1:length(y);
% mask = zeros(1,length(y));
% y(LogI(startAt:endAt, 8) < 12) = [];
% for i=1:length(y)
%     mask(y(i)) = 1;
% end
% y = LogI(:,5);
% y(~mask) = [];
% x(~mask) = [];
% plot(x,y, 'm.')

lcAccepted = sum(LogI(startAt:endAt, 8) >= 10 & LogI(startAt:endAt, 8) <= 11)
lcReactivated = sum(LogI(startAt:endAt, 12) == 1)
lcRejected = sum(LogI(startAt:endAt, 8) > 11 | LogI(startAt:endAt, 8) == 3)
lcRejectedNotEnoughPairs = sum(LogI(startAt:endAt, 8) == 14)
lcRejectedEpipolarGeo = sum(LogI(startAt:endAt, 8) > 14)

%figure;
%plot([1.0 * (LogI(startAt:endAt, 8) == 10) ... 
%      1.01 * (LogI(startAt:endAt, 8) == 11) ...
%      1.02 * (LogI(startAt:endAt, 8) == 14) ...
%      1.03 * (LogI(startAt:endAt, 8) == 15)], '.');
%title('Reject loop reason')
%legend('UNDEFINED', 'ACCEPTED', 'NOT ENOUGH MATCHING PAIRS', 'EPIPOLAR CONSTRAINT FAILED')

% -----------------
% Squared matrix



%%
%Precision-Recall graph

GroundTruth = []; 
if exist(GroundTruthFile, 'file')
    PR = getPrecisionRecall(LogI, LogF, GroundTruthFile, 0.03);  
    
    Precision = PR(:,1);
    Recall = PR(:,2);
    PrecisionVerified = PR(:,3);
    RecallVerified = PR(:,4);
    
    %plot the Precision-Recall
    figure
    plot([Recall RecallVerified], [Precision PrecisionVerified])
    legend('Without verification', 'With verification')
    title('Precision - Recall')
    xlabel('Recall (%)')
    ylabel('Precision (%)')
end

%%
% count = 0;
% for i=2:length(LogF(:,10))
%     if(LogF(i,10) > 0.45 && LogF(i,10) < LogF(i-1,10)*0.9)
%         display(['i=' num2str(i) ' with=' num2str(LogI(i,2)) ' ratio=' num2str(LogF(i,10)/LogF(i-1,10))])
%         count = count +1;
%     end
% end
% count

%%
% figure
% hold on
% K=100;
% %plot(1./(K*LogF(:,15)), 'r')
% %plot(log10(1./(LogF(:,15))), 'c')
% scale=1;
% %plot(log10(1./(LogF(:,15))).^2 ./ ((LogF(:,12)-LogF(:,15))./LogF(:,14)), 'k')
% %plot(log10(1./(LogF(:,15))), 'm')
% plot((LogF(:,12)-LogF(:,15))./LogF(:,14), 'g')
% plot([0, length(LogF(:,15))], [1 1], 'k:')
% plot(LogF(:,11), 'b')
% %legend(['K=' num2str(K)], 'ln', 'ln scaled', 'log10', 'max sim', '1', 'Vp likelihood')