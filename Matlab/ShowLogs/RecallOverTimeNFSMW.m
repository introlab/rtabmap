%% Recall over time
% Here we assume that all accepted loop closures (over T_loop defined below) are true positives (inspected manually)

%close all
%clear all

%Prefix = 'ResultsTRO_nfs/NFS/';
Prefix = './';

%Adjust T_loop
T_loop = 0.05;
%T_loop = 0.07939;

set(0,'defaultAxesFontName', 'Times')
set(0,'defaultTextFontName', 'Times')

LogI = importfile([ Prefix 'LogI.txt']);
LogF = importfile([ Prefix 'LogF.txt']);


loopIds = (LogI(:,2) .* (LogF(:,10)>T_loop) .* (LogI(:, 8) ~= 1));
Accepted = loopIds > 0;
falsePositives = loopIds(11620:end) < 11620 & loopIds(11620:end);
sumFalsePositives = sum(falsePositives)+sum(loopIds(1:144)>0)

if sumFalsePositives > 0
    figure
    subplot(211)
    plot([1:25098], [loopIds(1:144)>0; zeros(11619-144, 1); falsePositives])
    title('False positives')
    hyp = LogF(:,10);
    id=LogI(:,2);
    id(145:11619) = 0;
    hyp(id>11619 | id == 0) = 0;
    subplot(212)
    plot(hyp)
    set(datacursormode,'UpdateFcn',@(Y,X){sprintf('X: %0.2f',X.Position(1)),sprintf('Y: %0.2f',X.Position(2))})
end

figure
plot(sum(LogF(:,2:7),2)*1000);
hold on
ylabel('Time (ms)')
xlabel('Location indexes')
plot([1 length(LogF(:,1))], [700 700], 'r')
set(datacursormode,'UpdateFcn',@(Y,X){sprintf('X: %0.2f',X.Position(1)),sprintf('Y: %0.2f',X.Position(2))})

%for showLogs.m
%accepted:
%T_loop = 0.0628;
%y = LogI(:,2);
%x = 1:length(y);
%y(LogF(:,10)<=T_loop | LogI(:, 8) == 3) = [];
%x(LogF(:,10)<=T_loop | LogI(:, 8) == 3) = [];

% from 145 to end, we assume that the robot always do the same loop, thus
% always revisiting previously visited places
%GroundTruth = [zeros(144,1); ones(11531-144,1)]; %Only area one
GroundTruth = [zeros(144,1); ones(11618-144,1); zeros(11785-11618,1); ones(25098-11785,1)];

Recall = zeros(length(GroundTruth),1);

for i=1:length(Recall)
    sumGT = sum(GroundTruth(1:i));
    sumAccepted = sum(Accepted(1:i));
    if sumGT == 0
        if sumAccepted > 0
            error('sumAccepted must be null?!')
        end
        Recall(i) = 0;
    elseif GroundTruth(i) == 0 && Accepted(i) ~= 0
        error(['Not supposed to have a false positive... index=' num2str(i)])
    else
        Recall(i) = sumAccepted / sumGT;
    end
    
end

%%
endSegments = [0 144 284 429 583 732 882 1030 1172 1315 1451 1585 1720 1852 1994 2124 2258 2391 2519 2641 2781 2896 3006 3116 3228 3343 3469 3594 3706 3827 3952 4063 4190 4305 4417 4535 4642 4758 4866 4976 5095 5203 5319 5430 5546 5663 5773 5884 5994 6097 6200 6285 6406 6519 6617 6721 6822 6926 7027 7128 7226 7330 7432 7531 7634 7724 7844 7945 8051 8161 8270 8386 8493 8598 8707 8829 8942 9046 9150 9245 9348 9470 9575 9666 9761 9858 9944 10082 10191 10294 10399 10501 10608 10714 10811 10914 11019 11118 11222 11328 11430 11531];
endSegments = [endSegments 11618 11785 11908 12031 12158 12285 12419 12547 12666 12786 12907 13023 13136 13246 13362 13484 13596 13751 13906 14046 14206 14346 14493 14634 14770 14909 15056 15211 15370 15520 15672 15809 15953 16095 16246 16392 16539 16690 16835 16962 17112 17251 17387 17532 17676 17820 17957 18087 18224 18353 18483 18614 18746 18882 19008 19132 19262 19386 19516 19643 19758 19886 20013 20145 20282 20414 20552 20703 20838 20983 21123 21260 21394 21522 21645 21770 21897 22019 22144 22279 22401 22525 22646 22770 22889 23011 23131 23247 23368 23490 23617 23735 23849 23959 24070 24176 24306 24415 24524 24635 24758 24877 24987 25098];
%maxSegmentsRecall = [0 0 ones(1,102) 0 ones(1,)];
RecallSegmented = zeros(length(endSegments),1);
for i=2:length(RecallSegmented)
    sumGT = sum(GroundTruth(1+endSegments(i-1):endSegments(i)));
    sumAccepted = sum(Accepted(1+endSegments(i-1):endSegments(i)));
    if sumGT == 0
        if sumAccepted > 0
            error('sumAccepted must be null?!')
        end
    elseif sumGT == 0 && sumAccepted ~= 0
        error(['Not supposed to have a false positive... index=' num2str(i)])
    else
        RecallSegmented(i) = sumAccepted / sumGT;
    end
end
RecallSegmentedMean = sum(RecallSegmented)/(length(RecallSegmented)-2)

%%

figure1 = figure
subplot(211)
plot(Recall*100)
ylabel('Recall (%)')
xlabel('Time (s)')

subplot(212)
plot(0:length(RecallSegmented(2:end))-1, RecallSegmented(2:end)*100, ':.')
ylabel('Recall (%)')
xlabel('Traversals')


%TOP
% Create line
annotation(figure1,'line',[0.432 0.432],...
    [0.595 0.92],'Color',[1 0 0]);

% Create textbox
annotation(figure1,'textbox',...
    [0.52 0.68 0.16 0.0645933014354067],...
    'String',{'AREA 2'},...
    'HorizontalAlignment','center',...
    'FitBoxToText','off',...
    'LineStyle','none');

% Create textbox
annotation(figure1,'textbox',...
    [0.2 0.68 0.16 0.0645933014354067],...
    'String',{'AREA 1'},...
    'HorizontalAlignment','center',...
    'FitBoxToText','off',...
    'LineStyle','none');

%BOTTOM
% Create line
annotation(figure1,'line',[0.446 0.446],...
    [0.118 0.445],'Color',[1 0 0]);

% Create textbox
annotation(figure1,'textbox',...
    [0.23 0.15 0.16 0.0645933014354067],...
    'String',{'AREA 1'},...
    'HorizontalAlignment','center',...
    'FitBoxToText','off',...
    'LineStyle','none');

% Create textbox
annotation(figure1,'textbox',...
    [0.55 0.15 0.16 0.0645933014354067],...
    'String',{'AREA 2'},...
    'HorizontalAlignment','center',...
    'FitBoxToText','off',...
    'LineStyle','none');
