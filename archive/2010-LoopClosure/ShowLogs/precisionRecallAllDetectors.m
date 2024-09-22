
#set(0,'defaultAxesFontName', 'Times')
#set(0,'defaultTextFontName', 'Times')

Prefix = 'loop_closure_detection_datasets';
Dataset= 'CityCentre'
Detectors = {'Surf'; 'Sift'; 'CudaSift'; 'GfttBrief'};

% The Ground Truth is a squared bmp file (size must match the log files 
% length) where white dots mean loop closures.
% Grey dots mean 'loop closures to ignore', this happens when the rehearsal
% doesn't match consecutive images together.
GroundTruthFile = [Prefix '/' Dataset '.png'];

colors = 'kbgrcm';
figure
xlabel('Recall (%)')
ylabel('Precision (%)')
hold on;

Results = {};
TimeResults = {};

for i=1:length(Detectors)
    LogI = importfile([Prefix '/' Dataset '/' Detectors{i} 'LogI.txt']);
    LogF = importfile([Prefix '/' Dataset '/' Detectors{i} 'LogF.txt']);

    PR = getPrecisionRecall(LogI, LogF, GroundTruthFile, 0.07);
    plot(100*PR(:,2), 100*PR(:,1), colors(mod(i,6)+1));
   % hold on;
   Results{i} = PR;
   time = sum(LogF(:,2:7),2)+LogF(:,17);%LogF(:,1)
   TimeResults{i} = time;
   meanTime = mean(time)
   meanWm = mean(LogI(:,7))
   meanDict = mean(LogI(:,6))
   maxTime = max(time)
   maxWm = max(LogI(:,7))
   maxDict = max(LogI(:,6))
    
    %figure(2)
    %plot(PR(:,4), PR(:,3), colors(mod(i,6)+1));
    %hold on;
end
legend(Detectors)
title(Dataset)

figure
rows=floor(length(Detectors)/2 )+ mod(length(Detectors),  2)
for i=1:length(TimeResults)
    subplot(rows, 2, i)
    plot(TimeResults{i})
    ylabel('Time (s)')
    title([Detectors{i} ' (' num2str(mean(TimeResults{i})) 's)'])
end
xlabel('Location indexes')
