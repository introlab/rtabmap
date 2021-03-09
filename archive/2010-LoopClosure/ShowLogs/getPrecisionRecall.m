function [ PR ] = getPrecisionRecall( LogI, LogF, GT_file, LoopThr )
%GETPRECISIONRECALL Calculate the precision-recall results from the log
%files of RTAB-Map and a Ground Truth file (a bmp).
%   PR(:,1) = Precision
%   PR(:,2) = Recall
%   PR(:,3) = Precision with verification
%   PR(:,4) = Recall with verification
%
%   LogI:     The 'LogI.txt' generated file
%   LogF:     The 'LogF.txt' generated file
%   GT_file:  The related Ground truth file of the dataset ('GT.bmp')
%   LoopThr:  Display false positives over the loop thr (>=0.0 && < 1.0)

GroundTruth = []; 
if exist(GT_file, 'file')
    display('--- getPrecisionRecall ---');
    display(['Loading GroundTruth ''' GT_file ''' ...']);
    GroundTruth = imread(GT_file);
    if max(max(GroundTruth)) == 1
        GroundTruth=GroundTruth*255;
    end
else
    error(['The ground truth ''' GT_file '''doesn''t exist.'])
end

if ~isempty(GroundTruth)
    %display('Calculating Precision-Recall graph')
    %figure
    %imshow(GroundTruth)
    %title('GroundTruth')
    
    if size(GroundTruth, 1) ~=  length(LogF(:,1)) ||  size(GroundTruth, 1) ~=  length(LogI(:,1))
        error(['The ground truth size doesn''t match the log files (LogI=' num2str(length(LogI(:,1))) ', LogF=' num2str(length(LogF(:,1))) ', GT=' num2str(size(GroundTruth, 1)) ')'])
    end
    
    
    %[highestHypot, CorrespondingID, GT, Accepted, Good, Index, UnderLoopRatio] descending order
    if(sum(LogI(:,8) == 10) > 0)
        %OLD
        warning('Detected old Log format...');
        lc = [LogF(:,10) LogI(:,2) sum(GroundTruth == 255, 2)>0 (LogI(:, 8) == 10 | LogI(:, 8) == 11) zeros(length(LogI(:,1)),1) (1:length(LogF(:,10)))' LogI(:, 8) == 3];
    else
        %NEW 
        lc = [LogF(:,10) LogI(:,2) sum(GroundTruth == 255, 2)>0 LogI(:, 1) > 0 zeros(length(LogI(:,1)),1) (1:length(LogF(:,10)))' LogI(:, 8) == 1];
    end
    
    %eliminate loops on diagonal
    ignored = 0;
    for i=1:length(lc)
        index = find(GroundTruth(:,i) > 0 & GroundTruth(:,i) < 255);
        if ~isempty(index)
            row = GroundTruth(index(1), :);
            if lc(i,2) >= min(index) && lc(i,2) <= max(index)
                %display(['i=' NUM2STR(i) ' loop=' NUM2STR(LogI(i,2)) ' min(index)=' NUM2STR(min(index)) ' max(index)' NUM2STR(max(index))])
                lc(i,1) = 0;
                ignored = ignored + 1;
            end
        end
    end
    
    lc = sortrows(lc, -1);

    GT_total_positives = sum(sum(GroundTruth == 255, 2) > 0)
    if GT_total_positives == 0
        error(['The ground truth ''' GT_file '''doesn''t have any white pixels!?'])
    end
    
    %figure
    %plot(sum(GroundTruth > 0, 2)>0)
    %title('Ground truth (timeline)')
    
    sizeNonZero = sum(lc(:,1) > 0);
    
    PR = zeros(sizeNonZero, 4);
    for i=1:length(lc)
        if lc(i,1) == 0
            break;
        end
        
        id = lc(i,2);
        
        if id && sum(GroundTruth(lc(i,6), id)) > 0
            lc(i,5) = 1;
        end
       
        %Recall = Loop closures detected / GT loop closures
        PR(i,2) = sum(lc(1:i,5) & ~lc(1:i,7) & lc(1:i,2)) / GT_total_positives;
        
        %Precision = Good loop closures / total loop closure detected
        PR(i,1) = sum(lc(1:i,5) & ~lc(1:i,7) & lc(1:i,2)) / sum(~lc(1:i, 7) & lc(1:i,2));
        
        if ~lc(i,5) && ~lc(i,7) && id && lc(i,1) >= LoopThr
            display(['False positive! id=' num2str(lc(i,6)) ' with old=' num2str(id) ' (p=' num2str(lc(i,1)) ')'] )
        end
        
        %Recall = Loop closures detected / GT loop closures
        PR(i,4) = sum(lc(i,4) & lc(1:i,5) & ~lc(1:i,7) & lc(1:i,2)) / GT_total_positives;
        
        %Precision = Good loop closures / total loop closure detected
        if sum(~lc(1:i, 7) & lc(1:i,2) & lc(i,4)) > 0
            PR(i,3) = sum(lc(i,4) & lc(1:i,5) & ~lc(1:i,7) & lc(1:i,2)) / sum(~lc(1:i, 7) & lc(1:i,2) & lc(i,4));
        else
            PR(i,3) = 0;
        end
        
        if lc(i,4) && ~lc(i,5) && ~lc(i,7) && id && lc(i,1) >= LoopThr
            display(['False positive accepted! id=' num2str(lc(i,6)) ' with old=' num2str(id) ' (p=' num2str(lc(i,1)) ')'] )
        end
    end
    
    index = find(PR(:,1) == 1);
    if ~isempty(index)
        maxRecall = PR(index(end),2) * 100;
        display(['Recall max (Precision=100%) = ' num2str(maxRecall) '% (p=' num2str(lc(index(end),1)) '), accepted=' num2str(sum(lc(1:index(end),5) & ~lc(1:index(end),7) & lc(1:index(end),2)))])
    else
        display('Recall max (Precision=100%) = 0')
    end
    indexAccepted = find(PR(:,3) == 1);
    if ~isempty(indexAccepted)
        maxRecall = PR(indexAccepted(end),2) * 100;
        display(['Recall max accepted (Precision=100%) = ' num2str(maxRecall) '% (p=' num2str(lc(indexAccepted(end),1)) '), accepted=' num2str(sum(lc(1:indexAccepted(end),5) & ~lc(1:indexAccepted(end),7) & lc(1:indexAccepted(end),2)))])
    else
        display('Recall max accepted (Precision=100%) = 0')
    end
    display(['ignored = ' num2str(ignored)])
end

end

