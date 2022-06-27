##close all
##clear all

%% Use Export Poses in TORO format, then copy columns
load vertexes.txt;
load edges.txt;

set(0,'defaultAxesFontName', 'Times')
set(0,'defaultTextFontName', 'Times')

%matlab indexes  % rtabmap indexes
endMap1 = 201; % ID=206 
endMap2 = 401; % ID=411 
endMap3 = 604; % ID=621  
endMap4 = 794; % ID=814
endMap5 = 968; % ID=990
endMap6 = 1201; % ID=1230

%% 3D

t = vertexes(:,1);

##figure
##plot3(vertexes(1:endMap1,2), vertexes(1:endMap1,3), vertexes(1:endMap1,1))
##hold on
##plot3(vertexes(endMap1+1:endMap2,2), vertexes(endMap1+1:endMap2,3), vertexes(endMap1+1:endMap2,1))
##plot3(vertexes(endMap2+1:endMap3,2), vertexes(endMap2+1:endMap3,3), vertexes(endMap2+1:endMap3,1))
##plot3(vertexes(endMap3+1:endMap4,2), vertexes(endMap3+1:endMap4,3), vertexes(endMap3+1:endMap4,1))
##plot3(vertexes(endMap4+1:endMap5,2), vertexes(endMap4+1:endMap5,3), vertexes(endMap4+1:endMap5,1))
##plot3(vertexes(endMap5+1:end,2), vertexes(endMap5+1:end,3), vertexes(endMap5+1:end,1))

mapIds = zeros(vertexes(end,1), 2); % matlab index to vertexes, map id
for i=1:size(vertexes,1)
    mapIds(vertexes(i,1),1) = i;
    if i <= endMap1
        mapIds(vertexes(i,1),2) = 1;
    elseif i<=endMap2
        mapIds(vertexes(i,1),2) = 2;
    elseif i<=endMap3
        mapIds(vertexes(i,1),2) = 3;
    elseif i<=endMap4
        mapIds(vertexes(i,1),2) = 4;
    elseif i<=endMap5
        mapIds(vertexes(i,1),2) = 5;
    else
        mapIds(vertexes(i,1),2) = 6;
    end
        
end

##interLoopClosures = 0;
##intraLoopClosures = 0;
##
##for i=1:size(edges, 1)
##    if edges(i,2) > edges(i,1)+1
##        x = [vertexes(mapIds(edges(i,1),1), 2) vertexes(mapIds(edges(i,2),1), 2)];
##        y = [vertexes(mapIds(edges(i,1),1), 3) vertexes(mapIds(edges(i,2),1), 3)];
##        t = [vertexes(mapIds(edges(i,1),1), 1) vertexes(mapIds(edges(i,2),1), 1)];
##        if mapIds(edges(i,1),2) ~= mapIds(edges(i,2),2)
##            plot3(x,y,t, 'g')
##            interLoopClosures = interLoopClosures+1;
##        else
##            plot3(x,y,t, 'r')
##            intraLoopClosures = intraLoopClosures + 1;
##        end
##    end
##end
##xlabel('x')
##ylabel('y')
##zlabel('Node indexes')
##
##interLoopClosures
##intraLoopClosures

%% 2D
figure
hold on
plot([-8 6], [vertexes(endMap1,1) vertexes(endMap1,1)], 'k:')
plot([-8 6], [vertexes(endMap2,1) vertexes(endMap2,1)], 'k:')
plot([-8 6], [vertexes(endMap3,1) vertexes(endMap3,1)], 'k:')
plot([-8 6], [vertexes(endMap4,1) vertexes(endMap4,1)], 'k:')
plot([-8 6], [vertexes(endMap5,1) vertexes(endMap5,1)], 'k:')

colors = {'r:', 'g:', 'c:', 'y:', 'm:', 'c'};

for i=1:size(edges, 1)
    if edges(i,2) > edges(i,1)+1
        y = [vertexes(mapIds(edges(i,1),1), 3) vertexes(mapIds(edges(i,2),1), 3)];
        t = [vertexes(mapIds(edges(i,1),1), 1) vertexes(mapIds(edges(i,2),1), 1)];
        mapId = mapIds(edges(i,1),2);
        if mapId ~= mapIds(edges(i,2),2) && (mapId == 1 || mapIds(edges(i,2),2) == 1)
            plot(y,t, 'r')
        else
            %plot(y,t, 'r')
        end
    end
end

curveColor = 'b'
plot(vertexes(1:endMap1,3), vertexes(1:endMap1,1), curveColor)
plot(vertexes(endMap1+1:endMap2,3), vertexes(endMap1+1:endMap2,1), curveColor)
plot(vertexes(endMap2+1:endMap3,3), vertexes(endMap2+1:endMap3,1), curveColor)
plot(vertexes(endMap3+1:endMap4,3), vertexes(endMap3+1:endMap4,1), curveColor)
plot(vertexes(endMap4+1:endMap5,3), vertexes(endMap4+1:endMap5,1), curveColor)
plot(vertexes(endMap5+1:end,3), vertexes(endMap5+1:end,1), 'k')

xlabel('y')
ylabel('Node indexes')
