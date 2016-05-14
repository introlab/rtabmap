

clc
%close all

particles = 300;
noiseXYZ = 0.02;
lambdaXYZ = 1;
noiseRPY = 0.005;
lambdaRPY = 250;
noise = [noiseXYZ noiseXYZ noiseXYZ noiseRPY noiseRPY noiseRPY];
lambda = [lambdaXYZ lambdaXYZ lambdaXYZ lambdaRPY lambdaRPY lambdaRPY];
axeNames = ['x' 'y' 'z' 'R' 'P' 'Y'];

% P = [13 x t] (id, incremental odom, incremental ground truth)
T = P(2:7,:);
G = P(8:end,:);

T(4:end,:) = T(4:end,:)*pi/180;
G(4:end,:) = G(4:end,:)*pi/180;


x_filtered = zeros(6, size(T, 2));
for i=1:6
    x=T(i,:);
    x=x';
    x_filtered(i,:) = pf_filter([x(2:end); 0], particles, noise(i), lambda(i));
   
end

figure
for i=1:6
    subplot(2,3,i)
    x=T(i,:);
    x_gt = G(i,:);
    plot(1:length(x),cumsum(x),'b', 1:length(x),cumsum(x_filtered(i,:)),'r', 1:length(x),cumsum(x_gt),'g');
    legend(axeNames(i), [axeNames(i) ' filtered'], [axeNames(i) ' gt'])
end

figure
for i=1:6
    subplot(2,3,i)
    x=T(i,:);
    x_gt = G(i,:);
    plot(1:length(x),x,'b', 1:length(x),x_filtered(i,:),'r', 1:length(x),x_gt,'g');
    legend(axeNames(i), [axeNames(i) ' filtered'], [axeNames(i) ' gt'])
end
