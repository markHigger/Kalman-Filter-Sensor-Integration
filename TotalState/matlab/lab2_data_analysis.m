%Joseph Lynch
%EECE 5554
%Lab 2
%This script is used for data processing and analysis of the rtk gps
% data collected for the purpose of lab 2
close all;
clear;
clc;


%Load UTM data
stationary = 0;
filename = ' ISEC Walk';
bag=rosbag('ISEC_new_walk_2.bag');
utm_topic=select(bag,'Topic','/utm_fix');
msgStructs = readMessages(utm_topic,'DataFormat','struct');
xData = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
yData = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs);
zData = cellfun(@(m) double(m.Pose.Pose.Position.Z),msgStructs);

%Transform/clean UTM data
xData=xData-min(xData(:));
yData=yData-min(yData(:));
%zData=zData-min(zData(:));

max_range=max(max(xData(:)), max(yData(:)));

%2D UTM scatter plot
figure;
hold on;
plot(xData, yData,'.');
if stationary == 0
    plot(xData(1), yData(1), 'ro', 'MarkerFaceColor', 'r')
    legend('UTM GPS Data', 'Start + End Point')
else
    plot(mean(xData(:)), mean(yData(:)), 'ro', 'MarkerFaceColor', 'r')
    legend('UTM GPS Data', 'Average GPS Data')
end
ylim([0 max_range]);
xlim([0 max_range]);
title(strcat('2D UTM coordinates - ' , filename))
xlabel('UTM_x (m)')
ylabel('UTM_y (m)')

%3D UTM scatter plot
figure;
hold off
scatter3(xData,yData,zData,'.')
if stationary == 0
    hold on
    scatter3(xData(1),yData(1),zData(1),'ro', 'MarkerFaceColor', 'r')
    hold off
    legend('UTM GPS Data', 'Start + End Point')
else
    hold on
    scatter3(mean(xData(:)),mean(yData(:)),mean(zData(:)),'ro', 'MarkerFaceColor', 'r')
    hold off
    legend('UTM GPS Data', 'Average GPS Data')
end
ylim([0 max_range]);
xlim([0 max_range]);
title(strcat('3D UTM coordinates - ' , filename))
xlabel('UTM_x (m)')
ylabel('UTM_y (m)')
zlabel('UTM_z (m)')

%Analyze data
disp(['X range: ' , num2str(range(xData))]);
disp(['Y range: ', num2str(range(yData(:)))]);
disp(['Z range: ', num2str(range(zData(:)))]);

disp(['Avg X: ', num2str(mean(xData(:)))]);
disp(['Avg Y: ', num2str(mean(yData(:)))]);
disp(['Avg Z: ', num2str(mean(zData(:)))]);

disp(['Std X: ', num2str(std(xData(:)))]);
disp(['Std Y: ', num2str(std(yData(:)))]);
disp(['Std Z: ', num2str(std(zData(:)))]);

if stationary == 1
    %Look for gaussian distribution
    %Plot histograms
    figure;
    hist(xData)
    title(strcat('UTM_X Histogram - ' , filename))
    figure;
    hist(yData)
    title(strcat('UTM_Y Histogram - ' , filename))
    figure;
    hist(zData)
    title(strcat('UTM_Z Histogram - ' , filename))
    %Re-plot scatter plot in real time
    
    debug_gaussian = 0;
    if debug_gaussian == 1
        figure(7);
        hold on;
        ylim([0 max_range]);
        xlim([0 max_range]);
        for i=1:numel(xData)
            plot(xData(1:i), yData(1:i),'b')
            pause(0.01);
        end
        hold off;
    end
    
else
    %Calculate distance of walk
    distance = 0;
    distancexy=0;
    for i=2:numel(xData)
        x_dist=xData(i)-xData(i-1);
        y_dist=yData(i)-yData(i-1);
        z_dist=zData(i)-zData(i-1);
        
        distancexy = distancexy + sqrt(x_dist^2 + y_dist^2);        
        distance = distance + sqrt(x_dist^2 + y_dist^2 + z_dist^2);
    end
    
    disp(['Walking distancexy: ', num2str(distancexy)]);
    disp(['Walking distance: ', num2str(distance)]);
end




