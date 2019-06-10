%Joseph Lynch
%EECE 5554
%Final Project
%This script is used to calculate the position of the car based on the
%ackermann model
close all;
%clear;
%clc;


%Load data
load_data=0;
if(load_data == 1)
    bag=rosbag('g11/g11_city_data.bag');
    %bag=rosbag('ack_imu_angle_test_2.bag');
    imu_topic=select(bag,'Topic','/imu/imu');
    msgStructs = readMessages(imu_topic,'DataFormat','struct');
    imu_time = cellfun(@(m) m.Header.Stamp,msgStructs);
    x_orient = cellfun(@(m) double(m.Orientation.X),msgStructs);
    y_orient = cellfun(@(m) double(m.Orientation.Y),msgStructs);
    z_orient = cellfun(@(m) double(m.Orientation.Z),msgStructs);
    w_orient = cellfun(@(m) double(m.Orientation.W),msgStructs);
    
    gps_topic=select(bag,'Topic','/vehicle/gps/fix');
    gps_msgStructs = readMessages(gps_topic,'DataFormat','struct');
    gps_time = cellfun(@(gm) gm.Header.Stamp,gps_msgStructs);
    gps_lat = cellfun(@(gm) double(gm.Latitude),gps_msgStructs);
    gps_lon = cellfun(@(gm) double(gm.Longitude),gps_msgStructs);
end
[utm_x, utm_y, ~]=deg2utm(gps_lat(:), gps_lon(:));

g_time=transpose(cell2mat(struct2cell(gps_time)));
%gt=double(g_time(:,1)-min(g_time(:,1)));
gt=double(g_time(:,1));
%gt(:,1)=gt(:,1)+double(g_time(:,2));

i_time=transpose(cell2mat(struct2cell(imu_time)));
%it=double(i_time(:,1)-min(i_time(:,1)));
it=double(i_time(:,1));

[yaw, ~, ~] = quat2angle([w_orient, x_orient, y_orient, z_orient]);
yaw=rad2deg(yaw);

figure
hold on;
plot3(utm_x(:), utm_y(:), gt(:));
%plot(utm_x(:), utm_y(:));
%plot(gps_lat(:), gt(:));

figure
hold on
plot(it(:), yaw(:))


