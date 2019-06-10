%Joseph Lynch
%EECE 5554
%Final Project
%This script is used to calculate the position of the car based on the
%ackermann model
close all;
%clear;
%clc;


%Load data
load_data=1;
if(load_data == 1)
    %bag=rosbag('ekf_ack_imu_gps_test_2/ekf_ack_imu_gps_test_2.bag');
    bag=rosbag('bag_files/2019-04-06-02-22-10.bag');
    filt_odom_topic=select(bag,'Topic','/odometry/filtered');
    msgStructs = readMessages(filt_odom_topic,'DataFormat','struct');
    x_position = cellfun(@(m) double(m.Pose.Pose.Position.X),msgStructs);
    y_position = cellfun(@(m) double(m.Pose.Pose.Position.Y),msgStructs); 

    
    int_odom_topic=select(bag,'Topic','/eece5554/ackermann_steering');
    int_msgStructs = readMessages(int_odom_topic,'DataFormat','struct');
    int_x_position = cellfun(@(im) double(im.Pose.Pose.Position.X),int_msgStructs);
    int_y_position = cellfun(@(im) double(im.Pose.Pose.Position.Y),int_msgStructs); 
    
    gps_topic=select(bag,'Topic','/vehicle/gps/fix');
    gps_msgStructs = readMessages(gps_topic,'DataFormat','struct');
    gps_lat = cellfun(@(gm) double(gm.Latitude),gps_msgStructs);
    gps_lon = cellfun(@(gm) double(gm.Longitude),gps_msgStructs);
end

[utm_x, utm_y, ~]=deg2utm(gps_lat(:), gps_lon(:));

%Convert Ackermann positions to utm
utm_x_odom_filt(:,1)=x_position(:,1)+utm_x(1,1);
utm_y_odom_filt(:,1)=y_position(:,1)+utm_y(1,1);

utm_x_odom_int(:,1)=int_x_position(:,1)+utm_x(1,1);
utm_y_odom_int(:,1)=int_y_position(:,1)+utm_y(1,1);

%Plot utm
figure
hold on;
plot(utm_x_odom_filt(:,1), utm_y_odom_filt(:,1));
plot(utm_x(:,1), utm_y(:,1));
legend('odom filtered', 'gps')
title('Ackermann Odometry + IMU Kalman Filtered vs GPS')

%Rotate filtered data
origin_x=utm_x_odom_filt(1,1);
origin_y=utm_y_odom_filt(1,1);

rot_x=utm_x_odom_filt(:)-origin_x;
rot_y=utm_y_odom_filt(:)-origin_y;

alpha=deg2rad(28); % -28
R  = [cos(alpha) -sin(alpha); sin(alpha)  cos(alpha)];
rCoords = R*[transpose(rot_x) ; transpose(rot_y)];
xr = rCoords(1,:)';      
yr = rCoords(2,:)';  

xr=xr+origin_x;
yr=yr+origin_y;

%Plot rotated odom filtered
figure
hold on;
plot(xr(:,1), yr(:,1));
plot(utm_x(:,1), utm_y(:,1));
legend('rotated odom filtered', 'gps')
title('Rotated Ackermann Odometry + IMU Kalman Filtered vs GPS')

%Plot filtered ackermann and integrated ackermann
idx=458000;
figure
hold on;
plot(utm_x_odom_filt(:,1), utm_y_odom_filt(:,1));
plot(utm_x_odom_int(:,1), utm_y_odom_int(:,1));
plot(utm_x(:,1), utm_y(:,1));
legend('ekf filtered', 'integrated', 'gps')
title('Ackermann Odometry + IMU Kalman Filtered vs GPS')
