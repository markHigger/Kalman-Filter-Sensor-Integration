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
    bag=rosbag('bag_files/g11/g11_statiomary.bag');
    %bag=rosbag('ack_imu_angle_test_2.bag');
    imu_topic=select(bag,'Topic','/vehicle/imu/data_raw');
    msgStructs = readMessages(imu_topic,'DataFormat','struct');
    x_accel = cellfun(@(m) double(m.LinearAcceleration.X),msgStructs);
    y_accel = cellfun(@(m) double(m.LinearAcceleration.Y),msgStructs);
    z_accel = cellfun(@(m) double(m.LinearAcceleration.Z),msgStructs);
    
    
    imuimu_topic=select(bag,'Topic','/imu/imu');
    imu_msgStructs = readMessages(imuimu_topic,'DataFormat','struct');
    x_accel_imu = cellfun(@(im) double(im.LinearAcceleration.X),imu_msgStructs);
    y_accel_imu = cellfun(@(im) double(im.LinearAcceleration.Y),imu_msgStructs);
    z_accel_imu = cellfun(@(im) double(im.LinearAcceleration.Z),imu_msgStructs);
end

disp('Raw data:')