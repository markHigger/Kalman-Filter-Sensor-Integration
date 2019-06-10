path = '~/Documents/eece-5554/Project_actual/data/';
%Attitude and Velocity Tests for IMU at Stationary
dataName_stat = 'g11_statiomary.bag';
filename_stat = [path, dataName_stat];
if ~exist('stat_existFlag')
    %get rosbag
    bag_stat = rosbag(filename_stat);
    %select only IMU data 
    imuBag = select(bag_stat,'Topic','/vehicle/imu/data_raw');
    %Read IMU messages
    imuData_stat = readMessages(imuBag);
    %get angular velocity into matrix form
    angVel_stat_struct = cellfun(@(m) m.AngularVelocity,imuData_stat);
    angVel_stat = [angVel_stat_struct(:).X; angVel_stat_struct(:).Y; angVel_stat_struct(:).Z];
    
    %get linear acceleration into matrix form 
    acc_stat_struct = cellfun(@(m) m.LinearAcceleration,imuData_stat);
    acc_stat = [acc_stat_struct(:).X; acc_stat_struct(:).Y; acc_stat_struct(:).Z];
    save([path, 'imu_stat.mat'], 'angVel_stat', 'acc_stat');
    stat_existFlag = 1;
end
fs = 100;
tao = 1/fs;

%test without Kalman Filter
g = 9.80365; %average gravity in Boston
att_i(1,:) = [pi()/4, pi()/4, pi()/4]';
V(:,1) = [0, 0, 0]';
r(:,1) = [0, 0, 0]';
w_x_i = angVel_stat(1,:);
w_y_i = angVel_stat(2,:);
w_z_i = angVel_stat(3,:);

%find attitude in reference to inertial frame
for idx = [2:length(acc_stat)]
    Omg = ...
    [1, -1 * w_z_i(idx-1) * tao, w_y_i(idx-1) * tao;
     w_z_i(idx-1) * tao, 1, -1 * w_x_i(idx-1) * tao;
     -1 * w_y_i(idx-1) * tao, w_x_i(idx-1) * tao, 1;];
    att_i(idx,:) = att_i(idx-1, :) * Omg;
end

f_i = att_i .* acc_stat';

