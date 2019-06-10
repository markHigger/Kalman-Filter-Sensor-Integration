path = '~/Documents/eece-5554/Project_actual/data/';
dataName_city = 'g11_statiomary.bag';
filename_city = [path, dataName_city];
if ~exist('stat_imu_existFlag')
    %get rosbag
    bag_stat = rosbag(filename_city);
    %select only IMU data 
    imuBag = select(bag_stat,'Topic','/vehicle/imu/data_raw');
    %Read IMU messages
%     imuData_city = readMessages(imuBag);
    LinAcc_city = timeseries(imuBag, ...
        'LinearAcceleration.X', ...
        'LinearAcceleration.Y', ...
        'LinearAcceleration.Z');
    angVel_city = timeseries(imuBag, ...
        'AngularVelocity.X', ...
        'AngularVelocity.Y', ...
        'AngularVelocity.Z');
    
    %get angular velocity into matrix form
    save([path, 'imu_stat.mat'], 'angVel_city', 'LinAcc_city');
    city_imu_existFlag = 1;
end

if ~exist('stat_gps_existFlag')
    %get rosbag
    bag_stat = rosbag(filename_city);
    %select o
    gpsFixBag = select(bag_stat,'Topic','/vehicle/gps/fix');
    gpsVelBag = select(bag_stat,'Topic','/vehicle/gps/vel');
    %Read IMU messages
    gps_fix = timeseries(gpsFixBag, ...
        'Latitude', ...
        'Longitude', ...
        'Altitude');
    gps_vel = timeseries(gpsVelBag, ...
        'Twist.Linear.X', ...
        'Twist.Linear.Y', ...
        'Twist.Linear.Z');
    
   

    %get angular velocity into matrix form
    save([path, 'gps_stat.mat'], 'gps_fix', 'gps_vel');
    city_gps_existFlag = 1;
end