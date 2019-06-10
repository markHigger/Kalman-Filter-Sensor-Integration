%Joseph Lynch
%EECE 5554
%Final Project
%This script is used to calculate the position of the car based on the
%ackermann model
close all;
%clear;
%clc;


%Load data
WHEEL_RL=3;
WHEEL_RR=4;
STEER_FL=5;
STEER_FR=6;
SAMPLE_PD=0.0067; %Seconds between joint_state samples - 150 Hz
WHEEL_RADIUS=0.33909; % meters
T = 1.574; %Length of rear car axle in meters
L = 2.850; %Length between 2 car axles in meters


load_data=0;
if(load_data == 1)
    %bag=rosbag('g11/g11_city_data.bag');
    bag=rosbag('bag_files/paper_settings_ack_ekf/paper_settings_ack_ekf.bag');
    %bag=rosbag('snow_data/snow_car.bag');
    joint_states_topic=select(bag,'Topic','/vehicle/joint_states');
    msgStructs = readMessages(joint_states_topic,'DataFormat','struct');
    ang_vel_rl = cellfun(@(m) double(m.Velocity(WHEEL_RL)),msgStructs);
    ang_vel_rr = cellfun(@(m) double(m.Velocity(WHEEL_RR)),msgStructs);
    steer_angle_fl = cellfun(@(m) double(m.Position(STEER_FL)),msgStructs);
    steer_angle_fr = cellfun(@(m) double(m.Position(STEER_FR)),msgStructs);
    
    gps_topic=select(bag,'Topic','/vehicle/gps/fix');
    gps_msgStructs = readMessages(gps_topic,'DataFormat','struct');
    gps_lat = cellfun(@(gm) double(gm.Latitude),gps_msgStructs);
    gps_lon = cellfun(@(gm) double(gm.Longitude),gps_msgStructs);
    
    
    ack_topic=select(bag,'Topic','/eece5554/ackermann_steering');
    ack_msgStructs = readMessages(ack_topic,'DataFormat','struct');
    ros_ack_x = cellfun(@(ackm) double(ackm.Pose.Pose.Position.X),ack_msgStructs);
    ros_ack_y = cellfun(@(ackm) double(ackm.Pose.Pose.Position.Y),ack_msgStructs);
end

[utm_x, utm_y, ~]=deg2utm(gps_lat(:), gps_lon(:));

ang_vel_car=(ang_vel_rl+ang_vel_rr)./2;
lin_vel_car=ang_vel_car(:,1)*WHEEL_RADIUS;

num_samples=length(ang_vel_car);

position=zeros(num_samples, 2);
r_i=zeros(num_samples, 1);
r_o=zeros(num_samples, 1);
r=zeros(num_samples, 1);
omega=zeros(num_samples, 1);
thetazzz=zeros(num_samples, 2);
anglezzz=zeros(num_samples, 1);
angle=-43.348; %Angle car is currently facing - in reference to pointing east
anglzzz(1,1)=angle;

for a=0.012:0.0001:0.014
    position=zeros(num_samples, 2);
    r_i=zeros(num_samples, 1);
    r_o=zeros(num_samples, 1);
    r=zeros(num_samples, 1);
    omega=zeros(num_samples, 1);
    thetazzz=zeros(num_samples, 2);
    anglezzz=zeros(num_samples, 1);
    angle=-43.348; %Angle car is currently facing - in reference to pointing east
    anglzzz(1,1)=angle;
    
    for i=2:num_samples
        if abs(steer_angle_fr(i,1)) > a %Steering angle not 0 - turning - use Ackermann
            %Determine inside/outside angle
            theta_i=0;
            theta_o=0;
            if steer_angle_fr(i,1) > 0 %Left turn
                theta_i=steer_angle_fl(i,1);
                theta_o=steer_angle_fr(i,1);
            else
                theta_i=steer_angle_fr(i,1);
                theta_o=steer_angle_fl(i,1);
            end

            thetazzz(i,1)=theta_i;
            thetazzz(i,2)=theta_o;

            %Calculate turn radius
            r_i(i,1)=L/tan(theta_i);
            r_o(i,1)=L/tan(theta_o);

            r(i,1)=(r_i(i,1)+r_o(i,1))/2;

            %Calculate angular velocity
            omega(i,1)=lin_vel_car(i,1)/r(i,1);

            %Integrate to calculate angle
            angle=angle+rad2deg(omega(i,1)*SAMPLE_PD);

        end

        %Integrate wheel encoders to get distance traveled
        dist_traveled=lin_vel_car(i,1)*SAMPLE_PD;
        x_dist=dist_traveled*cos(deg2rad(angle));
        y_dist=dist_traveled*sin(deg2rad(angle));

        %Add distance traveled in angle
        position(i,1)=position(i-1,1)+x_dist;
        position(i,2)=position(i-1,2)+y_dist;

        anglzzz(i,1)=angle;
    end
    %Convert Ackermann positions to utm
    utm_ack_position=zeros(num_samples, 2);
    utm_ack_position(:,1)=position(:,1)+utm_x(1,1);
    utm_ack_position(:,2)=position(:,2)+utm_y(1,1);

    figure
    hold on;
    plot(utm_ack_position(:,1), utm_ack_position(:,2));
    plot(utm_x(:,1), utm_y(:,1));
    legend('ros ackermann', 'gps')
    title(num2str(a))
    
    disp([num2str(a), ': ', num2str((sqrt((position(end,1)-position(1,1))^2 + (position(end,2)-position(1,2))^2 )))])
end

%Convert Ackermann positions to utm
utm_ack_position=zeros(num_samples, 2);
utm_ack_position(:,1)=position(:,1)+utm_x(1,1);
utm_ack_position(:,2)=position(:,2)+utm_y(1,1);

ros_ack_x_utm=ros_ack_x+utm_x(1,1);
ros_ack_y_utm=ros_ack_y+utm_y(1,1);

figure
hold on;
%plot(position(:,1), position(:,2));
plot(utm_ack_position(:,1), utm_ack_position(:,2));
%plot(ros_ack_x_utm(:,1), ros_ack_y_utm(:,1));
plot(utm_x(:,1), utm_y(:,1));
legend('ros ackermann', 'gps')
%legend('ackermann', 'gps')


