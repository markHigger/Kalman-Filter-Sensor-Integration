% Unsucessful 2D integration tests for IMU data through City using UTM coordinates from GPS
clear
close all
%% Load in Data
filei = load('data/imu_city.mat');
Acc = filei.LinAcc_city;
angVel = filei.angVel_city;
fileg = load('data/gps_city.mat');

p_G = fileg.gps_fix.Data;
v_G = fileg.gps_vel.Data;
time_G = fileg.gps_fix.Time;
[x_G, y_G, zone] = deg2utm(p_G(:,1), p_G(:,2));

p_G_utm = [x_G - x_G(1), y_G - y_G(1), p_G(:,3) -  p_G(1,3)];
%Guess bias (temp) in specific force
f_b = Acc.Data(:,1) + 0.0717 ;
w_b = -1 * angVel.Data(:,3);

time_i = angVel.Time;
numData = length(w_b);

v_n = zeros(2,numData);

P = zeros(2,numData);
P(:,1) = [0, 0];

theta = zeros(1,numData);
theta(1) = deg2rad(-43.3355);

vel(1) = 0;
C_bn = [cos(theta(1)), sin(theta(1))]';
GIdx = 1;
for imuIdx = 2:numData
    %update time difference 
    Ti = time_i(imuIdx) - time_i(imuIdx-1);
    
    %update angle
    theta(imuIdx) = theta(imuIdx - 1) + w_b(imuIdx - 1)*Ti;
    C_bn = [sin(theta(imuIdx)), cos(theta(imuIdx))]';
    
    v_n(:, imuIdx) = v_n(:, imuIdx-1) + (C_bn.*(f_b(imuIdx-1)*Ti));
    
    P(:,imuIdx) = P(:,imuIdx-1) + v_n(:, imuIdx)*Ti;
    if GIdx < length(time_G) && time_G(GIdx) <= time_i(imuIdx)
        v_n(:,imuIdx) = v_G(GIdx,[1:2]);
%         Lat(imuIdx) = p_G(GIdx,1);
%         Lon(imuIdx) = p_G(GIdx,2);
%         h(imuIdx) = p_G(GIdx,3);
        GIdx = GIdx+15;
    end
end
    figure
    hold on
    plot(p_G_utm(:,2), p_G_utm(:,1))
    plot(P(1,:), P(2,:))
    
    figure
    hold on;
    plot(time_G, v_G(:,2) + 15)
    plot(time_i, v_n(1,:) + 15)
    plot(time_G, v_G(:,1))
    plot(time_i, v_n(2,:))
    
    