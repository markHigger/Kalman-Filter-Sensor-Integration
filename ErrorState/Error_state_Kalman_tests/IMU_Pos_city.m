%Full Tests from position using IMU as basis for error state kalman filtering 
clear
close all
file = load('data/imu_city.mat');
Acc = file.LinAcc_city;
angVel = file.angVel_city;

file = load('data/gps_city.mat');

p_G = file.gps_fix.Data;
v_G = file.gps_vel.Data;
time_G = file.gps_fix.Time;

w_b = angVel.Data;
f_b = Acc.Data;
time_i = angVel.Time;

Lat_i = p_G(1,1);
Lon_i = p_G(1,2);
g_b = [0, 0, 9.7];

%get frame transform
%assumed aligned with NED
phi = deg2rad(0); %N (roll) offset (rad)
theta = deg2rad(0); %E (pitch) offset (rad)
psi = deg2rad(43.348 + 90); %Down (yaw) offset (rad)

C_Phi = [1, 0, 0; 0, cos(phi), sin(phi); 0, -sin(phi), cos(phi)];
C_Theta = [cos(theta), 0, -sin(theta); 0, 1, 0; sin(theta), 0, cos(theta)];
C_Psi = [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0; 0, 0, 1]; 
C_bn_0 = C_Phi*C_Theta*C_Psi;

C_ne = ...
    [-sin(Lat_i) * cos(Lon_i), -sin(Lon_i), -cos(Lat_i) * cos(Lon_i); ...
    -sin(Lat_i) * sin(Lon_i), cos(Lon_i), -cos(Lat_i) * sin(Lon_i); ...
    cos(Lat_i), 0, -sin(Lat_i)];

T_i = 0.01;%timestep should be gotten from data
numData = length(w_b);

% Transformation matricies
C_bn = zeros(3,3,numData); 
C_bn(:,:,1) = C_bn_0; 

%Specific force in NED
f_bn = zeros(numData, 3); 
f_bn(1,:) = C_bn_0*f_b(1,:)';

%velocity in NED curvilnear meters
v_ben = zeros(numData, 3); 
v_ben(1,:) = [0, 0, 0];

%estimated height of Boston
h = zeros(1,numData);
h(1) = p_G(1,3); 


Lat = zeros(1,numData);
Lat(1) = Lat_i;

Lon = zeros(1,numData);
Lon(1) = Lon_i;

aBias = [mean(f_b(:,1)), mean(f_b(:,1)), 0]';

gBias = mean(w_b)';
%estimated error 
Er = zeros(15, 5000);
Er(:,1) = zeros(15,1);
Er(13:15, 1) = gBias;
Er(11:13, 1) = aBias;
%esitmated co-varience
P = zeros(15, 15, 5000);
P(:,:,1) = eye(15) .* [0.1, 0.1, 0.1, ... %att
                        0.1, 0.1, 0.1, ... %vel
                        0.1, 0.1, 0.1, ... %Phi
                        0.1, 0.1, 0.1, ...
                        0.1, 0.1, 0.1 ];
GIdx = 1;
KIdx = 1;
KInterval = 1; %in number of GNSS messages(@1Hz)
biasLog = zeros(numData, 3);
biasLog(1,:) = aBias;
for imuIdx = 2:length(w_b)  
    T_i = time_i(imuIdx) - time_i(imuIdx - 1);
    %Find tranform matrix, specific force, velocity, height, lat and lon
    [C_bn(:,:,imuIdx), f_bn(imuIdx,:), v_ben(imuIdx,:) , ...
     h(imuIdx), Lat(imuIdx), Lon(imuIdx)] = ...
        ImuNavSol_bn(C_bn(:,:,imuIdx - 1), w_b(imuIdx,:), ...
        (f_b(imuIdx,:) )', v_ben(imuIdx-1, :), ...
        h(imuIdx - 1), Lat(imuIdx - 1), Lon(imuIdx - 1), T_i);
    TILOG(imuIdx) = T_i;
    sig2_gy = 0.01;
    sig2_a = 0.01;
    sig2_ab = 0.01;
    sig2_gyb = 0.01;
    biasLog(imuIdx,:) = aBias;

     if GIdx < length(time_G) && time_G(GIdx) <= time_i(imuIdx)
         T_k = KInterval;
         %Compute error with Kalman Filter
         [Er(:, KIdx + 1), P(:,:,KIdx+1)] = IMUErrorEst_NED(...
            C_bn(:,:,imuIdx), w_b(imuIdx,:), v_ben(imuIdx,:), ...
            f_bn(imuIdx,:), Lat(imuIdx), Lon(imuIdx), h(imuIdx), ...
            Er(:,KIdx), P(:,:,KIdx), sig2_gy, sig2_a, sig2_ab, sig2_gyb, ...
            p_G(GIdx,:), v_G(GIdx,:), T_i, T_k);
        
         %Update indecies
         GIdx = GIdx + KInterval;
         KIdx = KIdx + 1;
        
        %Update Error
         d_Psi = Er(1:3, KIdx);
        d_v = Er(4:6, KIdx);
        d_Lat = Er(7, KIdx);
        d_Lon = Er(8, KIdx);
        d_h = Er(9, KIdx);
        
         aBias = Er(10:12, KIdx);
        gBias = Er(13:15, KIdx); 
         biasLog(imuIdx,:) = aBias;
        
        %Update navigation solutions with Kalman errors
        C_bn(:,:,imuIdx) = (eye(3) - skew(d_Psi)) * C_bn(:,:,imuIdx - 1);
        v_ben(imuIdx,:) = v_ben(imuIdx - 1,:) - d_v';
        Lat(imuIdx) = Lat(imuIdx - 1) + d_Lat;
        Lon(imuIdx) = Lon(imuIdx - 1) + d_Lon;
        h(imuIdx) = h(imuIdx - 1) + d_h;
     end

    if GIdx < length(time_G) && time_G(GIdx) <= time_i(imuIdx)
        v_ben(imuIdx,:) = v_G(GIdx,:);
%         Lat(imuIdx) = p_G(GIdx,1);
%         Lon(imuIdx) = p_G(GIdx,2);
%         h(imuIdx) = p_G(GIdx,3);
        GIdx = GIdx+1;
    end

    
end

 figure
 hold on
 plot(p_G(:,2), p_G(:,1))
 plot(Lon, Lat)
 
%  figure
% plot(time_G, p_G(:,1))
% hold
% plot(time_i, Lat)

% eul = zeros(3, numData);
% for rotIdx = 1:numData
%     eul(:,rotIdx) = rad2deg(rotm2eul(C_bn(:,:,rotIdx)));
% end
% figure
% hold on
% plot(eul(1,:))
% plot(eul(2,:))
% plot(eul(3,:))

figure 
plot(f_bn(:,1));
hold on;
plot(f_bn(:,2));
plot(f_bn(:,3));
