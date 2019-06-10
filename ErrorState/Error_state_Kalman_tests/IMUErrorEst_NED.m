function [Er_new, P_new] = ...
    IMUErrorEst_NED(C, w, v, f, lat, lon, h, Er_old, P_old, ...
                       sig2_gy, sig2_a, sig2_ab, sig2_gyb, ...
                       p_G, v_G, Ti, Ts)
%Find Error estimates in IMU with Kalman Filter 
%Takes in inputs from IMU navigation solution and fixed GPS coordinates and
%Inputs:
%   C [3 x 3] - Attitude transformation matrix between body and NED fzrame
%   w [1 x 3] - angular Velocity from IMU Gryos (rad/s)
%   v [1 x 3] - Velocity of IMU
%   f [1 x 3] - Specific force of IMU
%   lat - IMU lattitude (deg)
%   lon - IMU longitude (deg)
%   h - IMU height from Geiod (m)
%   Er_old [15 x 15] - Measurement errors of 
%                       [att, vel, pos, acc bias, g bias]
%   P_old [15 x 15] - Measurement co-varience of 
%                       [att, vel, pos, acc bias, g bias]
%   sig2_gy - sig^2 varience of Gyro
%   sig2_a - sig^2 varience of Accelerometer 
%   sig2_ab - sig^2 varience of Accelerometer bias
%   sig2_gyb - sig^2 varirnce of Gyro Bias
%   p_G [1 x 3] - position from GPS in Lat, Lon, height
%   v_G [1 x 3] - Velocity from  GPS in NED;
%   Ti - Time interval of Kalman Filter
%   Ts - Time interval of IMU
%
%Outputs:
%   d_psi [1 x 3] - error in attitude in NED
%   d_v [1 x 3] - error in velocity (m/s)
%   d_lat - error in Lattitude (deg)
%   d_lon - error in Longitude (deg)
%   d_h - error in height (meters)
%   b_a - bias in accelerometer (m/s^2)
%   b_g - bias in Gyroscope (rad/s)
    
    %% DEFINE CONSTANTS
    %Radius of Earth
    [Rn, Re] = Radii_of_curvature(lat);
    RadN = Rn + h;
    RadE = Re + h;
    R_0 = 6378137; %WGS84 Equatorial radius in meters
    e = 0.0818191908425; %WGS84 eccentricity
    R_geo = R_0 / sqrt(1 - (e * sind(lat))^2) *...
        sqrt(cosd(lat)^2 + (1 - e^2)^2 * sind(lat)^2); % from (2.137)
    z3 = zeros(3);
    
    %% Find state relationships between vars
    %Attitude State transistion
    F_11 = -skew(w);
    
    %Attitude-Velocity State transisiton
    F_12 = [0, -1/(Re + h), 0; ...
            1/(Rn + h), 0, 0; ...
            0, tand(lat) / (Re + h), 0];
    %Attitude-position state Transisiton
    %NOTE: Ommited factors dependent on w_ie as IMU accounts for this
    F_13 = [0, 0, v(2)/((Re+h)^2); ...
            0, 0, v(1)/((Re+h)^2); ...
            v(2)/ ((Re + h)*(cosd(lat)^2)), 0, (-v(2)*tand(lat))/(Re + h)^2];
    %Velocity-attitude state transition
    F_21 = -skew(C*f');
    
    %Velocity update 12.63 Groves
    F_22_11 = v(3)/RadN;
    F_22_12 = -(2*tand(lat))/RadE;
    F_22_13 = v(1)/RadN;
    F_22_21 = v(2)*tand(lat)/RadE;
    F_22_22 = (v(1)*tand(lat) + v(3)) / RadE;
    F_22_23 = v(2) / RadE;
    F_22_31 = -2*v(1) / RadN;
    F_22_32 = -2*v(2) / RadE;
    F_22 = [F_22_11, F_22_12, F_22_13; ...
            F_22_21, F_22_22, F_22_23; ...
            F_22_31, F_22_32, 0];
    
    %Velocity-Positon Update 12.64 groves
    F_23_11 = -((v(2)^2*(secd(lat)^2)) / RadE);
    F_23_13 = (v(2)^2*tand(lat)/RadE^2) - (v(1)*v(3)/RadN^2);
    F_23_21 = (v(1)*v(2)*(secd(lat)^2)/RadE);
    F_23_23 = (v(1)*v(2)*tand(lat) + v(2)*v(3)) / RadE^2;
    F_23_33 = (v(2)^2/RadE^2) + v(1)^2/RadN^2 - (2*min(Gravity_NED(lat,h)) / R_geo);
    F_23 = [F_23_11, 0, F_23_13; ...
            F_23_21, 0, F_23_23; ...
            0, 0, F_23_33];
    %Postion-Velocity Update 12.65 Groves
    F_32 = [1/RadN, 0, 0; 0, 1/(RadE*cosd(lat)), 0; 0, 0, -1];
    
    %Position-Position Update 12.66 Groves
    F_33 = [0, 0, -v(1)/RadN^2; ...
            v(2)*sind(lat)/(RadE*cosd(lat)^2), 0, v(2)/(RadE^2*cosd(lat)); ...
            0, 0, 0];
    
    
    F = [F_11, F_12, F_13, z3, C;...
        F_21, F_22, F_23, C, z3;...
        z3, F_32, F_33, z3, z3; ...
        z3, z3, z3, z3, z3; ...
        z3, z3, z3, z3, z3];
    Phi = F.*Ti;
    Phi = Phi + eye(size(F));
    
    %% get IMU varience 
    n2g = sig2_gy*Ti;
    n2a = sig2_a*Ti;
    n2ab = sig2_ab*Ti;
    n2gb = sig2_gyb*Ti;
    
    Q = (eye(5*3) .* [n2g, n2g, n2g, ...
                      n2a, n2a, n2a, ...
                       0, 0, 0, ...
                        n2ab, n2ab, n2ab, ...
                          n2gb, n2gb, n2gb]) * Ts;
    
    %% measurement Models
    
    % measurement innovation is diff between gnss and IMU
    d_z = [p_G - [lat, lon, h], v_G - v]';
    
    % measurement matrix
    H = [z3, z3, -eye(3), z3, z3; ...
        z3, -eye(3), z3, z3, z3];
    
    %% Kalman Filter
    
    Er_pred = Phi*Er_old;
    P_pred = Phi*P_old*Phi' + Q;
    S = H*P_pred*H' + (eye(6)*0.1);
    K = P_pred*H'*inv(S);
    Er_new = Er_pred + K*d_z;
    P_new = (eye(15) - K*H)*P_pred;
    
    
end