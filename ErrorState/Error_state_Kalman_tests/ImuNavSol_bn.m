function [C_new, f_new, v_new, h_new, Lat_new, Lon_new] = ...
    ImuNavSol_bn(C_old, w, f_body, v_old, h_old, Lat_old, Lon_old, Ti)
%Navigation solution for NED frame of IMU
%INPUTS:
%   C_old [3x3] - Previos Transform matrix between body and navigation
%       frame
%   w [3x1] - Angular Velocity of body frame (rads)
%   f_body [3x1] - Prev Specific Force (a + g) in Body Frame
%   v_old [3x1]- Prev Velocity in NED frame (m)
%   h_old - Prev Height from Geiod (m)
%   Lat_old - Prev Latitude (deg dec)
%   Lon_old - Prev Longitude (deg dec)
%   Ti - Time between last measurement 
%OUTPUTS:
%   C_new [3x3] - New Transform Matrix between body and NED frame
%   f_new [3x1] - New Specific Force (a + g) in NED frame
%   v_new [3x1]- New Velocity in NED frame (m)
%   h_new - New Height from Geiod (m)
%   Lat_new - New Latitude (deg dec)
%   Lon_new - New Longitude (deg dec)
    
    %Gravity is fixed in NED
    g_n = [0 0 -9.8]';
   
    %% calculate attitude transform matrix
    C_new = C_old * (eye(3) + (skew(w)*Ti));
    
    %% calculate specific force as C*fb
    
    f_new = C_new * (f_body);
    
    %% calculate new velocity as V- + Ts(f + g)
    v_new = v_old + ((f_new - g_n) * Ti)';
    
    %% calculate new position in lat,lon and heaight
    %find height
    h_new = h_old - ((Ti/2) * (v_old(3) + v_new(3)));
    
    %Get North and east radii of earth
    [Rn, Re] = Radii_of_curvature(Lat_old);
    
    %Find Latitude
    Lat_new = Lat_old + (Ti/2) * ...
        ((v_old(1) / (Rn + h_old)) + (v_new(1) / (Rn + h_new)));
    
    LonVel_old = v_old(2) / (Re + h_old*cosd(Lat_old));
    LonVel_new = v_new(2) / (Re + h_new*cosd(Lat_new));
    Lon_new = Lon_old + (Ti/2) * (LonVel_old + LonVel_new); 
end