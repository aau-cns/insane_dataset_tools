%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [R_gpsw_vgps, ang_mag_gps] = get_rotation_from_two_vectors(gps_vec,mag_vec)
%GET_ROTATION_FROM_TWO_VECTORS Summary of this function goes here
%   Detailed explanation goes here

for k = 1:length(gps_vec)
    % Normalize GPS and Magnetometer vectors
    gps_normalized = gps_vec(:,k)/vecnorm(gps_vec(:,k));
    mag_normalized = mag_vec(:,k)/vecnorm(mag_vec(:,k));
    
    x = gps_normalized;
    z = cross(x,mag_normalized);
    % The gps and magnetometer Vector are not guaranteed to have an angle
    % of 90° toward each other. Thus we need to normalize the result of the
    % cross product to ensure the resulting rotation matrix has a
    % determinant of 1.
    z = z / norm(z);
    
    y = cross(-x,z); 
    y = y / norm(y);
    
    R_gpsw_vgps{k} = [x,y,z];
    
    % Angle between the vectors
    ang_mag_gps(k) = acos(dot(gps_normalized,mag_normalized)/(norm(gps_normalized)*norm(mag_normalized)));
end

end
