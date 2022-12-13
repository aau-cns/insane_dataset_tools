%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ gps_vel ] = read_gps_vel( bag )
%READ_GPS_VEL

disp('Reading GPS Velocity msgs...');

topic = '/gps_velocity_gpstime';
msgs  = bag.readAll(topic);

sec  = zeros(1,length(msgs));
nsec = zeros(1,length(msgs));

gps_vel.vel = zeros(3,length(msgs));

for k = 1:length(msgs)
    gps_vel.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    gps_vel.vel(:,k) = msgs{1,k}.twist.linear;
end

disp('DONE Reading GPS Velocity msgs...');

end
