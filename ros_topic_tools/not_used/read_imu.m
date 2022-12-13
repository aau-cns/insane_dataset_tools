%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ imu, pressure ] = read_imu( bag )
%READ_IMU

disp('Reading IMU msgs...');

topic = '/fcu/imu_custom';
msgs  = bag.readAll(topic);

sec  = zeros(1,length(msgs));
nsec = zeros(1,length(msgs));

pressure.stamp = zeros(1,length(msgs));
imu.stamp = zeros(1,length(msgs));

for k = 1:length(msgs)
    imu.stamp(k) = msgs{1,k}.header.stamp.time;
    imu.acceleration(:,k) = msgs{1,k}.acceleration;
    imu.angular_velocity(:,k) = msgs{1,k}.angular_velocity;
    imu.orientation(k) = Quaternion(msgs{1,k}.orientation);
    pressure.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    pressure.height(k) = msgs{1,k}.height;
    pressure.differential_height(k) = msgs{1,k}.differential_height;
end

disp('DONE Reading IMU msgs...');

end

