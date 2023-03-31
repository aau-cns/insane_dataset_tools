%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, linear_acc, angular_vel, orientation] = read_imu_sensor_msgs( bag, topic )
%READ_IMU_SENSOR_MSGS

disp('Reading IMU msgs...');

topic = char(topic);

seg_begin = 1;
stamp = [];
linear_acc = [];
angular_vel = [];
orientation = [];

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);
    
    if isempty(msgs)
        fprintf("[WARNING] No Messages Found in Bag " + int2str(k) + "\n");
        continue;
    end
    
    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;
    
    if isempty(stamp)
        [stamp, linear_acc, angular_vel, orientation] = imu_sensor_msgs(msgs);
    else
        [stamp(seg_begin:seg_end), linear_acc(:,seg_begin:seg_end),...
            angular_vel(:,seg_begin:seg_end), ...
            orientation(seg_begin:seg_end)] = imu_sensor_msgs(msgs);
    end
    
    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;
if isempty(stamp)
    fprintf("[WARNING] No Messages Found\n");
end
disp('DONE Reading IMU msgs...');

end

function [stamp, linear_acc, angular_vel, orientation] = imu_sensor_msgs(msgs)

len = length(msgs);

stamp(len) = 0;
linear_acc(:,len) = [0 0 0].';
angular_vel(:,len) = [0 0 0].';
orientation(:,len) = Quaternion;

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    linear_acc(:,k) = cat(1,msgs{1,k}.linear_acceleration);
    angular_vel(:,k) = cat(1,msgs{1,k}.angular_velocity);
    orientation(k) = Quaternion(cat(1,msgs{1,k}.orientation)).unit;
end
end