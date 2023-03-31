%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, position, orientation] = read_pose_transform_stamped(bag, topic)
%READ_POSE_TRANSFORM_STAMPED

disp('Reading Pose Transform msgs...');

topic = char(topic);

seg_begin = 1;
stamp = [];
position = [];
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
        [stamp, position, orientation] = imu_pose_transform_msgs(msgs);
    else
        [stamp(seg_begin:seg_end), position(:,seg_begin:seg_end), orientation(:,seg_begin:seg_end)] = imu_pose_transform_msgs(msgs);
    end
    
    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;
if isempty(stamp)
    fprintf("[WARNING] No Messages Found\n");
end
disp('DONE Reading Pose Transform msgs...');

end

function [stamp, position, orientation] = imu_pose_transform_msgs(msgs)
len = length(msgs);

stamp(len) = 0;
position(:,len) = [0 0 0].';
orientation(:,len) = Quaternion;

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    position(:,k) = cat(1,msgs{1,k}.transform.translation);
    
    rotation = cat(1,msgs{1,k}.transform.rotation);
    orientation(k) = Quaternion([rotation(4); rotation(1:3)]).unit;
end
end