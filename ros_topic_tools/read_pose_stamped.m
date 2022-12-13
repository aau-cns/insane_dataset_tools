%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, position, orientation] = read_pose_stamped(bag,topic)
%READ_POSE_STAMPED

disp('Reading Pose msgs...');

topic = char(topic);

seg_begin = 1;

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);

    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;

    [stamp(seg_begin:seg_end), position(:,seg_begin:seg_end),...
        orientation(:,seg_begin:seg_end)] = pose_msgs(msgs);

    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;

disp('DONE Reading Pose msgs...');

end

function [stamp, position, orientation] = pose_msgs(msgs)
len = length(msgs);

stamp(len) = 0;
position(:,len) = [0 0 0].';
orientation(:,len) = [0 0 0 0].';

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    position(:,k) = cat(1,msgs{1,k}.pose.position);

    q_tmp = cat(1,msgs{1,k}.pose.orientation);
    orientation(:,k) = [q_tmp(4); q_tmp(1:3)]; % w,x,y,z
end

end