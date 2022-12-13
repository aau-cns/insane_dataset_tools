%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, position, orientation] = read_transform_geometry_msgs(bag, topic)
%READ_TRANSFORM_GEOMETRY_MSGS

disp('Reading Transform Geometry msgs...');

topic = char(topic);

seg_begin = 1;

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);

    if isempty(msgs)
        stamp=[];
        position=[];
        orientation=[];
        fprintf("[Warning] No Messages Found\n")
        return
    end


    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;

    [stamp(seg_begin:seg_end), position(:,seg_begin:seg_end),...
        orientation(:,seg_begin:seg_end)] = odometry_nav_msgs(msgs);

    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;

disp('DONE Reading Transform Geometry msgs...');

end

function [stamp, position, orientation] = odometry_nav_msgs(msgs)
len = length(msgs);

stamp(len) = 0;
position(:,len) = [0 0 0].';
orientation(:,len) = [0 0 0 0].';

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    position(:,k) = msgs{1,k}.transform.translation;

    q_tmp = cat(1,msgs{1,k}.transform.rotation);
    orientation(:,k) = [q_tmp(4); q_tmp(1:3)]; % w,x,y,z
end
end
