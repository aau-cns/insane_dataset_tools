%% Copyright (C) 2023 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>


function [tag_data, tags_detected] = read_fiducial_marker_msgs(bag, topic)
% Read Fiducial Marker Pose Messages

disp('Reading Fiducial Marker msgs...');

topic = char(topic);

seg_begin = 1;
tags_detected = [];

for k = 1 : length(bag)
    [tag_data_bag, tags_detected_bag] = read_aruco_rostopic(bag(k), topic);
    
    if isempty(tag_data_bag)
        fprintf("[WARNING] No Messages Found in Bag " + int2str(k) + "\n");
        continue;
    end
    
    next_size = length(tag_data_bag);
    seg_end = seg_begin + next_size-1;
    
    tag_data(seg_begin:seg_end) = tag_data_bag;
    tags_detected = unique([tags_detected, tags_detected_bag]);
    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;
if isempty(tags_detected)
    fprintf("[WARNING] No Messages Found\n");
end
disp('DONE Reading Fiducial Marker msgs...');
end