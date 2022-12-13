%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [tag_data, tags_detected] = read_aruco_rostopic(bag, topic)
%READ_ARUCO_ROSTOPIC Writes all tag detections timeslot in an array structure
%   The transformations are the position of the camera w.r.t. the tag frame

msgs  = bag.readAll(char(topic));

%% Read all tag data into a defined tag datastructure
local_counter = 1;
tags_available = [];
for k = 1:length(msgs)
    stamp = cat(1,msgs{1,k}.header.stamp.time);
    id = horzcat(msgs{k}.transforms.fiducial_id);
    pose = horzcat(msgs{k}.transforms.transform);

    len_ids = length(id);

    if len_ids == 0
        continue
    end

    tags_available = unique([tags_available, id]);

    aruco_detections(local_counter).t = stamp;
    aruco_detections(local_counter).detections = len_ids;

    for i = 1:len_ids
        aruco_detections(local_counter).data(i).id = id(i);
        aruco_detections(local_counter).data(i).pose.translation = pose(i).translation;

        % transform msg defines the quaternion xyzw, here we use wxyz
        rotation = pose(i).rotation;
        aruco_detections(local_counter).data(i).pose.rotation = [rotation(4); rotation(1:3)];
    end

    local_counter = local_counter + 1;
end

tag_data = aruco_detections;
tags_detected = tags_available;

end
