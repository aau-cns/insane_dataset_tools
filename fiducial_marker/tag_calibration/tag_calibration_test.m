%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

close all
clear all

main_tag_id = 1;

%% Generate simulated data
%Tag poses by frames
[cam_tag_pose, tag_gt] = gen_tag_test_data;

%% Calibrate tags
[G,tag_calibrations_trans,tag_calibrations_pose] = generate_tag_calibration(cam_tag_pose, main_tag_id);

% Plot tags in main tag world
[p_mt_t, cal_id] = plot_tag_calib_in_world(G,tag_calibrations_pose, main_tag_id);

p_mt_t = [p_mt_t.x;p_mt_t.y;p_mt_t.z];

%% TODO remove invisible tags

tag_cal_in_world = (tag_gt.position(:,1) + tag_gt.orientation(1).R.' * p_mt_t);
tag_gt_in_world = tag_gt.position;


% Test Position
tag_gt_in_world - tag_cal_in_world

% Test Orientation

for k = 1:length(tag_calibrations_pose)
    if isempty(tag_calibrations_pose{k})
        continue
    else
        tag_calibrations_pose{k}.orientation
    end
end
