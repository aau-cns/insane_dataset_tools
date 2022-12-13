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
[cam_tag_test_data, tag_pose_in_world, camera_pose_in_world_gt] = gen_tag_test_data;

%% Calibrate tags
[G,tag_calibration_trans,tag_calibration_pose] = generate_tag_calibration(cam_tag_test_data, main_tag_id);

% Plot tags in main tag world
% Tags (tx) w.r.t. main tag (mt)
p_mt_tx = plot_tag_calib_in_world(G,tag_calibration_pose, main_tag_id);

% Camera trajectory expressed in the main tag frame
cam_traj = tags_to_trajectory(cam_tag_test_data, tag_calibration_pose, main_tag_id);

% Position of the main tag (mt) w.r.t. world (w)
p_w_mt = tag_pose_in_world.position(:,1);
q_w_mt = tag_pose_in_world.orientation(1);

%% Evaluate test
% Position
p_w_c_gt = camera_pose_in_world_gt.position;
p_mt_c = cam_traj.position.';
p_w_c = p_w_mt + q_w_mt.R * p_mt_c;
position_error = p_w_c - p_w_c_gt;

figure
plot3(p_w_c(1,:),p_w_c(2,:),p_w_c(3,:))
axis equal
grid on

max_position_error = max(max(abs(position_error)));
if max_position_error < 1e-13
    fprintf("Camera position error test PASSED\n")
else
    fprintf("Camera position error test NOT passed, max error %d\n", max_position_error)
end

% Rotation
q_w_c_gt = camera_pose_in_world_gt.orientation;

for k=1:length(cam_traj.t)
    q_mt_c = cam_traj.orientation(k);
    q_w_c = q_w_mt * q_mt_c;
    orientation_error_euler(k,:) = tr2rpy(Quaternion(q_w_c_gt(k) * q_w_c.inv).T,'deg');
end

max_orientation_error = max(max(abs(orientation_error_euler)));
if max_orientation_error < 1e-13
    fprintf("Camera orientation error test PASSED\n")
else
    fprintf("Camera orientation error test NOT passed, max error %d\n", max_orientation_error)
end
