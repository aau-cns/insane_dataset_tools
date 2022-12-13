%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

clear all
close all
addpath(genpath('matlab_rosbag-0.5.0-linux64'));

data_path = "/tmp/";

%% Define Data Input (file tree)
bagfile_path = fullfile(data_path, "ids/tag_detections.bag");
marker_cam_topic_name = "/marker_detections";

calib_data_path = "/tmp/ct5/export/tag_calibration.mat";

%% Define Data Output
data_export_dir = fullfile(data_path, "/export/");
plot_dir = fullfile(data_export_dir, "/plots/");

%% Options
save_plots = true;  % Store the plots as .png
save_ws = true;     % Save workspace variables for GPS coordinates and ENU data

show_frames_in_plot = true;

%% Read Tag Calibration
load(calib_data_path,'tag_calibration_trans','tag_calibration_pose','main_tag_id','G');

%% Read Tag Detection Topic
bag = ros.Bag(char(bagfile_path));
bag.info()

% Read Aruco marker pose detected by camera
[tag_data,tags_detected] = read_aruco_rostopic(bag, marker_cam_topic_name);

% Pose of the Marker frame (t) w.r.t. the camera (c)
cam_tag_pose = extract_tags_to_cells(tag_data);

%% Calculate camera trajectory

% Camera trajectory expressed in the main tag frame
cam_tag_traj = tags_to_trajectory(cam_tag_pose, tag_calibration_trans, main_tag_id, 1.5);

%% Plot tags in main tag world
% Tags (tx) w.r.t. main tag / tag world frame(tw)
figure
hold on
title("Camera position in main tag frame")
p_mt_tx = plot_tag_calib_in_world(G,tag_calibration_pose, main_tag_id);
view(3)

% Camera Pose w.r.t. main tag / tag world frame(tw)
p_tw_c = cam_tag_traj.position;
plot3(p_tw_c(:,1),p_tw_c(:,2),p_tw_c(:,3),'.')
grid on
axis equal
xlabel("X [m]");ylabel("Y [m]");zlabel("Z [m]");

% Plot [x,y,z] frames into the trajectory
if show_frames_in_plot
    for k=1:100:length(p_tw_c)
        t = Quaternion(cam_tag_traj.orientation(k,:)).T;
        t(1:3,4) = [p_tw_c(k,1),p_tw_c(k,2),p_tw_c(k,3)].';
        trplot(t,'rviz');
    end
end

figure
hold on
title("Camera position in main tag frame 2D")
plot(cam_tag_traj.t - cam_tag_traj.t(1), cam_tag_traj.position,'.')
grid on
xlabel("Time [s]");ylabel("Position [m]");

if ~exist(data_export_dir, 'dir')
    fprintf("[Info] " + data_export_dir + " does not exist. Creating folder...\n");
    mkdir(data_export_dir)
end

fprintf("Saving camera trajectory in tag world...\n");
save(data_export_dir + "/tag_trajectory", 'cam_tag_traj');
fprintf("Done\n");
