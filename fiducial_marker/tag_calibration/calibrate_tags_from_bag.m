%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

% This script is intended to read the pose of a vehicle and the pose of
% a virtual marker (Aruco by default) in a MoCap environment. It assumes a calibration
% of the MoCap object and the IMU as well as the calibration between
% camera and IMU. The script determines the accuracy of the marker detection (STD and RMSE).

% Variable names:
% q_A_B rotation of frame B expressed in frame A
% p_A_B_in_X translation from A to B expressed in frame X.
% X is dropped if X = A, thus p_A_B_in_A == p_A_B.
% Example: p_A_B_in_C = R(q_C_A) * p_A_B_in_A

clear all
close all
addpath(genpath('matlab_rosbag-0.5.0-linux64'));

data_path = "/tmp/ct5";
bagfile_path = fullfile(data_path, "out.bag");
marker_cam_topic_name = "/marker_detections";

main_tag_id = 102;

replace_inter_board_transforms = false;

save_plots = false;  % Store the plots as .png
save_ws = false;     % Save workspace variables for GPS coordinates and ENU data

%% Define Data Output
data_export_dir = data_path + "/export/";
plot_dir = data_export_dir + "/plots/";

%% specify whether/how to print figures
savefigimages.doit                = true; % false if you do not want to save figures as images
savefigimages.figOptions.Units    = 'pixels';
savefigimages.figOptions.Position = [0 0 800 380];
savefigimages.figOptions.Color    = 'white';
savefigimages.figOptions.Renderer = 'opengl';
savefigimages.printOptions        = {'-m4','-pdf','-png' '-nocrop'};
savefigimages.Visibility      = 'on';
savefigimages.closeAfterSave      = false; % true to close figure after saving it to image

set(groot, 'DefaultFigureVisible', savefigimages.Visibility);

%% Read topics
bag = ros.Bag(char(bagfile_path));
bag.info()

% Read Aruco marker pose detected by camera
[tag_data,tags_detected] = read_aruco_rostopic(bag, marker_cam_topic_name);

% Crop data
%tag_data = tag_data(1:2000);

% Pose of the Marker frame (t) w.r.t. the camera (c)
cam_tag_pose = extract_tags_to_cells(tag_data);

%% Calibrate tags
% Extract relative tag to tag transformations
fprintf("Extract Tag to Tag Transformations...\n");
tag_poses = get_relativ_tag_pose(cam_tag_pose);

%% Replace board local transformations with known tag design
if replace_inter_board_transforms

    gt_transform_idx = [];
    for b = 2:8
        % Get board layout
        [calibration, board_ids, main_board_id] = gen_tag_board_calib(b);

        % Remove board internal poses
        tag_poses(board_ids(1):board_ids(end),board_ids(1):board_ids(end)) = cell(length(board_ids));

        % Write new transforms in cell structure
        for k = board_ids(1):board_ids(end)
            if isempty(calibration{main_board_id,k})
                tag_poses(main_board_id,k) = {[]};
            else
                tag_poses(main_board_id,k) = {calibration(main_board_id,k)};
            end
        end

        gt_transform_idx = vertcat(gt_transform_idx, [ones(length(board_ids),1)*main_board_id, board_ids(:)]);
    end

end

fprintf("Performing calibration...\n");
%[G,tag_calibration_trans,tag_calibration_pose] = generate_tag_calibration(tag_poses, main_tag_id, gt_transform_idx);

[G,tag_calibration_trans,tag_calibration_pose] = generate_tag_calibration(tag_poses, main_tag_id);

% Plot tags in main tag world
% Tags (tx) w.r.t. main tag (mt)

figure_name = "Tag Calibration";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
title(figure_name)

hold on

[p_mt_tx,~,h] = plot_tag_calib_in_world(G,tag_calibration_pose, main_tag_id);
view(3)

highlight(h,[1:14],'NodeColor','b');
highlight(h,[15:29],'NodeColor','k');
highlight(h,[30:44],'NodeColor','r');
highlight(h,[45:59],'NodeColor','g');
highlight(h,[60:74],'NodeColor','y');
highlight(h,[75:85],'NodeColor','c');
highlight(h,[86:96],'NodeColor','m');
highlight(h,[97:107],'NodeColor','b');
highlight(h,[123],'NodeColor','b');
h.MarkerSize=7;
h.LineWidth=0.1;
zlim([-0.4 1])
axis equal

% Test on error
tmp = [];
[x,y]=size(tag_calibration_pose)
for k = 1:x
    for t = 1:y
        if isempty(tag_calibration_pose{k,t})
            continue
        else
            tmp(end+1,:) = tag_calibration_pose{k,t}.position;
        end
    end
end

% Absolut hight change for all datapoints
max(tmp(:,3))-min(tmp(:,3))


if ~exist(data_export_dir, 'dir')
    fprintf("[Info] " + data_export_dir + " does not exist. Creating folder...\n");
    mkdir(data_export_dir)
end

fprintf("Saving calibration data\n");
save(data_export_dir + "/tag_calibration", 'tag_calibration_pose', 'tag_calibration_trans', 'main_tag_id', 'G');
fprintf("Done\n");

%% Plot Export
if ~exist(plot_dir, 'dir')
    fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
    mkdir(plot_dir)
end
export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
export_name = lower(export_name);

plot_full_path = fullfile(plot_dir,export_name);
export_fig(fh1, plot_full_path, savefigimages.printOptions{:}, '-nocrop');
savefig(plot_full_path + ".fig");
