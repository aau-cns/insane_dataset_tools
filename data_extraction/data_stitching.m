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

%% Set all data paths
data_path="/tmp/";
sensor_data_path = fullfile(data_path, "export/matlab_sensor_data.mat");
tag_traj_data_path = fullfile(data_path, "export/tag_trajectory.mat");
gps_ground_truth_path = fullfile(data_path, "export/outdoor_ground_truth_data.mat");

tag_calib_data_path = "/tmp/export/tag_calibration.mat";

sensor_calib_data_path = "/tmp/matlab/sensor_calibration.m";

use_only_valid_gps = false;
save_plots = false;  % Store the plots as .png
save_ws = true;      % Save workspace variables for GPS coordinates and ENU data
save_csv = true;

%% Define Data Output
data_export_dir = fullfile(data_path, "/export");
csv_export_dir = fullfile(data_export_dir, "/csv/ground_truth");
plot_dir = fullfile(data_export_dir, "/plots/alignment/");

%% Read Settings
yml_sett = ReadYaml(fullfile(data_path,  "settings.yaml"));

%% Read Sensor Data
load(sensor_data_path)

%% Read Sensor calibration (run script)
run(sensor_calib_data_path)

%% Read Tag calibration
load(tag_calib_data_path,'G'); % 'tag_calibration_trans','tag_calibration_pose','main_tag_id'

%% Read Tag Trajectory
load(tag_traj_data_path);

%% Read GPS Ground-Truth
gt = load(gps_ground_truth_path);

%% specify whether/how to print figures
savefigimages.doit                = true; % false if you do not want to save figures as images
savefigimages.figOptions.Units    = 'pixels';
savefigimages.figOptions.Position = [0 0 800 450];
savefigimages.figOptions.Color    = 'white';
savefigimages.figOptions.Renderer = 'opengl';
savefigimages.printOptions        = {'-m4','-pdf','-png' '-nocrop'};
savefigimages.Visibility          = 'on'; % 'on'/'off'
savefigimages.closeAfterSave      = false; % true to close figure after saving it to image

set(groot, 'DefaultFigureVisible', savefigimages.Visibility);

%% Map Data
% Get median transform of tag world (tw) (main tag mocap object) w.r.t. mocap world frame (mw)
p_mw_tw = mocap_tag_board.mean.p;
q_mw_tw = mocap_tag_board.mean.q;
R_mw_tw = quat2rotm(q_mw_tw);

% Mission camera pose (cam) w.r.t. the tag world frame (tw)
t_cam = cam_tag_traj.t;
tagbased.p_tw_cam = cam_tag_traj.position.';
tagbased.q_tw_cam = cam_tag_traj.orientation;
for k=1:length(t_cam)
    tagbased.R_tw_cam{k} = quat2rotm(tagbased.q_tw_cam(k,:));
end

t_mocap = mocap_vehicle.t;
t_gt = gt.gpsw_pximu_interp.t_imu;

% Rename Calibrations
% Cam w.r.t. imu
R_pximu_cam = q_pximu_mc.R;
p_pximu_cam = p_pximu_mc;

% MoCap vehicle w.r.t. imu
%p_pximu_mv = p_pximu_mv;
R_pximu_mv = quat2rotm(q_pximu_mv.double);

%% Transform tag based IMU trajectory to MoCap reference frame
% Transform tag based camera position to IMU positions. The result will be
% expressed in the MoCap world frame
% The main tag mocap object (mto) is equal to the main tag position in MoCap
% World (mt)

for k = 1:length(t_cam)
    % Navigation camera pose (cam) w.r.t. the MoCap world frame (mw)
    tagbased.p_mw_cam(:,k) = p_mw_tw + R_mw_tw * tagbased.p_tw_cam(:,k);
    tagbased.R_mw_cam{k} = R_mw_tw * tagbased.R_tw_cam{k};

    % Vehicle IMU (i) w.r.t. the MoCap world frame (mw)
    tagbased.p_mw_pximu(:,k) = tagbased.p_mw_cam(:,k) + tagbased.R_mw_cam{k} * R_pximu_cam.' * (-p_pximu_cam);
    tagbased.R_mw_pximu{k} = tagbased.R_mw_cam{k} * R_pximu_cam.';

    % MoCap Vehicle Object (mv) w.r.t. the MoCap world frame (mw) based on
    % tag detections
    tagbased.p_mw_mv(:,k) =  tagbased.p_mw_pximu(:,k) + tagbased.R_mw_cam{k} * R_pximu_cam.' * p_pximu_mv;
end

%% Show results of Tag and MoCap Trajectory alignment
% Calculate alignment error of tag based imu trajectory and MoCap vehicle
[idx_mw_mv, ids_mw_v] = sync_data_struct(t_cam, t_mocap, 0.005);
e_pximu_mov_in_mv = sqrt(mean((tagbased.p_mw_mv(:,idx_mw_mv).' - mocap_vehicle.position(:,ids_mw_v).').^2));
fprintf("PX4 IMU and MoCap trajectory alignment RMSE [%.4f,%.4f,%.4f] over %.2f seconds\n", e_pximu_mov_in_mv, mocap_vehicle.t(ids_mw_v(end)) -  mocap_vehicle.t(ids_mw_v(1)))

% Show aligned MoCap and Tag Trajectory segments
t_mocap_tag_overlap = [max([min(t_cam), min(t_mocap)]), min([max(t_cam), max(t_mocap)]) ];

overlap_mocap = (t_mocap > t_mocap_tag_overlap(1)) & (t_mocap < t_mocap_tag_overlap(2));
overlap_tag = (t_cam > t_mocap_tag_overlap(1)) & (t_cam < t_mocap_tag_overlap(2));

figure_name = "MoCap Tag Position Overlap";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot(t_mocap(overlap_mocap), mocap_vehicle.position(1,overlap_mocap),'.','MarkerSize',12, 'DisplayName', 'mocap-vehicle_x');
plot(t_mocap(overlap_mocap), mocap_vehicle.position(2,overlap_mocap),'.','MarkerSize',12, 'DisplayName', 'mocap-vehicle_y');
plot(t_mocap(overlap_mocap), mocap_vehicle.position(3,overlap_mocap),'.','MarkerSize',12, 'DisplayName', 'mocap-vehicle_z');
hold on
plot(t_cam(overlap_tag), tagbased.p_mw_mv(1,overlap_tag),'.','MarkerSize',12, 'DisplayName', 'tag-vehicle_x');
plot(t_cam(overlap_tag), tagbased.p_mw_mv(2,overlap_tag),'.','MarkerSize',12, 'DisplayName', 'tag-vehicle_y');
plot(t_cam(overlap_tag), tagbased.p_mw_mv(3,overlap_tag),'.','MarkerSize',12, 'DisplayName', 'tag-vehicle_z');
legend
grid on
xlabel("Time [s]"); ylabel("Position [m]");

%% Align tag world frame with the GPS world frame
% The alignment transformation is T_tw_gw, gps world w.r.t. tag world

% Calculate Tag based IMU pose in Tag Frame
for k=1:length(t_cam)
    % Vehicle IMU (i) w.r.t. the main tag (mto)
    tagbased.p_tw_pximu(:,k) = tagbased.p_tw_cam(:,k) + tagbased.R_tw_cam{k} * R_pximu_cam.' * (-p_pximu_cam);
    tagbased.R_tw_pximu{k} = tagbased.R_tw_cam{k} * R_pximu_cam.';
end

% Crop GPS and Tag based poses
t_start_align = yml_sett.t_gps_align_start;
t_stop_align = yml_sett.t_gps_align_stop;

max_gps_tag_delta_t = 0.008;
[idx_gt, idx_tag] = sync_data_struct(gt.gpsw_pximu_interp.t_imu-0.2, t_cam, max_gps_tag_delta_t);

t_sync = t_cam(idx_tag);
tagbased.p_tw_pximu_sync = tagbased.p_tw_pximu(:,idx_tag);
tagbased.R_tw_pximu_sync = tagbased.R_tw_pximu(idx_tag);

gpsbased.p_gw_pximu = gt.gpsw_pximu_interp.p;
gpsbased.p_gw_pximu_sync = gpsbased.p_gw_pximu(:,idx_gt);
gpsbased.q_gw_pximu = gt.gpsw_pximu_interp.q;
gpsbased.q_gw_pximu_sync = gpsbased.q_gw_pximu(idx_gt,:);

for k=1:length(gpsbased.q_gw_pximu_sync)
    gpsbased.R_gw_pximu_sync{k} = quat2rotm(gpsbased.q_gw_pximu_sync(k,:));
end
for k=1:length(gpsbased.q_gw_pximu)
    gpsbased.R_gw_pximu{k} = quat2rotm(gpsbased.q_gw_pximu(k,:));
end

% Get absolute time
t_start_align = t_start_align + t_sync(1);
t_stop_align = t_stop_align + t_sync(1);

% Crop and overwrite syncs
tagbased.p_tw_pximu_crop = tagbased.p_tw_pximu_sync(:,t_sync > t_start_align & t_sync < t_stop_align);
tagbased.R_tw_pximu_crop = tagbased.R_tw_pximu_sync(t_sync > t_start_align & t_sync < t_stop_align);
gpsbased.p_gw_pximu_crop = gpsbased.p_gw_pximu_sync(:,t_sync > t_start_align & t_sync < t_stop_align);
gpsbased.R_gw_pximu_crop = gpsbased.R_gw_pximu_sync(t_sync > t_start_align & t_sync < t_stop_align);

t_sync_crop = t_sync(t_sync > t_start_align & t_sync < t_stop_align);

va = sgolayfilt(tagbased.p_tw_pximu_crop.',5,101).';

%[p_tw_gw, R_tw_gw] = transform_between_trajectories(p_tw_pximu_sync.', p_gw_pximu_sync.');
[p_tw_gw, R_tw_gw] = transform_between_trajectories(va.', gpsbased.p_gw_pximu_crop.');

gpsbased.p_tw_gt_crop = p_tw_gw + R_tw_gw * gpsbased.p_gw_pximu_crop;
gpsbased.p_tw_gt = p_tw_gw + R_tw_gw * gpsbased.p_gw_pximu;

gpsbased.R_tw_gt_crop = {};
for k=1:length(gpsbased.R_gw_pximu_crop)
    gpsbased.R_tw_gt_crop{k} = R_tw_gw * gpsbased.R_gw_pximu_crop{k};
end
for k=1:length(gpsbased.R_gw_pximu)
    gpsbased.R_tw_gt{k} = R_tw_gw * gpsbased.R_gw_pximu{k};
end

e_gps_tag_in_mt = sqrt(mean((tagbased.p_tw_pximu_crop.' - gpsbased.p_tw_gt_crop.').^2));
fprintf("RTK GPS and Tag based RTK GPS trajectory alignment RMSE [%.4f,%.4f,%.4f] over %.2f seconds\n",...
    e_gps_tag_in_mt, t_sync_crop(end) - t_sync_crop(1))

gpsbased.R_eul_tw_gt_crop = [];
tagbased.R_eul_tw_pximu_crop = [];
for k=1:length(gpsbased.R_tw_gt_crop)
    gpsbased.R_eul_tw_gt_crop(k,:) = rad2deg(rotm2eul(gpsbased.R_tw_gt_crop{k},'zyx'));
    tagbased.R_eul_tw_pximu_crop(k,:) = rad2deg(rotm2eul(tagbased.R_tw_pximu_crop{k} ,'zyx'));
end

% Plot Rotation alignment
figure_name = "GPS Tag Overlay - Rotation";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)

legend
grid on
xlabel("Time [s]"); ylabel("Rotation [deg]");
t_off = t_sync_crop(1);
plot(t_sync_crop-t_off, gpsbased.R_eul_tw_gt_crop(:,1),'.', 'DisplayName', 'gt_x');
plot(t_sync_crop-t_off, gpsbased.R_eul_tw_gt_crop(:,2),'.', 'DisplayName', 'gt_y');
plot(t_sync_crop-t_off, gpsbased.R_eul_tw_gt_crop(:,3),'.', 'DisplayName', 'gt_z');

plot(t_sync_crop-t_off, tagbased.R_eul_tw_pximu_crop(:,1),'.', 'DisplayName', 'tag_x');
plot(t_sync_crop-t_off, tagbased.R_eul_tw_pximu_crop(:,2),'.', 'DisplayName', 'tag_y');
plot(t_sync_crop-t_off, tagbased.R_eul_tw_pximu_crop(:,3),'.', 'DisplayName', 'tag_z');

% Plot Position Alignment
figure_name = "GPS Tag Overlap";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
legend
grid on
xlabel("Time [s]"); ylabel("Position [m]");
t_off = t_sync_crop(1);
plot(t_sync_crop-t_off, gpsbased.p_tw_gt_crop(1,:),'.', 'DisplayName', 'gt-based-imu_x');
plot(t_sync_crop-t_off, gpsbased.p_tw_gt_crop(2,:),'.', 'DisplayName', 'gt-based-imu_y');
plot(t_sync_crop-t_off, gpsbased.p_tw_gt_crop(3,:),'.', 'DisplayName', 'gt-based-imu_z');

plot(t_sync_crop-t_off, tagbased.p_tw_pximu_crop(1,:),'.', 'DisplayName', 'Tag-based-imu_x');
plot(t_sync_crop-t_off, tagbased.p_tw_pximu_crop(2,:),'.', 'DisplayName', 'Tag-based-imu_y');
plot(t_sync_crop-t_off, tagbased.p_tw_pximu_crop(3,:),'.', 'DisplayName', 'Tag-based-imu_z');

plot(t_sync_crop-t_off, va(1,:), 'DisplayName', 'Tag-based-imu-filt_x');
plot(t_sync_crop-t_off, va(2,:), 'DisplayName', 'Tag-based-imu-filt_y');
plot(t_sync_crop-t_off, va(3,:), 'DisplayName', 'Tag-based-imu-filt_z');


%% Transform all data in a common frame and plot it
% The common frame is the MoCap World frame

% Vehicle IMU w.r.t. MoCap world Frame
mocapbased.p_mw_pximu = mocap_vehicle.position + R_pximu_mv.' * (-p_pximu_mv);

for k = 1:length(t_mocap)
    mocapbased.R_mw_pximu{k} = quat2rotm(mocap_vehicle.orientation(:,k).') * R_pximu_mv.';
end

% GT data w.r.t. MoCap world Frame
for k = 1:length(gpsbased.R_tw_gt)
    gpsbased.R_mw_gt{k} = R_mw_tw * gpsbased.R_tw_gt{k};
    gpsbased.R_eul_mw_gt(k,:) = rad2deg(rotm2eul(gpsbased.R_mw_gt{k},'xyz'));
end

gpsbased.p_mw_gt = p_mw_tw + R_mw_tw * gpsbased.p_tw_gt;


%% Plot MoCap, Tag Trajectory and GPS
figure_name = "MoCap, Tag Trajectory and GPS";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)

t_plot_off = t_gt(1);
marker_size=13;

plot(t_gt - t_plot_off, gpsbased.p_mw_gt(1,:),'.','MarkerSize',marker_size, 'DisplayName', 'gt-vehicle_x');
plot(t_gt - t_plot_off, gpsbased.p_mw_gt(2,:),'.','MarkerSize',marker_size, 'DisplayName', 'gt-vehicle_y');
plot(t_gt - t_plot_off, gpsbased.p_mw_gt(3,:),'.','MarkerSize',marker_size, 'DisplayName', 'gt-vehicle_z');

plot(t_cam - t_plot_off, tagbased.p_mw_pximu(1,:),'.','MarkerSize',marker_size, 'DisplayName', 'tag-vehicle_x');
plot(t_cam - t_plot_off, tagbased.p_mw_pximu(2,:),'.','MarkerSize',marker_size, 'DisplayName', 'tag-vehicle_y');
plot(t_cam - t_plot_off, tagbased.p_mw_pximu(3,:),'.','MarkerSize',marker_size, 'DisplayName', 'tag-vehicle_z');

plot(t_mocap - t_plot_off, mocapbased.p_mw_pximu(1,:),'.','MarkerSize',marker_size, 'DisplayName', 'mocap-vehicle_x');
plot(t_mocap - t_plot_off, mocapbased.p_mw_pximu(2,:),'.','MarkerSize',marker_size, 'DisplayName', 'mocap-vehicle_y');
plot(t_mocap - t_plot_off, mocapbased.p_mw_pximu(3,:),'.','MarkerSize',marker_size, 'DisplayName', 'mocap-vehicle_z');

legend
grid on
xlabel("Time [s]"); ylabel("Position [m]");

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(fh1, plot_dir+export_name, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_dir+export_name + ".fig");
end


figure_name = "MoCap, Tag Trajectory and GPS 3D";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)

plot3(mocapbased.p_mw_pximu(1,:),mocapbased.p_mw_pximu(2,:),mocapbased.p_mw_pximu(3,:),'.', 'DisplayName', 'mocap-vehicle');
plot3(tagbased.p_mw_pximu(1,:),tagbased.p_mw_pximu(2,:),tagbased.p_mw_pximu(3,:),'.', 'DisplayName', 'tag-vehicle');
plot3(gpsbased.p_mw_gt(1,:),gpsbased.p_mw_gt(2,:),gpsbased.p_mw_gt(3,:),'.', 'DisplayName', 'gps-vehicle');
legend
grid on
axis equal
view(3)

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(fh1, plot_dir+export_name, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_dir+export_name + ".fig");
end

%% Hard cut and merge data
t_cut_gps_tag_rel = yml_sett.crop_gps_tag;
t_cut_tag_mocap_rel = yml_sett.crop_tag_mocap;
t_cut_mocap_end_rel = yml_sett.crop_mocap_end;
t_cut_gps_tag = t_plot_off + t_cut_gps_tag_rel;
t_cut_tag_mocap = t_plot_off + t_cut_tag_mocap_rel;
t_cut_mocap_end = t_plot_off + t_cut_mocap_end_rel;

idx_gps_cut = t_gt < t_cut_gps_tag;
idx_tag_cut = (t_cam > t_cut_gps_tag) & (t_cam < t_cut_tag_mocap);
idx_mocap_cut = (t_mocap > t_cut_tag_mocap) & (t_mocap < t_cut_mocap_end);

pose_in_mocap.t = [t_gt(idx_gps_cut).', t_cam(idx_tag_cut), t_mocap(idx_mocap_cut)];
pose_in_mocap.p = [gpsbased.p_mw_gt(:,idx_gps_cut), tagbased.p_mw_pximu(:,idx_tag_cut), mocapbased.p_mw_pximu(:,idx_mocap_cut)];
pose_in_mocap.R = [gpsbased.R_mw_gt(idx_gps_cut.'), tagbased.R_mw_pximu(idx_tag_cut), mocapbased.R_mw_pximu(idx_mocap_cut)];


for k=1:length(t_cam)
    tagbased.R_eul_mw_pximu(k,:) = rad2deg(rotm2eul(tagbased.R_mw_pximu{k},'xyz'));
end
for k=1:length(t_mocap)
    mocapbased.R_eul_mw_pximu(k,:) = rad2deg(rotm2eul(mocapbased.R_mw_pximu{k},'xyz'));
end

figure_name = "Ground Truth Overlaping Orientations";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot(t_gt-t_plot_off, gpsbased.R_eul_mw_gt)
plot(t_cam-t_plot_off, tagbased.R_eul_mw_pximu)
plot(t_mocap-t_plot_off, mocapbased.R_eul_mw_pximu)
xline(t_cut_gps_tag_rel)
xline(t_cut_tag_mocap_rel)
grid on

for k=1:length(pose_in_mocap.R)
    pose_in_mocap.q(k,:) = rotm2quat(pose_in_mocap.R{k});
    pose_in_mocap.R_eul(:,k) = rad2deg(rotm2eul(pose_in_mocap.R{k},'xyz'));
end

figure_name = "Ground Truth Stitched in MoCap Rotation";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot(pose_in_mocap.t-t_plot_off, pose_in_mocap.R_eul(1,:),'.')
plot(pose_in_mocap.t-t_plot_off, pose_in_mocap.R_eul(2,:),'.')
plot(pose_in_mocap.t-t_plot_off, pose_in_mocap.R_eul(3,:),'.')
xline(t_cut_gps_tag_rel)
xline(t_cut_tag_mocap_rel)
grid on

figure_name = "Ground Truth Stitched in MoCap - 2D";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot(pose_in_mocap.t-t_plot_off, pose_in_mocap.p,'.')
xline(t_cut_gps_tag_rel)
xline(t_cut_tag_mocap_rel)
grid on

figure_name = "Ground Truth Stitched in MoCap - 3D";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot3(pose_in_mocap.p(1,:),pose_in_mocap.p(2,:),pose_in_mocap.p(3,:),'.', 'DisplayName', 'mocap-vehicle');
grid on
axis equal
view(3)
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

%% Transform
% Calibration MoCap World w.r.t. GPS world
R_gw_mw = R_tw_gw.' * R_mw_tw.';

P_gw_mw = R_tw_gw.' * (-p_tw_gw) + R_gw_mw * (-p_mw_tw);

pose_in_gps.t = pose_in_mocap.t;
pose_in_gps.p = P_gw_mw + (R_gw_mw * pose_in_mocap.p);

for k=1:length(pose_in_mocap.R)
    pose_in_gps.q(k,:) = rotm2quat(R_gw_mw * pose_in_mocap.R{k});
    pose_in_gps.R{k} = quat2rotm(pose_in_gps.q(k,:));
    pose_in_gps.R_eul(:,k) = rad2deg(rotm2eul(pose_in_gps.R{k},'xyz'));
end

figure_name = "Ground Truth Stitched in GPS Rotation";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot(pose_in_mocap.t-t_plot_off, pose_in_gps.R_eul(1,:),'.')
plot(pose_in_mocap.t-t_plot_off, pose_in_gps.R_eul(2,:),'.')
plot(pose_in_mocap.t-t_plot_off, pose_in_gps.R_eul(3,:),'.')
xline(t_cut_gps_tag_rel)
xline(t_cut_tag_mocap_rel)
grid on
legend

figure_name = "Ground Truth Stitched in GPS Frame - 3D";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot(pose_in_mocap.t-t_plot_off, pose_in_gps.p,'.')
grid on

figure_name = "Ground Truth Stitched in GPS Frame - 3D";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot3(pose_in_gps.p(1,:),pose_in_gps.p(2,:),pose_in_gps.p(3,:),'.');
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");
legend
grid on
axis equal
view(3)

%% Write Data
if save_csv

    fprintf("Saving CSV data...\n");

    if ~exist(csv_export_dir, 'dir')
        fprintf("[Info] " + csv_export_dir + " does not exist. Creating folder...\n");
        mkdir(csv_export_dir)
    end

    fprintf("[Info] " + "Writing CSV files and README...\n");

    filename_gt_in_mocap = "transition_gt_in_moap.csv";
    filename_gt_in_gps = "transition_gt_in_gps.csv";
    filename_gt_in_tag = "transition_gt_in_tag.csv";

    csv_t = pose_in_gps.t;
    csv_p = pose_in_gps.p;
    csv_q = pose_in_gps.q;

    %% Write Timeinfo YAML
    fprintf("[Info] " + "Writing time information yaml file...\n");
    time_filename = fullfile(data_export_dir,"time_info_aligned.yaml");
    time_fid = fopen(time_filename, 'wt' );
    fprintf( time_fid, "# Time Info\n");
    fprintf( time_fid, "t_total: %.3f # in seconds\n", csv_t(end) - csv_t(1));
    fprintf( time_fid, "t_begin: %.23f # in seconds\n", csv_t(1));
    fprintf( time_fid, "t_end: %.23f # in seconds\n\n", csv_t(end));
    fclose(time_fid);

    %% Write CSV
    filename = filename_gt_in_gps;
    file_path = fullfile(csv_export_dir, filename);
    fprintf("Writing %s\n", filename);

    header = "t, p_x, p_y, p_z, q_w, q_x, q_y, q_z"; % csv header
    csv_fid = fopen(file_path, 'w+');
    fprintf(csv_fid,'%s\n', header);
    fclose(csv_fid);
    csv_outdoor_gt = [csv_t.', csv_p.', csv_q];
    dlmwrite(file_path, csv_outdoor_gt, '-append', 'delimiter', ',', 'precision', 17);

    csv_t = pose_in_mocap.t;
    csv_p = pose_in_mocap.p;
    csv_q = pose_in_mocap.q;

    filename = filename_gt_in_mocap;
    file_path = fullfile(csv_export_dir, filename);
    fprintf("Writing %s\n", filename);

    header = "t, p_x, p_y, p_z, q_w, q_x, q_y, q_z"; % csv header
    csv_fid = fopen(file_path, 'w+');
    fprintf(csv_fid,'%s\n', header);
    fclose(csv_fid);
    csv_outdoor_gt = [csv_t.', csv_p.', csv_q];
    dlmwrite(file_path, csv_outdoor_gt, '-append', 'delimiter', ',', 'precision', 17);
end

if save_ws
    fprintf("[Info] Write Matlab Workspace to File\n");
    gt_pose_in_gps.t = pose_in_gps.t;
    gt_pose_in_gps.p = pose_in_gps.p;
    gt_pose_in_gps.q = pose_in_gps.q;
    gt_pose_in_mocap.t = pose_in_mocap.t;
    gt_pose_in_mocap.p = pose_in_mocap.p;
    gt_pose_in_mocap.q = pose_in_mocap.q;
    save(fullfile(data_export_dir, "transition_gt.mat"),'gt_pose_in_mocap','gt_pose_in_gps');
end
