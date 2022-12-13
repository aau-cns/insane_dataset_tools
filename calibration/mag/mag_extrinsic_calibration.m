%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

%% Perform extrinsic calibration to find the rotation between the IMU and magnetic sensor
% It reads all folders that have the defined folder_prefix and searches for
% exported data (matlab workspaces). The script will then open a plot for
% each data sequence one after the other. In this plot, you can select the
% range of valid data by directly clicking into the plot. Click once at the
% beginning and once at the end of the graph you want to use for the
% calibration routine. Then close the window and perform the same in the
% next plot, till all data sequences have been processed.
%
% Short:
% 1. Run the script on a sequence of exported data (set data_path=/xyz/)
% 2. Plots will open one after the other
% 3. Select valid/stable graph segments
%    - (click once at the beginning and once at the end)
% 4. Close the window and do the same for all following plot windows
% 5. After all data sequences have been selected, the calibration will be
%    performed
% 6. Results will be stored for further steps

clear all
close all

%% Set all data paths
data_path = "/tmp/";

mag_cal_folder_prefix = "six_rot*";
mag_matlab_ws_token = "matlab_sensor_data.mat";

save_plots = true;  % Store the plots as .png
show_plots = true;  % Show plots during generation
save_ws = true;     % Save workspace variables for GPS coordinates and ENU data

matlab_ws_exp_filename = "mag_calib_extrinsic.mat";

use_mag_intrinsic_cal = true;
mag_intrinsic_cal = "/tmp/export/mag_calib_intrinsic.mat";

do_lsm_calibration = true;

%% Define Data Output
data_export_dir = fullfile(data_path, "/export/mag_calib");
plot_dir = fullfile(data_export_dir, "/plots/");

%% specify whether/how to print figures
savefigimages.doit                = true; % false if you do not want to save figures as images
savefigimages.figOptions.Units    = 'pixels';
savefigimages.figOptions.Position = [0 0 800 450];
savefigimages.figOptions.Color    = 'white';
savefigimages.figOptions.Renderer = 'opengl';
savefigimages.printOptions        = {'-m4','-pdf','-png' '-nocrop'};

if show_plots
    savefigimages.Visibility      = 'on';
else
    savefigimages.Visibility      = 'off';
end

savefigimages.closeAfterSave      = false; % true to close figure after saving it to image
set(groot, 'DefaultFigureVisible', savefigimages.Visibility);

%% Read Sensor Data

% Find all matlab workspace files that correspond to the
% 'mag_cal_folder_prefix' and 'mag_matlab_ws_token'
mag_ws_files = dir(fullfile(data_path,mag_cal_folder_prefix,"**/",mag_matlab_ws_token));
fprintf("[Info] Found %i workspace files\n", length(mag_ws_files) )

mag_sensor_data = cell(0);
for k=1:length(mag_ws_files)
    mag_sensor_data_path = string(fullfile(mag_ws_files(k).folder, mag_ws_files(k).name));
    mag_sensor_data{k} = load(mag_sensor_data_path);
end

%% Show plots and select range

% Select ranges
for k= 1:length(mag_sensor_data)
    raw_px4mag_px4imu{k}.imu = mag_sensor_data{k}.px4_imu;
    raw_px4mag_px4imu{k}.mag = mag_sensor_data{k}.px4_mag;

    raw_lsmmag_px4imu{k}.imu = mag_sensor_data{k}.px4_imu;
    raw_lsmmag_px4imu{k}.mag = mag_sensor_data{k}.lsm_mag;

    raw_lsmmag_lsmimu{k}.imu = mag_sensor_data{k}.lsm_imu;
    raw_lsmmag_lsmimu{k}.mag = mag_sensor_data{k}.lsm_mag;

    raw_px4mag_lsmimu{k}.imu = mag_sensor_data{k}.lsm_imu;
    raw_px4mag_lsmimu{k}.mag = mag_sensor_data{k}.px4_mag;
end

[sel_px4mag_px4imu,~] = select_and_concatinate_data(raw_px4mag_px4imu);
fh1 = plot_mag_imu(sel_px4mag_px4imu, savefigimages);
[sel_lsmmag_px4imu,~] = select_and_concatinate_data(raw_lsmmag_px4imu);
fh2 = plot_mag_imu(sel_lsmmag_px4imu, savefigimages);
[sel_lsmmag_lsmimu,~] = select_and_concatinate_data(raw_lsmmag_lsmimu);
fh3 = plot_mag_imu(sel_lsmmag_lsmimu, savefigimages);
[sel_px4mag_lsmimu,~] = select_and_concatinate_data(raw_px4mag_lsmimu);
fh3 = plot_mag_imu(sel_px4mag_lsmimu, savefigimages);

%% Apply magnetometer intrinsic calibration
if use_mag_intrinsic_cal
    intrinsic = load(mag_intrinsic_cal);

    % v_corr_offset = v_mag - center
    % v_corr = transform * v_corr_offset
    sel_px4mag_px4imu.mag.cart_vec = intrinsic.px4_mag_elipsoid_trans * (sel_px4mag_px4imu.mag.cart_vec - intrinsic.px4_mag_center_offset);
    sel_px4mag_lsmimu.mag.cart_vec = intrinsic.px4_mag_elipsoid_trans * (sel_px4mag_lsmimu.mag.cart_vec - intrinsic.px4_mag_center_offset);

    sel_lsmmag_px4imu.mag.cart_vec = intrinsic.lsm_mag_elipsoid_trans * (sel_lsmmag_px4imu.mag.cart_vec - intrinsic.lsm_mag_center_offset);
    sel_lsmmag_lsmimu.mag.cart_vec = intrinsic.lsm_mag_elipsoid_trans * (sel_lsmmag_lsmimu.mag.cart_vec - intrinsic.lsm_mag_center_offset);
end

%% Calculate extrinsics

% Calibrate Mag to IMU
[R_px4mag_px4imu, mag_incl_px4mag_px4imu] = mag_imu_calibration(sel_px4mag_px4imu.mag.cart_vec, sel_px4mag_px4imu.imu.linear_acceleration);
fprintf("[Info] R_px4mag_px4imu (r,p,y) [deg] [%.2f, %.2f, %.2f]\n",rotm2eul(R_px4mag_px4imu,'xyz') *180/pi);
fprintf("[Info] Mag inclination [deg] %.2f\n",mag_incl_px4mag_px4imu * 180/pi);

[R_lsmmag_px4imu, mag_incl_lsmmag_px4imu] = mag_imu_calibration(sel_lsmmag_px4imu.mag.cart_vec, sel_lsmmag_px4imu.imu.linear_acceleration);
fprintf("[Info] R_lsmmag_px4imu (r,p,y) [deg] [%.2f, %.2f, %.2f]\n",rotm2eul(R_lsmmag_px4imu,'xyz') *180/pi);
fprintf("[Info] Mag inclination [deg] %.2f\n",mag_incl_lsmmag_px4imu * 180/pi);

[R_lsmmag_lsmimu, mag_incl_lsmmag_lsmimu] = mag_imu_calibration(sel_lsmmag_lsmimu.mag.cart_vec, sel_lsmmag_lsmimu.imu.linear_acceleration);
fprintf("[Info] R_lsmmag_lsmimu (r,p,y) [deg] [%.2f, %.2f, %.2f]\n",rotm2eul(R_lsmmag_lsmimu,'xyz') *180/pi);
fprintf("[Info] Mag inclination [deg] %.2f\n",mag_incl_lsmmag_lsmimu * 180/pi);

[R_px4mag_lsmimu, mag_incl_px4mag_lsmimu] = mag_imu_calibration(sel_px4mag_lsmimu.mag.cart_vec, sel_px4mag_lsmimu.imu.linear_acceleration);
fprintf("[Info] R_px4mag_lsmimu (r,p,y) [deg] [%.2f, %.2f, %.2f]\n",rotm2eul(R_px4mag_lsmimu,'xyz') *180/pi);
fprintf("[Info] Mag inclination [deg] %.2f\n",mag_incl_px4mag_lsmimu * 180/pi);

fprintf("Saving data range selection...\n");

if ~exist(data_export_dir, 'dir')
    fprintf("[Info] " + data_export_dir + " does not exist. Creating folder...\n");
    mkdir(data_export_dir)
end

if save_ws
    fprintf("[Info] Write Matlab Workspace to File\n");
    save(fullfile(data_export_dir, matlab_ws_exp_filename),...
        "sel_px4mag_px4imu","R_px4mag_px4imu","mag_incl_px4mag_px4imu", ...
        "sel_lsmmag_px4imu","R_lsmmag_px4imu","mag_incl_lsmmag_px4imu", ...
        "sel_lsmmag_lsmimu","R_lsmmag_lsmimu","mag_incl_lsmmag_lsmimu", ...
        "sel_px4mag_lsmimu","R_px4mag_lsmimu","mag_incl_px4mag_lsmimu");
end


function [mag_imu_data_out, selected_range] = select_and_concatinate_data(mag_imu_data_in)

mag_imu_data_out.mag.t = [];
mag_imu_data_out.mag.cart_vec = [];
mag_imu_data_out.imu.t = [];
mag_imu_data_out.imu.linear_acceleration = [];

for k= 1:length(mag_imu_data_in)

    ws_data = mag_imu_data_in{k};

    % Synch mag and imu data
    [idx_mag,idx_imu] = sync_data_struct(ws_data.mag.t,  ws_data.imu.t, 0.002);

    mag.t = ws_data.mag.t(idx_mag);
    mag.cart_vec = ws_data.mag.cart_vec(:,idx_mag);

    imu.t = ws_data.imu.t(idx_imu);
    imu.linear_acceleration = ws_data.imu.linear_acceleration(:,idx_imu);

    h = figure;
    hold on;
    subplot(2,1,1)
    hold on
    title("IMU Linear Acceleration")
    plot(1:length(imu.t), imu.linear_acceleration(1,:),'.','DisplayName','acc_x');
    plot(1:length(imu.t), imu.linear_acceleration(2,:),'.','DisplayName','acc_y');
    plot(1:length(imu.t), imu.linear_acceleration(3,:),'.','DisplayName','acc_z');
    ylabel("[m/s^2]");xlabel("Data Index");
    grid on
    legend

    subplot(2,1,2)
    title("Magnetometer Cart. Vector")
    hold on
    plot(1:length(mag.t), mag.cart_vec(1,:),'.','DisplayName','mag_x');
    plot(1:length(mag.t), mag.cart_vec(2,:),'.','DisplayName','mag_y');
    plot(1:length(mag.t), mag.cart_vec(3,:),'.','DisplayName','mag_z');
    ylabel("[Tesla]");xlabel("Data Index");
    grid on
    legend

    plot_interact = PointsFromPlot(h);
    selected_range(k,:) = ceil(plot_interact.points(1,:));

    idx_lower = min(selected_range(k,:));
    idx_upper = max(selected_range(k,:));

    mag_imu_data_out.mag.t = [mag_imu_data_out.mag.t mag.t(idx_lower:idx_upper)];
    mag_imu_data_out.mag.cart_vec = [mag_imu_data_out.mag.cart_vec mag.cart_vec(:,idx_lower:idx_upper)];

    mag_imu_data_out.imu.t = [mag_imu_data_out.imu.t imu.t(idx_lower:idx_upper)];
    mag_imu_data_out.imu.linear_acceleration = [mag_imu_data_out.imu.linear_acceleration imu.linear_acceleration(:,idx_lower:idx_upper)];

end
end

function [fh1] = plot_mag_imu(mag_imu_data,savefigimages)
figure_name = "Selected Mag IMU Data";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)

subplot(2,1,1)
hold on
title("IMU Linear Acceleration")
plot(1:size(mag_imu_data.imu.linear_acceleration,2), mag_imu_data.imu.linear_acceleration(1,:),'.','DisplayName','acc_x');
plot(1:size(mag_imu_data.imu.linear_acceleration,2), mag_imu_data.imu.linear_acceleration(2,:),'.','DisplayName','acc_y');
plot(1:size(mag_imu_data.imu.linear_acceleration,2), mag_imu_data.imu.linear_acceleration(3,:),'.','DisplayName','acc_z');
ylabel("[m/s^2]");xlabel("Data Index");
grid on
legend

subplot(2,1,2)
title("Magnetometer Cart. Vector")
hold on
plot(1:size(mag_imu_data.mag.cart_vec,2), mag_imu_data.mag.cart_vec(1,:),'.','DisplayName','mag_x');
plot(1:size(mag_imu_data.mag.cart_vec,2), mag_imu_data.mag.cart_vec(2,:),'.','DisplayName','mag_y');
plot(1:size(mag_imu_data.mag.cart_vec,2), mag_imu_data.mag.cart_vec(3,:),'.','DisplayName','mag_z');
ylabel("[Tesla]");xlabel("Data Index");
grid on
legend

end
