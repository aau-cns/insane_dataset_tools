%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

%% This script reads magnetometer data from the PX4 and LSM sensor, 
% generates a calibration for the intrinsics and stores a .mat file 
% with the results
%
% Compensation is applied as:
% v_corr_offset = v_mag - center
% v_corr = transform * v_corr_offset
% 
% Sensor data already needs to be exportet using the 
% "extract_data.m" script

clear all
close all

format long

%% Set all data paths
data_path = "/tmp/";

% Path to the exported data using (data_extraction/extract_data.m)
sensor_data_path = fullfile(data_path, "export/matlab_sensor_data.mat");

save_plots = true;  % Store the plots as .png
show_plots = true;  % Show plots during generation
save_ws = true;     % Save workspace variables for GPS coordinates and ENU data

%% Define Data Output
data_export_dir = fullfile(data_path, "/export/");
plot_dir = fullfile(data_export_dir, "/plots/");

%% Read Sensor Data
load(sensor_data_path);

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

%% Calculate PX4 magnetometer intrinsic data
disp("[Info] PX4 Magnetometer");
[px4_mag_elipsoid_trans, px4_mag_center_offset] = mag_intrinsic_cal(px4_mag.cart_vec.', false);
px4_mag.cart_vec_corr = px4_mag_elipsoid_trans * (px4_mag.cart_vec - px4_mag_center_offset);

%% Plot PX4 magnetometer intrinsic data
h_px4_mag = plot_mag_calibration(px4_mag.cart_vec,px4_mag.cart_vec_corr,'PX4 Mag', savefigimages);

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(h_px4_mag.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(h_px4_mag, plot_dir+export_name, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_dir+export_name + ".fig");
end

%% Calculate LSM magnetometer intrinsic data
disp("[Info] LSM Magnetometer");
[lsm_mag_elipsoid_trans, lsm_mag_center_offset] = mag_intrinsic_cal(lsm_mag.cart_vec.', false);
lsm_mag.cart_vec_corr = lsm_mag_elipsoid_trans * (lsm_mag.cart_vec - lsm_mag_center_offset);

%% Plot LSM magnetometer intrinsic data
h_lsm_mag = plot_mag_calibration(lsm_mag.cart_vec,lsm_mag.cart_vec_corr,'LSM Mag', savefigimages);

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(h_lsm_mag.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(h_lsm_mag, plot_dir+export_name, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_dir+export_name + ".fig");
end

%% Safe Matlab Workspace Variables
if save_ws
    if ~exist(data_export_dir, 'dir')
        fprintf("[Info] " + data_export_dir + " does not exist. Creating folder...\n");
        mkdir(data_export_dir)
    end

    fprintf("[Info] " + "Writing Magnetometer Intrinsic Calibration...\n");
    save(data_export_dir + "mag_calib_intrinsic.mat",'px4_mag_elipsoid_trans','px4_mag_center_offset','lsm_mag_elipsoid_trans','lsm_mag_center_offset');
end
