%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

%% This script reads the magnetometer extrinsics and intrinsics and writes it to a YAML file

clear all
close all

%% Set all data paths
data_path = "/tmp/export/";

px4_folder = "mag_calib_px4";
lsm_folder = "mag_calib_lsm_mag_to_px_imu";

px4_yaml_file_name = "px4_mag.yaml";
lsm_yaml_file_name = "lsm_mag.yaml";

save_plots = true;  % Store the plots as .png
show_plots = true;  % Show plots during generation
save_ws = true;     % Save workspace variables for GPS coordinates and ENU data

gps_coord.lat = 46.61355929;
gps_coord.long = 14.26238830;
gps_coord.alt = 489.4940;

matlab_intrinsic_file = "mag_calib_intrinsic.mat";
matlab_extrinsic_file = "mag_calib_extrinsic.mat";

data_export_dir = fullfile(data_path, "/");

%% Load calibration data
px4_intr = load( fullfile(data_path, px4_folder, matlab_intrinsic_file));
px4_extr = load( fullfile(data_path, px4_folder, matlab_extrinsic_file));

lsm_intr = load( fullfile(data_path, lsm_folder, matlab_intrinsic_file));
lsm_extr = load( fullfile(data_path, lsm_folder, matlab_extrinsic_file));

%% Write YAML file

if ~exist(data_export_dir, 'dir')
    fprintf("[Info] " + data_export_dir + " does not exist. Creating folder...\n");
    mkdir(data_export_dir)
end

fprintf("[Info] Write Magnetometer Calibration to YAML File\n");

csv_fid = fopen(fullfile(data_export_dir, px4_yaml_file_name), 'w+');
mag_cal_yaml_format(csv_fid, "PX4", "pxmag", px4_intr.px4_mag_elipsoid_trans, px4_intr.px4_mag_center_offset, px4_extr.R_mag_imu, px4_extr.mag_incl, gps_coord )
fclose(csv_fid);

csv_fid = fopen(fullfile(data_export_dir, lsm_yaml_file_name), 'w+');
mag_cal_yaml_format(csv_fid, "LSM", "lsmmag", lsm_intr.lsm_mag_elipsoid_trans, lsm_intr.lsm_mag_center_offset, lsm_extr.R_mag_imu, lsm_extr.mag_incl, gps_coord )
fclose(csv_fid);
