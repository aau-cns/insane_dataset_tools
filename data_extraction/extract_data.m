%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

%% The script is divided in three sections:
% - Read messages from bagfiles given the folder structure and file tokens
% - Plot all data for visual evaluation and store the plots
% - Export sensor data to csv files and write a README with sensor stats
% - Export sensor data to matlab file

clear all
close all
addpath(genpath('matlab_rosbag-0.5.0-linux64'));

data_path = "/tmp/";

%% Read Settings
yml_sett = ReadYaml(fullfile(data_path,  "settings.yaml"));

%% Options
insane_file_structure = true; % Input files follow data set structure or are called xyz.bag
save_plots = false;  % Store the plots as .png
show_plots = false;  % Show plots during generation
save_csv = true;     % Export CSV files for the GPS coordinates and ENU data
save_ws = true;      % Save workspace variables for GPS coordinates and ENU data
set_t_zero = false;  % Common timestamp, begining at t=0s
additional_gps_plots = false; % Plot additional 3D GPS data plots
verbose = false;

fix_lsm_imu_scaling = yml_sett.fix_lsm_imu_scaling; % Fix the wring LSMDS1 IMU scaling dut to wrong documentation (Resolution of 16g do have a resolution of 24g)

gps_ref_location = string(yml_sett.gps_ref_location); % "negev","klu" or "klu_ma"
show_building_entry = yml_sett.show_building_entry;

% Choose UWB Modules
uwb_module_type = 1; % 1: Decawave, 2: Neuhold
rs_imu_combined = 1; % 0: IMU seperate, 1: IMU combined

do_crop = yml_sett.do_crop;
t_crop_begin = yml_sett.crop_begin;
t_crop_end = yml_sett.crop_end;

if isfield(yml_sett, 't_rpm_offset')
    t_rpm_offset = yml_sett.t_rpm_offset;
else
    t_rpm_offset = 0;
end

%% Define Data Input (file tree)
ids_folder = "/ids/";
sensor_folder = "/sensors/";
realsense_folder = "/rs/";

mocap_vehicle_topic = '/twins_three/vrpn_client/raw_transform';

% Define file tokens
if insane_file_structure
    mod1_sensor_token = "*mod1*";
    mod2_sensor_token = "*mod2*";
    mocap_sensor_token = "*_mocap_*";
    fiducial_sensor_token = "*_fiducial_*";
    rs_img_token = "*rs_img*";
    ids_img = "*ids*";
else
    mod1_sensor_token = "*.bag";
    mod2_sensor_token = "*.bag";
    mocap_sensor_token = "*.bag";
    fiducial_sensor_token = "*.bag";
    rs_img_token = "*.bag";
    ids_img = "*.bag";
end

%% Define Data Output
data_export_dir = fullfile(data_path, "/export/");
plot_dir = fullfile(data_export_dir, "/plots/");
csv_export_dir = fullfile(data_export_dir, "/csv/");

% Additional Dataset Information
uwb_main_id = 268444576; % only for Neuhold UWB

all_gps_t_crop = 160;

if gps_ref_location == "negev"
    % GPS reference coordinates for ENU conversion
    % Israel
    gps_ref_coord.lat = 30.5999294;
    gps_ref_coord.long = 34.867308;
    gps_ref_coord.alt = 526.594;
elseif gps_ref_location == "klu"
    % Klagenfurt
    gps_ref_coord.lat = 46.61355929;
    gps_ref_coord.long = 14.26238830;
    gps_ref_coord.alt = 489.4940;
elseif gps_ref_location == "klu_ma"
    % Klagenfurt Model Airport
    gps_ref_coord.lat = 46.606867399;
    gps_ref_coord.long = 14.279121199;
    gps_ref_coord.alt = 484.017;
else
    % Maunal
    gps_ref_coord.lat = 46.61333069;
    gps_ref_coord.long = 14.26216569;
    gps_ref_coord.alt = 489.474;
end

% Klagenfurt Building entry coordinates
gps_entry_coord.lat = 46.61333069;
gps_entry_coord.long = 14.26216569;
gps_entry_coord.alt = 489.474;


% Get GPS Reference Conversion
[ gps_ref_coord.rotation, gps_ref_coord.position ] = gps_init_ref(gps_ref_coord.lat, gps_ref_coord.long, gps_ref_coord.alt);

% Get Dronehall entry position in ENU
door_position = wgs84ToEnu(gps_entry_coord.lat,gps_entry_coord.long,gps_entry_coord.alt,gps_ref_coord.rotation,gps_ref_coord.position);

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

%% Read Module 1 Topics
mod1_sensor_bagfiles_list = dir(fullfile(data_path+sensor_folder, mod1_sensor_token));

for k=1:length(mod1_sensor_bagfiles_list)
    mod1_sensor_bagfiles(k) = string(fullfile(mod1_sensor_bagfiles_list(k).folder, mod1_sensor_bagfiles_list(k).name));
    
    mod1_sensor_bag(k) = ros.Bag(char(mod1_sensor_bagfiles(k)));
    
    if verbose
        mod1_sensor_bag(k).info()
    end
end

% Read PX4 IMU
[px4_imu.t, px4_imu.linear_acceleration, px4_imu.angular_velocity, ~] = read_imu_sensor_msgs( mod1_sensor_bag, '/mavros/imu/data_raw' );

% Read PX4 Fluidpressure
[px4_baro.t, px4_baro.pressure] = read_fluidpressure_sensor_msgs(mod1_sensor_bag,'/mavros/imu/static_pressure');

% Read PX4 Magnetometer
[px4_mag.t, px4_mag.cart_vec] = read_magneticfield_msgs(mod1_sensor_bag,'/mavros/imu/mag');

if ~isempty(px4_mag.t)
    [px4_mag.az,px4_mag.el,px4_mag.norm] = cart2sph(px4_mag.cart_vec(1,:),px4_mag.cart_vec(2,:),px4_mag.cart_vec(3,:));
    
    px4_mag.el = rad2deg(px4_mag.el);
    px4_mag.az = rad2deg(px4_mag.az);
end

% Read PX4 Motor Speeds
[px4_motor_speed.t, px4_motor_speed.rpm] = read_motor_speed_msgs(mod1_sensor_bag,'/mavros/motor_speeds/speed');
px4_motor_speed.t = px4_motor_speed.t + t_rpm_offset;

% Read PX4 GPS
[px4_gps.t, px4_gps.lat, px4_gps.long, px4_gps.alt, px4_gps.enu_cov] = read_gps_navsat(mod1_sensor_bag, '/mavros/global_position/raw/fix');
[px4_gps_vel.t, px4_gps_vel.velocity,~] = read_geometry_twist_msgs(mod1_sensor_bag, '/mavros/global_position/raw/gps_vel');

% Convert from GPS coordinates to local ENU
for i = 1:length(px4_gps.t)
    px4_gps.enu(:,i) = wgs84ToEnu(px4_gps.lat(i),px4_gps.long(i),px4_gps.alt(i),gps_ref_coord.rotation,gps_ref_coord.position);
end

%% Read RTK GPS1
[rtk_gps1_fix_raw.t, rtk_gps1_fix_raw.lat, rtk_gps1_fix_raw.long, rtk_gps1_fix_raw.alt, rtk_gps1_fix_raw.enu_cov] = read_gps_navsat(mod1_sensor_bag, '/rtk_gps_1/fix');
[rtk_gps1_vel_raw.t, rtk_gps1_vel_raw.velocity,~] = read_geometry_twist_w_cov_msgs(mod1_sensor_bag, '/rtk_gps_1/fix_velocity');

% Read RTK GPS1 flags and determine when RTK had FIXED accuracy
[rtk_gps1_pvt_raw.gps_time, rtk_gps1_pvt_raw.gps_fix_type, rtk_gps1_pvt_raw.rtk_status, rtk_gps1_pvt_raw.lat, rtk_gps1_pvt_raw.long, rtk_gps1_pvt_raw.alt] = read_ublox_navpvt(mod1_sensor_bag, '/rtk_gps_1/navpvt');

rtk_gps1 = align_gps_topics(rtk_gps1_fix_raw, rtk_gps1_vel_raw, rtk_gps1_pvt_raw);

% Convert from GPS coordinates to local ENU
for i = 1:length(rtk_gps1.t)
    rtk_gps1.enu(:,i) = wgs84ToEnu(rtk_gps1.lat(i), rtk_gps1.long(i), rtk_gps1.alt(i), gps_ref_coord.rotation, gps_ref_coord.position);
end

%% Read RTK GPS2
[rtk_gps2_fix_raw.t, rtk_gps2_fix_raw.lat, rtk_gps2_fix_raw.long, rtk_gps2_fix_raw.alt, rtk_gps2_fix_raw.enu_cov] = read_gps_navsat(mod1_sensor_bag, '/rtk_gps_2/fix');
[rtk_gps2_vel_raw.t, rtk_gps2_vel_raw.velocity,~] = read_geometry_twist_w_cov_msgs(mod1_sensor_bag, '/rtk_gps_2/fix_velocity');

% Read RTK GPS2 flags and determine when RTK had FIXED accuracy
[rtk_gps2_pvt_raw.gps_time, rtk_gps2_pvt_raw.gps_fix_type, rtk_gps2_pvt_raw.rtk_status, rtk_gps2_pvt_raw.lat, rtk_gps2_pvt_raw.long, rtk_gps2_pvt_raw.alt] = read_ublox_navpvt(mod1_sensor_bag, '/rtk_gps_2/navpvt');

rtk_gps2 = align_gps_topics(rtk_gps2_fix_raw, rtk_gps2_vel_raw, rtk_gps2_pvt_raw);

% Convert from GPS coordinates to local ENU
for i = 1:length(rtk_gps2.t)
    rtk_gps2.enu(:,i) = wgs84ToEnu(rtk_gps2.lat(i), rtk_gps2.long(i), rtk_gps2.alt(i), gps_ref_coord.rotation, gps_ref_coord.position);
end

%% LRF Range topics
[lrf.t, lrf.range] = read_range_sensor_msgs(mod1_sensor_bag,'/lidar_lite/range');

%% MoCap Topic
% Check if dedicated MoCap file exists, if not, use the module 1 bag file
if ~isempty(dir(data_path+sensor_folder + mocap_sensor_token))
    mocap_sensor_bagfiles_list = dir(fullfile(data_path+sensor_folder, mocap_sensor_token));
    for k=1:length(mocap_sensor_bagfiles_list)
        mocap_sensor_bagfiles(k) = string(fullfile(mocap_sensor_bagfiles_list(k).folder, mocap_sensor_bagfiles_list(k).name));

        mocap_sensor_bag(k) = ros.Bag(char(mocap_sensor_bagfiles(k)));

        if verbose
            mocap_sensor_bag(k).info()
        end
    end
else
    fprintf("[INFO] No dedicated MoCap Bagfile found, assuming its in the Module 1 Bagfile\n")
    mocap_sensor_bag = mod1_sensor_bag;
end

[mocap_vehicle.t, mocap_vehicle.position, mocap_vehicle.orientation] = read_transform_geometry_msgs(mocap_sensor_bag, mocap_vehicle_topic);
[mocap_tag_board.t, mocap_tag_board.position, mocap_tag_board.orientation] = read_transform_geometry_msgs(mocap_sensor_bag, '/tag_board_8/vrpn_client/raw_transform');

% Get median transform of main tag mocap object (mto) w.r.t. mocap world frame (mw)
if ~isempty(mocap_tag_board.t)
    
    for k = 1: length(mocap_tag_board.t)
        T_mw_mtoN{k} = Quaternion(mocap_tag_board.orientation(:,k)).T;
        T_mw_mtoN{k}(1:3, 4) = mocap_tag_board.position(:,k);
    end
    
    T_mw_mto = median_of_transform(T_mw_mtoN);
    
    mocap_tag_board.mean.T = T_mw_mto;
    mocap_tag_board.mean.p = T_mw_mto(1:3,4:4);
    mocap_tag_board.mean.q = Quaternion(T_mw_mto).double;
    
end

%% Fiducial Marker Detection Messages
% Check if dedicated fiducial bag exists, if not use module 1 bag file
if ~isempty(dir(data_path+sensor_folder + fiducial_sensor_token))
    fiducial_marker_bagfiles_list = dir(fullfile(data_path+sensor_folder, fiducial_sensor_token));
    for k=1:length(fiducial_marker_bagfiles_list)
        fiducial_marker_bagfiles(k) = string(fullfile(fiducial_marker_bagfiles_list(k).folder, fiducial_marker_bagfiles_list(k).name));

        fiducial_marker_bag(k) = ros.Bag(char(fiducial_marker_bagfiles(k)));

        if verbose
            fiducial_marker_bag(k).info()
        end
    end
else
    fprintf("[INFO] No dedicated Fiducial Bagfile found, assuming its in the Module 1 Bagfile\n")
    fiducial_marker_bag = mod1_sensor_bag;
end

[fiducial_markers.tag_data, fiducial_markers.tags_detected] = read_fiducial_marker_msgs(fiducial_marker_bag, "/marker_detections");

%% Module 2 topics
mod2_sensor_bagfiles_list = dir(fullfile(data_path+sensor_folder, mod2_sensor_token));

if ~isempty(mod2_sensor_bagfiles_list)
    
    for k=1:length(mod2_sensor_bagfiles_list)
        mod2_sensor_bagfiles(k) = string(fullfile(mod2_sensor_bagfiles_list(k).folder, mod2_sensor_bagfiles_list(k).name));
        
        mod2_sensor_bag(k) = ros.Bag(char(mod2_sensor_bagfiles(k)));
        
        if verbose
            mod2_sensor_bag(k).info()
        end
    end
    
    %% LSM9DS1
    % IMU
    [lsm_imu.t, lsm_imu.linear_acceleration, lsm_imu.angular_velocity, ~] = read_imu_sensor_msgs(mod2_sensor_bag, '/imu_lsm9ds1/imu');
    
    if fix_lsm_imu_scaling
        % Fix incorrect IMU scaling when 16G was selected. The documentation shows
        % 16G but the corresponding resolution suggests that this setting provides
        % 24G scaling
        lsm_a_res_corr = 24/16;
        lsm_imu.linear_acceleration = lsm_imu.linear_acceleration * lsm_a_res_corr;
    end
    
    
    % Mag
    [lsm_mag.t, lsm_mag.cart_vec] = read_magneticfield_msgs(mod2_sensor_bag,'/imu_lsm9ds1/mag');
    
    if ~isempty(lsm_mag.t)
        [lsm_mag.az,lsm_mag.el,lsm_mag.norm] = cart2sph(lsm_mag.cart_vec(1,:),lsm_mag.cart_vec(2,:),lsm_mag.cart_vec(3,:));
        lsm_mag.el = rad2deg(lsm_mag.el);
        lsm_mag.az = rad2deg(lsm_mag.az);
    end
    
    %% RealSense
    if rs_imu_combined
        % Combined IMU measurements
        [rs_imu.t, rs_imu.linear_acceleration, rs_imu.angular_velocity, ~] = read_imu_sensor_msgs(mod2_sensor_bag, '/realsense/imu');
    else
        %Dedicated IMU measurements
        [rs_imu_accel.t, rs_imu_accel.linear_acceleration, ~, ~] = read_imu_sensor_msgs(mod2_sensor_bag, '/realsense/accel/sample');
        [rs_imu_gyro.t, ~, rs_imu_gyro.angular_velocity, ~] = read_imu_sensor_msgs(mod2_sensor_bag, '/realsense/gyro/sample');
    end
    
    % Odometry
    [rs_odom.t, rs_odom.position, rs_odom.orientation] = read_odometry_nav_msgs(mod2_sensor_bag, '/realsense/odom/sample');
    
    %% UWB Modules
    if uwb_module_type == 1% Decawave UWB topics
        [uwb_range.t, uwb_range.dist, uwb_range.valid] = read_decawave_uwb_msgs( mod2_sensor_bag, "/uwb_trek/tagDistance_raw" );
        
    else % Neuhold UWB topics
        [uwb_range.t, uwb_range.id_from, uwb_range.id_to, uwb_range.dist] = read_uwb_bridge_msgs( mod2_sensor_bag, "/uwb_bridge/uwb_meas" );
        
        if ~isempty(uwb_range.t)
            [main_dist, inter_dist] = process_uwb(uwb_range.t, uwb_range.id_from, uwb_range.id_to, uwb_range.dist, uwb_main_id);
        end
        
        % UWB IMU
        [uwb_imu.t, uwb_imu.linear_acceleration, ~, ~] = read_imu_sensor_msgs( mod2_sensor_bag, "/uwb_bridge/imu" );
    end
    
end

%% Time adjustment

all_stamps = [px4_imu.t, px4_mag.t,px4_baro.t,px4_gps.t, px4_gps_vel.t, rtk_gps1.t, rtk_gps2.t,rs_odom.t,lrf.t,mocap_vehicle.t, mocap_tag_board.t,lsm_imu.t,lsm_mag.t, uwb_range.t];
%px4_motor_speed.t,
if rs_imu_combined
    all_stamps = [ all_stamps rs_imu.t];
else
    all_stamps = [ all_stamps rs_imu_accel.t];
    all_stamps = [ all_stamps rs_imu_gyro.t];
end

all_stamps(all_stamps==0) = [];
t_min_ref = min(all_stamps);
t_max_ref = max(all_stamps);

% Crop original data
if do_crop
    %
    t_crop_b_rel = t_min_ref + t_crop_begin;
    if t_crop_end == 0
        t_crop_e_rel  = t_max_ref;
    else
        t_crop_e_rel = t_min_ref + t_crop_end;
    end
    %px4_imu
    t_crop = px4_imu.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    px4_imu = crop_data_struct(px4_imu, idx_b,idx_e);
    %px4_baro
    t_crop = px4_baro.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    px4_baro = crop_data_struct(px4_baro, idx_b,idx_e);
    %px4_mag
    t_crop = px4_mag.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    px4_mag = crop_data_struct(px4_mag, idx_b,idx_e);
    %px4_motor_speed
    t_crop = px4_motor_speed.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    px4_motor_speed = crop_data_struct(px4_motor_speed, idx_b,idx_e);
    %px4_gps
    t_crop = px4_gps.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    px4_gps = crop_data_struct(px4_gps, idx_b,idx_e);
    %px4_gps_vel
    t_crop = px4_gps_vel.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    px4_gps_vel = crop_data_struct(px4_gps_vel, idx_b,idx_e);
    %rtk_gps1
    t_crop = rtk_gps1.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    rtk_gps1 = crop_data_struct(rtk_gps1, idx_b,idx_e);
    %rtk_gps2
    t_crop = rtk_gps2.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    rtk_gps2 = crop_data_struct(rtk_gps2, idx_b,idx_e);
    %lrf
    t_crop = lrf.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    lrf = crop_data_struct(lrf, idx_b,idx_e);
    %mocap_vehicle
    t_crop = mocap_vehicle.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    mocap_vehicle = crop_data_struct(mocap_vehicle, idx_b,idx_e);
    %mocap_tag_board
    t_crop = mocap_tag_board.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    mocap_tag_board.t = mocap_tag_board.t(idx_b:idx_e);
    mocap_tag_board.position = mocap_tag_board.position(:,idx_b:idx_e);
    mocap_tag_board.orientation = mocap_tag_board.orientation(:,idx_b:idx_e);
    %mocap_tag_board = crop_data_struct(mocap_tag_board, idx_b,idx_e);
    %lsm_imu
    t_crop = lsm_imu.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    lsm_imu = crop_data_struct(lsm_imu, idx_b,idx_e);
    %lsm_mag
    t_crop = lsm_mag.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    lsm_mag = crop_data_struct(lsm_mag, idx_b,idx_e);
    %rs_imu
    t_crop = rs_imu.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    rs_imu = crop_data_struct(rs_imu, idx_b,idx_e);
    %rs_odom
    t_crop = rs_odom.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    rs_odom = crop_data_struct(rs_odom, idx_b,idx_e);
    %uwb_range
    t_crop = uwb_range.t;
    [~, idx_b] = min( abs( t_crop-t_crop_b_rel ) );
    [~, idx_e] = min( abs( t_crop-t_crop_e_rel ) );
    uwb_range = crop_data_struct(uwb_range, idx_b,idx_e);
    
    % Redo min time stamp for plots
    all_stamps = [px4_imu.t, px4_mag.t,px4_baro.t,px4_gps.t, px4_gps_vel.t, rtk_gps1.t, rtk_gps2.t,rs_odom.t,lrf.t,mocap_vehicle.t, mocap_tag_board.t,lsm_imu.t,lsm_mag.t, uwb_range.t];
    %px4_motor_speed.t,
    if rs_imu_combined
        all_stamps = [ all_stamps rs_imu.t];
    else
        all_stamps = [ all_stamps rs_imu_accel.t];
        all_stamps = [ all_stamps rs_imu_gyro.t];
    end
    
    all_stamps(all_stamps==0) = [];
    t_min_ref = min(all_stamps);
    t_max_ref = max(all_stamps);
end

if set_t_zero
    px4_imu.t = px4_imu.t - t_min_ref;
    px4_mag.t = px4_mag.t - t_min_ref;
    px4_baro.t = px4_baro.t - t_min_ref;
    px4_gps.t = px4_gps.t - t_min_ref;
    px4_gps_vel.t = px4_gps_vel.t - t_min_ref;
    px4_motor_speed.t = px4_motor_speed.t - t_min_ref;
    rtk_gps1.t = rtk_gps1.t - t_min_ref;
    rtk_gps2.t = rtk_gps2.t - t_min_ref;
    
    lrf.t = lrf.t - t_min_ref;
    mocap_vehicle.t = mocap_vehicle.t - t_min_ref;
    mocap_tag_board.t = mocap_tag_board.t - t_min_ref;
    lsm_imu.t = lsm_imu.t - t_min_ref;
    lsm_mag.t = lsm_mag.t - t_min_ref;
    uwb_range.t = uwb_range.t - t_min_ref;
    
    rs_odom.t = rs_odom.t - t_min_ref;
    
    if rs_imu_combined
        rs_imu.t = rs_imu.t - t_min_ref;
    else
        rs_imu_accel.t = rs_imu_accel.t - t_min_ref;
        rs_imu_gyro.t = rs_imu_gyro.t - t_min_ref;
    end
end

%% Generate Plots
% PX4 IMU
figure_name = "PX4 IMU";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);

plot_imu(px4_imu.t - t_min_ref, px4_imu.linear_acceleration, px4_imu.t - t_min_ref, px4_imu.angular_velocity);

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

% PX4 IMU 3D
figure_name = "PX4 IMU 3D";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);

plot_imu_3d(px4_imu.t - t_min_ref, px4_imu.linear_acceleration, px4_imu.t - t_min_ref, px4_imu.angular_velocity);

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

% PX4 Pressure
figure_name = "PX4 Fluid Pressure";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name);
plot(px4_baro.t - t_min_ref, px4_baro.pressure, 'DisplayName', 'Fluid Pressure')
grid on
legend
xlabel("Time [s]");ylabel("Fluid Pressure [pa]");

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

% PX4 Magnetometer
figure_name = "PX4 Magnetometer";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);

plot_mag(px4_mag.t - t_min_ref, px4_mag.cart_vec)

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

% PX4 Motor Speeds
if ~isempty(px4_motor_speed.t)
    figure_name = "PX4 Motor Speeds";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    plot(px4_motor_speed.t - t_min_ref, px4_motor_speed.rpm(1,:), 'DisplayName', 'Motor 1')
    plot(px4_motor_speed.t - t_min_ref, px4_motor_speed.rpm(2,:), 'DisplayName', 'Motor 2')
    plot(px4_motor_speed.t - t_min_ref, px4_motor_speed.rpm(3,:), 'DisplayName', 'Motor 3')
    plot(px4_motor_speed.t - t_min_ref, px4_motor_speed.rpm(4,:), 'DisplayName', 'Motor 4')
    grid on
    legend
    xlabel("Time [s]");ylabel("Normalized RPM");
    
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
end

% LSM9DS1 IMU
if exist("lsm_imu")
    figure_name = "LSM9DS1 IMU";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    
    plot_imu(lsm_imu.t - t_min_ref, lsm_imu.linear_acceleration, lsm_imu.t - t_min_ref, lsm_imu.angular_velocity);
    
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
end

if exist("lsm_mag")
    figure_name = "LSM9DS1 MAG";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    
    plot_mag(lsm_mag.t - t_min_ref, lsm_mag.cart_vec)
    
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
end

if ~isempty(mocap_vehicle.t)
    % MoCap Vehicle Position
    figure_name = "MoCap Vehicle Position";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    
    plot_position(mocap_vehicle.t - t_min_ref, mocap_vehicle.position);
    
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
    
    % MoCap Velocity Norm
    figure_name = "MoCap Velocity Norm";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    
    plot(mocap_vehicle.t(1:end-1)  - t_min_ref, sgolayfilt(diff(vecnorm(mocap_vehicle.position))./diff(mocap_vehicle.t),3,21));
    grid on
    xlabel("Time [s]");ylabel("Velocity [m/s^2]");
    
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
end

if ~isempty(mocap_tag_board.t)
    % MoCap Tag Board Position
    figure_name = "MoCap Tag Board Position";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    
    plot_position(mocap_tag_board.t - t_min_ref, mocap_tag_board.position);
    
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
end

% RealSense IMU
if exist("rs_imu")
    figure_name = "RealSense IMU";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    
    if rs_imu_combined
        plot_imu(rs_imu.t - t_min_ref, rs_imu.linear_acceleration, rs_imu.t - t_min_ref, rs_imu.angular_velocity);
    else
        plot_imu(rs_imu_accel.t - t_min_ref, rs_imu_accel.linear_acceleration, rs_imu_gyro.t - t_min_ref, rs_imu_gyro.angular_velocity);
    end
    
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
end

if exist("rs_imu")
    % RealSense IMU 3D
    figure_name = "RealSense IMU 3D";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    
    if rs_imu_combined
        plot_imu_3d(rs_imu.t - t_min_ref, rs_imu.linear_acceleration, rs_imu.t - t_min_ref, rs_imu.angular_velocity);
    else
        plot_imu_3d(rs_imu_accel.t - t_min_ref, rs_imu_accel.linear_acceleration, rs_imu_gyro.t - t_min_ref, rs_imu_gyro.angular_velocity);
    end
    
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
end

% RealSense Pose
if exist("rs_odom")
    figure_name = "RealSense Pose";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    
    plot_position(rs_odom.t - t_min_ref, rs_odom.position);
    
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
end

if exist("lrf")
    % Laser Range Finder
    figure_name = "Laser Range Finder";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    plot(lrf.t - t_min_ref, lrf.range, 'DisplayName', 'range')
    grid on
    legend
    xlabel("Time [s]");ylabel("Range [m]");
    
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
    
end

% Plot UWB Measurements
if uwb_module_type == 1 % Decawave UWB topics
    if exist("uwb_range")
        figure_name = "Decawave UWB Modules";
        fh1 = figure('Name',figure_name);
        clf(fh1);
        set(fh1, savefigimages.figOptions);
        hold on
        title(figure_name)
        plot_decawave_uwb(uwb_range.t - t_min_ref, uwb_range.dist, uwb_range.valid);
        
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
    end
else % Neuhold UWB topics
    
end

%% PX4 GPS
if ~isempty(px4_gps.t)
    figure_name = "PX4 GPS ENU";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    [~,~,~,h3d]=plot_position(px4_gps.t - t_min_ref, px4_gps.enu);
    
    if show_building_entry
        plot3(h3d,door_position(1),door_position(2),door_position(3),'r.','MarkerSize',30,'DisplayName', 'Building Entry');
    end
    
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
    
    % PX4 GPS Covariance
    figure_name = "PX4 GPS ENU Standard Deviation";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    
    plot_position_std(px4_gps.t - t_min_ref, sqrt(squeeze(px4_gps.enu_cov)));
    
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
    
    % PX4 GPS Velocity
    figure_name = "PX4 GPS Velocity";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    plot(px4_gps_vel.t - t_min_ref, px4_gps_vel.velocity(1,:), 'DisplayName', 'x');
    plot(px4_gps_vel.t - t_min_ref, px4_gps_vel.velocity(2,:), 'DisplayName', 'y');
    plot(px4_gps_vel.t - t_min_ref, px4_gps_vel.velocity(3,:), 'DisplayName', 'z');
    grid on
    legend
    xlabel("Time [s]");ylabel("Velocity [m/s]");
    
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
end

%% RTK 1 GPS
if ~isempty(rtk_gps1.t)
    
    figure_name = "RTK 1 GPS ENU";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    
    rtk_gps1_fix = rtk_gps1.gps_fix_type==3;
    rtk_gps1_rtk_status = rtk_gps1.rtk_status==1;
    
    [g1h1,g1h2,g1h3,g1h3d]= plot_gps_position(rtk_gps1.t - t_min_ref, rtk_gps1.enu, rtk_gps1_fix,rtk_gps1_rtk_status);
    
    if show_building_entry
        plot3(g1h3d,door_position(1),door_position(2),door_position(3),'r.','MarkerSize',30,'DisplayName', 'Building Entry');
    end
    
    legend
    
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
    
    % RTK 1 Covariance
    figure_name = "RTK 1 GPS ENU Standard Deviation";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    
    rtk_gps1_fix = rtk_gps1.gps_fix_type==3;
    rtk_gps1_rtk_status = rtk_gps1.rtk_status==1;
    
    plot_position_std(rtk_gps1.t(rtk_gps1_fix) - t_min_ref, sqrt(rtk_gps1.enu_cov(:,rtk_gps1_fix)));
    
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
    
    
    % RTK 1 GPS Velocity
    figure_name = "RTK 1 GPS Velocity";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    plot(rtk_gps1.t(rtk_gps1_fix) - t_min_ref, rtk_gps1.vel(1,rtk_gps1_fix),'.', 'DisplayName', 'x');
    plot(rtk_gps1.t(rtk_gps1_fix) - t_min_ref, rtk_gps1.vel(2,rtk_gps1_fix),'.', 'DisplayName', 'y');
    plot(rtk_gps1.t(rtk_gps1_fix) - t_min_ref, rtk_gps1.vel(3,rtk_gps1_fix),'.', 'DisplayName', 'z');
    grid on
    legend
    xlabel("Time [s]");ylabel("Velocity [m/s]");
    
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
    
    % RTK 1 GPS Velocity Norm
    figure_name = "RTK 1 GPS Velocity Norm";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    plot(rtk_gps1.t(rtk_gps1_fix) - t_min_ref, vecnorm(rtk_gps1.vel(:,rtk_gps1_fix)),'.', 'DisplayName', 'norm');
    grid on
    legend
    xlabel("Time [s]");ylabel("Velocity [m/s]");
    
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
    
end

%% RTK 2 GPS
if ~isempty(rtk_gps2.t)
    
    figure_name = "RTK 2 GPS ENU";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    
    rtk_gps2_fix = rtk_gps2.gps_fix_type==3;
    rtk_gps2_rtk_status = rtk_gps2.rtk_status==1;
    
    [g2h1,g2h2,g2h3,g2h3d]= plot_gps_position(rtk_gps2.t - t_min_ref, rtk_gps2.enu, rtk_gps2_fix, rtk_gps2_rtk_status);
    
    if show_building_entry
        plot3(h3d,door_position(1),door_position(2),door_position(3),'r.','MarkerSize',30,'DisplayName', 'Building Entry');
    end
    
    legend
    
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
    
    % RTK 2 Covariance
    figure_name = "RTK 2 GPS ENU Standard Deviation";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    
    rtk_gps2_fix = rtk_gps2.gps_fix_type==3;
    rtk_gps2_rtk_status = rtk_gps2.rtk_status==1;
    
    plot_position_std(rtk_gps2.t(rtk_gps2_fix) - t_min_ref, sqrt(rtk_gps2.enu_cov(:,rtk_gps2_fix)));
    
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
    
    % RTK 2 GPS Velocity
    figure_name = "RTK 2 GPS Velocity";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    plot(rtk_gps2.t(rtk_gps2_fix) - t_min_ref, rtk_gps2.vel(1,rtk_gps2_fix),'.', 'DisplayName', 'x');
    plot(rtk_gps2.t(rtk_gps2_fix) - t_min_ref, rtk_gps2.vel(2,rtk_gps2_fix),'.', 'DisplayName', 'y');
    plot(rtk_gps2.t(rtk_gps2_fix) - t_min_ref, rtk_gps2.vel(3,rtk_gps2_fix),'.', 'DisplayName', 'z');
    grid on
    legend
    xlabel("Time [s]");ylabel("Velocity [m/s]");
    
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
    
    
    % RTK 2 GPS Velocity Norm
    figure_name = "RTK 2 GPS Velocity Norm";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    plot(rtk_gps2.t(rtk_gps2_fix) - t_min_ref, vecnorm(rtk_gps2.vel(:,rtk_gps2_fix)),'.', 'DisplayName', 'norm');
    grid on
    legend
    xlabel("Time [s]");ylabel("Velocity [m/s]");
    
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
end

if additional_gps_plots
    % Plot all GPS
    figure_name = "All GPS ENU";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    
    idx_end_px4 = find(px4_gps.t - px4_gps.t(1) < all_gps_t_crop,1,'last');
    idx_end_rtk1 = find(rtk_gps1.t - rtk_gps1.t(1) < all_gps_t_crop,1,'last');
    idx_end_rtk2 = find(rtk_gps2.t - rtk_gps2.t(1) < all_gps_t_crop,1,'last');
    
    plot3(px4_gps.enu(1:idx_end_px4,1),px4_gps.enu(1:idx_end_px4,2),px4_gps.enu(1:idx_end_px4,3),'DisplayName','PX4')
    plot3(rtk_gps1.enu(1:idx_end_rtk1,1),rtk_gps1.enu(1:idx_end_rtk1,2),rtk_gps1.enu(1:idx_end_rtk1,3),'DisplayName','RTK1')
    plot3(rtk_gps2.enu(1:idx_end_rtk1,1),rtk_gps2.enu(1:idx_end_rtk1,2),rtk_gps2.enu(1:idx_end_rtk1,3),'DisplayName','RTK2')
    if show_building_entry
        plot3(door_position(1),door_position(2),door_position(3),'r.','MarkerSize',30,'DisplayName', 'Building Entry');
    end
    legend
    axis equal
    xlabel("x [m]");ylabel("y [m]");zlabel("z [m]");
    grid on
    view(3)
end

%% CSV export
mat_save_var = {}; % Cell array to accumulate the workspace variables to be saved
if save_csv
    fprintf("Saving CSV data...\n");
    
    if ~exist(data_export_dir, 'dir')
        fprintf("[Info] " + data_export_dir + " does not exist. Creating folder...\n");
        mkdir(data_export_dir)
    end
    fprintf("[Info] " + "Writing CSV files and README...\n");
    rm_fid = fopen(data_export_dir + "README.txt", 'wt' );
    
    % Write time info file
    fprintf("[Info] " + "Writing time offset yaml file...\n");
    time_filename = fullfile(data_export_dir,"time_info.yaml");
    time_fid = fopen(time_filename, 'wt' );
    fprintf( time_fid, "# Time Info\n");
    fprintf( time_fid, "t_total: %.3f # in seconds\n", max(all_stamps) - min(all_stamps));
    fprintf( time_fid, "t_begin: %.23f # in seconds\n", min(all_stamps));
    fprintf( time_fid, "t_end: %.23f # in seconds\n\n", max(all_stamps));
    fclose(time_fid);
    
    % General information to README
    % GPS
    if ~isempty(rtk_gps1) % Only add this information if GPS data exists
        fprintf(rm_fid,"Reference GPS coordinates for local ENU conversion\n\t [Latitude,Longitude,Altitude] = %f, %f, %f\n", gps_ref_coord.lat,gps_ref_coord.long,gps_ref_coord.alt);
        mat_save_var{end+1}='gps_ref_coord';
        if show_building_entry
            fprintf(rm_fid,"Building entry GPS coordinates\n\t [Latitude,Longitude,Altitude] = %f, %f, %f\n", gps_entry_coord.lat,gps_entry_coord.long,gps_entry_coord.alt);
            mat_save_var{end+1}='gps_entry_coord';
        end
        fprintf(rm_fid,"\n");
    end
    
    fprintf(rm_fid,"Data Stream Information:\n");
    
    if ~exist(csv_export_dir, 'dir')
        fprintf("[Info] " + csv_export_dir + " does not exist. Creating folder...\n");
        mkdir(csv_export_dir)
    end
    
    if ~isempty(px4_imu.t)
        filename = "px4_imu.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, a_x, a_y, a_z, w_x, w_y, w_z"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_px4_imu = [px4_imu.t.', px4_imu.linear_acceleration.', px4_imu.angular_velocity.'];
        mat_save_var{end+1}='px4_imu';
        fprintf( rm_fid, "PX4 IMU format: \n\tENU [" + header + "]\n" + sensor_stats_from_csv(csv_px4_imu));
        dlmwrite(csv_export_dir + filename, csv_px4_imu, '-append', 'delimiter', ',', 'precision', 17);
    end
    
    if ~isempty(px4_mag.t)
        filename = "px4_mag.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, cart_x, cart_y, cart_z, spher_az, spher_el, spher_norm"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_px4_mag = [px4_mag.t.', px4_mag.cart_vec.', px4_mag.az.', px4_mag.el.', px4_mag.norm.'];
        mat_save_var{end+1}='px4_mag';
        fprintf( rm_fid, "PX4 Mag format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_px4_mag));
        dlmwrite(csv_export_dir + filename, csv_px4_mag, '-append', 'delimiter', ',', 'precision', 17);
    end
    
    if ~isempty(px4_baro.t)
        filename = "px4_baro.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, p"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_px4_baro = [px4_baro.t.', px4_baro.pressure.'];
        mat_save_var{end+1}='px4_baro';
        fprintf( rm_fid, "PX4 Baro [pa] format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_px4_baro));
        dlmwrite(csv_export_dir + filename, csv_px4_baro, '-append', 'delimiter', ',', 'precision', 17);
    end
    
    if ~isempty(px4_gps.t)
        filename = "px4_gps.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, lat, long, alt, p_x, p_y, p_z, cov_p_x, cov_p_y, cov_p_z,"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_px4_gps = [px4_gps.t.' , px4_gps.lat.', px4_gps.long.', px4_gps.alt.', px4_gps.enu.', px4_gps.enu_cov.'];
        mat_save_var{end+1}='px4_gps';
        fprintf( rm_fid, "PX4 GPS format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_px4_gps));
        dlmwrite(csv_export_dir + filename, csv_px4_gps, '-append', 'delimiter', ',', 'precision', 17);
        fprintf( rm_fid, "\tDistance: " + sum(abs(diff(vecnorm(px4_gps.enu)))) + "m\n");
    end
    
    if ~isempty(px4_gps_vel.t)
        filename = "px4_gps_vel_data.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, v_x, v_y, v_z"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_px4_gps_vel = [px4_gps_vel.t.' , px4_gps_vel.velocity.'];
        mat_save_var{end+1}='px4_gps_vel';
        fprintf( rm_fid, "PX4 GPS Velocity format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_px4_gps_vel));
        dlmwrite(csv_export_dir + filename, csv_px4_gps_vel, '-append', 'delimiter', ',', 'precision', 17);
    end
    
    if ~isempty(px4_motor_speed.t)
        filename = "px4_rpm.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, motor1, motor2, motor3, motor4"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_px4_motor_rpm = [px4_motor_speed.t.', px4_motor_speed.rpm.'];
        mat_save_var{end+1}='px4_motor_speed';
        fprintf( rm_fid, "PX4 Normalized Motor RPM format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_px4_motor_rpm));
        dlmwrite(csv_export_dir + filename, csv_px4_motor_rpm, '-append', 'delimiter', ',', 'precision', 17);
    end
    
    if exist("lsm_imu")
        if ~isempty(lsm_imu.t)
            filename = "lsm_imu.csv";
            fprintf("Writing %s\n", filename);
            
            % Write csv header to the file
            header = "t, a_x, a_y, a_z, w_x, w_y, w_z"; % csv header
            csv_fid = fopen(csv_export_dir + filename, 'w+');
            fprintf(csv_fid,'%s\n', header);
            fclose(csv_fid);
            
            % Generate csv data and write to file
            csv_lsm_imu = [lsm_imu.t.', lsm_imu.linear_acceleration.', lsm_imu.angular_velocity.'];
            mat_save_var{end+1}='lsm_imu';
            fprintf( rm_fid, "LSM9DS1 IMU format: \n\tENU [" + header + "]\n" + sensor_stats_from_csv(csv_lsm_imu));
            dlmwrite(csv_export_dir + filename, csv_lsm_imu, '-append', 'delimiter', ',', 'precision', 17);
        end
    end
    
    if exist("lsm_mag")
        if ~isempty(lsm_mag.t)
            filename = "lsm_mag.csv";
            fprintf("Writing %s\n", filename);
            
            % Write csv header to the file
            header = "t, cart_x, cart_y, cart_z, spher_az, spher_el, spher_norm"; % csv header
            csv_fid = fopen(csv_export_dir + filename, 'w+');
            fprintf(csv_fid,'%s\n', header);
            fclose(csv_fid);
            
            % Generate csv data and write to file
            csv_lsm_mag = [lsm_mag.t.', lsm_mag.cart_vec.', lsm_mag.az.', lsm_mag.el.', lsm_mag.norm.'];
            mat_save_var{end+1}='lsm_mag';
            fprintf( rm_fid, "LSM9DS1 Mag format:\n\t["+header+"]\n" + sensor_stats_from_csv(csv_lsm_mag));
            dlmwrite(csv_export_dir + filename, csv_lsm_mag, '-append', 'delimiter', ',', 'precision', 17);
        end
    end
    
    if exist("uwb_range")
        if ~isempty(uwb_range.t)
            filename = "uwb_range.csv";
            fprintf("Writing %s\n", filename);
            
            % Write csv header to the file
            header = "t, dist_m1, dist_m2, dist_m3, valid_m1, valid_m2, valid_m3"; % csv header
            csv_fid = fopen(csv_export_dir + filename, 'w+');
            fprintf(csv_fid,'%s\n', header);
            fclose(csv_fid);
            
            % Generate csv data and write to file
            csv_lsm_mag = [uwb_range.t.', uwb_range.dist(1,:).', uwb_range.dist(2,:).', uwb_range.dist(3,:).', ...
                uwb_range.valid(1,:).', uwb_range.valid(2,:).', uwb_range.valid(3,:).'];
            mat_save_var{end+1}='uwb_range';
            fprintf( rm_fid, "Decawave UWB Range format:\n\t["+header+"]\n" + sensor_stats_from_csv(csv_lsm_mag));
            dlmwrite(csv_export_dir + filename, csv_lsm_mag, '-append', 'delimiter', ',', 'precision', 17);
        end
    end
    
    if exist("rtk_gps1")
        if ~isempty(rtk_gps1.t)
            filename = "rtk_gps1.csv";
            fprintf("Writing %s\n", filename);
            
            % Write csv header to the file
            header = "t, t_gps, lat, long, alt, p_x, p_y, p_z, v_x, v_y, v_z, cov_p_x, cov_p_y, cov_p_z, gps_fix_type, rtk_status"; % csv header
            csv_fid = fopen(csv_export_dir + filename, 'w+');
            fprintf(csv_fid,'%s\n', header);
            fclose(csv_fid);
            
            % Generate csv data and write to file
            csv_rtk_gps_1 = [rtk_gps1.t(:), rtk_gps1.gps_time(:),  rtk_gps1.lat(:), rtk_gps1.long(:), rtk_gps1.alt(:), rtk_gps1.enu.' ,rtk_gps1.vel.', rtk_gps1.enu_cov.', rtk_gps1.gps_fix_type(:), rtk_gps1.rtk_status(:)];
            mat_save_var{end+1}='rtk_gps1';
            fprintf( rm_fid, "RTK GPS1 format:\n\t["+header+"]\n" + sensor_stats_from_csv(csv_rtk_gps_1));
            dlmwrite(csv_export_dir + filename, csv_rtk_gps_1, '-append', 'delimiter', ',', 'precision', 17);
            fprintf( rm_fid, "\tDistance: " + sum(abs(diff(vecnorm(rtk_gps1.enu)))) + "m\n");
            
        end
    end
    
    if exist("rtk_gps2")
        if ~isempty(rtk_gps2.t)
            filename = "rtk_gps2.csv";
            fprintf("Writing %s\n", filename);
            
            % Write csv header to the file
            header = "t, t_gps, lat, long, alt, p_x, p_y, p_z, v_x, v_y, v_z, cov_p_x, cov_p_y, cov_p_z, gps_fix_type, rtk_status"; % csv header
            csv_fid = fopen(csv_export_dir + filename, 'w+');
            fprintf(csv_fid,'%s\n', header);
            fclose(csv_fid);
            
            % Generate csv data and write to file
            csv_rtk_gps_2 = [rtk_gps2.t(:), rtk_gps2.gps_time(:), rtk_gps2.lat(:), rtk_gps2.long(:), rtk_gps2.alt(:),rtk_gps2.enu.' , rtk_gps2.vel.', rtk_gps2.enu_cov.', rtk_gps2.gps_fix_type(:), rtk_gps2.rtk_status(:)];
            mat_save_var{end+1}='rtk_gps2';
            fprintf( rm_fid, "RTK GPS 2 format:\n\t["+ header +"]\n" + sensor_stats_from_csv(csv_rtk_gps_2));
            dlmwrite(csv_export_dir + filename, csv_rtk_gps_2, '-append', 'delimiter', ',', 'precision', 17);
            fprintf( rm_fid, "\tDistance: " + sum(abs(diff(vecnorm(rtk_gps2.enu)))) + "m\n");
        end
    end
    
    
    if ~isempty(mocap_vehicle.t)
        filename = "mocap_vehicle_data.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, p_x, p_y, p_z, q_w, q_x, q_y, q_z"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_mocap_vehicle = [mocap_vehicle.t.', mocap_vehicle.position.', mocap_vehicle.orientation.'];
        mat_save_var{end+1}='mocap_vehicle';
        fprintf( rm_fid, "MoCap format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_mocap_vehicle));
        dlmwrite(csv_export_dir + filename, csv_mocap_vehicle, '-append', 'delimiter', ',', 'precision', 17);
        fprintf( rm_fid, "\tDistance: " + sum(abs(diff(vecnorm(mocap_vehicle.position)))) + "m\n");
    end
    
    if ~isempty(mocap_tag_board.t)
        filename = "mocap_tag_board.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, p_x, p_y, p_z, q_w, q_x, q_y, q_z"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_mocap_tag_board = [mocap_tag_board.t.', mocap_tag_board.position.', mocap_tag_board.orientation.'];
        mat_save_var{end+1}='mocap_tag_board';
        fprintf( rm_fid, "MoCap format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_mocap_tag_board));
        dlmwrite(csv_export_dir + filename, csv_mocap_tag_board, '-append', 'delimiter', ',', 'precision', 17);
    end
    
    if ~isempty(fiducial_markers.tag_data)
        
        % Detected dags
        tag_ids_string = "[" + sprintf('%.0f,' , fiducial_markers.tags_detected) + "]";
        
        filename = "fiducial_marker.csv";
        fprintf("Writing %s\n", filename);
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        
        header = "t, ";
        for i = 1 : length(fiducial_markers.tags_detected)
            marker_id = fiducial_markers.tags_detected(i);
            header = header + "ex_" + int2str(marker_id) + ", p_x_" + int2str(marker_id) + ", p_y_" + int2str(marker_id) + ", p_z_" + int2str(marker_id) + ", q_w_" + int2str(marker_id) + ", q_x_" + int2str(marker_id) + ", q_y_" + int2str(marker_id) + ", q_z_" + int2str(marker_id) + ", ";
        end
        header_char = convertStringsToChars(header);
        header = convertCharsToStrings(header_char(1:end-2));  % Remove the last comma and space
        
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        num_columns = 1 + 8 * length(fiducial_markers.tags_detected);
        num_rows = length(fiducial_markers.tag_data);
        csv_fiducial_markers = zeros(num_rows, num_columns);
        
%         csv_fiducial_markers(:, 1) = fiducial_markers.tag_data.t;
        for i = 1:length(fiducial_markers.tag_data)
            %TODO: why is the code two lines before that not working?
            csv_fiducial_markers(i, 1) = fiducial_markers.tag_data(i).t;
            for j = 1:length(fiducial_markers.tag_data(i).data)
                marker_id = fiducial_markers.tag_data(i).data(j).id;
                idx = find(fiducial_markers.tags_detected == marker_id);
                csv_fiducial_markers(i, 2 + (idx-1)*8) = 1;  % measurement exists
                csv_fiducial_markers(i, 3 + (idx-1)*8 : 5 + (idx-1)*8) = fiducial_markers.tag_data(i).data(j).pose.translation;
                csv_fiducial_markers(i, 6 + (idx-1)*8 : 9 + (idx-1)*8) = fiducial_markers.tag_data(i).data(j).pose.rotation;
            end
        end
        
        % Generate csv data and write to file
        mat_save_var{end+1}='fiducial_marker';
        example_header = "t, ex_0, p_x_0, p_y_0, p_z_0, q_w_0, q_x_0, q_y_0, q_z_0, ..., ex_n, p_x_n, p_y_n, p_z_n, q_w_n, q_x_n, q_y_n, q_z_n";
        fprintf(rm_fid, "Fiducial Marker Pose format: \n\t[" + example_header + "]\n\tFiducial Tag IDs: " + tag_ids_string + "\n" + sensor_stats_from_csv(csv_fiducial_markers));
        dlmwrite(csv_export_dir + filename, csv_fiducial_markers, '-append', 'delimiter', ',', 'precision', 17);
        
    end
        
    if rs_imu_combined
        if exist("rs_imu")
            if ~isempty(rs_imu.t)
                filename = "rs_imu.csv";
                fprintf("Writing %s\n", filename);
                
                % Write csv header to the file
                header = "t, a_x, a_y, a_z, w_x, w_y, w_z"; % csv header
                csv_fid = fopen(csv_export_dir + filename, 'w+');
                fprintf(csv_fid,'%s\n', header);
                fclose(csv_fid);
                
                % Generate csv data and write to file
                csv_rs_imu = [rs_imu.t.', rs_imu.linear_acceleration.', rs_imu.angular_velocity.'];
                mat_save_var{end+1}='rs_imu';
                fprintf( rm_fid, "RealSense IMU format:\n\tENU [" + header + "]\n" + sensor_stats_from_csv(csv_rs_imu));
                dlmwrite(csv_export_dir + filename, csv_rs_imu, '-append', 'delimiter', ',', 'precision', 17);
            end
        end
    else
        if exist("rs_imu_accel")
            if ~isempty(rs_imu_accel.t)
                filename = "rs_imu_accel_data.csv";
                fprintf("Writing %s\n", filename);
                
                % Write csv header to the file
                header = "t, linear_accel_x, linear_accel_y, linear_accel_z"; % csv header
                csv_fid = fopen(csv_export_dir + filename, 'w+');
                fprintf(csv_fid,'%s\n', header);
                fclose(csv_fid);
                
                csv_rs_imu_accel = [rs_imu_accel.t.', rs_imu_accel.linear_acceleration.'];
                mat_save_var{end+1}='rs_imu_accel';
                fprintf( rm_fid, "RealSense linear acceleration IMU format:\n\tENU [" + header + "]\n" + sensor_stats_from_csv(csv_rs_imu_accel));
                dlmwrite(csv_export_dir + filename, csv_rs_imu_accel, '-append', 'delimiter', ',', 'precision', 17);
            end
        end
        
        if exist("rs_imu_accel")
            if ~isempty(rs_imu_gyro.t)
                filename = "rs_imu_gyro_data.csv";
                fprintf("Writing %s\n", filename);
                
                % Write csv header to the file
                header = "t, gyro_x, gyro_y, gyro_z"; % csv header
                csv_fid = fopen(csv_export_dir + filename, 'w+');
                fprintf(csv_fid,'%s\n', header);
                fclose(csv_fid);
                
                csv_rs_imu_gyro = [rs_imu_gyro.t.', rs_imu_gyro.angular_velocity.'];
                mat_save_var{end+1}='rs_imu_gyro';
                fprintf( rm_fid, "RealSense angular velocity IMU format:\n\tENU [" + header + "]\n" + sensor_stats_from_csv(csv_rs_imu_gyro));
                dlmwrite(csv_export_dir + filename, csv_rs_imu_gyro, '-append', 'delimiter', ',', 'precision', 17);
            end
        end
    end
    
    if exist("rs_odom")
        if ~isempty(rs_odom.t)
            filename = "rs_odom.csv";
            fprintf("Writing %s\n", filename);
            
            % Write csv header to the file
            header = "t, p_x, p_y, p_z, q_w, q_x, q_y, q_z"; % csv header
            csv_fid = fopen(csv_export_dir + filename, 'w+');
            fprintf(csv_fid,'%s\n', header);
            fclose(csv_fid);
            
            % Generate csv data and write to file
            csv_rs_odom = [rs_odom.t.', rs_odom.position.', rs_odom.orientation.'];
            mat_save_var{end+1}='rs_odom';
            fprintf( rm_fid, "RealSense odometry format:\n\t[" + header + "]\n"  + sensor_stats_from_csv(csv_rs_odom));
            dlmwrite(csv_export_dir + filename, csv_rs_odom, '-append', 'delimiter', ',', 'precision', 17);
        end
    end
    
    if ~isempty(lrf.t)
        filename = "lrf_range.csv";
        fprintf("Writing %s\n", filename);
        
        % Write csv header to the file
        header = "t, range"; % csv header
        csv_fid = fopen(csv_export_dir + filename, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);
        
        % Generate csv data and write to file
        csv_lrf_range = [lrf.t.', lrf.range.'];
        mat_save_var{end+1}='lrf';
        fprintf( rm_fid, "Laser Range Finder format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_lrf_range));
        fprintf( rm_fid, "\tMax. Measured Range: " + max(lrf.range) + "m\n");
        
        dlmwrite(csv_export_dir + filename, csv_lrf_range, '-append', 'delimiter', ',', 'precision', 17);
    end
    
    fclose(rm_fid);
    fprintf("...done saving CSV data\n");
end

if save_ws
    fprintf("[Info] Write Matlab Workspace to File\n");
    save(fullfile(data_export_dir, "matlab_sensor_data.mat"),mat_save_var{:});
end

fprintf("Finished Data Export\n");
