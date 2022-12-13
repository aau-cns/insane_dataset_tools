%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

%% Calculation of a 3Dof Vehicle Orientation based on two GPS vectors and a Magnetometer Measurement
%
% The vectors of the magnetometer and virtual GPS vector need to be on a
% rigid body. Thus, the angle between the two 3D vectors should be
% constant.
%
% The vector of the magnetic measurement is expressed in the magnetometer
% (senror) frame. This vector is constant in the GPS world frame /
% magnetometer world frame (given correct sensor calibration). Thus, the
% magnetic vector represents a passive rotation of the magnetic sensor frame.
%
% The two GPS measurements can be used to generate a virtual GPS sensor
% orientation. The rotational information relates to the virtual GPS sensor
% with respect to the GPS world frame.
%
% A combination of cross-products is used to calculate the 3DoF orientation
% of the virtual GPS sensor. The X-axis of the rotational frame is aligned
% with the vector of the virtual GPS sensors (p_gps2-p_gps1). This axis is
% used together with the magnetic vector, which is assumed to have a
% positive rotation with respect to the GPS vector (they do not have to
% be orthogonal). The result of this cross-product is the z axis. Because
% the given vectors were not orthogonal, the resuld needs to be normalized.
% Afterwards, the GPS Vector, basevector of the x-axis and basevector of
% the z-axis are used to calculate the y-axis (cross-product of (-x)
% and y axis).
% The result is the rotation matrix of the virtual GPS sensor expressed in
% the GPS world frame.
%
%
% Important note!
% The angle between the Magnetometer and GPS Vector needs to be high enough
% to ensure the quality of the resulting orientation. This is also
% necessaray because the noise on these measurements could lead to
% both vectors switching the side which would cause the flip of the
% coordinate system. The generated plot below shows the angle between the
% vectors.
%
% Steps:
% 1. Get valid and synchronized RTK GPS 1 and 2 measurements
% 2. Apply magnetometer calibration
% 3. Time sync Magnetometer and GPS
% 4. Synchronize Magnetometer and GPS Measurements
% 5. Get Magnetic Sensor Orientation in the GPS world
% 6. Get 3 DoF orientation of virtual GPS sensor in the GPS world
% 7. Get the rotation of the IMU in the GPS world
% 8. Get the position of the IMU based on RTK2 in the GPS world
% 9. Get the position of the IMU based on interpolated RTK2 (matching the magnetometer rate) in the GPS world
% 10. Time-synchronize the RTK based IMU ground truth to the IMU measurements
% 11. Add timestamps matched to the IMU time to the datastreams
% 12. Store ground truth data and revised sensor data in CSV files

clear all
close all

format long

%% Set all data paths
data_path="/tmp/";

sensor_calib_data_path = "/tmp/matlab/sensor_calibration.m";

sensor_data_path = fullfile(data_path, "export/matlab_sensor_data.mat");

save_plots = true;  % Store the plots as .png
show_plots = true;  % Show plots during generation
save_csv = true;    % Export CSV files for the GPS coordinates and ENU data
save_ws = true;     % Save workspace variables for GPS coordinates and ENU data
debug = false;

use_gps_float = true;
do_crop_rtk_gps = true;
t_crop_rtk_gps_begin = 0;
t_crop_rtk_gps_begin = 20;
t_crop_rtk_gps_end = 165;
t_crop_rtk_gps_end = 165;
use_movmed_for_mag_sync = true;
use_movmed_for_imu_sync = true;
do_crop_mag = true;
t_crop_mag_begin = 0;
% Time used to crop the datastreams before time syncronization
t_crop_mag_end = 150;

%% Define Data Output
data_export_dir = fullfile(data_path, "/export/");
plot_dir = fullfile(data_export_dir, "/plots/ground_truth");
csv_export_dir = fullfile(data_export_dir, "/csv/ground_truth");

%% Read Sensor Data
load(sensor_data_path);

%% Read Sensor calibration (run script)
run(sensor_calib_data_path);

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

%% Get valid and synchronized RTK GPS 1 and 2 measurements

% Get valid GPS measurements SELECT means to select all valid GPS
% measurements from the datastream

if use_gps_float
    % Use Float and Fix state data
    gps1_valid_idx = (rtk_gps1.rtk_status == 1) | (rtk_gps1.rtk_status == 0);
else
    % Use only Fix state RTK data
    gps1_valid_idx = (rtk_gps1.rtk_status == 1);
end

rtk_gps1_select = select_gps_entries(rtk_gps1,gps1_valid_idx);

if do_crop_rtk_gps
    idx_crop_rtk = ~((rtk_gps1_select.gps_time - rtk_gps1_select.gps_time(1) < t_crop_rtk_gps_begin) | (rtk_gps1_select.gps_time - rtk_gps1_select.gps_time(1) > t_crop_rtk_gps_end));
    rtk_gps1_select = select_gps_entries(rtk_gps1_select,idx_crop_rtk);
end

if use_gps_float
    % Use Float and Fix state data
    gps2_valid_idx = (rtk_gps2.rtk_status == 1) | (rtk_gps2.rtk_status == 0);
else
    % Use only Fix state RTK data
    gps2_valid_idx = (rtk_gps2.rtk_status == 1);
end

rtk_gps2_select = select_gps_entries(rtk_gps2,gps2_valid_idx);


if do_crop_rtk_gps
    idx_crop_rtk = ~((rtk_gps2_select.gps_time - rtk_gps2_select.gps_time(1) < t_crop_rtk_gps_begin) | (rtk_gps2_select.gps_time - rtk_gps2_select.gps_time(1) > t_crop_rtk_gps_end));
    rtk_gps2_select = select_gps_entries(rtk_gps2_select,idx_crop_rtk);
end

%% Sync gps measurements
[idx_g1, idx_g2] = sync_data_struct(rtk_gps1_select.gps_time, rtk_gps2_select.gps_time, 0.001);
rtk_gps1_sync = select_gps_entries(rtk_gps1_select,idx_g1);
rtk_gps2_sync = select_gps_entries(rtk_gps2_select,idx_g2);

%% Plot Raw GPS data
figure_name = "RTK GNSS Position (Raw)";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
title(figure_name)
hold on
plot3(rtk_gps1_sync.enu(1,:),rtk_gps1_sync.enu(2,:),rtk_gps1_sync.enu(3,:),'r.','DisplayName','RTK1')
plot3(rtk_gps2_sync.enu(1,:),rtk_gps2_sync.enu(2,:),rtk_gps2_sync.enu(3,:),'b.','DisplayName','RTK2')
grid on
axis equal
view(3)
xlabel("X [m]");ylabel("Y [m]");zlabel("Z [m]");
legend('Location','northwest','NumColumns',2)

% Get virtual GPS measurement
gps_rot.t = rtk_gps1_sync.gps_time;
gps_rot.vec = rtk_gps2_sync.enu - rtk_gps1_sync.enu;

[g_az,g_el,g_r] = cart2sph(gps_rot.vec(1,:),gps_rot.vec(2,:),gps_rot.vec(3,:));

gps_rot.g_az = g_az;
gps_rot.g_el = g_el;
gps_rot.g_r = g_r;

figure_name = "Distance between two RTK GPS (Sanity Check)";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
title(figure_name)
hold on
plot(gps_rot.t,g_r,'DisplayName', 'GPS distance');
yline(norm_rtk1_rtk2,'r','DisplayName','GPS Distance Calibration');
grid on
legend
ylabel("Distance [m]");xlabel("Time [s]");

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(fh1, fullfile(plot_dir,export_name), savefigimages.printOptions{:}, '-nocrop');
    savefig(fullfile(plot_dir,export_name) + ".fig");
end

%% Apply magnetometer calibration
% Map Raw data and apply intrinsic correction

mag_orig.t = px4_mag.t;
mag_orig.vec = px4_mag.cart_vec;%px4_mag_intr.transform * (px4_mag.cart_vec - px4_mag_intr.offset);
[mag_orig.az, mag_orig.el, mag_orig.r] = cart2sph(mag_orig.vec(1,:),mag_orig.vec(2,:),mag_orig.vec(3,:));

if debug
    % Plot RAW Magnetometer Data (no timesync)
    figure
    hold on
    title("Raw Magnetometer Measurements Elevation")

    plot(mag_orig.t, rad2deg(mag_orig.el),'.', 'DisplayName','raw')
    grid on

    xlabel("t [se]");
    ylabel("Angle [deg]");
    legend

    figure
    hold on
    title("Raw Magnetometer Measurements Azimuth")
    plot(mag_orig.t, rad2deg(mag_orig.az),'.', 'DisplayName','raw')
    grid on

    xlabel("t [s]");
    ylabel("Angle [deg]");
    legend
end


%% Time sync Magnetometer and GPS
% Using the angular rate of the yaw component

if do_crop_mag
    idx_crop_mag = ~(((mag_orig.t - mag_orig.t(1)) < t_crop_mag_begin) | ((mag_orig.t - mag_orig.t(1)) > t_crop_mag_end));
    mag_sync.t = mag_orig.t(idx_crop_mag);
    mag_sync.v = -mag_orig.az(idx_crop_mag); % Negative because the Mag vector describes a passive rotation
else
    mag_sync.t = mag_orig.t;
    mag_sync.v = -mag_orig.az; % Negative because the Mag vector describes a passive rotation
end

gps_sync.t = gps_rot.t;
gps_sync.v = gps_rot.g_az;

ta = gps_sync.t(1:end-1);
va = diff(gps_sync.v)./diff(gps_sync.t);
ta = movmedian(ta,4);
va = movmedian(va,4);

tb = mag_sync.t(1:end-1);
vb = diff(mag_sync.v)./diff(mag_sync.t);

if use_movmed_for_mag_sync
    vb = movmedian(vb,8);
end

vb = sgolayfilt(vb,2,51);

t_offset = time_reg(ta, va, tb, vb);
t_mag_gps = (ta(1)-tb(1)) - t_offset(1);

fprintf("Timeoffset Mag to GPS: %.3f[sec.]\n",t_mag_gps)

% Plot Result
figure_name = "Mag GPS Time Synchronization";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
title(figure_name)

h1 = subplot(2,1,1);
hold on
title("No Synchronization")
plot(tb,vb,'DisplayName','Mag')
plot(ta,va,'DisplayName','GPS')
xlabel("Time [s]");ylabel("yaw rate [rad/s]");
grid on
legend

h2 = subplot(2,1,2);
hold on
title("With Synchronization")
plot(tb+t_mag_gps,vb,'DisplayName','Mag')
plot(ta,va,'DisplayName','GPS')
xlabel("Time [s]");ylabel("yaw rate [rad/s]");
grid on
legend

linkaxes([h1,h2],'xy');

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(fh1, fullfile(plot_dir,export_name), savefigimages.printOptions{:}, '-nocrop');
    savefig(fullfile(plot_dir,export_name) + ".fig");
end

% Apply time offset - correct mag time to gps time
mag_orig.t_corr = mag_orig.t + t_mag_gps;


%% Interpolated GPS data to match Magnetometer samples
% Crop both vectors to only interpolate overlaping entries
% GPS Crop
idx_g_crop = ~(gps_rot.t > mag_orig.t_corr(end) | gps_rot.t < mag_orig.t_corr(1));
% Mag Crop
idx_m_crop = ~(mag_orig.t_corr > gps_rot.t(end) | mag_orig.t_corr < gps_rot.t(1));

mag_interp.t = mag_orig.t_corr(idx_m_crop);
mag_interp.vec = mag_orig.vec(:,idx_m_crop);

gps_crop.t = gps_rot.t(idx_g_crop);
gps_crop.vec = gps_rot.vec(:,idx_g_crop);

gps_interp.vec(1,:) = interp1(gps_crop.t, gps_crop.vec(1,:), mag_interp.t, 'pchip');
gps_interp.vec(2,:) = interp1(gps_crop.t, gps_crop.vec(2,:), mag_interp.t, 'pchip');
gps_interp.vec(3,:) = interp1(gps_crop.t, gps_crop.vec(3,:), mag_interp.t, 'pchip');
gps_interp.t = mag_interp.t;

% Common timestamp in GPS time
t_com_interp = mag_interp.t;

figure_name = "GNSS Position - Original and Interpolation (Sanity Check)";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
title(figure_name)
hold on
plot(gps_interp.t - gps_crop.t(1),gps_interp.vec,'.');
plot(gps_crop.t - gps_crop.t(1), gps_crop.vec,'.');
xlabel("Time [s]");ylabel("Position [m]");
grid on
legend

%% Non interpolated data
[idx_g, idx_m] = sync_data_struct(gps_rot.t, mag_orig.t+t_mag_gps, 0.003);

% Get the delay between the measurementstrue
mag.t = mag_orig.t(idx_m);
mag.vec = mag_orig.vec(:,idx_m);

gps.t = gps_rot.t(idx_g);
gps.vec = gps_rot.vec(:,idx_g);

% Common timestamp in GPS time
t_com = gps.t;

% Get RTK2 data synched to the magnetometer data
rtk_gps2_mag_sync.gps_time = rtk_gps2_sync.gps_time(idx_g);
rtk_gps2_mag_sync.enu = rtk_gps2_sync.enu(:,idx_g);
rtk_gps2_mag_sync.rtk_status = rtk_gps2_sync.rtk_status(idx_g);


%% Generate 3 DoF IMU OrientationPlot GPS and Magnetometer Yaw (Sanity Check)
[mag_var_w(1,1),mag_var_w(2,1),mag_var_w(3,1)] = sph2cart((pi/2)+px4_mag_var.dec, px4_mag_var.inc, 1);

disp("Calculate GT Orientation...");
%R_wi = mag_gps_non_linear_lsq(gps.vec, R_pxmag_pximu.' * mag.vec ,R_pximu_vgps * VG_vg, mag_var_w);
R_wi = mag_gps_wahba(gps.vec, R_pxmag_pximu.' * mag.vec ,R_pximu_vgps * VG_vg, mag_var_w);

len_R_wi = length(R_wi);
q_wi = zeros(len_R_wi,4);
for k=1:len_R_wi
    q_wi(k,:) = rotm2quat(R_wi{k});
end

disp("Calculate GT Interpolated Orientation...");
R_wi_interp = mag_gps_wahba(gps_interp.vec, R_pxmag_pximu.' * mag_interp.vec, R_pximu_vgps * VG_vg, mag_var_w);
len_R_wi_interp = length(R_wi_interp);
q_wi_interp = zeros(len_R_wi_interp,4);
for k=1:length(R_wi_interp)
    q_wi_interp(k,:) = rotm2quat(R_wi_interp{k});
end

%% Compare different GT Rotation Methods
if false
    R_wi_whb = mag_gps_wahba(gps.vec, R_pxmag_pximu.' * mag.vec ,R_pximu_vgps * VG_vg, mag_var_w);
    %R_wi_interp = mag_gps_wahba(gps_interp.vec, R_pxmag_pximu.' * mag_interp.vec, R_pximu_vgps * VG_vg, mag_var_w);

    R_wi_nlsq = mag_gps_non_linear_lsq(gps.vec, R_pxmag_pximu.' * mag.vec ,R_pximu_vgps * VG_vg, mag_var_w);
    %R_wi_interp = mag_gps_non_linear_lsq(gps_interp.vec, R_pxmag_pximu.' * mag_interp.vec, R_pximu_vgps * VG_vg, mag_var_w);

    R_wi_lsq = mag_gps_lsq(gps.vec, R_pxmag_pximu.' * mag.vec ,R_pximu_vgps * VG_vg, mag_var_w);
    %R_wi_interp = mag_gps_lsq(gps_interp.vec, R_pxmag_pximu.' * mag_interp.vec, R_pximu_vgps * VG_vg, mag_var_w);

    figure_name = "Orientation Comparison";
    fh1 = figure('Name',figure_name);
    clf(fh1);
    set(fh1, savefigimages.figOptions);
    hold on
    title(figure_name)
    hold on
    title('Orientation (y,p,r)')
    for k=1:length(R_wi_whb)
        R_gpsw_pximu_eul_interp(k,:) = rad2deg(rotm2eul(R_wi_whb{k},'zyx'));
    end
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,1).','DisplayName','wahba_{yaw}')
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,2).','DisplayName','wahba_{pitch}')
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,3).','DisplayName','wahba_{roll}')

    for k=1:length(R_wi_nlsq)
        R_gpsw_pximu_eul_interp(k,:) = rad2deg(rotm2eul(R_wi_nlsq{k},'zyx'));
    end
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,1).','DisplayName','nlsq_{yaw}')
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,2).','DisplayName','nlsq_{pitch}')
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,3).','DisplayName','nlsq_{roll}')

    for k=1:length(R_wi_lsq)
        R_gpsw_pximu_eul_interp(k,:) = rad2deg(rotm2eul(R_wi_lsq{k},'zyx'));
    end
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,1).','DisplayName','lsq_{yaw}')
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,2).','DisplayName','lsq_{pitch}')
    plot(t_com-t_com(1), R_gpsw_pximu_eul_interp(:,3).','DisplayName','lsq_{roll}')

    grid on
    ylabel("Angle [deg]");xlabel("Time [s]");
    legend
end

%% Visualization, Plot Rotation Axis and Vectors for GPS and Mag
if debug
    p_orig = [0,0,0];
    figure
    hold on
    axis equal
    grid on
    view(3)
    xlim([-1.2 1.2])
    ylim([-1.2 1.2])
    zlim([-1.2 1.2])
    xlabel("x");ylabel("y");zlabel("z");
    title("GPS=Magenta, Mag=Black")

    % Plot world axis
    R = eye(3);
    R_x = [p_orig; R(:,1).'];
    R_y = [p_orig; R(:,2).'];
    R_z = [p_orig; R(:,3).'];
    plot3(R_x(:,1),R_x(:,2),R_x(:,3),'r','LineWidth',3);
    plot3(R_y(:,1),R_y(:,2),R_y(:,3),'g','LineWidth',3);
    plot3(R_z(:,1),R_z(:,2),R_z(:,3),'b','LineWidth',3);

    %For Debug Visualization
    for k = 340:length(gps.vec)
        fprintf("Index %i, Time(Sync): %.3f, Time(orig): %.3f\n",k,t_com(k)-t_com(1), t_com(k)-t_com(1) + t_mag_gps);
        if exist("h_mag")
            delete(h_mag);
        end

        if exist("h_gps")
            delete(h_gps);
        end

        if exist("h_r")
            delete(h_r.x);
            delete(h_r.y);
            delete(h_r.z);
        end

        g_vec = [p_orig; gps.vec(:,k).'];
        g_vec = g_vec/norm(g_vec)*0.5;

        m_vec = [p_orig; mag.cart_vec(:,k).'];
        m_vec = m_vec / norm(m_vec) *0.5;

        h_mag = plot3(g_vec(:,1),g_vec(:,2),g_vec(:,3),'m','LineWidth',3);
        h_gps = plot3(m_vec(:,1),m_vec(:,2),m_vec(:,3),'k','LineWidth',3);

        R = R_gpsw_vgps{k};

        % Plot world axis
        R_x = [p_orig; R(:,1).'];
        R_y = [p_orig; R(:,2).'];
        R_z = [p_orig; R(:,3).'];
        h_r.x = plot3(R_x(:,1),R_x(:,2),R_x(:,3),'r');
        h_r.y = plot3(R_y(:,1),R_y(:,2),R_y(:,3),'g');
        h_r.z = plot3(R_z(:,1),R_z(:,2),R_z(:,3),'b');
        pause(0.01)
    end
end

%% RTK 2 based PX4 IMU position
% Position of the IMU based on RTK2, IMU-RTK2 Calibration, and virtual GPS
% orientation
disp("Calculate GT Positon...");

p_gw_g2 = rtk_gps2_mag_sync.enu;
p_i_g2 = p_pximu_rtk2;
R_gv_i = R_pximu_vgps.';

for k=1:length(R_wi)
    R_gw_gv{k} = R_wi{k} * R_pximu_vgps;
end

for k=1:size(rtk_gps2_mag_sync.enu,2)
    p_gpsw_pximu(:,k) = p_gw_g2(:,k) + R_gw_gv{k} * R_gv_i * (-p_i_g2);
end

figure_name = "3D RTK positions and RTK2 based IMU position";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot3(p_gpsw_pximu(1,:),p_gpsw_pximu(2,:),p_gpsw_pximu(3,:),'DisplayName', 'IMU');
plot3(rtk_gps1_select.enu(1,:),rtk_gps1_select.enu(2,:),rtk_gps1_select.enu(3,:),'DisplayName', 'RTK1');
plot3(p_gw_g2(1,:),p_gw_g2(2,:),p_gw_g2(3,:),'DisplayName', 'RTK2');
axis equal
xlabel('X [m]');ylabel('Y [m]');zlabel('Z [m]');
grid on
view(3)
legend

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);

    plot_full_path = fullfile(plot_dir,export_name);
    export_fig(fh1, plot_full_path, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_full_path + ".fig");
end

% Interpolate RTK GPS 2 to higher Magnetometer rate and generate the IMU position
rtk_gps2_select_interp.enu(1,:) = interp1(rtk_gps2_select.gps_time, rtk_gps2_select.enu(1,:), mag_interp.t, 'pchip');
rtk_gps2_select_interp.enu(2,:) = interp1(rtk_gps2_select.gps_time, rtk_gps2_select.enu(2,:), mag_interp.t, 'pchip');
rtk_gps2_select_interp.enu(3,:) = interp1(rtk_gps2_select.gps_time, rtk_gps2_select.enu(3,:), mag_interp.t, 'pchip');
rtk_gps2_select_interp.t = mag_interp.t;

p_gw_g2 = rtk_gps2_select_interp.enu;
p_i_g2 = p_pximu_rtk2;
R_gv_i = R_pximu_vgps.';


for k=1:length(R_wi_interp)
    R_gw_gv{k} = R_wi_interp{k} * R_pximu_vgps;
end

for k=1:size(rtk_gps2_select_interp.enu,2)
    p_gpsw_pximu_interp(:,k) = p_gw_g2(:,k) + R_gw_gv{k} * R_gv_i * (-p_i_g2);
end

figure_name = "3D RTK positions and RTK2 based IMU position (interpolated)";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
plot3(p_gpsw_pximu_interp(1,:),p_gpsw_pximu_interp(2,:),p_gpsw_pximu_interp(3,:),'DisplayName', 'IMU');
plot3(p_gw_g2(1,:),p_gw_g2(2,:),p_gw_g2(3,:),'DisplayName', 'RTK2');
plot3(rtk_gps1_select.enu(1,:),rtk_gps1_select.enu(2,:),rtk_gps1_select.enu(3,:),'DisplayName', 'RTK1');
axis equal
xlabel('X [m]');ylabel('Y [m]');zlabel('Z [m]');
grid on
view(3)
legend('Location','northwest','NumColumns',1)

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);

    plot_full_path = fullfile(plot_dir,export_name);
    export_fig(fh1, plot_full_path, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_full_path + ".fig");
end

%% Final ground truth plots
% IMU Position and Orientation
figure_name = "IMU Position and Orientation Groundtruth";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
subplot(2,1,1)
hold on
title('Position')
plot(t_com-t_com(1), p_gpsw_pximu(1,:),'DisplayName','px4imu_{x}')
plot(t_com-t_com(1), p_gpsw_pximu(2,:),'DisplayName','px4imu_{y}')
plot(t_com-t_com(1), p_gpsw_pximu(3,:),'DisplayName','px4imu_{z}')
grid on
ylabel("Position [m]");xlabel("Time [s]");
legend

subplot(2,1,2)
hold on
title('Orientation (r,p,y)')
for k=1:length(R_wi)
    R_gpsw_pximu_eul(k,:) = rad2deg(rotm2eul(R_wi{k},'xyz'));
end
plot(t_com-t_com(1), R_gpsw_pximu_eul(:,1).','DisplayName','px4imu_{roll}')
plot(t_com-t_com(1), R_gpsw_pximu_eul(:,2).','DisplayName','px4imu_{pitch}')
plot(t_com-t_com(1), R_gpsw_pximu_eul(:,3).','DisplayName','px4imu_{yaw}')
grid on
ylabel("Angle [deg]");xlabel("Time [s]");
legend

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);

    plot_full_path = fullfile(plot_dir,export_name);
    export_fig(fh1, plot_full_path, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_full_path + ".fig");
end

% IMU Position and Orientation Interpolated
figure_name = "IMU Position and Orientation Groundtruth (Interpolated)";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
hold on
title(figure_name)
subplot(2,1,1)
hold on
title('Position')
plot(t_com_interp-t_com_interp(1), p_gpsw_pximu_interp(1,:),'DisplayName','px4imu_{x}')
plot(t_com_interp-t_com_interp(1), p_gpsw_pximu_interp(2,:),'DisplayName','px4imu_{y}')
plot(t_com_interp-t_com_interp(1), p_gpsw_pximu_interp(3,:),'DisplayName','px4imu_{z}')
grid on
ylabel("Position [m]");xlabel("Time [s]");
legend

subplot(2,1,2)
hold on
title('Orientation (r,p,y)')
for k=1:length(R_wi_interp)
    R_gpsw_pximu_eul_interp(k,:) = rad2deg(rotm2eul(R_wi_interp{k},'xyz'));
end
plot(t_com_interp-t_com_interp(1), R_gpsw_pximu_eul_interp(:,1).','DisplayName','px4imu_{roll}')
plot(t_com_interp-t_com_interp(1), R_gpsw_pximu_eul_interp(:,2).','DisplayName','px4imu_{pitch}')
plot(t_com_interp-t_com_interp(1), R_gpsw_pximu_eul_interp(:,3).','DisplayName','px4imu_{yaw}')
grid on
ylabel("Angle [deg]");xlabel("Time [s]");
legend

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);

    plot_full_path = fullfile(plot_dir,export_name);
    export_fig(fh1, plot_full_path, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_full_path + ".fig");
end

%% Map final data
gpsw_pximu_interp.t_gps = t_com_interp.';
gpsw_pximu_interp.p = p_gpsw_pximu_interp;
gpsw_pximu_interp.q = q_wi_interp;

gpsw_pximu.t_gps = t_com.';
gpsw_pximu.p = p_gpsw_pximu;
gpsw_pximu.q = q_wi;

%% Time synchronize ground truth to PX4 IMU measurements

for k = 1:length(gpsw_pximu.t_gps)-1
    qt_0 = Quaternion(gpsw_pximu.q(k,:));
    qt_1 = Quaternion(gpsw_pximu.q(k+1,:));
    q_dt = qt_1 * qt_0.inv;

    w_imu_gt(k,:) = quat2eul(q_dt.double,'xyz')/(gpsw_pximu.t_gps(k+1) - gpsw_pximu.t_gps(k));
end

imu_gt.t = gpsw_pximu.t_gps(1:end-1);
imu_gt.v = w_imu_gt;

imu.t = px4_imu.t;
imu.v = px4_imu.angular_velocity;

ta = imu_gt.t;
%va = vecnorm(imu_gt.v.').';
va = imu_gt.v(:,3);

if use_movmed_for_imu_sync
    va = movmedian(va, 3);
end

va = sgolayfilt(va,2,5);

tb = imu.t.';
vb = imu.v(3,:);
vb = sgolayfilt(vb,2,51);

t_offset = time_reg(ta, va, tb, vb);
t_pximu_imugt = (ta(1)-tb(1)) - t_offset(1);

fprintf("Timeoffset GT to IMU: %.3f[sec.]\n",t_pximu_imugt)

% Plot Result
figure_name = "RTK based IMU groundtruth to IMU measurement Synchronization";
fh1 = figure('Name',figure_name);
clf(fh1);
set(fh1, savefigimages.figOptions);
title(figure_name)

h1 = subplot(2,1,1);
hold on
title("No Synchronization")
plot(tb,vb,'DisplayName','IMU Meas.')
plot(ta,va,'DisplayName','IMU GT')
xlabel("Time [s]");ylabel("yaw rate [rad/s]");
grid on
legend

h2 = subplot(2,1,2);
hold on
title("With Synchronization")
plot(tb+t_pximu_imugt,vb,'DisplayName','IMU Meas.')
plot(ta,va,'DisplayName','IMU GT')
xlabel("Time [s]");ylabel("yaw rate [rad/s]");
grid on
legend

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);

    plot_full_path = fullfile(plot_dir,export_name);
    export_fig(fh1, plot_full_path, savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_full_path + ".fig");
end


%% Apply IMU GPS time offset - t_imu matches the px4 IMU timeframe
gpsw_pximu_interp.t_imu = t_com_interp.' - t_pximu_imugt;
gpsw_pximu.t_imu = t_com.' - t_pximu_imugt;

t_mag_pximu = t_mag_gps - t_pximu_imugt;
px4_mag.t_imu = px4_mag.t + t_mag_pximu;


%% CSV data export GT data
if save_csv

    fprintf("Saving CSV data...\n");

    if ~exist(csv_export_dir, 'dir')
        fprintf("[Info] " + csv_export_dir + " does not exist. Creating folder...\n");
        mkdir(csv_export_dir)
    end

    %% Write time offset file

    fprintf("[Info] " + "Writing time offset yaml file...\n");
    time_filename = fullfile(data_export_dir,"time_info.yaml");
    time_fid = fopen(time_filename, 'wt' );
    fprintf( time_fid, "# Time Info\n");
    fprintf( time_fid, "t_total: %.3f # in seconds\n", gpsw_pximu_interp.t_imu(end) - gpsw_pximu_interp.t_imu(1));
    fprintf( time_fid, "t_begin: %.23f # in seconds\n", gpsw_pximu_interp.t_imu(1));
    fprintf( time_fid, "t_end: %.23f # in seconds\n\n", gpsw_pximu_interp.t_imu(end));
    fprintf( time_fid, "# Time Offsets\n");
    fprintf( time_fid, "t_mag_gps: %.6f # in seconds\n",t_mag_gps);
    fprintf( time_fid, "t_pximu_imugt: %.6f # in seconds\n",t_pximu_imugt);
    fclose(time_fid);

    fprintf("[Info] " + "Writing CSV files and README...\n");

    %% Data limited to the lowest sensor rate
    % Write csv header to the file
    filename = "ground_truth_8hz.csv";
    file_path = fullfile(csv_export_dir, filename);
    fprintf("Writing %s\n", filename);

    header = "t, p_x, p_y, p_z, q_w, q_x, q_y, q_z"; % csv header
    csv_fid = fopen(file_path, 'w+');
    fprintf(csv_fid,'%s\n', header);
    fclose(csv_fid);

    csv_outdoor_gt = [gpsw_pximu.t_imu, gpsw_pximu.p.', gpsw_pximu.q];
    dlmwrite(file_path, csv_outdoor_gt, '-append', 'delimiter', ',', 'precision', 17);

    %% Data with interpolating the lower rate sensor to the higher rate sensor
    % Write csv header to the file
    filename = "ground_truth_80hz.csv";
    file_path = fullfile(csv_export_dir, filename);
    fprintf("Writing %s\n", filename);

    header = "t, p_x, p_y, p_z, q_w, q_x, q_y, q_z"; % csv header
    csv_fid = fopen(file_path, 'w+');
    fprintf(csv_fid,'%s\n', header);
    fclose(csv_fid);

    csv_outdoor_gt_interp = [gpsw_pximu_interp.t_imu, gpsw_pximu_interp.p.', gpsw_pximu_interp.q];
    dlmwrite(file_path, csv_outdoor_gt_interp, '-append', 'delimiter', ',', 'precision', 17);

    %% Store revised data
    fprintf("[Info] " + "Writing CSV files and README...\n");
    rm_fid = fopen(fullfile(csv_export_dir, "README.txt"), 'wt' );

    if ~isempty(px4_imu.t)
        filename = "px4_imu_data.csv";
        file_path = fullfile(csv_export_dir, filename);
        fprintf("Writing %s\n", filename);

        % Write csv header to the file
        header = "t, a_x, a_y, a_z, w_x, w_y, w_z"; % csv header
        csv_fid = fopen(file_path, 'w+');
        %fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);

        % Generate csv data and write to file
        csv_px4_imu = [px4_imu.t.', px4_imu.linear_acceleration.', px4_imu.angular_velocity.'];
        fprintf( rm_fid, "PX4 IMU format: \n\tENU [" + header + "]\n" + sensor_stats_from_csv(csv_px4_imu));
        dlmwrite(file_path, csv_px4_imu, '-append', 'delimiter', ',', 'precision', 17);
    end

    if ~isempty(rtk_gps1.t)
        filename = "rtk_gps1_data_revised.csv";
        file_path = fullfile(csv_export_dir, filename);
        fprintf("Writing %s\n", filename);

        % Write csv header to the file
        header = "t, lat, long, alt, x, y, z, v_x, v_y, v_z"; % csv header
        csv_fid = fopen(file_path, 'w+');
        %fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);

        % Generate csv data and write to file
        %csv_rtk_gps_1 = [rtk_gps1_select.gps_time(:) - t_pximu_imugt, rtk_gps1_select.lat(:), rtk_gps1_select.long(:), rtk_gps1_select.alt(:),rtk_gps1_select.enu.', rtk_gps1_select.vel.'];
        csv_rtk_gps_1 = [rtk_gps1_select.gps_time(:) - t_pximu_imugt, rtk_gps1_select.lat(:), rtk_gps1_select.long(:), rtk_gps1_select.alt(:), rtk_gps1_select.vel.'];

        fprintf( rm_fid, "RTK GPS 1 format:\n\t["+ header +"]\n" + sensor_stats_from_csv(csv_rtk_gps_1));
        dlmwrite(file_path, csv_rtk_gps_1, '-append', 'delimiter', ',', 'precision', 17);
    end

    if ~isempty(rtk_gps2.t)
        filename = "rtk_gps2_data_revised.csv";
        file_path = fullfile(csv_export_dir, filename);
        fprintf("Writing %s\n", filename);

        % Write csv header to the file
        header = "t, lat, long, alt, x, y, z, v_x, v_y, v_z"; % csv header
        csv_fid = fopen(file_path, 'w+');
        %fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);

        % Generate csv data and write to file
        %csv_rtk_gps_2 = [rtk_gps2_select.gps_time(:) - t_pximu_imugt, rtk_gps2_select.lat(:), rtk_gps2_select.long(:), rtk_gps2_select.alt(:),rtk_gps2_select.enu.' , rtk_gps2_select.vel.'];
        csv_rtk_gps_2 = [rtk_gps2_select.gps_time(:) - t_pximu_imugt, rtk_gps2_select.lat(:), rtk_gps2_select.long(:), rtk_gps2_select.alt(:), rtk_gps2_select.vel.'];

        fprintf( rm_fid, "RTK GPS 2 format:\n\t["+ header +"]\n" + sensor_stats_from_csv(csv_rtk_gps_2));
        dlmwrite(file_path, csv_rtk_gps_2, '-append', 'delimiter', ',', 'precision', 17);
    end

    if ~isempty(px4_mag.t)
        filename = "px4_mag_data_revised.csv";
        file_path = fullfile(csv_export_dir, filename);
        fprintf("Writing %s\n", filename);

        % Write csv header to the file
        header = "t, cart_x, cart_y, cart_z, spher_az, spher_el, spher_norm"; % csv header
        csv_fid = fopen(file_path, 'w+');
        fprintf(csv_fid,'%s\n', header);
        fclose(csv_fid);

        % Generate csv data and write to file
        csv_px4_mag = [px4_mag.t_imu.', px4_mag.cart_vec.', px4_mag.az.', px4_mag.el.', px4_mag.norm.'];
        fprintf( rm_fid, "PX4 Mag format:\n\t[" + header + "]\n" + sensor_stats_from_csv(csv_px4_mag));
        dlmwrite(file_path, csv_px4_mag, '-append', 'delimiter', ',', 'precision', 17);
    end

end

if save_ws
    fprintf("[Info] Write Matlab Workspace to File\n");
    save(fullfile(data_export_dir, "outdoor_ground_truth_data.mat"),'gpsw_pximu','gpsw_pximu_interp','rtk_gps1_select','rtk_gps2_select', 'px4_imu', 'px4_mag');
end

%% Function definitions

function selected_data = select_gps_entries(gps_data, select_idx)

selected_data.gps_time = gps_data.gps_time(select_idx);
selected_data.enu = gps_data.enu(:,select_idx);
selected_data.vel = gps_data.vel(:,select_idx);
selected_data.lat = gps_data.lat(:,select_idx);
selected_data.long = gps_data.long(:,select_idx);
selected_data.alt = gps_data.alt(:,select_idx);
selected_data.rtk_status = gps_data.rtk_status(:,select_idx);

end
