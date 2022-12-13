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

save_plots = true;  % Store the plots as .png
save_csv = true;    % Export CSV files for the GPS coordinates and ENU data
save_ws = true;     % Save workspace variables for GPS coordinates and ENU data
set_t_zero = true;  % Common timestamp, begining at t=0s

verbose_plots = false;

plot_dir = "./plots/";
data_dir = "./data/";

bagfile_path = './post_data.bag';

ref_marker_id = 102;
max_synch_window = 0.01;
do_mocap_imu_cal = true;
mocap_imu_cal_use_mean = true; % true calculates the mean, false uses the horns method

marker_cam_topic_name = "/fiducial_transforms";
marker_mocap_topic_name = "/tag_board_8/vrpn_client/raw_transform";
vehicle_mocap_topic_name = "/twins_one/vrpn_client/raw_transform";

%% Set calibrations

% Camera calibration
% Pose of the camera expressed in the IMU frame
T_ic=[[-0.99723635 -0.01524964 -0.07271249 -0.25206375]
    [-0.01282577  0.99935016 -0.03368611 -0.05610139]
    [ 0.07317894 -0.03266042 -0.9967839   0.01000359]
    [ 0.          0.          0.          1.        ]];

% Position of the camera (c) w.r.t the IMU frame (i)
p_i_c = T_ic(1:3,4:4);
q_i_c = Quaternion(T_ic);

% Translation of the MoCap object vehicle pose (mv) w.r.t. the IMU frame (i)
% Dataset 1
%p_i_mv = [-0.10 0 0.045].';
%q_i_mv = Quaternion(rpy2r([deg2rad(-2) deg2rad(4) deg2rad(0)]));

% Dataset 8 and 9
p_i_mv = [-0.1 0 0.03].';
q_i_mv = Quaternion(rpy2r([deg2rad(-3) deg2rad(2) deg2rad(3.15)]));

% Translation of the tag (t) on the tag board (tb)
p_tb_t = [0.0;0.0;0];
q_tb_t = Quaternion(rpy2r([deg2rad(0) deg2rad(0) deg2rad(0)]));

%% Read topics
bag = ros.Bag(char(bagfile_path));
bag.info()

% Read Aruco marker pose detected by camera
[tag_data,tags_detected] = read_aruco_rostopic(bag, marker_cam_topic_name);
% Pose of the Marker frame (t) w.r.t. the camera (c)
p_c_t = extract_tag_id(ref_marker_id, tag_data);

% Read the Pose of the tag board (tb) w.r.t the MoCap frame (w) (ground truth)
[t_mocap_marker, p_w_tb, q_w_tb] = read_pose_transform_stamped(bag, marker_mocap_topic_name);
% Read the Pose of the MoCap vehicle body (mv) w.r.t the MoCap frame (w) (ground truth)
[t_mocap_vehicle, p_w_mv, q_w_mv] = read_pose_transform_stamped(bag, vehicle_mocap_topic_name);

if verbose_plots

    % Plot marker detection
    % Position
    figure
    hold on
    title('Position of the camera, w.r.t. the tag')
    plot(p_c_t.t, p_c_t.p(:,1),'-');
    plot(p_c_t.t, p_c_t.p(:,2),'-');
    plot(p_c_t.t, p_c_t.p(:,3),'-');
    xlabel("t [s]"), ylabel("Position [m]")
    grid on

    % Orientation
    for k = 1:length(p_c_t.q)
        r_c_m_eul(k,:) = tr2rpy(Quaternion(p_c_t.q(k,:)).T,'deg');
    end

    figure
    hold on
    plot(p_c_t.t, r_c_m_eul(:,1), 'Displayname', 'Roll')
    plot(p_c_t.t, r_c_m_eul(:,2), 'Displayname', 'Pitch')
    plot(p_c_t.t, r_c_m_eul(:,3), 'Displayname', 'Yaw')
    legend
    xlabel("t [s]"), ylabel("Orientation [deg]")
    grid on

    figure
    hold on
    title('Marker detection')
    plot(p_c_t.t, p_c_t.p(:,1),'-');
    plot(p_c_t.t, p_c_t.p(:,2),'-');
    plot(p_c_t.t, p_c_t.p(:,3),'-');
    xlabel("t [s]"), ylabel("Position [m]")
    grid on

    % Plot vehicle data
    figure
    hold on
    title('Vehicle Pose in MoCap')
    plot(t_mocap_vehicle, p_w_mv(1,:),'-');
    plot(t_mocap_vehicle, p_w_mv(2,:),'-');
    plot(t_mocap_vehicle, p_w_mv(3,:),'-');
    xlabel("t [s]"), ylabel("Position [m]")
    grid on

    figure
    hold on
    title('Marker Pose in MoCap')
    plot(t_mocap_marker, p_w_tb(1,:),'-');
    plot(t_mocap_marker, p_w_tb(2,:),'-');
    plot(t_mocap_marker, p_w_tb(3,:),'-');
    xlabel("t [s]"), ylabel("Position [m]")
    grid on

    figure
    hold on
    title('3D Marker and Vehicle Pose in MoCap')
    plot3(p_w_mv(1,:),p_w_mv(2,:),p_w_mv(3,:),'DisplayName', 'Vehicle Body')
    plot3(p_w_tb(1,:),p_w_tb(2,:),p_w_tb(3,:), 'DisplayName', 'Marker')
    xlabel("X [m]"), ylabel("Y [m]"), zlabel("Z [m]")
    view(3)
    axis equal
    grid on
    legend

end

%% Sync all data to the set with the least data points

% Store pre-synch data for later comparison
pre_sync_t_cam_marker = p_c_t.t;
pre_sync_p_c_t = p_c_t.p;
pre_sync_t_mocap_marker = t_mocap_marker;
pre_sync_p_w_tb = p_w_tb;
pre_sync_t_mocap_vehicle = t_mocap_vehicle;
pre_sync_p_w_mv = p_w_mv;

% Sync MoCap messages
t_data_a = t_mocap_marker;
t_data_b = t_mocap_vehicle;
synch_match_b_to_a = true;

if length(t_data_a) <= length(t_data_b)
    fprintf('Synching data B to A...\n');
    synch_match_b_to_a = true;

    data_base = t_data_a;
    data_sync = t_data_b;
else
    fprintf('Synching data A to B...\n');
    synch_match_b_to_a = false;

    data_base = t_data_b;
    data_sync = t_data_a;
end

sync_idx=[];
base_idx=[];
for k = 1:length(data_base)
    delta = abs(data_sync - data_base(k));
    if min(delta) > max_synch_window
        continue
    end
    b = find(delta == min(delta));
    sync_idx(end+1)=  b(1);
    base_idx(end+1) = k;
end
fprintf("Discarded %i measurements due to specified synch window of %.4f sec.\n", k-length(base_idx), max_synch_window);

if synch_match_b_to_a
    a_idx = base_idx;
    b_idx = sync_idx;

    % Data A remains original
    t_mocap_marker = t_mocap_marker(a_idx);
    p_w_tb = p_w_tb(:,a_idx);
    q_w_tb = q_w_tb(:,a_idx);

    % Data B will be reduced
    t_mocap_vehicle = t_mocap_vehicle(b_idx); % Use same timestamps t_b = t_a
    %t_mocap_vehicle = t_mocap_vehicle(sync_idx); % Use original timestamps
    p_w_mv = p_w_mv(:,b_idx);
    q_w_mv = q_w_mv(b_idx);

else

    a_idx = base_idx;
    b_idx = sync_idx;

    % Data A remains original
    t_mocap_marker = t_mocap_marker(a_idx);
    p_w_tb = p_w_tb(:,a_idx);
    q_w_tb = q_w_tb(:,a_idx);

    % Data B will be reduced
    t_mocap_vehicle = t_mocap_marker(b_idx); % Use same timestamps t_b = t_a
    %t_mocap_vehicle = t_mocap_vehicle(sync_idx); % Use original timestamps
    p_w_mv = p_w_mv(:,b_idx);
    q_w_mv = q_w_mv(b_idx);

end

% Synchronize marker to MoCap messages

t_data_a = t_mocap_marker(:);
t_data_b = p_c_t.t(:);
synch_match_b_to_a = true;

if length(t_data_a) <= length(t_data_b)
    fprintf('Synching data B to A...\n');
    synch_match_b_to_a = true;

    data_base = t_data_a;
    data_sync = t_data_b;
else
    fprintf('Synching data A to B...\n');
    synch_match_b_to_a = false;

    data_base = t_data_b;
    data_sync = t_data_a;
end

clear sync_idx
sync_idx = [];
base_idx = [];
for k = 1:length(data_base)
    delta = abs(data_sync - data_base(k));
    if min(delta) > max_synch_window
        continue
    end
    b = find(delta == min(delta));
    sync_idx(end+1)=  b(1);
    base_idx(end+1) = k;
end
fprintf("Discarded %i measurements due to specified synch window of %.4f sec.\n", k-length(base_idx), max_synch_window);


if synch_match_b_to_a
    a_idx = base_idx;
    b_idx = sync_idx;

    % Data A remains original
    t_mocap_marker = t_mocap_marker(a_idx);
    p_w_tb = p_w_tb(:,a_idx);
    q_w_tb = q_w_tb(:,a_idx);

    t_mocap_vehicle = t_mocap_vehicle(a_idx); % Use same timestamps t_b = t_a
    %t_mocap_vehicle = t_mocap_vehicle(sync_idx); % Use original timestamps
    p_w_mv = p_w_mv(:,a_idx);
    q_w_mv = q_w_mv(a_idx);

    % Data B will be reduced
    p_c_t.t = t_mocap_marker(b_idx); % Use same timestamps t_b = t_a
    %p_c_m.t = p_c_m.t(sync_idx); % Use original timestamps
    p_c_t.p =  p_c_t.p(:,b_idx);
    p_c_t.q =  p_c_t.q(:,b_idx);

else
    a_idx = sync_idx;
    b_idx = base_idx;

    % Data A remains original
    t_mocap_marker = t_mocap_marker(a_idx);
    p_w_tb = p_w_tb(:,a_idx);
    q_w_tb = q_w_tb(:,a_idx);

    t_mocap_vehicle = t_mocap_vehicle(a_idx); % Use same timestamps t_b = t_a
    %t_mocap_vehicle = t_mocap_vehicle(sync_idx); % Use original timestamps
    p_w_mv = p_w_mv(:,a_idx);
    q_w_mv = q_w_mv(a_idx);

    % Data B will be reduced
    p_c_t.t = p_c_t.t(b_idx); % Use same timestamps t_b = t_a
    %p_c_m.t = p_c_m.t(sync_idx); % Use original timestamps
    p_c_t.p =  p_c_t.p(b_idx,:);
    p_c_t.q =  p_c_t.q(b_idx,:);

end

% Sanity Check, ensure all timestamps are the same
% if sum(p_c_m.t(:) - t_mocap_vehicle(:)) && sum(p_c_m.t(:) - t_mocap_marker(:))
%     fprintf("Synched Timestamps do not Match")
%     return
% end

%% Set time vector
% Timestamp, t_0 staring at zero seconds
% Ensure vertical vector with (:)
t_zero = (t_mocap_vehicle(:)-t_mocap_vehicle(1));
% Timestamp, original recording stamps
t_orig = t_mocap_vehicle(:);

if set_t_zero
    t_plot = t_zero;
else
    t_plot = t_orig;
end

figure
hold on
title("All data streams before Synchronization")
plot(pre_sync_t_cam_marker, pre_sync_p_c_t, 'DisplayName','Marker in Camera')
plot(pre_sync_t_mocap_marker, pre_sync_p_w_tb, 'DisplayName','MoCap Marker Board')
plot(pre_sync_t_mocap_vehicle, pre_sync_p_w_mv, 'DisplayName','MoCap Vehicle')
legend
grid on

figure
hold on
title("All data streams after Synchronization")
plot(t_plot, p_c_t.p, 'DisplayName','Marker in Camera')
plot(t_plot, p_w_tb, 'DisplayName','MoCap Marker Board')
plot(t_plot, p_w_mv, 'DisplayName','MoCap Vehicle')
legend
grid on


if do_mocap_imu_cal
    % Vehicle MoCap object to IMU calibration
    for k = 1:length(t_plot)
        r_i_c = q_i_c.R;
        r_c_t = Quaternion(p_c_t.q(k,:)).R;
        r_tb_t = q_tb_t.R;
        r_w_tb = q_w_tb(k).R;

        % Position
        p_w_i_trough_marker(:,k) = p_w_tb(:,k) + (r_w_tb * p_tb_t) + (r_w_tb * r_tb_t * r_c_t.' * (-p_c_t.p(k,:).')) + (r_w_tb * r_tb_t * r_c_t.' * r_i_c.' * (-p_i_c));

        % Orientation
        q_w_i_trough_marker(k) = Quaternion(r_w_tb * r_tb_t * r_c_t.' * r_i_c.');
        q_w_i_trough_marker_euler(:,k) = tr2rpy(q_w_i_trough_marker(k).T,'deg').';

        q_w_mv_euler(:,k) = tr2rpy(q_w_mv(k).T,'deg').';

        q_i_mv_cal(k) = q_w_i_trough_marker(k).inv * q_w_mv(k);
        q_i_mv_cal_double(k,:) = Quaternion(q_w_i_trough_marker(k).inv * q_w_mv(k)).double;
        q_i_mv_cal_euler(:,k) = tr2rpy(q_i_mv_cal(k).T,'deg').';
    end

    % Overwrite Calibration for Vehicle MoCap object and IMU transformation
    if mocap_imu_cal_use_mean
        p_i_mv = mean((p_w_mv - p_w_i_trough_marker).').';
        q_i_mv = Quaternion(mean_quaternion(q_i_mv_cal_double));
    else
        [p_i_mv, R_i_vb_cal] = transform_between_trajectories(p_w_i_trough_marker.',p_w_mv.');
        q_i_mv = Quaternion(R_i_vb_cal);
    end

    fprintf("p_i_mv calibration(x,y,z)[m]: %f %f %f\n",p_i_mv)
    fprintf("q_i_mv calibration(w,x,y,z): %f %f %f %f\n",q_i_mv.double)
    fprintf("q_i_mv calibration(rpy)[deg]: %f %f %f\n",tr2rpy(q_i_mv.T,'deg'))

    figure
    subplot(2,2,1)
    % Rotation Calibration
    hold on
    title(sprintf('Rotation of MoCap Vehicle Object\n and Vehicle IMU determined trough Marker'))
    plot(t_plot, q_w_i_trough_marker_euler(1,:),'displayname','x_{imu}')
    plot(t_plot, q_w_i_trough_marker_euler(2,:),'displayname','y_{imu}')
    plot(t_plot, q_w_i_trough_marker_euler(3,:),'displayname','z_{imu}')

    plot(t_plot, q_w_mv_euler(1,:),'displayname','x_{mocap}')
    plot(t_plot, q_w_mv_euler(2,:),'displayname','y_{mocap}')
    plot(t_plot, q_w_mv_euler(3,:),'displayname','z_{mocap}')
    legend
    grid on

    subplot(2,2,3)
    hold on
    title(sprintf('Rotation Calibration between MoCap Vehicle Object\n and Vehicle IMU determined trough Marker'))
    plot(t_plot, q_i_mv_cal_euler(1,:),'displayname','x')
    plot(t_plot, q_i_mv_cal_euler(2,:),'displayname','y')
    plot(t_plot, q_i_mv_cal_euler(3,:),'displayname','z')

    r_i_mv_cal_euler = tr2rpy(Quaternion(q_i_mv).T,'deg');
    yline(r_i_mv_cal_euler(1));
    yline(r_i_mv_cal_euler(2));
    yline(r_i_mv_cal_euler(3));
    legend
    grid on


    % Translation Calibration
    subplot(2,2,2)
    hold on
    title(sprintf('Translations of MoCap Vehicle Object\n and Vehicle IMU determined trough Marker'))
    plot(t_plot, p_w_i_trough_marker(1,:),'displayname','x_imu')
    plot(t_plot, p_w_i_trough_marker(2,:),'displayname','y_imu')
    plot(t_plot, p_w_i_trough_marker(3,:),'displayname','z_imu')

    plot(t_plot, p_w_mv(1,:),'displayname','x_mocap')
    plot(t_plot, p_w_mv(2,:),'displayname','y_mocap')
    plot(t_plot, p_w_mv(3,:),'displayname','z_mocap')
    legend
    grid on

    subplot(2,2,4)
    hold on
    title(sprintf('Translation Calibration of MoCap Vehicle Object\n and Vehicle IMU determined trough Marker'))
    p_imu_mocap_diff = (p_w_i_trough_marker - p_w_mv);
    plot(t_plot, p_imu_mocap_diff(1,:),'displayname','x')
    plot(t_plot, p_imu_mocap_diff(2,:),'displayname','y')
    plot(t_plot, p_imu_mocap_diff(3,:),'displayname','z')

    yline(p_i_mv(1));
    yline(p_i_mv(2));
    yline(p_i_mv(3));
    legend
    grid on

    % MoCap object trough imu by adding the IMU MoCap vehicle calibration
    for k = 1:length(t_plot)

        % Position
        r_i_c = q_i_c.R;
        r_c_t = Quaternion(p_c_t.q(k,:)).R;
        r_tb_t = q_tb_t.R;
        r_w_tb = q_w_tb(k).R;
        r_i_mv = q_i_mv.R;

        p_w_mv_trough_marker(:,k) = p_w_i_trough_marker(:,k) + (r_w_tb * r_tb_t * r_c_t.' * r_i_c.') * p_i_mv;

        % Orientation
        q_w_mv_trough_marker(k) = Quaternion(r_w_tb * r_tb_t * r_c_t.' * r_i_c.' * r_i_mv);
    end

    % MoCap Vehicle calibration error
    figure('Name','MoCap Vehicle Object to IMU Calibration','NumberTitle','off')
    subplot(2,1,1)
    %Position
    hold on
    title('Translation Error of Vehicle MoCap alignment')

    p_w_mv_error = p_w_mv_trough_marker - p_w_mv;

    plot(t_plot, p_w_mv_error(1,:),'displayname','x')
    plot(t_plot, p_w_mv_error(2,:),'displayname','y')
    plot(t_plot, p_w_mv_error(3,:),'displayname','z')
    grid on
    legend

    % Rotation
    subplot(2,1,2)
    hold on
    title('Rotation Error of Vehicle MoCap alignment')

    for k=1:length(q_w_mv)
        q_w_mv_error = q_w_mv_trough_marker(k).inv * q_w_mv(k);
        q_w_mv_error_euler(:,k) = tr2rpy(q_w_mv_error.T,'deg');
    end

    plot(t_plot, q_w_mv_error_euler(1,:),'displayname','x')
    plot(t_plot, q_w_mv_error_euler(2,:),'displayname','y')
    plot(t_plot, q_w_mv_error_euler(3,:),'displayname','z')
    grid on
    legend
end

%% Generate projected marker positions
% Transform the Aruco tag,  into world coordinates
% Sync aruco tag reading against mocap

% Determine marker groundtruth, marker board with additional marker
% calibration (offset) expressed in MoCap world
for k=1:length(t_plot)
    p_w_t_gt(:,k) = p_w_tb(:,k) + q_w_tb(k).R * p_tb_t;
    q_w_t_gt(k) = q_w_tb(k) * q_tb_t;
end

% Calculate transformation of marker seen by the camera into MoCap world
for k = 1:length(t_plot)
    p_w_t_trough_cam(:,k) = p_w_mv(:,k) + q_w_mv(k).R * q_i_mv.R.' * ( (-p_i_mv) + p_i_c + ( q_i_c.R * p_c_t.p(k,:).') );
    q_w_t_trough_cam(k) = Quaternion(q_w_mv(k).R * q_i_mv.R.' * q_i_c.R * Quaternion(p_c_t.q(k,:)).R);
end

%% Translation evaluation
figure
hold on
title('Tag Position, Camera Based and Groundtruth')
plot(t_plot, p_w_t_trough_cam(1,:),'displayname','t-cam_x')
plot(t_plot, p_w_t_trough_cam(2,:),'displayname','t-cam_y')
plot(t_plot, p_w_t_trough_cam(3,:),'displayname','t-cam_z')
plot(t_plot, p_w_t_gt(1,:),'displayname','t-gt_x')
plot(t_plot, p_w_t_gt(2,:),'displayname','t-gt_y')
plot(t_plot, p_w_t_gt(3,:),'displayname','t-gt_z')
legend
grid on

figure('Name','Visual Tag Error Evaluation','NumberTitle','off')
subplot(2,1,1)
hold on
title('Tag Position Error (meas - gt)')
plot(t_plot, p_w_t_trough_cam(1,:) - p_w_t_gt(1,:),'displayname','x')
plot(t_plot, p_w_t_trough_cam(2,:) - p_w_t_gt(2,:),'displayname','y')
plot(t_plot, p_w_t_trough_cam(3,:) - p_w_t_gt(3,:),'displayname','z')
legend
grid on

%% Dataset information
fprintf("Dataset time: %.2f sec., samples: %i\n", t_plot(end)-t_plot(1), length(t_plot))

fprintf("Position\n")
% Standard deviation position
fprintf("Tag STD: %.3f %.3f %.3f\n", std(p_w_t_trough_cam.'))
% RMSE
fprintf("Tag RMSE: %.3f %.3f %.3f\n", sqrt(mean(((p_w_t_trough_cam.' - p_w_t_gt.').^2))))

% Tag Rotation Error
for k = 1:length(t_plot)
    q_error(k) = q_w_t_trough_cam(k) * q_w_t_gt(k).inv;
    q_w_t_euler(:,k) = tr2rpy(q_error(k).T,'deg');

    %q_w_m_trough_cam_euler(:,k) = tr2rpy(q_w_t_trough_cam(k).T,'deg');
    %q_w_m_gt_euler(:,k) = tr2rpy(q_w_t_gt(k).T,'deg');
end

subplot(2,1,2)
hold on
title('Tag Rotation Error (meas - gt)')
plot(t_plot, q_w_t_euler(1,:),'displayname','x')
plot(t_plot, q_w_t_euler(2,:),'displayname','y')
plot(t_plot, q_w_t_euler(3,:),'displayname','z')
legend
grid on

fprintf("Orientation\n")
% Standard deviation
fprintf("Tag STD: %.3f %.3f %.3f\n", std(q_w_t_euler.'))
% RMSE
fprintf("Tag RMSE: %.3f %.3f %.3f\n", sqrt(mean(((q_w_t_euler.').^2))))
