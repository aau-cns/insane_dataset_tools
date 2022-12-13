%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [traj] = tags_to_trajectory(tag_data, tag_calib_trans_dict, main_tag_id, outlier_std_multiplier)
%TAGS_TO_TRAJECTORY

do_outlier_rejection = false;

fprintf("Processing %i frames\n", size(tag_data,1));

cam_pose.t = [];
cam_pose.position = [];
cam_pose.orientation = [];

% Timestep Loop
for k=1:size(tag_data,1)

    frame_tag_poses = tag_data(k,:);
    frame_tag_poses(cellfun(@isnumeric,frame_tag_poses)) = []; % remove empty entries

    stamp = frame_tag_poses{1}.stamp;

    % Loop over each tag visible during this timestep
    p_tw_c = [];
    q_tw_c = [];
    for i=1:size(frame_tag_poses,2)

        q_c_t = Quaternion(frame_tag_poses{i}.orientation);
        p_c_t = frame_tag_poses{i}.position;
        tag_id = frame_tag_poses{i}.id;

        % Skip zero entries for now
        if tag_id == 0
            continue
        end

        T_tw_t = tag_calib_trans_dict{main_tag_id, tag_id};

        p_tw_t = T_tw_t(1:3,4);
        q_tw_t = Quaternion(T_tw_t);

        p_tw_c(end+1,:) = ( p_tw_t + q_tw_t.R * q_c_t.R.' * (-p_c_t) ).';
        q_tw_c(end+1,:) = Quaternion(q_tw_t * q_c_t.inv).double;

    end

    %% Outlier Rejection
    % Skip outlier detection for single transformations

    trans_vect = p_tw_c;
    quat_vect = q_tw_c;

    if do_outlier_rejection && size(trans_vect,1) ~=1
        % Find outlier for translation and rotation. Then discard
        % transformations where either translation or rotation
        % is out of bounds (oob)

        % Position outlier
        trans_oob = find_position_outlier(trans_vect, outlier_std_multiplier);

        % Orientation outlier
        rot_oob = find_quaternion_outlier(quat_vect, outlier_std_multiplier);

        %% Collect all outlier
        outlier = trans_oob(:,1) | trans_oob(:,2) | trans_oob(:,3) | rot_oob(:,1) | rot_oob(:,2) | rot_oob(:,3);

        % Mean of all translations and rotations for a given, relativ, transformation
        % between tags
        if sum(outlier==0) % Check if all measurements are discarted, if yes, skip entry
            %mean_trans = mean(trans_vect(~outlier,:),1).'
            mean_trans = geometric_median(trans_vect(~outlier,:)).';
            mean_rot = mean_quaternion(quat_vect(~outlier,:)).';
        else
            fprintf("[Warning] All transformations from ID %i to ID %i were rejected because of outlier\n", main_tag_id, tag_id)
            continue
        end

    else
        mean_trans = geometric_median(trans_vect).';
        mean_rot = mean_quaternion(quat_vect);
    end

    cam_pose.t(end+1) = stamp;
    cam_pose.position(end+1,:) = mean_trans;
    cam_pose.orientation(end+1,:) = mean_rot;

end

traj = cam_pose;
