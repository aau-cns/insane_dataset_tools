%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [tag_transform_all] = get_relativ_tag_pose(tag_data)
%GET_RELATIV_TAG_POSE Summary of this function goes here
%
% get available tags for each timeslot and generate a data structure that
% is time independend
% For tag_transform_all{X,Y}{Z} X,Y are the tag ID's, Z is a collection off
% all translations between ID's X and Y
% Translations are only mapped in the fashing of [from,to] [lower_id, higher_id]

verbose = false;

%% Begin data extraction

% Get all tags from a single camera frame and collect an array of relative
% transformations
num_timeslots = size(tag_data,1);

fprintf("Found %i camera frames\n", num_timeslots);


tag_transform_all = cell(1,1);
num_total_translations = 0;
num_skipped_single_tags = 0;

for k = 1:num_timeslots

    % Sort ID's for each timeslot
    tag_poses_unsorted = tag_data(k,:);
    tag_poses_unsorted(cellfun(@isnumeric,tag_poses_unsorted)) = []; % remove empty entries

    num_tag_poses = length(tag_poses_unsorted);

    % Remove entrys with single tags, having only one visible tag does not provide information
    % about relative transformations
    if num_tag_poses <= 1
        num_skipped_single_tags = num_skipped_single_tags+1;
        continue
    end

    % Sort according to ID's acending
    current_ids = zeros(num_tag_poses,1);
    id_order = zeros(num_tag_poses,1);

    % Extract ID's
    for i = 1:num_tag_poses
        current_ids(i) = tag_poses_unsorted{i}.id;
    end

    [~,id_order] = sortrows(current_ids);
    tag_poses_sorted = tag_poses_unsorted(id_order);

    % Build relative tag transformations and store all relative
    % transformations for this timeslot in a global collection
    for i = 1:num_tag_poses-1
        for j = i+1:num_tag_poses

            from_idx=i;
            to_idx=j;

            from_id = tag_poses_sorted{from_idx}.id;
            to_id = tag_poses_sorted{to_idx}.id;

            % Ignore zero ID for now TODO
            if from_id == 0 || to_id == 0
                continue
            end

            if verbose
                [from_id,to_id]
            end

            % Calculate transformations
            p_w_from = tag_poses_sorted{from_idx}.position;
            p_w_to = tag_poses_sorted{to_idx}.position;

            q_w_from = Quaternion(tag_poses_sorted{from_idx}.orientation);
            q_w_to = Quaternion(tag_poses_sorted{to_idx}.orientation);

            % Tag translation p_from_to expresed in frame from
            tag_trans.position = q_w_from.R.' * ((-p_w_from) + p_w_to);
            tag_trans.orientation = Quaternion(q_w_from.inv * q_w_to).double;

            % Attach to existing transformations these ID's [from,to]
            try
                cell_z_index = length(tag_transform_all{from_id,to_id}) + 1;
            catch
                cell_z_index = 1;
            end

            tag_transform_all{from_id, to_id}{cell_z_index} = tag_trans;
            num_total_translations = num_total_translations + 1;

        end
    end

end

fprintf("Number of total relative tag translations: %i\n", num_total_translations);
fprintf("Skipped %i single tag entries\n", num_skipped_single_tags);

end
