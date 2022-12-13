%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [G,tag_calibrations_trans,tag_calibrations_pose] = generate_tag_calibration(tag_transforms, main_tag_id)
%GENERATE_TAG_CALIBRATION Perform the calibration of a tagvield based on a time series of tag transforations
% For each point in time, transformations of multiple tags should be observed to generate
% a relative transformation. This is done for every point in time. The collection of individuall
% relative translations is then used to generate all tag transformations w.r.t. a choosen main tag.

verbose = false;
do_outlier_rejection = true;
outlier_std_multiplier = 1;
rand_weight = false;
num_transform_paths = 1; % Number of randomly generated path for a single transform calibration


[row_tag_transitions,col_tag_transitions] = size(tag_transforms);

% Ensure that only the uper triangular of the input transformation is
% provided, if not, fix it here
% Translations are only mapped in the form of [from,to] [lower_id, higher_id]

%% Perform calibration for relative Tag rotation and translation

% Average translations for all information on the same relative
% transformation and change the information to homogenious transformations
% Add all given translations to a graph structure

graph_trans_id = [];

for from_id = 1:row_tag_transitions

    for to_id= 1:col_tag_transitions

        if isempty(tag_transforms{from_id,to_id})
            continue
        end

        num_trans = length(tag_transforms{from_id,to_id});
        trans_vect = zeros(num_trans,3);
        quat_vect = zeros(num_trans,4);

        for k = 1:length(tag_transforms{from_id,to_id})
            trans_vect(k,:) = tag_transforms{from_id,to_id}{k}.position;
            quat_vect(k,:) = tag_transforms{from_id,to_id}{k}.orientation;
        end

        % Skip outlier detection for single transformations
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
                fprintf("[Warning] All transformations from ID %i to ID %i were rejected because of outlier\n", from_id, to_id)
                continue
            end

        else
            mean_trans = geometric_median(trans_vect).';
            mean_rot = mean_quaternion(quat_vect);
        end

        % Convert translation and orientation to homogenious transformation
        transform = Quaternion(mean_rot).T;
        transform(1:3,4) = mean_trans(:);
        tag_transform{from_id,to_id} = transform;

        % Store relative translation (association) graph
        graph_trans_id(end+1,:) = [from_id,to_id];

    end
end

% Map source nodes
s = graph_trans_id(:,1);

% Map Target nodes
t = graph_trans_id(:,2);
G = graph(s,t, ones(length(s),1));

% Change the edge weight of for tr
%[loc,~] = ismember(table2array(G.Edges(:,1)), gt_transform_idx, 'rows');
%G.Edges(loc,2) = table(0.01);

figure
p = plot(G, 'NodeLabelMode', 'Auto');
grid on


%% Build translations of each tag w.r.t. a main tag

% Check if main tag ID exist
if ~ismember(main_tag_id, graph_trans_id)
    error("Main Tag ID is not present in given data")
    return
end

for k=unique(graph_trans_id).'

    % Find path for calibration w.r.t. the main tag
    for p = 1:num_transform_paths
        if rand_weight
            G.Edges.Weight = abs(randn(length(G.Edges.Weight),1));
        end
        path = shortestpath(G,main_tag_id,k,'Method','positive');

        %highlight(p,path,'EdgeColor','g')

        if isempty(path)
            warning("Not transformation between tag %i and %i was found", main_tag_id, k);
        end

        % Reset init transformation
        % This also covers the case of the calibration for the master tag to it self
        % P = [0 0 0] and R = Identity
        transform = eye(4);

        % Concatinate path transformations
        for i = 1:length(path)-1

            from_id = path(i);
            to_id = path(i+1);

            % Check direction and adapt (invert) the transformation because an undirected graph is
            % used
            if from_id < to_id
                transform = transform * tag_transform{from_id, to_id};
            else
                transform = transform / tag_transform{to_id, from_id}; % Replace b*inv(A) with b/A
            end

        end
        transform_accum{p} = transform;
    end

    transform = median_of_transform(transform_accum);

    % Write final homogenious transformation
    tag_calibrations_trans{main_tag_id,k} = transform;

    % Write final pose
    pose.position = transform(1:3,4);
    pose.orientation = Quaternion(transform).double;
    tag_calibrations_pose{main_tag_id,k} = pose;
end

end
