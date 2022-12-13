%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [cam_tag_pose, tag_in_world, camera_in_world] = gen_tag_test_data()
%GEN_TAG_TEST_DATA This function returns tag poses seen by a camera troughout a random trajectory

% Generate camera positions
num_cam_pose = 10;
b = 0; m=0;
x = 1:num_cam_pose;
y = b + m*x + 2 * sin(0.6 * x); % Some function for a trajectory
z = ones(1,length(x));

p_w_c = [x;y;z];
q_w_c(1:num_cam_pose) = Quaternion(rpy2r([deg2rad(180) deg2rad(0) deg2rad(0)]));
camera_in_world.position = p_w_c;
camera_in_world.orientation = q_w_c;

plot3(p_w_c(1,:),p_w_c(2,:),p_w_c(3,:),'o-');
hold on
xlabel('x'),ylabel('y'),zlabel('z');
axis equal
grid on

% Define tag positions
num_tags = 20;
s = rng; rng(20); % Store random generator settings, then set seed to 0
rand_upper = max(x); rand_lower=min(x);
t_x = rand_upper + (rand_lower-rand_upper).*rand(1,num_tags);
rand_upper = max(y); rand_lower=min(y);
t_y = rand_upper + (rand_lower-rand_upper).*rand(1,num_tags);
t_z = zeros(1,num_tags);
rng(s); % Restore previous settings of the random generator
plot3(t_x, t_y , t_z, '.','MarkerSize',10,...
    'MarkerEdgeColor','black',...
    'MarkerFaceColor',[1 1 1]);

tag_in_world.position = [t_x; t_y; t_z];
tag_in_world.orientation(1:num_tags) = Quaternion(rpy2r([deg2rad(-10) deg2rad(20) deg2rad(30)]));
tag_in_world.id = 1:num_tags; % the tag should equal the index

tag_in_world.position(:,1) = [0,0,0];
tag_in_world.orientation(1) = Quaternion();

tag_in_world = tag_in_world;

for k=1:num_tags
    text(tag_in_world.position(1,k),tag_in_world.position(2,k),tag_in_world.position(3,k),sprintf("%i",tag_in_world.id(k)))
end


% Determine visible tags, restricted to the x,y plane
max_tag_dist = 2;

for k = 1:length(p_w_c) % for each camera position

    cam_xy = p_w_c(1:2,k);
    tag_xy = tag_in_world.position(1:2,:);

    dist = vecnorm(tag_xy - cam_xy);

    vis_tag(k,:) = dist < max_tag_dist;
end

% Generate cam to tag poses
for k = 1:length(p_w_c)

    % get visible tags
    active_tag_ids = tag_in_world.id(vis_tag(k,:));

    for i = 1:length(active_tag_ids)

        p_w_c_ = p_w_c(:,k);
        r_w_c = q_w_c(k).R;

        r_w_t = tag_in_world.orientation(active_tag_ids(i)).R;
        p_w_t = tag_in_world.position(:,active_tag_ids(i));

        q_c_t = Quaternion(r_w_c.' * r_w_t);
        p_c_t =  r_w_c.' * (-p_w_c_ +  p_w_t);

        tag_info.position = p_c_t;
        tag_info.orientation = q_c_t;

        tag_info.id = active_tag_ids(i);
        tag_info.stamp = k;
        cam_tag_pose{k,i} = tag_info;

        % Draw line from cam frame to tag
        w_P_ct = r_w_c * tag_info.position;
        quiver3(p_w_c_(1),p_w_c_(2),p_w_c_(3),w_P_ct(1),w_P_ct(2),w_P_ct(3),'AutoScale','off');

    end
end

end

