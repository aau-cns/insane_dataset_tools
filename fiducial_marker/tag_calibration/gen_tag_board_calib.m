%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [calibration,ids,main_id] = gen_tag_board_calib(board_no)
%GEN_TAG_BOARD_CALIB Generate Board Pose Calibrations w.r.t. a main tag
%   Valid board numbers are between 1 and 8
%
% Generates calibrations in the fashion of Tag N Transformation
% w.r.t. the main tag
%
% IMPORTANT: Tag 4 of board 1 is not rotated by 45 degrees as it should be,
% this is done correct for the other board layouts of this type
%
% The positions of the tags are defined w.r.t the board world frame,
% bottom left corner = [0,0,0]
% The final result which is returned as 'calibration' is expressed w.r.t.
% to a main tag

% Start tag id for each board, this is used to provide the range for each
% board
start_id = [0,15,30,45,60,75,86,97];

% Generate the calibration depending on the board type
switch board_no

    case 1
        % 15 Tags
        num_tags = 15;

        % position of tag n w.r.t. board world frame [0,0,0]
        p_w_n = [11.5, 48.5, 0; %1
            11.5, 11.5, 0; %2
            36.0, 30.0, 0; %3
            60.5, 41.0, 0; %4
            60.5, 11.5, 0; %5
            48.0, 46.0, 0; %6
            48.0, 14.0, 0; %7
            100.0, 30.0, 0; %8
            116.0, 46.0, 0; %9
            116.0, 14.0, 0; %10
            139.5, 48.5, 0; %11
            139.5, 19.0, 0; %12
            164.0, 30.0, 0; %13
            188.5, 48.5, 0; %14
            188.5, 11.5, 0] / 100; %15

        % IMPORTANT: Tag 4 of board 1 is not rotated by 45 degrees
        r_world_yaw = [0,0,45,0,0,0,0,45,0,0,0,45,45,0,0];

    case {2,3,4,5}
        % 15 Tags
        num_tags = 15;

        % position of tag n w.r.t. board world frame [0,0,0]
        p_w_n = [11.5, 48.5, 0; %1
            11.5, 11.5, 0; %2
            36.0, 30.0, 0; %3
            60.5, 41.0, 0; %4
            60.5, 11.5, 0; %5
            48.0, 46.0, 0; %6
            48.0, 14.0, 0; %7
            100.0, 30.0, 0; %8
            116.0, 46.0, 0; %9
            116.0, 14.0, 0; %10
            139.5, 48.5, 0; %11
            139.5, 19.0, 0; %12
            164.0, 30.0, 0; %13
            188.5, 48.5, 0; %14
            188.5, 11.5, 0] / 100; %15

        r_world_yaw = [0,0,45,45,0,0,0,45,0,0,0,45,45,0,0];

    case {6,7,8}
        % 11 Tags
        num_tags = 11;

        % position of tag n w.r.t. board world frame [0,0,0]
        p_w_n = [11.5, 48.5, 0; %1
            11.5, 11.5, 0; %2
            36.0, 30.0, 0; %3
            60.5, 41.0, 0; %4
            60.5, 11.5, 0; %5
            100.0, 30.0, 0; %6
            139.5, 48.5, 0; %7
            139.5, 19.0, 0; %8
            164.0, 30.0, 0; %9
            188.5, 48.5, 0; %10
            188.5, 11.5, 0] / 100; %11

        r_world_yaw = [0,0,45,45,0,0,0,45,45,0,0];

end

% Generate transformations relative to a main tag
main_tag_local_idx = 3;

% Translation of the main tag (m) in the world frame (w)
p_w_m = p_w_n(main_tag_local_idx,:);
% Translation of tag n (n) in the main tag frame (m)
p_m_n = p_w_n - p_w_m;

first_tag_id = start_id(board_no);
last_tag_id = start_id(board_no) + num_tags-1;

ids = [first_tag_id:last_tag_id];
main_id = ids(main_tag_local_idx);

% Rotation of the main tag w.r.t. the board world frame
q_w_m = Quaternion(rpy2r([0,0,r_world_yaw(main_tag_local_idx)],'deg'));

for k = 1:num_tags

    if ids(k) == main_id
        % skip main_id to main_id transform, it does not add information
        calibration{main_id,ids(k)} = [];
    else
        calibration{main_id,ids(k)}.position = p_m_n(k,:);

        % Rotation of tag n w.r.t. the board world frame
        q_w_n = Quaternion(rpy2r([0,0,r_world_yaw(k)],'deg'));
        % Rotation of tag n w.r.t. the main tag
        calibration{main_id,ids(k)}.orientation = q_w_m.inv * q_w_n;
    end

end

end
