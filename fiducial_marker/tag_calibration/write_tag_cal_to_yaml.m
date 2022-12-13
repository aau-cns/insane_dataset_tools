%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

clear all
close all

dir = "/tmp";

for k = 2:8

    board_no = k;
    filename = sprintf("tagboard_%i.yaml", board_no);
    file_path = fullfile(dir, filename);

    %Tag N Transformation w.r.t. the main tag
    [calibration,ids,main_id] = gen_tag_board_calib(board_no);

    yaml_fid = fopen(file_path, 'w+');

    from_id = main_id;

    for to_id = ids
        if from_id == to_id
            T=eye(4);
        else
            T = quat2rotm(calibration{from_id,to_id}.orientation.double);
            T(1:3,4) = calibration{from_id,to_id}.position;
            T(4,4) = 1;
        end

        fprintf(yaml_fid,'T_%i_%i: ', from_id, to_id);
        fprintf(yaml_fid,"[[ %f, %f, %f, %f],",T(1,:));
        fprintf(yaml_fid,"[ %f, %f, %f, %f],",T(2,:));
        fprintf(yaml_fid,"[ %f, %f, %f, %f],",T(3,:));
        fprintf(yaml_fid,"[ %f, %f, %f, %f]]\n",T(4,:));
        fprintf(yaml_fid,"\n",T(4,:));
    end

    fclose(yaml_fid);
end