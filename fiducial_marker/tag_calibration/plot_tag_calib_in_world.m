%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [p_mt_t,id,h] = plot_tag_calib_in_world(G,tag_calibrations_pose,main_tag_id)
%PLOT_TAG_CALIB_IN_WORLD Plot tag calibrations w.r.t. the main tag id
% This associated the graph entries with the pose of each tag w.r.t. the main tag id
% p_mt_tx tag positions (t) expressed in the main tag frame

%% Map in Tag world
for k=1:length(tag_calibrations_pose)
    if isempty(tag_calibrations_pose{main_tag_id,k})
        position = [0,0,0];
    else
        position = tag_calibrations_pose{main_tag_id,k}.position;
    end

    p_mt_t.x(k) = position(1);
    p_mt_t.y(k) = position(2);
    p_mt_t.z(k) = position(3);

    id(k)=k;
end

h = plot(G,'XData',p_mt_t.x,'YData',p_mt_t.y,'ZData',p_mt_t.z);
%highlight(h,path,'EdgeColor','g')
grid on
xlabel("X [m]");ylabel("Y [m]");zlabel("Z [m]");
axis equal

end
