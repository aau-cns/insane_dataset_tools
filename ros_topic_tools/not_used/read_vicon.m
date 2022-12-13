%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ output_args ] = read_vicon( bag, topic )
%READ_VICON

disp('Reading vicon msgs...');

msgs  = bag.readAll(topic);

sec  = zeros(1,length(msgs));
nsec = zeros(1,length(msgs));

vicon.stamp = zeros(1,length(msgs));
vicon.position = zeros(3,length(msgs));

for k = 1:length(msgs)
    vicon.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    vicon.position(:,k) = msgs{1,k}.transform.translation;

    tmp_quat = msgs{1,k}.transform.rotation;

    vicon.orientation(k) = Quaternion([tmp_quat(4),tmp_quat(1),tmp_quat(2),tmp_quat(3)]);
end

output_args = vicon;
disp('DONE Reading vicon msgs...');

end
