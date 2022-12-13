%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ gps ] = read_ssf_gps( bag )
%READ_SSF_GPS

disp('Reading SSF GPS Position msgs...');

topic = '/position_sensor/gps_position_stamped';
msgs  = bag.readAll(topic);

sec  = zeros(1,length(msgs));
nsec = zeros(1,length(msgs));

gps.stamp = zeros(1,length(msgs));
gps.position = zeros(3,length(msgs));

for k = 1:length(msgs)
    gps.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    gps.position(:,k) = msgs{1,k}.position;
end

disp('DONE Reading SSF GPS Position msgs...');

end

