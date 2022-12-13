%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ gps_coordinates ] = read_gps_coordinates( bag, topic )
%READ_GPS_COORDINATES

disp('Reading GPS coordinates msgs...');

msgs = bag.readAll(topic);
gps_coordinates.stamp = zeros(1,length(msgs));

for k = 1:length(msgs)
    gps_coordinates.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    gps_coordinates.latitude(:,k) = msgs{1,k}.latitude;
    gps_coordinates.longitude(:,k) = msgs{1,k}.longitude;
    gps_coordinates.altitude(:,k) = msgs{1,k}.altitude;
end

disp('DONE Reading GPS coordinates msgs...');

end
