%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ time ] = read_gps_time( bag )
%READ_GPS_TIME

disp('Reading GPS time msg...');

%% GPS Position
topic = '/gps_time';
msgs  = bag.readAll(topic);

msg_length = length(msgs);

sec  = zeros(1,msg_length);
nsec = zeros(1,msg_length);

time.stamp = zeros(1,msg_length);

for k = 1:msg_length
    time.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    time.time(k) = double(msgs{1,k}.time_ref.sec) + double(msgs{1,k}.time_ref.nsec) * 1.0e-9;
end

disp('DONE GPS time diff msgs...');

end
