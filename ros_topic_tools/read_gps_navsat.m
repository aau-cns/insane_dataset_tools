%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ stamp, latitude, longitude, altitude, cov ] = read_gps_navsat( bag, topic )
%READ_GPS_NAVSAT

disp('Reading NavSat GPS Coordinates msgs...');

topic = char(topic);

seg_begin = 1;

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);

    if isempty(msgs)
        stamp=[];
        latitude=[];
        longitude=[];
        altitude=[];
        cov=[];
        fprintf("[WARNING] No Messages Found\n");
        return
    end

    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;

    [stamp(seg_begin:seg_end), latitude(seg_begin:seg_end),...
        longitude(seg_begin:seg_end), ...
        altitude(seg_begin:seg_end), cov(:,seg_begin:seg_end)] = gps_navsat(msgs);

    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;

disp('DONE Reading NavSat GPS Coordinates msgs...');

end

function [stamp, latitude, longitude, altitude, cov] = gps_navsat(msgs)

len = length(msgs);

stamp(len) = 0;
latitude(len) = 0;
longitude(len) = 0;
altitude(len) = 0;
cov(3,len) = 0;

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    latitude(k) = msgs{1,k}.latitude;
    longitude(k) = msgs{1,k}.longitude;
    altitude(k) = msgs{1,k}.altitude;
    cov(:,k) = diag(reshape(msgs{1,k}.position_covariance,3,3));
end
end