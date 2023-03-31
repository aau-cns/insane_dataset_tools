%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [gps_time, gnss_fix_type, rtk_status, latitude, longitude, altitude] = read_ublox_navpvt(bag,topic)
%READ_UBLOX_NAVPVT

% GNSSfix Type status:
% 0: no fix
% 1: dead reckoning only
% 2: 2D-fix
% 3: 3D-fix
% 4: GNSS + dead reckoning combined
% 5: time only fix

% RTK Status
%
%  1: RTK Fix status
%  0: RTK Float Status
% -1: No RTK

disp('Reading ublox NavPVT msgs...');

topic = char(topic);

seg_begin = 1;
gps_time = [];
latitude = [];
longitude = [];
altitude = [];
gnss_fix_type = [];
rtk_status = [];

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);
    
    if isempty(msgs)
        fprintf("[WARNING] No Messages Found in Bag " + int2str(k) + "\n");
        continue;
    end
    
    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;
    
    if isempty(gps_time)
        [gps_time, gnss_fix_type, rtk_status, latitude,...
            longitude, altitude] = ublox_navpvt(msgs);
    else
        [gps_time(seg_begin:seg_end), gnss_fix_type(seg_begin:seg_end), rtk_status(seg_begin:seg_end), ...
            latitude(seg_begin:seg_end), longitude(seg_begin:seg_end), altitude(seg_begin:seg_end)...
            ] = ublox_navpvt(msgs);
    end
    
    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;
if isempty(gps_time)
    fprintf("[WARNING] No Messages Found\n");
end
disp('DONE Reading ublox NavPVT msgs...');

end

function [gps_time, fix_type, rtk_status, latitude, longitude, altitude] = ublox_navpvt(msgs)

% Const Variables
RTK_CARRIER_PHASE_FLOAT = uint8(64);  %0b01000000 - bit 7
RTK_CARRIER_PHASE_FIXED = uint8(128); %0b10000000 - bit 8

len = length(msgs);

gps_time(len) = 0;
latitude(len) = 0;
longitude(len) = 0;
altitude(len) = 0;
fix_type(len) = 0;
rtk_status(len) = 0;

for k = 1:length(msgs)
    
    gps_time(k) = gpstime2utc(msgs{k}.year,msgs{k}.month,msgs{k}.day,msgs{k}.hour,msgs{k}.min,msgs{k}.sec,msgs{k}.nano,true);
    
    if logical(bitand(msgs{k}.flags, RTK_CARRIER_PHASE_FIXED))
        rtk_status(k) = 1; % RTK Fix Status
    elseif logical(bitand(msgs{k}.flags, RTK_CARRIER_PHASE_FLOAT))
        rtk_status(k) = 0; % RTK Float Status
    else
        rtk_status(k) = -1; % None
    end
    
    fix_type(k) = msgs{1,k}.fixType;
    
    latitude(k) = msgs{1,k}.lat;
    longitude(k) = msgs{1,k}.lon;
    altitude(k) = msgs{1,k}.height;
    
end

end
