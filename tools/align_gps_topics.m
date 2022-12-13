%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [rtk_gps] = align_gps_topics(rtk_gps_fix_raw, rtk_gps_vel_raw, rtk_gps_pvt_raw)
%ALIGN_GPS_TOPICS Summary of this function goes here
%   Detailed explanation goes here

if isempty(rtk_gps_fix_raw.t)
    rtk_gps.t = [];
    rtk_gps.gps_time = [];
    rtk_gps.lat = [];
    rtk_gps.long = [];
    rtk_gps.alt = [];
    rtk_gps.enu_cov = [];
    rtk_gps.fix_type = [];
    rtk_gps.rtk_status = [];
    return
end

%% Find common entries and combine GPS fix and PVT Status message
% In the bigger string, find the sequence of the smaller and crop both
% strings accordingly

if length(rtk_gps_fix_raw.lat) > length(rtk_gps_pvt_raw.lat)
    idx = strfind(int32(rtk_gps_fix_raw.lat*1e7), rtk_gps_pvt_raw.lat);
    
    fix_crop = idx(1) : idx(1) + length(rtk_gps_pvt_raw.lat)-1;
    pvt_crop = 1:length(fix_crop);
else
    idx = strfind(rtk_gps_pvt_raw.lat, int32(rtk_gps_fix_raw.lat*1e7));
    
    pvt_crop = idx(1) : idx(1) + length(rtk_gps_fix_raw.lat)-1;
    fix_crop = 1:length(pvt_crop);
end

%% Map croped data
rtk_gps_fix.t = rtk_gps_fix_raw.t(fix_crop);
rtk_gps_fix.lat = rtk_gps_fix_raw.lat(fix_crop);
rtk_gps_fix.long = rtk_gps_fix_raw.long(fix_crop);
rtk_gps_fix.alt =  rtk_gps_fix_raw.alt(fix_crop);
rtk_gps_fix.enu_cov =  rtk_gps_fix_raw.enu_cov(:,fix_crop);

rtk_gps_pvt.gps_time = rtk_gps_pvt_raw.gps_time(pvt_crop);
rtk_gps_pvt.lat = rtk_gps_pvt_raw.lat(pvt_crop);
rtk_gps_pvt.long = rtk_gps_pvt_raw.long(pvt_crop);
rtk_gps_pvt.alt = rtk_gps_pvt_raw.alt(pvt_crop);
rtk_gps_pvt.gps_fix_type =  rtk_gps_pvt_raw.gps_fix_type(pvt_crop);
rtk_gps_pvt.rtk_status =  rtk_gps_pvt_raw.rtk_status(pvt_crop);

% Sanity Check, ensure the allignment was correct
if ~all((int32(rtk_gps_fix.lat*1e7) - int32(rtk_gps_pvt.lat)) == 0)
    fprintf("[Error] Alignment of GPS topics is did not work")
end


%% Map Fix and PVT message to common struct
rtk_gps.t = rtk_gps_fix.t;
rtk_gps.gps_time = rtk_gps_pvt.gps_time;
rtk_gps.lat = rtk_gps_pvt.lat*1e-7;
rtk_gps.long = rtk_gps_pvt.long*1e-7;
rtk_gps.alt = rtk_gps_pvt.alt*1e-3;
rtk_gps.gps_fix_type = rtk_gps_pvt.gps_fix_type;
rtk_gps.rtk_status = rtk_gps_pvt.rtk_status;


%% Find common entries in prepared GPS struct and GPS velocity
% GPS timestamps for the same iTOW are equal. This is done by the ublox GPS
% node. Since the timestamps for corresponding GPS messages is the same, we
% can find the index for each matching timestamp and select the individual
% entries accordingly

[~,gps_idx] = intersect(rtk_gps.t, rtk_gps_vel_raw.t);
[~,gps_vel_idx] = intersect(rtk_gps_vel_raw.t, rtk_gps.t);

rtk_gps.t = rtk_gps.t(gps_idx);
rtk_gps.gps_time = rtk_gps.gps_time(gps_idx);

rtk_gps.lat = rtk_gps.lat(gps_idx);
rtk_gps.long = rtk_gps.long(gps_idx);
rtk_gps.alt = rtk_gps.alt(gps_idx);

rtk_gps.enu_cov = rtk_gps_fix.enu_cov(:,gps_idx);

rtk_gps.gps_fix_type =  rtk_gps.gps_fix_type(gps_idx);
rtk_gps.rtk_status =  rtk_gps.rtk_status(gps_idx);

rtk_gps.vel = rtk_gps_vel_raw.velocity(:,gps_vel_idx);

end
