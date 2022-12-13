%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function posix_sec = gpstime2utc(year,month,day,hour,minutes,sec,nanosec,does_incl_leap_sec)
%GPSTIME2UTC Convert GPS UTC time to Posix seconds
%
% Reference: https://de.mathworks.com/matlabcentral/answers/660293-get-gps-time-to-calulate-time-since-referencje-epoch

% Time to be converted to UTC
t = datetime(year,month,day,hour,minutes,sec,'TimeZone','UTC') + milliseconds(double(nanosec)*1e-6);

if does_incl_leap_sec
    tLS = t;
    
    % GPS time epoch
    GPS0 = datetime(1980,1,6,0,0,0,'TimeZone','UTC');
else
    tLS = t; tLS.TimeZone = 'UTCLeapSeconds';
    
    % GPS time epoch
    GPS0 = datetime(1980,1,6,0,0,0,'TimeZone','UTCLeapSeconds');
end

% Delta time since gps epoch till time t
deltaT = tLS - GPS0; deltaT.Format = 's';

% Current UTC time
t_gps = datetime(1980,1,6) + deltaT;

% Current UTC time in seconds
posix_sec = posixtime(t_gps);
end
