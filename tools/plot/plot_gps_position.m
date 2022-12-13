%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [h1,h2,h3,h3d] = plot_gps_position(t, enu, gps_fix_type, rtk_type)
%PLOT_GPS_POSITION Summary of this function goes here
%   Detailed explanation goes here

    [h1,h2,h3,h3d]=plot_position(t(gps_fix_type), enu(:,gps_fix_type));
    
    % Mark RTK 'FIX' Status
    plot3(h3d,enu(1,rtk_type),enu(2,rtk_type),enu(3,rtk_type),'go','MarkerSize',3,'DisplayName', 'RTK Fix');
    plot(h1,t(rtk_type),enu(1,rtk_type),'go','MarkerSize',3,'DisplayName', 'RTK Fix');
    plot(h2,t(rtk_type),enu(2,rtk_type),'go','MarkerSize',3,'DisplayName', 'RTK Fix');
    plot(h3,t(rtk_type),enu(3,rtk_type),'go','MarkerSize',3,'DisplayName', 'RTK Fix');
    
    % Mark RTK 'Float' Status
    plot3(h3d,enu(1,~rtk_type),enu(2,~rtk_type),enu(3,~rtk_type),'ro','MarkerSize',3,'DisplayName', 'RTK Float');
    
    plot(h1,t(~rtk_type),enu(1,~rtk_type),'ro','MarkerSize',3,'DisplayName', 'RTK Float');
    plot(h2,t(~rtk_type),enu(2,~rtk_type),'ro','MarkerSize',3,'DisplayName', 'RTK Float');
    plot(h3,t(~rtk_type),enu(3,~rtk_type),'ro','MarkerSize',3,'DisplayName', 'RTK Float');
    

end

