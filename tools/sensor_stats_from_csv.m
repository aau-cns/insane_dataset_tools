%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [info] = sensor_stats_from_csv(csv_data)
%SENSOR_STATS_FROM_CSV Summary of this function goes here
%   Detailed explanation goes here
info = sprintf("\tMeasurements: %i \n\tSensor rate: %.2fHz \n",length(csv_data),length(csv_data)/(csv_data(end,1)-csv_data(1,1)));
end

