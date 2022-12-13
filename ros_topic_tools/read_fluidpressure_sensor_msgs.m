%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, fluid_pressure] = read_fluidpressure_sensor_msgs( bag, topic )
%READ_FLUIDPRESSURE_SENSOR_MSGS

disp('Reading Fluid Pressure msgs...');

topic = char(topic);

seg_begin = 1;

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);
    
    if isempty(msgs)
        stamp = [];
        fluid_pressure = [];
        
        fprintf("[WARNING] No Messages Found\n");
        return
    end
    
    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;
    
    [stamp(seg_begin:seg_end), fluid_pressure(:,seg_begin:seg_end)] = fluid_pressure_sensor_msgs(msgs);
    
    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;

disp('DONE Reading Fluid Pressure msgs...');

end

function [stamp, fluid_pressure] = fluid_pressure_sensor_msgs(msgs)
len = length(msgs);

stamp(len) = 0;
fluid_pressure(:,len) = 0;

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    fluid_pressure(k) = cat(1,msgs{1,k}.fluid_pressure);
end

end