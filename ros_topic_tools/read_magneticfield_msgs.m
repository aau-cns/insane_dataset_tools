%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, mag_vector] = read_magneticfield_msgs(bag,topic)
%READ_MAGNETICFIELD_MSGS

disp('Reading Magnetic Field msgs...');

seg_begin = 1;

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);

    if isempty(msgs)
        stamp = [];
        mag_vector = [];
        fprintf("[WARNING] No Messages Found\n");
        return
    end

    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;

    [stamp(seg_begin:seg_end), mag_vector(:,seg_begin:seg_end)] = MagneticFieldSensorMessage(msgs);

    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;

disp('DONE Reading Magnetometer msgs...');

end

function [ stamp, mag_vector ] = MagneticFieldSensorMessage( msgs )

len = length(msgs);

stamp(len) = 0;
mag_vector(:,len) = [0 0 0].';

for k = 1:len
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    mag_vector(:,k) = msgs{1,k}.magnetic_field;
end

end