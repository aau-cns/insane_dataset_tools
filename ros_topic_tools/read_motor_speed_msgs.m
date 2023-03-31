%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, motor_speed] = read_motor_speed_msgs( bag, topic )
%READ_MOTOR_SPEED_MSGS

disp('Reading Motor Speed msgs...');

topic = char(topic);

seg_begin = 1;
stamp = [];
motor_speed = [];

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);
    
    if isempty(msgs)
        fprintf("[WARNING] No Messages Found in Bag " + int2str(k) + "\n");
        continue;
    end
    
    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;
    
    if isempty(stamp)
        [stamp, motor_speed] = motor_speed_msgs(msgs);
    else
        [stamp(seg_begin:seg_end), motor_speed(:,seg_begin:seg_end)] = motor_speed_msgs(msgs);
    end
    
    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;
if isempty(stamp)
    fprintf("[WARNING] No Messages Found\n");
end
disp('DONE Reading Motor Speeds msgs...');

end

function [stamp, motor_speed] = motor_speed_msgs(msgs)

len = length(msgs);

stamp(len) = 0;
motor_speed(:,len) = [0 0 0 0].';

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    motor_speed(:,k) = msgs{1,k}.speed.';
end
end