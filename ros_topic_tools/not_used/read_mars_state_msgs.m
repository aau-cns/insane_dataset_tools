%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, p_wi, v_wi, q_wi, r_wi_eul, b_w, b_a, cov_full] = read_mars_state_msgs(bag,topic)
%READ_MAGNETICFIELD_MSGS

disp('Reading Mars States msgs...');
topic = char(topic);
seg_begin = 1;
stamp = [];
p_wi = [];
v_wi = [];
q_wi = [];
r_wi_eul = [];
b_w = [];
b_a = [];
cov_full = [];

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);

    if isempty(msgs)
        fprintf("[WARNING] No Messages Found in Bag " + int2str(k) + "\n");
        continue;
    end

    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;
    
    if isempty(stamp)
        [stamp, p_wi, v_wi, q_wi, r_wi_eul, b_w, b_a, cov_full] = MagneticFieldSensorMessage(msgs);
    else
        [stamp(seg_begin:seg_end), p_wi(:,seg_begin:seg_end),v_wi(:,seg_begin:seg_end),...
        q_wi(:,seg_begin:seg_end),r_wi_eul(:,seg_begin:seg_end), b_w(:,seg_begin:seg_end),b_a(:,seg_begin:seg_end), cov_full(:,:,seg_begin:seg_end)]...
        = MagneticFieldSensorMessage(msgs);
    end

    

    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;
if isempty(stamp)
    fprintf("[WARNING] No Messages Found\n");
end
disp('DONE Reading Mars States msgs...');

end

function [ stamp, p_wi, v_wi, q_wi, r_wi_eul, b_w, b_a, cov_full, cov] = MagneticFieldSensorMessage( msgs )

len = length(msgs);

stamp(len) = 0;
p_wi(:,len) = [0 0 0].';
v_wi(:,len) = [0 0 0].';
q_wi(:,len) = [0 0 0 0].';
r_wi_eul(:,len) = [0 0 0].';
b_w(:,len) = [0 0 0].';
b_a(:,len) = [0 0 0].';
cov_full(:,:,len) = zeros(15,15);

for k = 1:len
    stamp(k) = msgs{1,k}.header.stamp.time;
    p_wi(:,k) = msgs{1,k}.p_wi;
    v_wi(:,k) = msgs{1,k}.v_wi;

    q = cat(1,msgs{1,k}.q_wi);
    q_wi(:,k) = [q(4); q(1:3)];
    r_wi_eul(:,k) = (180/pi) * quat2eul(q_wi(:,k).','xyz').';

    b_w(:,k) = msgs{1,k}.b_w;
    b_a(:,k) = msgs{1,k}.b_a;
    cov_full(:,:,k) = reshape(msgs{1,k}.cov,15,15);
end

end