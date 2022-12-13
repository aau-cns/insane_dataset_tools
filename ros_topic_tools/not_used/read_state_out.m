%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ state ] = read_state_out( bag, topic )
%READ_STATE_OUT

disp('Reading SSF state out msgs...');

msgs  = bag.readAll(topic);

sec  = zeros(1,length(msgs));
nsec = zeros(1,length(msgs));

state_out.stamp = zeros(1,length(msgs));

for k = 1:length(msgs)
    % Time for messages
    state_out.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);

    state_out.position(k,:) = cat(1,msgs{1,k}.data(1:3));
    state_out.velocity(k,:) = cat(1,msgs{1,k}.data(4:6));
    state_out.orientation(k,:) = Quaternion(cat(1,msgs{1,k}.data(7:10)));
    state_out.b_w(k,:) = cat(1,msgs{1,k}.data(11:13));
    state_out.b_a(k,:) = cat(1,msgs{1,k}.data(14:16));

    if length(msgs{1,1}.data) > 16
        state_out.L(k) = cat(1,msgs{1,k}.data(17));
    end

    if length(msgs{1,1}.data) > 17
        state_out.q_wv(k,:) = cat(1,msgs{1,k}.data(18:21));
    end

    if length(msgs{1,1}.data) > 21
        state_out.q_ci(k,:) = cat(1,msgs{1,k}.data(22:25));
    end

    if length(msgs{1,1}.data) > 25
        state_out.p_ci(k,:) = cat(1,msgs{1,k}.data(26:28));
    end

    if length(msgs{1,1}.data) > 28
        state_out.q_mi(k,:) = cat(1,msgs{1,k}.data(29:32));
    end

    if length(msgs{1,1}.data) > 32
        state_out.mag(k,:)  = cat(1,msgs{1,k}.data(33:35));
    end

    if length(msgs{1,1}.data) > 35
        state_out.b_pressure(k)  = cat(1,msgs{1,k}.data(36));
    end

    if length(msgs{1,1}.data) > 36
        state_out.s_pressure(k)  = cat(1,msgs{1,k}.data(37));
    end

    if length(msgs{1,1}.data) > 37
        state_out.p_pi(k,:)  = cat(1,msgs{1,k}.data(38:40));
    end

end

state = state_out;
