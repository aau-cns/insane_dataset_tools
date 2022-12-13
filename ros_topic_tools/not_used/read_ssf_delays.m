%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ delay ] = read_ssf_delays( bag, topic )
%READ_SSF_DELAYS

disp('Reading SSF delay msgs...');

%% GPS Position
msgs  = bag.readAll(topic);

msg_length = length(msgs);

sec  = zeros(1,msg_length);
nsec = zeros(1,msg_length);

delay.stamp = zeros(1,msg_length);

for k = 1:msg_length
    delay.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    delay.delay(k) = msgs{1,k}.data;
end

disp('DONE SSF delay msgs...');

end
