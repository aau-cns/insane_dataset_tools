%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, id_from, id_to, dist] = read_uwb_bridge_msgs( bag, topic )
%READ_UWB_BRIDGE_MSGS

disp('Reading UWB Bridge msgs...');

topic = char(topic);

seg_begin = 1;
stamp = [];
id_from = [];
id_to = [];
dist = [];

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);
    
    if isempty(msgs)
        fprintf("[WARNING] No Messages Found in Bag " + int2str(k) + "\n");
        continue;
    end
    
    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;
    
    if isempty(stamp)
        [stamp, id_from, id_to, dist] = uwb_bridge_msgs(msgs);
    else
        [stamp(seg_begin:seg_end), id_from(seg_begin:seg_end),...
            id_to(seg_begin:seg_end), ...
            dist(seg_begin:seg_end)] = uwb_bridge_msgs(msgs);
    end

    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;
if isempty(stamp)
    fprintf("[WARNING] No Messages Found\n");
end
disp('DONE UWB Bridge msgs...');

end

function [stamp, id_from, id_to, dist] = uwb_bridge_msgs(msgs)
len = length(msgs);

stamp(len) = 0;
id_from(len) = 0;
id_to(len) = 0;
dist(len) = 0;

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    id_from(k) = msgs{1,k}.ID1;
    id_to(k) = msgs{1,k}.ID2;
    dist(k) = msgs{1,k}.dist;
end
end