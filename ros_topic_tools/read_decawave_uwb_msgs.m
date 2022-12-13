%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [stamp, dist, valid] = read_decawave_uwb_msgs( bag, topic )
%READ_DECAWAVE_UWB_MSGS

disp('Reading Decawave UWB msgs...');

topic = char(topic);

seg_begin = 1;

for k = 1 : length(bag)
    msgs  = bag(k).readAll(topic);
    
    if isempty(msgs)
        stamp = [];
        dist = [];
        valid = [];
        fprintf("[WARNING] No Messages Found\n");
        return
    end
    
    next_size = length(msgs);
    seg_end = seg_begin + next_size-1;
    
    [stamp(seg_begin:seg_end), dist(:,seg_begin:seg_end),...
        valid(:,seg_begin:seg_end)] = decawave_uwb_msgs(msgs);
    
    seg_begin = seg_begin + next_size;
    fprintf('Done: File %i\n',k)
end
clear msgs;

disp('DONE Reading Decawave UWB msgs...');

end

function [stamp, dist, valid] = decawave_uwb_msgs(msgs)

len = length(msgs);

stamp(len) = 0;
dist(:,len) = [0 0 0 0].';
valid(:,len) = [0 0 0 0].';

for k = 1:length(msgs)
    stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    dist(:,k) = cat(1,msgs{1,k}.distance).';
    valid(:,k) = cat(1,msgs{1,k}.valid).';
end
end