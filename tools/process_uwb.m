%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [main_dist, inter_dist] = process_uwb(t,id_from,id_to,dist, main_id)

if isempty(t)
    main_dist = [];
    inter_dist = [];
    fprintf("[Warning] No UWB data to process\n");
    return
end

uwb_data.t=t;
uwb_data.dist=dist;
uwb_data.id_from=id_from;
uwb_data.id_to=id_to;

% Get all distances to the main node
main_dist_idx=0;
for k = 1:length(uwb_data.t)
    if uwb_data.id_to(k) == main_id
        main_dist_idx = main_dist_idx+1;
        main_dist.t(main_dist_idx) = uwb_data.t(k);
        main_dist.dist(main_dist_idx) = uwb_data.dist(k);
        main_dist.id_from(main_dist_idx) = uwb_data.id_from(k);
    end
end

% Get distances to the main ancore
source_ids = unique(main_dist.id_from);

for k=1:length(source_ids)
    id_idx = find(main_dist.id_from == source_ids(k));
    
    figure
    hold on
    title(sprintf("Main Dist. ID %i", source_ids(k)))
    plot(main_dist.t(id_idx) - main_dist.t(1),main_dist.dist(id_idx))
    grid on
    fprintf("ID: %i, std:%.2f, mean:%.2f\n",source_ids(k), std(main_dist.dist(id_idx)), mean(main_dist.dist(id_idx)))
end

% Get inter sensor measurements
source_ids = unique([uwb_data.id_from, uwb_data.id_to]);
source_ids(source_ids==main_id) = [];

% Consider all ID's but only store small ID to bigger ID
inter_dist_idx = 0;
for id_from=1:length(source_ids)-1
    for id_to = id_from+1:length(source_ids)
        inter_dist_idx = inter_dist_idx+1;
        
        % Find id combination independend if it is a from or to ID (set of two ID's)
        id_pair_index = find((uwb_data.id_from == source_ids(id_from) & uwb_data.id_to == source_ids(id_to)) );% | (uwb_data.id_to == source_ids(id_from) & uwb_data.id_from == source_ids(id_to)));
        
        inter_dist(inter_dist_idx).id_from = source_ids(id_from);
        inter_dist(inter_dist_idx).id_to = source_ids(id_to);
        inter_dist(inter_dist_idx).t = uwb_data.t(id_pair_index);
        inter_dist(inter_dist_idx).dist = uwb_data.dist(id_pair_index);
    end
end

for k=1:length(inter_dist)
    figure
    hold on
    title(sprintf("Inter Dist. from %i, to %i", inter_dist(k).id_from(1), inter_dist(k).id_to(1) ))
    plot(inter_dist(k).t - inter_dist(k).t(1),inter_dist(k).dist)
    grid on
    fprintf("ID from %i, to %i, std %.2f, mean %.2f\n", inter_dist(k).id_from, inter_dist(k).id_to, std(inter_dist(k).dist), mean(inter_dist(k).dist))
end

end