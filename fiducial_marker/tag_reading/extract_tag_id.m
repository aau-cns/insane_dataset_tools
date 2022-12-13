%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [result] = extract_tag_id(tag_id, tag_data)
%EXTRACT_TAG_ID Extract a specific tag from the tag datastructure

local_counter = 1;

for k = 1:length(tag_data)

    tab = table(tag_data(k).data.id);
    idx = find(tab{:,:}==tag_id);

    if isempty(idx)
        continue
    end

    result.t(local_counter,1) = tag_data(k).t;
    result.p(local_counter,:) = tag_data(k).data(idx).pose.translation;
    result.q(local_counter,:) = tag_data(k).data(idx).pose.rotation;

    local_counter = local_counter + 1;
end

end
