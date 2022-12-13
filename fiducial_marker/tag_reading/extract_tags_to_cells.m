%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [tag_cell_data] = extract_tags_to_cells(tag_data)
%EXTRACT_TAGS_TO_CELLS The cell format is used for the tag calibration
% Each row contains all detections for a single time frame

tag_cell_data = cell(length(tag_data),1);

for k = 1:length(tag_data)

    for i=1:length(tag_data(k).data)

        tag.id = tag_data(k).data(i).id;
        tag.position = tag_data(k).data(i).pose.translation;
        tag.orientation = tag_data(k).data(i).pose.rotation;
        tag.stamp = tag_data(k).t;

        tag_cell_data{k,i} = tag;
    end

end
