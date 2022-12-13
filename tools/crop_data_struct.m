%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [data] = crop_data_struct(data, idx_begin,idx_end)
%CROP_DATA_STRUCT Summary of this function goes here
%   Detailed explanation goes here

datafield = fieldnames(data);
for iField = 1:numel(datafield)
    Field        = datafield{iField};
    data.(Field) = data.(Field)(:,idx_begin:idx_end);
end

end

