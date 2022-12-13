%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [outlier] = find_position_outlier(position_vec,sigma)
%FIND_POSITION_OUTLIER Finds outlier outside (sigma*mad) median absolute deviation in a nx3 position vector
%   This function uses the geometric median and the absolute median
%   deviation to find outlier. This method is more resilient against
%   outliers in the data.

% Do not perform outlier detection on data with less than 2 datapoints
if size(position_vec,1) <= 2
    outlier = zeros(size(position_vec));
    return
end

%% Position outlier
% Geometric median
position_med = geometric_median(position_vec);

% Median absolute deviation
position_mad = mad(position_vec);

% Lower and upper Sigma bound
upper_bound = position_med + sigma*position_mad;
lower_bound = position_med - sigma*position_mad;

% Determine out of bound entries
outlier = (position_vec <= lower_bound | position_vec >= upper_bound);

end
