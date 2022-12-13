%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [outlier] = find_quaternion_outlier(quat_vect,sigma)
%FIND_QUATERNION_OUTLIER Finds outlier outside (sigma*std) in a vector of
%type Quaternion
%   Detailed explanation goes here

% Do not perform outlier detection on data with less than 2 datapoints
if size(quat_vect,1) <= 2
    outlier = zeros(size(quat_vect));
    return
end

rot_mean = mean_quaternion(quat_vect);
rot_mean_euler = tr2rpy(Quaternion(rot_mean).T, 'deg');

for k=1:size(quat_vect,1)
    q_v(k) = Quaternion(quat_vect(k,:));
    q_v_euler(k,:) = tr2rpy(q_v(k).T, 'deg');
end

rot_std_euler = mad(q_v_euler);

% Lower and upper Sigma bound
upper_bound = rot_mean_euler + sigma*rot_std_euler;
lower_bound = rot_mean_euler - sigma*rot_std_euler;

% Determine out of bound entries
outlier = (q_v_euler < lower_bound | q_v_euler > upper_bound);

end