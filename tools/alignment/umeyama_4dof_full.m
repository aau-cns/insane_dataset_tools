%% Copyright (C) 2022 Alessandro Fornasier, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <alessandro.fornasier@aau.at>

function [R,t] = umeyama_4dof_full(est,gt)

%
%   4 DOF trajectory alignment using full trajectory
%
%   alessandro.fornasier@aau.at
%

% Check dimensions
assert(size(est,1) == size(gt,1) && size(est,2) == size(gt,2), 'Dimensions must match!')

% Flip to correfct size
if size(est,1) == 3 && size(est,2) > 3
    est = est';
end
if size(gt,1) == 3 && size(gt,2) > 3
    gt = gt';   
end

% Compute means
mean_est = mean(est);
mean_gt = mean(gt);

% Compute Rotation
P_gt = gt' - mean_gt';
P_est = est' - mean_est';
P = P_est*P_gt';
theta = atan2(P(1,2)-P(2,1),P(1,1)+P(2,2));
R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

% Compute tranlation
t = mean_gt' - R*(mean_est');

end

