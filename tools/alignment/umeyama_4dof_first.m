%% Copyright (C) 2022 Alessandro Fornasier, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <alessandro.fornasier@aau.at>

function [R,t] = umeyama_4dof_first(est_p, est_q, gt_p, gt_q)

%
%   4 DOF trajectory alignment using first trajectory element
%
%   alessandro.fornasier@aau.at
%

% Check dimensions
assert(size(est_p,1) == size(gt_p,1) && size(est_p,2) == size(gt_p,2), 'Dimensions must match!')
assert(size(est_q,1) == size(gt_q,1) && size(est_q,2) == size(gt_q,2), 'Dimensions must match!')

% Flip to correfct size
if size(est_p,1) == 3 && size(est_p,2) > 3
    est_p = est_p';
end
if size(gt_p,1) == 3 && size(gt_p,2) > 3
    gt_p = gt_p';   
end
if size(est_q,1) == 4 && size(est_q,2) > 4
    est_q = est_q';
end
if size(gt_q,1) == 4 && size(gt_q,2) > 4
    gt_q = gt_q';   
end

% Compute Rotation
R0_gt = quat2rotm(gt_q(1,:));
R0_est = quat2rotm(est_q(1,:));
P = R0_est*R0_gt';
theta = atan2(P(1,2)-P(2,1),P(1,1)+P(2,2));
R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

% Compute tranlation
t = gt_p(1,:)' - R*(est_p(1,:)');

end
