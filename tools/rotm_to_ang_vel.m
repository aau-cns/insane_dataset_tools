%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [omega] = rotm_to_ang_vel(r_0,r_1, dt)
%ROTM_TO_ANG_VEL Summary of this function goes here
%   Detailed explanation goes here
omega = (1/dt) * skew_inv(logm(r_0.' * r_1));
end

