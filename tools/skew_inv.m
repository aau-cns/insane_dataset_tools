%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [v] = skew_inv(m_skew)
%SKEW_INV Summary of this function goes here
%   Detailed explanation goes here
v(1) = m_skew(3,2);
v(2) = m_skew(1,3);
v(3) = m_skew(2,1);
end

