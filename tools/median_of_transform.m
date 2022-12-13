%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [T_out] = median_of_transform(T_in)
%MEDIAN_OF_TRANSFORM Returns the median of a transform, passed as a cell
%structure
%   Position Median and Rotation Mean

[~,len] = size(T_in);
T_out(4,4)=1;

% Get positions
p(3,len) = 0;

for k = 1:len
    p(:,k) = T_in{k}(1:3,4);
end

T_out(1:3,4) = geometric_median(p.').';

% Get rotations
q(4,len) = 0;
for k = 1:len
    q(:,k) = Quaternion(T_in{k}(1:3,1:3)).double;
end

T_out(1:3,1:3) = Quaternion(mean_quaternion(q.')).R;

end

