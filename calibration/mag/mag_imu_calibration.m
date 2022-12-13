%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [R_hat, inclination] = mag_imu_calibration(mag_data,imu_data)
%MAG_IMU_CALIBRATION Summary of this function goes here
%   Detailed explanation goes here

len_m = length(mag_data);

% Normalize Data Vectors
m = mag_data./vecnorm(mag_data);
g = imu_data./vecnorm(imu_data);

for k = 1:len_m
    A(k,:) = kron(m(:,k), g(:,k)).';
end

Hv = (A' * A) \ A' * ones(len_m,1); %inv(A)*b is equal to A\b
H = reshape(Hv,3,3);

[U,~,V] = svd(H);

U_hat = sign(det(H)) * U;
R_hat = U_hat * V.';

for k = 1:len_m
    s_all(k) = g(:,k).' * R_hat * m(:,k);
end

s_hat = 1/len_m * sum(s_all);

inclination = asin(s_hat);
end

