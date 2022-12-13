%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function plot_imu_3d(t_accel,accel,t_gyro,gyro)
%PLOT_IMU Summary of this function goes here
%   Detailed explanation goes here

if isempty(t_accel) || isempty(t_gyro)
    fprintf("No IMU Data to Plot\n")
    return
end

subplot(1,2,1)
hold on
title("Linear Accelectation")
plot3(accel(1,:),accel(2,:),accel(3,:),'.')
axis equal
maxlim = max(abs([xlim,ylim]));
xlim([-maxlim maxlim]);
ylim([-maxlim maxlim]);
xlabel("x [m/s^2]");ylabel("y [m/s^2]");zlabel("z [m/s^2]");
grid on

subplot(1,2,2)
hold on
title("Angular Velocity")
plot3(gyro(1,:),gyro(2,:),gyro(3,:),'.')
axis equal
maxlim = max(abs([xlim,ylim]));
xlim([-maxlim maxlim]);
ylim([-maxlim maxlim]);
xlabel("x [rad/s]");ylabel("y [rad/s]");zlabel("z [rad/s]");
grid on

end