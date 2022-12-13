%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function plot_imu(t_accel,accel,t_gyro,gyro)
%PLOT_IMU Summary of this function goes here
%   Detailed explanation goes here

if isempty(t_accel) || isempty(t_gyro)
    fprintf("No IMU Data to Plot\n")
    return
end

subplot(2,1,1)
hold on
title("Linear Accelectation")
plot(t_accel, accel(1,:), 'DisplayName', 'accel_x')
plot(t_accel, accel(2,:), 'DisplayName', 'accel_y')
plot(t_accel, accel(3,:), 'DisplayName', 'accel_z')
grid on
legend
xlabel("Time [s]");ylabel("m/s^2");
subplot(2,1,2)
hold on
title("Angular Velocity")
plot(t_gyro, gyro(1,:), 'DisplayName', 'gyro_x')
plot(t_gyro, gyro(2,:), 'DisplayName', 'gyro_y')
plot(t_gyro, gyro(3,:), 'DisplayName', 'gyro_z')
grid on
legend
xlabel("Time [s]");ylabel("rad/s");

end

