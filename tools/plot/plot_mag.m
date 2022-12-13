%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function plot_mag(t_mag,mag_vect)
%PLOT_MAG Summary of this function goes here
%   Detailed explanation goes here

if isempty(t_mag)
    fprintf("No Position Data to Plot")
    return
end

[mag_az,mag_el,mag_norm] = cart2sph(mag_vect(1,:),mag_vect(2,:),mag_vect(3,:));

mag_el = rad2deg(mag_el);
mag_az = rad2deg(mag_az);

subplot(3,2,1)
hold on
title("Magnetometer (Cartesian)")
plot(t_mag, mag_vect(1,:), 'DisplayName', 'mag_x')
grid on
legend
xlabel("Time [s]");ylabel("Fieldstrength [T]");
subplot(3,2,3)
hold on
plot(t_mag, mag_vect(2,:), 'DisplayName', 'mag_y')
grid on
legend
xlabel("Time [s]");ylabel("Fieldstrength [T]");
subplot(3,2,5)
hold on
plot(t_mag, mag_vect(3,:), 'DisplayName', 'mag_z')
grid on
legend
xlabel("Time [s]");ylabel("Fieldstrength [T]");
subplot(3,2,2)
title("Magnetometer (Spherical)")
hold on
plot(t_mag, mag_el, 'DisplayName', 'elevation')
grid on
legend
xlabel("Time [s]");ylabel("[deg]");
subplot(3,2,4)
hold on
plot(t_mag, mag_az, 'DisplayName', 'azimuth')
grid on
legend
xlabel("Time [s]");ylabel("[deg]");
subplot(3,2,6)
hold on
plot(t_mag, mag_norm, 'DisplayName', 'norm')
grid on
legend
xlabel("Time [s]");ylabel("Fieldstrength [T]");
end

