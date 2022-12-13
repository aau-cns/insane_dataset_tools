%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [h1] = plot_mag_calibration(raw_cart_vec,corr_cart_vec,mag_name, savefigimages)
%PLOT_MAG_CALIBRATION Summary of this function goes here
%   Detailed explanation goes here

%% Plot States
figure_name = strcat(mag_name, " Intrinsic Calibration Overlay");
h1 = figure('Name',figure_name);
clf(h1);
set(h1, savefigimages.figOptions);
hold on

title(figure_name)

%Plot origin axis
ax_scale = max(abs(raw_cart_vec),[],'all') * 1.5;
axis_linewidth = 6;

plot3([0,ax_scale],[0,0],[0,0], 'r-','LineWidth',axis_linewidth,'DisplayName', 'Origin X-Axis')
plot3([0,0],[0,ax_scale],[0,0], 'g-','LineWidth',axis_linewidth,'DisplayName', 'Origin Y-Axis')
plot3([0,0],[0,0],[0,ax_scale], 'b-','LineWidth',axis_linewidth,'DisplayName', 'Origin Z-Axis')

% Plot non-corrected magnetometer data
plot3(raw_cart_vec(1,:), raw_cart_vec(2,:), raw_cart_vec(3,:), 'b.', 'DisplayName', 'Original');

% Plot corrected magnetometer data
plot3(corr_cart_vec(1,:), corr_cart_vec(2,:), corr_cart_vec(3,:), 'r.', 'DisplayName', 'Corrected');

view(3)
axis equal
grid on
xlabel('x [Tesla]'),ylabel('y [Tesla]'),zlabel('z [Tesla]')
legend

end
