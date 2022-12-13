%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [h] = plot_position_std(t,std)
%PLOT_POSITION_STD Summary of this function goes here
%   This function receives a 3x3xN matrix of standard deviations and plots the std accordingly
h = gca;
hold on
plot(h, t, squeeze(std(1,:)),'DisplayName', 'x');
plot(h, t, squeeze(std(2,:)),'DisplayName', 'y');
plot(h, t, squeeze(std(3,:)),'DisplayName', 'z');

ylim([0 inf]);
grid on

xlabel("Time [s]");ylabel("STD [m]");

legend

end

