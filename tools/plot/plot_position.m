%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [h1, h2, h3, h4] = plot_position(stamp,position)
%PLOT_MAG Summary of this function goes here
%   Detailed explanation goes here

if isempty(stamp)
    fprintf("No Position Data to Plot\n")
    return
end

[h1, h2, h3, h4] = draw_plot(stamp(1:end), position(:,1:end));

len=length(stamp);

% s = uicontrol('style','slide',...
%     'unit','pix',...
%     'position',[20 10 260 30],...
%     'min',1,'max',len,'val',len,...
%     'sliderstep',[1/len 1/len]);
% 
% addlistener(s,'ContinuousValueChange',@(hObject,event) sl_call(hObject, stamp,position));
end

function [h1,h2,h3,h4]=draw_plot(stamp, position)
% Position over time
h1 = subplot(3,2,1);
cla
hold on
title("Position")
plot(stamp, position(1,:),'.', 'DisplayName', 'x')
grid on
xlabel("Time [s]");ylabel("x [m]");

h2 = subplot(3,2,3);
cla
hold on
plot(stamp, position(2,:),'.', 'DisplayName', 'y')
grid on
xlabel("Time [s]");ylabel("y [m]");

h3 = subplot(3,2,5);
cla
hold on
plot(stamp, position(3,:),'.', 'DisplayName', 'z')
grid on
xlabel("Time [s]");ylabel("z [m]");

% 3D Position
h4 = subplot(3,2,[2,4,6]);
cla
hold on
title("3D Position")
plot3(position(1,:),position(2,:),position(3,:));
view(3)
grid on
axis equal
xlabel("x [m]");ylabel("y [m]");zlabel("z [m]");

end

function [] = sl_call(hObject,stamp,position)
% Callback for slider.
n = round(get(hObject, 'Value'));
draw_plot(stamp(1:n), position(:,1:n));
end
