%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function plot_decawave_uwb(t,dist,valid)
%PLOT_DECAWAVE_UWB Summary of this function goes here
%   Detailed explanation goes here

if isempty(t)
    fprintf("No UWB Data to Plot")
    return
end

valid = logical(valid);

for k=1:3
    subplot(3,1,k)
    hold on
    title(sprintf("Module %i",k))
    
    plot(t(valid(k,:)), dist(k,valid(k,:)), 'g.', 'DisplayName', "Valid");
    plot(t(~valid(k,:)), dist(k,~valid(k,:)), 'r.', 'DisplayName', "Invalid");
    
    grid on
    legend
    xlabel("Time [s]");ylabel("Distance [m]");
end

end

