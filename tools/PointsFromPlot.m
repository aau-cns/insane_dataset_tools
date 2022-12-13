%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

classdef PointsFromPlot < handle
    %RECORD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        points
        handle
    end
    
    methods
        
        function obj = PointsFromPlot(plot_handle)
            
            set(plot_handle,'WindowButtonDownFcn',@obj.plot_callback)
            
            waitfor(plot_handle)
            fprintf("Done")
        end
        
        function plot_callback(obj,src,eventData)
            
            pt = get(gca,'CurrentPoint');
            xline(pt(1,1),'r');
            
            if size(obj.points,2) >=2
                close(src)
                return
            end
            
            obj.points(:,end+1) = [pt(1,1), pt(1,2)].';
            
        end
        
        function points = get_points()
            points = obj.points;
        end
        
    end
end