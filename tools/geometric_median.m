%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [geometric_median, valid_solution] = geometric_median(position_vec)
%GEOMETRIC_MEDIAN Calculation of the geometric median
%   The geoetric median requires optimization, thus 'fminsearch' is used to
%   minimize the error definition.

% If only one vector element is given, the median equals this single vector.
% No optimization required, stop early.
[sx,sy] = size(position_vec);
if sx == 1 || sy == 1
    geometric_median = position_vec;
    valid_solution = 1;
    return
end

% Heuristic test have shown that the optimization converges usually at
% 120-200 itterations.
optim_settings = optimset('MaxIter',500,'MaxFunEvals',500, ...
    'TolFun', 1e-8, 'TolX', 1e-8);%,'PlotFcns',@optimplotfval);

% Initial value for the optimization
y0 = median(position_vec);

% Error definition
geoMedianError = @(x,y)(sum(vecnorm(x-y)));
[x,~,exitflag,~] = fminsearch(@(y)geoMedianError(position_vec,y),y0,optim_settings);

valid_solution = exitflag;
geometric_median = x;

end

