%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [R] = mag_gps_wahba(gps_w, mag_i, vg_i, mag_w)
%MAG_GPS_WAHBA Wahba based rotation ground truth generation

for k =1:size(mag_i,2)

    m_w = mag_w(:);
    m_w = m_w ./ norm(m_w);

    m_i = mag_i(:,k);
    m_i = m_i ./ norm(m_i);

    g_w = gps_w(:,k);
    g_w = g_w ./ norm(g_w);

    g_i = vg_i;
    g_i = g_i ./ norm(g_i);

    % Do not normalize the cross products
    c_i = cross(m_i,g_i);
    c_w = cross(m_w,g_w);

    a = 50 * g_w*g_i.' + m_w*m_i.' + c_w*c_i.';

    [U,~,V] = svd(a);

    R{k} = U* diag([1,1,det(U)*det(V)])*V.';

end
