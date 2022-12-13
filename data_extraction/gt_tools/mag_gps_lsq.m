%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [R] = mag_gps_lsq(gps_w, mag_i, vg_i, mag_w)
%MAG_GPS_LSQ LSG based rotation ground truth generation

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

    %R_lsq = [g_w,m_i,c_w] * inv([g_i,m_w,c_i]);
    R_lsq = [g_w,m_w,c_w] / ([g_i,m_i,c_i]);

    R{k} = R_lsq * (R_lsq.'*R_lsq)^(-1/2);
end

% % In IMU
% figure
% hold on
% title("World")
% axis equal
% grid on
% view(3)
% % Plot frame axis
% R = eye(3);
% p_orig = [0,0,0];
% R_x = [p_orig; R(:,1).'];
% R_y = [p_orig; R(:,2).'];
% R_z = [p_orig; R(:,3).'];
% plot3(R_x(:,1),R_x(:,2),R_x(:,3),'r','LineWidth',3);
% plot3(R_y(:,1),R_y(:,2),R_y(:,3),'g','LineWidth',3);
% plot3(R_z(:,1),R_z(:,2),R_z(:,3),'b','LineWidth',3);
%
% % Plot m_w
%
% Vm_w = [p_orig; m_w.'];
% plot3(Vm_w(:,1),Vm_w(:,2),Vm_w(:,3),'k','LineWidth',3, 'DisplayName','Mag');
%
% Vg_w = [p_orig; g_w.'];
% plot3(Vg_w(:,1),Vg_w(:,2),Vg_w(:,3),'c','LineWidth',3, 'DisplayName','GPS');
%
%
% figure
% title("IMU")
% hold on
% axis equal
% grid on
% view(3)
% % Plot frame axis
% R = eye(3);
% p_orig = [0,0,0];
% R_x = [p_orig; R(:,1).'];
% R_y = [p_orig; R(:,2).'];
% R_z = [p_orig; R(:,3).'];
% plot3(R_x(:,1),R_x(:,2),R_x(:,3),'r','LineWidth',3);
% plot3(R_y(:,1),R_y(:,2),R_y(:,3),'g','LineWidth',3);
% plot3(R_z(:,1),R_z(:,2),R_z(:,3),'b','LineWidth',3);
%
% % Plot m_w
%
% Vm_i = [p_orig; m_i.'];
% plot3(Vm_i(:,1),Vm_i(:,2),Vm_i(:,3),'k','LineWidth',3, 'DisplayName','Mag');
%
% Vg_i = [p_orig; g_i.'];
% plot3(Vg_i(:,1),Vg_i(:,2),Vg_i(:,3),'c','LineWidth',3, 'DisplayName','GPS');
%
% %% Plot Result
% figure
% hold on
% title("World")
% axis equal
% grid on
% view(3)
% % Plot frame axis
% R_o = eye(3);
% p_orig = [0,0,0];
% R_x = [p_orig; R_o(:,1).'];
% R_y = [p_orig; R_o(:,2).'];
% R_z = [p_orig; R_o(:,3).'];
% plot3(R_x(:,1),R_x(:,2),R_x(:,3),'r','LineWidth',3);
% plot3(R_y(:,1),R_y(:,2),R_y(:,3),'g','LineWidth',3);
% plot3(R_z(:,1),R_z(:,2),R_z(:,3),'b','LineWidth',3);
%
% R_wi = R{k};
% p_orig = [0,0,0];
% R_x = [p_orig; R(:,1).'];
% R_y = [p_orig; R(:,2).'];
% R_z = [p_orig; R(:,3).'];
% plot3(R_x(:,1),R_x(:,2),R_x(:,3),'r','LineWidth',3);
% plot3(R_y(:,1),R_y(:,2),R_y(:,3),'g','LineWidth',3);
% plot3(R_z(:,1),R_z(:,2),R_z(:,3),'b','LineWidth',3);
%
% % Plot m_w
%
% Vm_w = [p_orig; m_w.'];
% plot3(Vm_w(:,1),Vm_w(:,2),Vm_w(:,3),'k','LineWidth',3, 'DisplayName','Mag');
%
% Vg_w = [p_orig; g_w.'];
% plot3(Vg_w(:,1),Vg_w(:,2),Vg_w(:,3),'c','LineWidth',3, 'DisplayName','GPS');
