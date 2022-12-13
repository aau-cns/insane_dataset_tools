%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [R] = mag_gps_non_linear_lsq(gps_w, mag_i, vg_i, mag_w)
%MAG_GPS_NON_LINEAR_LSQ LSQ based rotation ground truth generation

options = optimoptions('lsqnonlin','SpecifyObjectiveGradient',true,'Display','off');

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

    z = [g_w;m_w;c_w];
    A = [g_i;m_i;c_i];

    % Initial guess based on linear least squares (Wahabas problem)
    llsq = 3*g_w*g_i.' + m_w*m_i.' + c_w*c_i.';
    [U,~,V] = svd(llsq);
    R_llsq = U* diag([1,1,det(U)*det(V)])*V.';

    x0 = skew_inv(logm(R_llsq)).';

    fun = @(x)func(x, A, z);
    [x,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(fun, x0, [], [], options);

    R{k} = expm(skew(x));

end

function [F,J] = func(x, A, z)
% Function f(x)-z
F = [expm(skew(x)),zeros(3),zeros(3);...
    zeros(3),expm(skew(x)),zeros(3);...
    zeros(3),zeros(3),expm(skew(x))] * A - z;
if nargout > 1
    % Jacobian
    R_x = expm(skew(x));
    J = [-R_x*skew(A(1:3,:));-R_x*skew(A(4:6,:));-R_x*skew(A(7:9,:))];
end
