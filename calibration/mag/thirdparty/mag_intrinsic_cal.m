function [transform, center] = mag_intrinsic_cal(mag_data, do_plot)
%MAG_INTRINSIC_CAL Summary of this function goes here
% Compensation is applied as:
% v_corr_offset = v_mag - center
% v_corr = transform * v_corr_offset

m_x = mag_data(:,1);
m_y = mag_data(:,2);
m_z = mag_data(:,3);

% Elipsoid Fit
[e_center, e_radii, e_eigenvecs, e_algebraic] = ellipsoid_fit([m_x, m_y, m_z]);

% compensate distorted magnetometer data
% e_eigenvecs is an orthogonal matrix, so ' can be used instead of inv()
S = [m_x - e_center(1), m_y - e_center(2), m_z - e_center(3)]'; % translate and make array
scale = inv([e_radii(1) 0 0; 0 e_radii(2) 0; 0 0 e_radii(3)]) * min(e_radii); % scaling matrix
map = e_eigenvecs'; % transformation matrix to map ellipsoid axes to coordinate system axes
invmap = e_eigenvecs; % inverse of above
comp = invmap * scale * map;

transform = comp;
center = e_center;

% output info
fprintf('mag_ellipsoid_center = [%.12g, %.6g, %.6g];\n', e_center);
fprintf('mag_ellipsoid_transform = {{%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}};\n', comp);


if do_plot
    %% draw ellipsoid fit
    figure;
    hold on;
    plot3(m_x, m_y, m_z, '.r'); % original data
    
    maxd = max(e_radii);
    step = maxd / 50;
    [xp, yp, zp] = meshgrid(-maxd:step:maxd + e_center(1), -maxd:step:maxd + e_center(2), -maxd:step:maxd + e_center(3));
    
    Ellipsoid = e_algebraic(1) *xp.*xp +   e_algebraic(2) * yp.*yp + e_algebraic(3)   * zp.*zp + ...
        2*e_algebraic(4) *xp.*yp + 2*e_algebraic(5) * xp.*zp + 2*e_algebraic(6) * yp.*zp + ...
        2*e_algebraic(7) *xp     + 2*e_algebraic(8) * yp     + 2*e_algebraic(9) * zp;
    p = patch(isosurface(xp, yp, zp, Ellipsoid, 1));
    set(p, 'FaceColor', 'g', 'EdgeColor', 'none');
    
    alpha(0.5);
    xlabel 'm_x'
    ylabel 'm_y'
    zlabel 'm_z'
    
    view(-33, 33);
    axis vis3d;
    axis equal;
    camlight;
    lighting phong;
    grid on
    grid minor
    title 'Ellipsoid fit with raw magnetometer data'
    
    %% draw original and compensated data
    S = comp * S; % do compensation
    
    figure;
    hold on;
    plot3( m_x, m_y, m_z, '.r' ); % original magnetometer data
    plot3(S(1,:), S(2,:), S(3,:), 'b.'); % compensated data
    xlabel 'm_x'
    ylabel 'm_y'
    zlabel 'm_z'
    view(-33, 33);
    axis vis3d;
    axis equal;
    grid on
    grid minor
    title 'Comparison: Raw vs. corrected magnetometer data'
    legend('raw measurements','corrected measurements')
    
    
    figure;
    plot3( m_x, m_y, m_z);
    axis vis3d;
    axis equal;
    grid on
    grid minor
    title 'Raw magnetometer data'
    
    xlabel 'm_x'
    ylabel 'm_y'
    zlabel 'm_z'
    view(-33, 33);
end

end
