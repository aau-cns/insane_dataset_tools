%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

%% Test script for rotational ground truth generation
% Generate a number of evenly distribute rotations for the test of the mag
% gnss ground truth generation.

clear all
close all
clc

%% Settings
random_runs = 10;
degree_interval = 10;

a = 0;
b = 359;
ra = (b-a).*rand(random_runs,8) + a;

individual_plot = false;
verbose = false;

vg_g = [1,0,0].';

disp("Generating Sim Rotations");
R_wi = gen_sim_rot(degree_interval);

sim_length = length(R_wi);

% pre alloc
m_m(3,sim_length) = 0;
vg_w(3,sim_length) = 0;
rpy_wi_result(3,sim_length) = 0;
rpy_wi_gt(3,sim_length) = 0;

for k=1:random_runs
    % Reset
    m_m(:) = 0;
    vg_w(:) = 0;
    rpy_wi_result(:) = 0;
    rpy_wi_gt(:) = 0;

    % Run
    fprintf("Running Iteration %i\n", k);

    mag_var.inc = deg2rad(ra(k,1));
    mag_var.dec = deg2rad(ra(k,2));

    R_i_vg = eul2rotm(deg2rad([ra(k,3),ra(k,4),ra(k,5)]),'xyz');
    %R_i_vg = eul2rotm(deg2rad([0,0,0]),'xyz');
    R_i_m = eul2rotm(deg2rad([ra(k,6),ra(k,7),ra(k,8)]),'xyz');

    vg_i = R_i_vg * vg_g;
    [m_w(1,1),m_w(2,1),m_w(3,1)] = sph2cart((pi/2)+mag_var.dec, mag_var.inc, 1);


    for i=1:sim_length
        m_m(:,i) = R_i_m.' * R_wi{i}.' * m_w;
        vg_w(:,i) = R_wi{i} * vg_i;
    end

    if verbose
        figure
        hold on
        for i=1:length(R_wi)
            v = R_wi{i};
            plot3(v(1,1),v(2,1),v(3,1),'*');
        end
        grid on
        axis equal
        view(3)
    end

    if verbose
        figure
        hold on
        axis equal
        view(3)
        % Plot world axis
        R = eye(3);
        p_orig = [0,0,0];
        R_x = [p_orig; R(:,1).'];
        R_y = [p_orig; R(:,2).'];
        R_z = [p_orig; R(:,3).'];
        plot3(R_x(:,1),R_x(:,2),R_x(:,3),'r','LineWidth',3);
        plot3(R_y(:,1),R_y(:,2),R_y(:,3),'g','LineWidth',3);
        plot3(R_z(:,1),R_z(:,2),R_z(:,3),'b','LineWidth',3);
    end

    %% Test
    t = 1:size(vg_w,2);
    gps.vec = vg_w;
    gps.t = t;
    mag.vec = R_i_m * m_m;

    R_wi_res = mag_gps_non_linear_lsq(gps.vec, mag.vec, vg_i, m_w);

    %R_wi_res = mag_gps_wahaba(gps.vec, mag.vec, vg_i, m_w);
    %R_wi_res = mag_gps_lsq(gps.vec, mag.vec, vg_i, m_w);
    %R_wi_res = mag_gps_3dof(gps, mag,R_i_m, R_i_vg, mag_var);

    for i = 1:length(R_wi_res)
        r_error{i} = R_wi_res{i} * R_wi{i}.';
        r_eul(:,i) = rad2deg(rotm2eul(r_error{i},'xyz'));
        axisang_error(:,i) = rotm2axang(R_wi_res{i} * R_wi{i}.');
    end

    %figure
    %plot(r_eul(3,:))

    individual_error = rad2deg(sqrt(mean(axisang_error(4,:).^2)));
    fprintf("Rotation error in [deg]: %d \n",individual_error);
    error_collector(k) = individual_error;
end

%%
figure
title("Random Runs: " + length(error_collector) + "; Interval: " + degree_interval + " Degrees; " + length(R_wi) + " Orientations each")
hold on
bar(error_collector)
grid on
xlabel("run"); ylabel("Error [Deg]");

fprintf("Average Error: %d\n", mean(error_collector));
%%
if individual_plot
    for i = 1:length(R_wi_res)
        rpy_wi_result(i,:) = rad2deg(rotm2eul(R_wi_res{i},'xyz'));
        rpy_wi_gt(i,:) = rad2deg(rotm2eul(R_wi{i},'xyz'));
    end

    figure
    hold on
    plot(t, rpy_wi_result,'.','DisplayName', 'Result');
    plot(t, rpy_wi_gt,'o','DisplayName','GT');
    grid on
    legend

    figure
    hold on
    plot(t, rpy_wi_result(:,1),'.','DisplayName', 'Result');
    plot(t, rpy_wi_gt(:,1),'o','DisplayName','GT');
    grid on
    legend

    figure
    hold on
    plot(t, rad2deg(axisang_error(4,:)),'.','DisplayName', 'Result');
    grid on
    legend
end

disp("done");


function r = gen_sim_rot(incr)

e_r = (0:incr:360-incr).';
e_p = [];
for k = 1:length(e_r)
    dat = [e_r, ones(length(e_r),1)*e_r(k)];
    e_p = [e_p; dat];
end
e_y=[];
for k = 1:length(e_r)
    e_p(:,3) = ones(length(e_p),1)*e_r(k);
    e_y = [e_y; e_p];
end

for k=1:length(e_y)
    r{k} = eul2rotm(deg2rad(e_y(k,:)),'xyz');
end

end