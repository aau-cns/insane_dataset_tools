%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

clear all
close all
addpath(genpath('matlab_rosbag-0.5.0-linux64'));

bagfile_path = './gps_and_tags.bag';
tag_id = 102;

save_plots = true;  % Store the plots as .png
save_csv = true;    % Export CSV files for the GPS coordinates and ENU data
save_ws = true;     % Save workspace variables for GPS coordinates and ENU data
set_t_zero = true;  % Common timestamp, begining at t=0s

plot_dir = './plots/';
data_dir = './data/';

bag = ros.Bag(char(bagfile_path));
bag.info()

[tag_data,tags_detected] = read_aruco_rostopic(bag, '/fiducial_transforms');

ref_tag_data = extract_tag_id(tag_id, tag_data);

%% Generate Plots
if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
end

% Set time vector
% Timestamp, t_0 staring at zero seconds
% Ensure vertical vector with (:)
t_zero = (ref_tag_data.t(:)-ref_tag_data.t(1));
% Timestamp, original recording stamps
t_orig = ref_tag_data.t(:);

if set_t_zero
    t_plot = t_zero;
else
    t_plot = t_orig;
end


figure
title("3D Tag Position");
hold on
plot3(ref_tag_data.p(:,1),ref_tag_data.p(:,2),ref_tag_data.p(:,3),'.', 'Displayname', 'Ref Tag');
xlabel("X [m]"), ylabel("Y [m]"), zlabel("Z [m]")
view(3)
axis equal
grid on
legend
if save_plots
    saveas(gcf, plot_dir + "tag_3d_position.png")
end

figure
title("Tag Position over Time")
hold on
plot(t_plot, ref_tag_data.p(:,1),'displayname', 'Tag_x')
plot(t_plot, ref_tag_data.p(:,2),'displayname', 'Tag_y')
plot(t_plot, ref_tag_data.p(:,3),'displayname', 'Tag_z')
grid on
legend
ylabel("Tag in ENU [m]"),xlabel("Time [s]")

if save_plots
    saveas(gcf, plot_dir + "tag_enu_data.png")
end


%% Write CSV file
if save_csv
    fprintf("Saving CSV data\n");

    if ~exist(data_dir, 'dir')
        fprintf("Info: " + plot_dir + " does not exist. Creating folder...\n");
        mkdir(data_dir)
    end

    fprintf("Info: " + "Writing tag info to file...\n");
    fid = fopen(data_dir + "README_tag.txt", 'wt' );
    fprintf( fid, sprintf('ENU data for tag ID %i.\n\n', tag_id));
    fprintf( fid, 'CSV file format: ENU [t,p_x,p_y,p_z,q_w,q_x,q_y,q_z].');
    fclose(fid);

    csv_tag_enu = [t_plot, ref_tag_data.p, ref_tag_data.q];
    dlmwrite(data_dir + "/tag_enu.csv", csv_tag_enu , 'delimiter', ',', 'precision', 17);

end

%% Save essential GPS data from workspace
if save_ws
    fprintf("Saving workspace data\n");

    if ~exist(data_dir, 'dir')
        fprintf("Info: " + plot_dir + " does not exist. Creating folder...\n");
        mkdir(data_dir);
    end

    save(data_dir + "tag_data",'t_zero','t_orig','ref_tag_data');

end