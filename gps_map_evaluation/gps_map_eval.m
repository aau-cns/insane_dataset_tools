%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

%% Read GNSS coordinates from multiple bags and plot a map of position 
% correlated noise values. Bags need to contain static GNSS data at
% different locations.

clear all
close all
addpath(genpath('matlab_rosbag-0.5.0-linux64'));

data_path = "/tmp/";

save_plots = false;  % Store the plots as .png
save_csv = false;    % Export CSV files for the GPS coordinates and ENU data
save_ws = false;     % Save workspace variables for GPS coordinates and ENU data
set_t_zero = true;   % Common timestamp, begining at t=0s

verbose = false;

data_export_dir = data_path + "/export/";
plot_dir = data_export_dir + "/plots/";

%% specify whether/how to print figures
savefigimages.doit                = true; % false if you do not want to save figures as images
savefigimages.figOptions.Units    = 'pixels';
savefigimages.figOptions.Position = [0 0 800 450];
savefigimages.figOptions.Color    = 'white';
savefigimages.figOptions.Renderer = 'opengl';
savefigimages.printOptions        = {'-m4','-pdf','-png' '-nocrop'};
savefigimages.Visibility          = 'on'; % 'on'/'off'
savefigimages.closeAfterSave      = false; % true to close figure after saving it to image

set(groot, 'DefaultFigureVisible', savefigimages.Visibility);

%% Read Sensor topics
sensor_bagfiles = strcat(ls(data_path+ "*.bag"));

sensor_bag = ros.Bag(char(sensor_bagfiles));
if verbose
    sensor_bag.info()
end

% Read PX4 GPS
[px4_gps.t, px4_gps.lat, px4_gps.long, px4_gps.alt] = read_gps_navsat(sensor_bag, '/mavros/global_position/raw/fix');

% Read RTK GPS1
[rtk_gps1.t, rtk_gps1.lat, rtk_gps1.long, rtk_gps1.alt] = read_gps_navsat(sensor_bag, '/rtk_gps_1/fix');

% Read RTK GPS2
[rtk_gps2.t, rtk_gps2.lat, rtk_gps2.long, rtk_gps2.alt] = read_gps_navsat(sensor_bag, '/rtk_gps_2/fix');


%% GPS Coordinate ENU conversion
% Set origin of GPS coordinates

%coordinate_origin = [46.6131, 14.2621,488.1400]; % Dronehall entrance
coordinate_origin = [gps1.latitude(1), gps1.longitude(1), gps1.altitude(1)];

gps_ref.latitude = coordinate_origin(1);
gps_ref.longitude = coordinate_origin(2);
gps_ref.altitude = coordinate_origin(3);

[ gps_ref.rotation, gps_ref.position ] = gps_init_ref(gps_ref.latitude, gps_ref.longitude, gps_ref.altitude);

% Convert from GPS coordinates to local ENU
for i = 1:length(rtk_gps1.alt)
    rtk_gps1.enu(i,:) = wgs84ToEnu(rtk_gps1.lat(i),rtk_gps1.long(i),rtk_gps1.alt(i),gps_ref.rotation,gps_ref.position);
end

for i = 1:length(rtk_gps2.alt)
    rtk_gps2.enu(i,:) = wgs84ToEnu(rtk_gps2.lat(i),rtk_gps2.long(i),rtk_gps2.alt(i),gps_ref.rotation,gps_ref.position);
end

for i = 1:length(px4_gps.alt)
    px4_gps.enu(i,:) = wgs84ToEnu(px4_gps.lat(i),px4_gps.long(i),px4_gps.alt(i),gps_ref.rotation,gps_ref.position);
end

%% Time adjustment
if set_t_zero
    all_stamps = [px4_gps.t,rtk_gps1.t,rtk_gps2.t];
    all_stamps(all_stamps==0) = [];
    t_min_ref = min(all_stamps);
    px4_gps.t = px4_gps.t - t_min_ref;
    rtk_gps1.t = rtk_gps1.t - t_min_ref;
    rtk_gps2.t = rtk_gps2.t - t_min_ref;
end

%% Generate Plots
% PX4 GPS
fh1 = figure('Name','PX4 GPS');
clf(fh1);
set(fh1, savefigimages.figOptions);

plot_position(px4_gps.t, px4_gps.enu.')

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(fh1, sprintf('./%s', plot_dir+export_name), savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_dir+export_name + ".fig");
end

% RTK GPS 1
fh1 = figure('Name','RTK GPS 1');
clf(fh1);
set(fh1, savefigimages.figOptions);

plot_position(rtk_gps1.t, rtk_gps1.enu.')

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(fh1, sprintf('./%s', plot_dir+export_name), savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_dir+export_name + ".fig");
end

% RTK GPS 2
fh1 = figure('Name','RTK GPS 2');
clf(fh1);
set(fh1, savefigimages.figOptions);

plot_position(rtk_gps2.t, rtk_gps2.enu.')

if save_plots
    if ~exist(plot_dir, 'dir')
        fprintf("Info:" + plot_dir + " does not exist. Creating folder...\n");
        mkdir(plot_dir)
    end
    export_name = regexprep(fh1.Name,{'{','}',' '},{'','','_'});
    export_name = lower(export_name);
    export_fig(fh1, sprintf('./%s', plot_dir+export_name), savefigimages.printOptions{:}, '-nocrop');
    savefig(plot_dir+export_name + ".fig");
end

std(rtk_gps1.enu)
std(rtk_gps2.enu)
std(px4_gps.enu)

%% CSV export
mat_save_var = {};
if save_csv
    fprintf("Saving CSV data...\n");

    if ~exist(data_export_dir, 'dir')
        fprintf("Info: " + data_export_dir + " does not exist. Creating folder...\n");
        mkdir(data_export_dir)
    end
    fprintf("Info: " + "Writing README...\n");
    fid = fopen(data_export_dir + "README.txt", 'wt' );

    csv_px4_gps = [px4_gps.t.' , px4_gps.lat.', px4_gps.long.', px4_gps.alt.'];
    mat_save_var{end+1}='px4_gps';
    fprintf( fid, "PX4 GPS format:\n\t[t, lat, long, alt]\n" + sensor_stats_from_csv(csv_px4_gps));
    dlmwrite(data_export_dir + "/px4_gps_data.csv", csv_px4_gps , 'delimiter', ',', 'precision', 17);

    csv_rtk_gps_1 = [rtk_gps1.t.', rtk_gps1.lat.', rtk_gps1.long.', rtk_gps1.alt.'];
    mat_save_var{end+1}='rtk_gps1';
    fprintf( fid, "RTK GPS1 format:\n\t[t, lat, long, alt]\n" + sensor_stats_from_csv(csv_rtk_gps_1));
    dlmwrite(data_export_dir + "/rtk_gps1_data.csv", csv_rtk_gps_1 , 'delimiter', ',', 'precision', 17);

    csv_rtk_gps_2 = [rtk_gps2.t.', rtk_gps2.lat.', rtk_gps2.long.', rtk_gps2.alt.'];
    mat_save_var{end+1}='rtk_gps2';
    fprintf( fid, "RTK GPS2 format:\n\t[t, lat, long, alt]\n" + sensor_stats_from_csv(csv_rtk_gps_2));
    dlmwrite(data_export_dir + "/rtk_gps2_data.csv", csv_rtk_gps_2 , 'delimiter', ',', 'precision', 17);

    fclose(fid);
    fprintf("...done saving CSV data\n");
end

if save_ws
    save(data_export_dir + "matlab_data.mat",mat_save_var{:});
end

fprintf("Finished Data Export\n");
