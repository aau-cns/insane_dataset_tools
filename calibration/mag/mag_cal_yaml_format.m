%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [string] = mag_cal_yaml_format(file_id, name, var_name, elipsoid_trans, center_offset, R_mag_imu, mag_incl_rad, gps_coord )
%MAG_CAL_YAML_FORMAT Magnetometer extrinsic intrinsic as a YAML string

yaml_header = "# " + name + " Magnetometer Calibration";
gps_header = "# GPS location of magnetometer calibration";
intr_header = "# Intrinsic calibration for soft-iron effects (Elipsoid-Fit)";
incl_header = "# Calculated magnetic inclination in [degree] at calibration location";
rot_cal_header = "# Rotation of the PX4 IMU w.r.t. the " + name + " magnetometer frame";

fprintf(file_id,'%s\n\n', yaml_header);

% GPS Coordinates
fprintf(file_id,'%s\n', gps_header);
fprintf(file_id,'%s: %f\n', "gps_coord_cal_lat",gps_coord.lat);
fprintf(file_id,'%s: %f\n', "gps_coord_cal_long",gps_coord.long);
fprintf(file_id,'%s: %f\n\n', "gps_coord_cal_alt",gps_coord.alt);

% Intrinsics
fprintf(file_id,'%s\n', intr_header);
fprintf(file_id,'%s:\n', "mag_intrinsic");
fprintf(file_id,' - [[%f, %f, %f],\n', elipsoid_trans(1,:).');
fprintf(file_id,'    [%f, %f, %f],\n', elipsoid_trans(2,:).');
fprintf(file_id,'    [%f, %f, %f]]\n\n', elipsoid_trans(3,:).');

fprintf(file_id,'%s: [%d, %d, %d]\n\n', "mag_offset", center_offset);

% Inclination
fprintf(file_id,'%s\n', incl_header);
fprintf(file_id,'%s: %f\n\n', "mag_inclination", mag_incl_rad*180/pi);
fprintf(file_id,'%s\n', rot_cal_header);

% Extrinsics
fprintf(file_id,'%s:\n', "R_" + var_name + "_pximu");
fprintf(file_id,' - [[%f, %f, %f],\n', R_mag_imu(1,:).');
fprintf(file_id,'    [%f, %f, %f],\n', R_mag_imu(2,:).');
fprintf(file_id,'    [%f, %f, %f]]\n\n', R_mag_imu(3,:).');

end
