function [ ecef_ref_orientation, ecef_ref_point ] = gps_init_ref(latitude, longitude, altitude)
R = zeros(3);

rad_lat = deg2rad(latitude);
rad_long = deg2rad(longitude);

s_lat = sin(rad_lat);
c_lat = cos(rad_lat);

s_long = sin(rad_long);
c_long = cos(rad_long);

R(1, 1) = -s_long;
R(1, 2) = c_long;
R(1, 3) = 0;

R(2, 1) = -s_lat * c_long;
R(2, 2) = -s_lat * s_long;
R(2, 3) = c_lat;

R(3, 1) = c_lat * c_long;
R(3, 2) = c_lat * s_long;
R(3, 3) = s_lat;

ecef_ref_orientation = R;
ecef_ref_point = wgs84ToECEF(latitude, longitude, altitude);
end