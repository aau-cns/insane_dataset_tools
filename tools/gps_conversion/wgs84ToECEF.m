function [ ecef ] = wgs84ToECEF(lat, lon, alt)
%WGS84 ellipsoid constants:
a = 6378137.0;  % semi-major axis
%e = 8.1819190842622e-2  % first eccentricity squared
%e_sq = e^2
e_sq = 6.69437999014e-3;

rad_lat = deg2rad(lat);
rad_long = deg2rad(lon);

s_lat = sin(rad_lat);
c_lat = cos(rad_lat);
s_long = sin(rad_long);
c_long = cos(rad_long);

% intermediate calculation (prime vertical radius of curvature)
N = a / sqrt(1 - e_sq * s_lat * s_lat );

ecef = zeros(3, 1);

ecef(1) = (N + alt) * c_lat * c_long;
ecef(2) = (N + alt) * c_lat * s_long;
ecef(3) = (N * (1 - e_sq) + alt) * s_lat;
end