function [ enu ] = wgs84ToEnu( latitude, longitude, altitude,  ref_rotation, ref_position )
ecef = wgs84ToECEF(latitude, longitude, altitude);
enu = ECEFToENU(ecef, ref_rotation, ref_position);
end