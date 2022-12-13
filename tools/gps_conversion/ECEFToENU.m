function [ enu ] = ECEFToENU(ecef, ecef_ref_orientation, ecef_ref_point)
enu = ecef_ref_orientation * (ecef - ecef_ref_point);
end