%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [ gsv_array ] = read_gsv_array( bag )
%READ_GSV_ARRAY

disp('Reading gsv array msgs...');

msgs  = bag.readAll('/gsv_array');

sec  = zeros(1,length(msgs));
nsec = zeros(1,length(msgs));

gsv_array.stamp = zeros(1,length(msgs));

for k = 1:length(msgs)
    % Time for messages
    gsv_array.stamp(k) = cat(1,msgs{1,k}.header.stamp.time);
    
    gsv_array.number_of_sv(k) = cat(1,msgs{1,k}.number_of_sv);
    
    gsv_array.sat_prn_nr(:,k) = cat(1,msgs{1,k}.gsv_array.sat_prn_nr);
    gsv_array.snr(:,k) = cat(1,msgs{1,k}.gsv_array.snr);
    
end

disp('DONE Reading gsv array msgs...');

end
