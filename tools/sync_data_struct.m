%% Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
%
% All rights reserved.
%
% This software is licensed under the terms of the BSD-2-Clause-License with
% no commercial use allowed, the full terms of which are made available
% in the LICENSE file. No license in patents is granted.
%
% You can contact the author at <christian.brommer@aau.at>

function [idx_a,idx_b] = sync_data_struct(data_a, data_b, tol)
%DATA_STRUCT_SYNC Sync all data to the set with the least data points
%
% Find dataset with less datapoints
% Data is synched based on the dataset with less points

verbose = true;

synch_match_b_to_a = true;

if length(data_a) <= length(data_b)
    if verbose;   fprintf('Synching data B to A...\n'); end
    synch_match_b_to_a = true;
    
    data_base = data_a;
    data_sync = data_b;
else
    if verbose; fprintf('Synching data A to B...\n'); end
    synch_match_b_to_a = false;
    
    data_base = data_b;
    data_sync = data_a;
end

min_accum=[];
sync_idx = [];
base_idx = [];
for k = 1:length(data_base)
    
    delta = abs(data_sync - data_base(k));
    
    min_accum(end+1) = min(delta);
    if min(delta) > tol
        continue
    end
    
    min_idx = find(delta == min(delta));
    sync_idx(end+1)=  min_idx(1);
    base_idx(end+1) = k;
end

if isempty(sync_idx)
    fprintf("ERROR, no data synched, min delta was %.4f, tol was %.4f", min(min_accum), tol)
    return
end

if verbose
    fprintf("Discarded %i measurements due to specified synch window of %.4f sec.\n", k-length(base_idx), tol);
end

if synch_match_b_to_a
    idx_a = base_idx;
    idx_b = sync_idx;
else
    idx_a = sync_idx;
    idx_b = base_idx;
end

end

