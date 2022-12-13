function [p,angle] = absolute_trajectory_error(gt_p, gt_q, est_p, est_q)
%ABSOLUTE_TRAJECTORY_ERROR Summary of this function goes here
%   Detailed explanation goes here
%
% From "A Tutorial on Quantitative Trajectory Evaluation for
% Visual(-Inertial) Odometry", Zhang, Zichao and Scaramuzza, Davide

data_length = length(gt_p);

delta_r = {};
sum_delta_r = 0;
delta_p = [];
sum_delta_p = 0;

for k = 1:data_length
    % Absolut Trajectory Error (ATE) Orientation (Degree) RMSE
    delta_r{k} = quat2rotm(gt_q(:,k).') * transpose(quat2rotm(est_q(:,k).'));
    ax_ang_err = rotm2axang(delta_r{k});
    sum_delta_r = sum_delta_r + norm(ax_ang_err(4))^2;
    
    % Absolut Trajectory Error (ATE) Position (meter) RMSE
    delta_p(:,k) = gt_p(:,k) - delta_r{k} * est_p(:,k);
    sum_delta_p = sum_delta_p + norm(delta_p(:,k))^2;
end

angle = rad2deg(sqrt(sum_delta_r/data_length));

p = sqrt(sum_delta_p/data_length);

end
