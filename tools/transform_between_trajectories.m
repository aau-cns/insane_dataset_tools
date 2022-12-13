function [t,R] = transform_between_trajectories(t_a,t_b)
%TRANSFORM_BETWEEN_TRAJECTORIES get transformation between two trajectories
% returns p_a_b and r_a_b between two trajectories
%
% Variable names:
% q_A_B rotation of frame B expressed in frame A
% p_A_B_in_X translation from A to B expressed in frame X.
% X is dropped if X = A, thus p_A_B_in_A == p_A_B.
% Example: p_A_B_in_C = R(q_C_A) * p_A_B_in_A
%
% Reference: Sorkine, O., & Rabinovich, M. (2009). Least-squares rigid
% motion using svd. Technical Notes, February, 1–6.
% http://www.igl.ethz.ch/projects/ARAP/svd_rot.pdf

t_a_mean = mean(t_a);
t_a_center_vector = t_a - t_a_mean;
t_b_mean = mean(t_b);
t_b_center_vector = t_b - mean(t_b);

W = diag(ones(length(t_a_center_vector),1));
X = t_b_center_vector.';
Y = t_a_center_vector.';
S = X*W*Y.';
[U,~,V] = svd(S);

R = V * diag([1,1,det(V*U.')]) * U.';
t = t_a_mean.' - R * t_b_mean.';

end
