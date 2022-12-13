% Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman.
% "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30,
% no. 4 (2007): 1193-1197.
function [mean_q]=mean_quaternion(q_in)

% Form the symmetric accumulator matrix
A = zeros(4,4);
M = size(q_in,1);

for i=1:M
    q = q_in(i,:)';
    if(q(1)<0) % handle the antipodal configuration
        q = -q;
    end
    A = q*q'+A; % rank 1 update
end

% scale
A=(1.0/M)*A;
% Get the eigenvector corresponding to largest eigen value
[mean_q, Eval] = eigs(A,1);
end