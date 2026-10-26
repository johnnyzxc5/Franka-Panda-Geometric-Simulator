function domega = forward_dynamics(X, omega, tau, m, external_wrench)
%FORWARD_DYNAMICS Joint acceleration on (S1)^7: M(X)*domega + b(X,omega) = tau + Jb(X)'*F_env.
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   omega: 7-by-1 joint velocity vector [rad/s]
%   tau: 7-by-1 joint effort vector [N*m]
%   m: (optional) model struct from panda_s1.model()
%   external_wrench: (optional, default zeros) wrench applied BY environment on flange
% Output:
%   domega: 7-by-1 angular acceleration vector [rad/s^2]

if nargin < 5
    external_wrench = zeros(6, 1);
end
if nargin < 4 || isempty(m)
    m = panda_s1.model();
end

tau = panda.joint_vector(tau);
external_wrench = panda.wrench_vector(external_wrench);

[M, b] = panda_s1.dynamics(X, omega, m);
if any(external_wrench)
    tau = tau + panda_s1.jacobian(X, m, 'body')' * external_wrench;
end
domega = M \ (tau - b);
end
