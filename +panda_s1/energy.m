function [H, K, V] = energy(X, omega, m)
%ENERGY Mechanical Hamiltonian on (S1)^7: H(X, omega) = 1/2*omega'*M(X)*omega + V(X).
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   omega: 7-by-1 joint velocity vector [rad/s]
%   m: (optional) model struct from panda_s1.model()
% Outputs:
%   H: total mechanical energy [J]
%   K: kinetic energy [J]
%   V: gravitational potential energy [J]

if nargin < 3 || isempty(m)
    m = panda_s1.model();
end

X = panda_s1.config_matrix(X);
omega = panda.joint_vector(omega);

[M, ~] = panda_s1.dynamics(X, omega, m);
K = 0.5 * omega' * M * omega;

chain = panda_s1.frames(X, m);
V = 0;
for i = 1:m.n
    r = chain(1:3, 1:3, i) * m.com(:, i) + chain(1:3, 4, i);
    V = V - m.mass(i) * m.gravity' * r;
end
H = K + V;
end
