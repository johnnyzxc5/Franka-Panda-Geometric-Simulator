function [M, b, parts] = dynamics(X, omega, m)
%DYNAMICS Mass matrix M(X) and bias b(X, omega) on (S1)^7 with shared Lie kinematics.
%
% Reuses optimized SHAPE RNEA kernels (zero-state mass recursion and direct ad)
% combined with the S^1 Lie-group kinematics kernel rnea_shared_kinematics_S1.
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   omega: 7-by-1 angular velocity vector [rad/s]
%   m: (optional) model struct from panda_s1.model()
% Outputs:
%   M: 7-by-7 symmetric positive-definite mass matrix
%   b: 7-by-1 bias vector (coriolis + gravity + friction)
%   parts: (optional) struct with gravity, coriolis, friction, apparent_inertia

if nargin < 3 || isempty(m)
    m = panda_s1.model();
end

X = panda_s1.config_matrix(X);
omega = panda.joint_vector(omega);

kin = rnea_shared_kinematics_S1(X, m.Mlist, m.Slist);
M = MassMatrixZeroStateSharedKinematics_MR(m.Glist, kin) + m.Japp;
rigid = InverseDynamicsSharedKinematics_MR(omega, zeros(m.n, 1), m.gravity, zeros(6, 1), m.Glist, kin);
f = panda.friction(omega, m);
b = rigid + f;

if nargout > 2
    g = InverseDynamicsSharedKinematics_MR(zeros(m.n, 1), zeros(m.n, 1), m.gravity, zeros(6, 1), m.Glist, kin);
    parts = struct('gravity', g, 'coriolis', rigid - g, 'friction', f, 'apparent_inertia', m.Japp);
end
end
