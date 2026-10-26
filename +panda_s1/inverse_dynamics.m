function tau = inverse_dynamics(X, omega, domega, m, Ftip)
%INVERSE_DYNAMICS Total joint-side effort required on (S1)^7 manifold.
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   omega: 7-by-1 joint velocity vector [rad/s]
%   domega: 7-by-1 joint acceleration vector [rad/s^2]
%   m: (optional) model struct from panda_s1.model()
%   Ftip: (optional, default zeros) wrench applied BY the flange TO the environment [moment; force]
% Output:
%   tau: 7-by-1 required link-side effort vector [N*m]

if nargin < 5
    Ftip = zeros(6, 1);
end
if nargin < 4 || isempty(m)
    m = panda_s1.model();
end

X = panda_s1.config_matrix(X);
omega = panda.joint_vector(omega);
domega = panda.joint_vector(domega);
Ftip = panda.wrench_vector(Ftip);

kin = rnea_shared_kinematics_S1(X, m.Mlist, m.Slist);
tau = InverseDynamicsSharedKinematics_MR(omega, domega, m.gravity, Ftip, m.Glist, kin) ...
    + m.Japp * domega + panda.friction(omega, m);
end
