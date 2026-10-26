function report = test_s1_geometry_limits()
%TEST_S1_GEOMETRY_LIMITS Comprehensive test of (S1)^7 manifold geometry, limits, and chart maps.
%
% Verifies:
%   1. Embedding and centered lift roundtrip q <-> X on (S1)^7
%   2. Joint 4 (all negative) and Joint 6 (crosses +pi) chart continuity
%   3. Arc inequality u_i^T * x_i >= cos(h_i) discrimination at boundaries
%   4. Radian margin metric mu_i(x_i) == min(q_i - qmin_i, qmax_i - q_i)
%   5. Geodesic difference vs box difference for |q2 - q1| > pi
%   6. Exp_S1 group action unit-norm preservation
%   7. Initial kinematic & dynamic parity between +panda and +panda_s1

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_panda();

m_s1 = panda_s1.model();
m_orig = panda.model();
n = m_s1.n;

rng(13102026, 'twister');

% -------------------------------------------------------------------------
% 1. Embedding and centered lift roundtrip q <-> X
% -------------------------------------------------------------------------
max_roundtrip_err = 0;
N_samples = 200;
for s = 1:N_samples
    % Sample uniformly inside physical limits
    q_rand = m_s1.qmin + rand(n, 1) .* (m_s1.qmax - m_s1.qmin);
    X = panda_s1.from_q(q_rand);
    q_rec = panda_s1.to_q(X, m_s1);
    max_roundtrip_err = max(max_roundtrip_err, norm(q_rec - q_rand, inf));
end
assert(max_roundtrip_err < 1e-14, 'panda_s1:test', 'Roundtrip q <-> X failed');

% -------------------------------------------------------------------------
% 2. Critical Joint Boundary and Sector Testing (Joint 4 and Joint 6)
% -------------------------------------------------------------------------
% Test exact boundaries qmin and qmax for all joints
X_min = panda_s1.from_q(m_s1.qmin);
q_min_rec = panda_s1.to_q(X_min, m_s1);
assert(norm(q_min_rec - m_s1.qmin, inf) < 1e-14, 'panda_s1:test', 'qmin recovery failed');

X_max = panda_s1.from_q(m_s1.qmax);
q_max_rec = panda_s1.to_q(X_max, m_s1);
assert(norm(q_max_rec - m_s1.qmax, inf) < 1e-14, 'panda_s1:test', 'qmax recovery failed');

% Test Joint 6 specifically across +pi
q6_vals = [3.0, 3.14, pi, 3.15, 3.5, 3.75, m_s1.qmax(6)];
q_test = 0.5 * (m_s1.qmin + m_s1.qmax);
for v = q6_vals
    q_test(6) = v;
    X_test = panda_s1.from_q(q_test);
    q_rec = panda_s1.to_q(X_test, m_s1);
    assert(abs(q_rec(6) - v) < 1e-14, 'panda_s1:test', 'Joint 6 above pi recovery failed');
end

% -------------------------------------------------------------------------
% 3. Arc Inequality Discrimination at Boundaries
% -------------------------------------------------------------------------
[in_min, margins_min] = panda_s1.check_limits(X_min, m_s1);
assert(all(in_min), 'panda_s1:test', 'qmin should be inside limits');
assert(all(abs(margins_min) < 1e-13), 'panda_s1:test', 'qmin margin should be zero');

[in_max, margins_max] = panda_s1.check_limits(X_max, m_s1);
assert(all(in_max), 'panda_s1:test', 'qmax should be inside limits');
assert(all(abs(margins_max) < 1e-13), 'panda_s1:test', 'qmax margin should be zero');

% Check rejection of points just outside limits (1e-3 rad beyond)
eps_out = 1e-3;
q_below = m_s1.qmin - eps_out;
X_below = [cos(q_below)'; sin(q_below)'];
in_below = panda_s1.check_limits(X_below, m_s1);
assert(~any(in_below), 'panda_s1:test', 'Points below qmin must be rejected');

q_above = m_s1.qmax + eps_out;
X_above = [cos(q_above)'; sin(q_above)'];
in_above = panda_s1.check_limits(X_above, m_s1);
assert(~any(in_above), 'panda_s1:test', 'Points above qmax must be rejected');

% -------------------------------------------------------------------------
% 4. Geodesic Difference vs Box Difference across Forbidden Sector
% -------------------------------------------------------------------------
% In joints where 2*h > pi (Joints 1, 2, 3, 5, 6, 7), pick states near opposite limits
large_span_joints = find(2 * m_s1.qhalf > pi);
assert(numel(large_span_joints) == 6, 'panda_s1:test', 'Exactly 6 joints have 2h > pi');

q1_opp = m_s1.qmin + 0.02;
q2_opp = m_s1.qmax - 0.02;
X1_opp = panda_s1.from_q(q1_opp);
X2_opp = panda_s1.from_q(q2_opp);

[dtheta_geo, crosses, ~] = panda_s1.geodesic_diff(X2_opp, X1_opp, m_s1);
dq_box = panda_s1.box_diff(X2_opp, X1_opp, m_s1);

for j = large_span_joints'
    delta_q = q2_opp(j) - q1_opp(j);
    if delta_q > pi
        % Must detect crossing forbidden arc
        assert(crosses(j), 'panda_s1:test', 'Forbidden arc crossing not detected');
        % Box diff matches delta_q
        assert(abs(dq_box(j) - delta_q) < 1e-14, 'panda_s1:test', 'Box diff must match delta_q');
        % Geodesic diff has opposite sign
        assert(sign(dtheta_geo(j)) ~= sign(dq_box(j)), 'panda_s1:test', ...
            'Geodesic diff and box diff must have opposite signs when crossing stop');
    end
end

% Check local agreement when |q2 - q1| < pi
q_mid = m_s1.qmid;
q_near = q_mid + 0.1 * (m_s1.qmax - m_s1.qmin) / 2;
X_mid = panda_s1.from_q(q_mid);
X_near = panda_s1.from_q(q_near);
[dtheta_near, crosses_near] = panda_s1.geodesic_diff(X_near, X_mid, m_s1);
dq_box_near = panda_s1.box_diff(X_near, X_mid, m_s1);
assert(~any(crosses_near), 'panda_s1:test', 'Small motion must not cross forbidden arc');
assert(norm(dtheta_near - dq_box_near, inf) < 1e-14, 'panda_s1:test', ...
    'Geodesic diff and box diff must agree for small motions');

% -------------------------------------------------------------------------
% 5. Exp_S1 Group Action Unit-Norm Preservation
% -------------------------------------------------------------------------
X_test = panda_s1.from_q(m_s1.qmid);
for step = 1:100
    dtheta = 0.05 * randn(n, 1);
    X_test = panda_s1.exp_s1(dtheta, X_test, true);
end
[~, norm_err] = panda_s1.config_matrix(X_test);
assert(norm_err < 1e-15, 'panda_s1:test', 'Exp_S1 lost unit norm');

% -------------------------------------------------------------------------
% 6. Initial Parity with +panda (FK, Frames, Jacobian, Dynamics, Energy)
% -------------------------------------------------------------------------
max_fk_err = 0;
max_frames_err = 0;
max_jac_space_err = 0;
max_jac_body_err = 0;
max_mass_err = 0;
max_bias_err = 0;
max_energy_err = 0;

for s = 1:30
    q = m_s1.qmin + 0.05 + rand(n, 1) .* (m_s1.qmax - m_s1.qmin - 0.1);
    omega = 0.5 * randn(n, 1);
    X = panda_s1.from_q(q);

    % FK
    T_orig = panda.fk(q, m_orig);
    T_s1 = panda_s1.fk(X, m_s1);
    max_fk_err = max(max_fk_err, norm(T_orig - T_s1, 'fro'));

    % Frames
    [chain_orig, ~] = panda.frames(q, m_orig);
    [chain_s1, ~] = panda_s1.frames(X, m_s1);
    max_frames_err = max(max_frames_err, norm(chain_orig(:) - chain_s1(:), inf));

    % Jacobians
    Js_orig = panda.jacobian(q, m_orig, 'space');
    Js_s1 = panda_s1.jacobian(X, m_s1, 'space');
    max_jac_space_err = max(max_jac_space_err, norm(Js_orig - Js_s1, 'fro'));

    Jb_orig = panda.jacobian(q, m_orig, 'body');
    Jb_s1 = panda_s1.jacobian(X, m_s1, 'body');
    max_jac_body_err = max(max_jac_body_err, norm(Jb_orig - Jb_s1, 'fro'));

    % Dynamics
    [M_orig, b_orig] = panda.dynamics(q, omega, m_orig);
    [M_s1, b_s1] = panda_s1.dynamics(X, omega, m_s1);
    max_mass_err = max(max_mass_err, norm(M_orig - M_s1, 'fro') / norm(M_orig, 'fro'));
    max_bias_err = max(max_bias_err, norm(b_orig - b_s1));

    % Energy
    [H_orig, K_orig, V_orig] = panda.energy(q, omega, m_orig);
    [H_s1, K_s1, V_s1] = panda_s1.energy(X, omega, m_s1);
    max_energy_err = max(max_energy_err, abs(H_orig - H_s1) + abs(K_orig - K_s1) + abs(V_orig - V_s1));
end

assert(max_fk_err < 1e-14, 'panda_s1:test', 'FK parity failure');
assert(max_frames_err < 1e-14, 'panda_s1:test', 'Frames parity failure');
assert(max_jac_space_err < 1e-14, 'panda_s1:test', 'Space Jacobian parity failure');
assert(max_jac_body_err < 1e-14, 'panda_s1:test', 'Body Jacobian parity failure');
assert(max_mass_err < 1e-14, 'panda_s1:test', 'Mass matrix parity failure');
assert(max_bias_err < 1e-13, 'panda_s1:test', 'Bias vector parity failure');
assert(max_energy_err < 1e-13, 'panda_s1:test', 'Energy parity failure');

report = struct('roundtrip_max_error_rad', max_roundtrip_err, ...
    'fk_parity_error_fro', max_fk_err, ...
    'frames_parity_error_inf', max_frames_err, ...
    'jacobian_space_parity_error_fro', max_jac_space_err, ...
    'jacobian_body_parity_error_fro', max_jac_body_err, ...
    'mass_relative_parity_error', max_mass_err, ...
    'bias_parity_error_Nm', max_bias_err, ...
    'energy_parity_error_J', max_energy_err, ...
    'unit_norm_drift_100_steps', norm_err, ...
    'large_span_joints_count', numel(large_span_joints), ...
    'pass', true);

disp('=== TEST_S1_GEOMETRY_LIMITS: ALL TESTS PASSED ===');
disp(report);
end
