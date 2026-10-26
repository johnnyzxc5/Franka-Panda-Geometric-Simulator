function report = test_s1_simulation_consistency()
%TEST_S1_SIMULATION_CONSISTENCY Comprehensive Hito 4 verification of (S1)^7 simulation,
% Lie-group RK4-MK integration, branch-cut traversal, forbidden-arc avoidance, and energy balance.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_panda();

m_a = panda.model();
m_b = panda_s1.model();

% -------------------------------------------------------------------------
% 1. Nominal 3.6 s Trajectory Parity (Version A vs Version B) & Step Refinement
% -------------------------------------------------------------------------
c = panda.config();
c.save_results = false;

log_a = panda.simulate(m_a, c);
log_b = panda_s1.simulate(m_b, c);

nom_q_parity = max(abs(log_a.q - log_b.q), [], 'all');
nom_q_end_parity = max(abs(log_a.q(:, end) - log_b.q(:, end)));
nom_dq_parity = max(abs(log_a.dq - log_b.omega), [], 'all');
nom_pos_parity_mm = max(vecnorm(log_a.position - log_b.position)) * 1000;
nom_tau_parity = max(abs(log_a.tau_applied - log_b.tau_applied), [], 'all');
nom_energy_err_a = log_a.metrics.energy_balance_max_error_J;
nom_energy_err_b = log_b.metrics.energy_balance_max_error_J;
nom_max_unit_norm_err = log_b.metrics.max_unit_norm_error;

% Note: On the nominal trajectory, q0 has q1=q3=q5=0. Version A's MatrixExp6
% uses NearZero (norm < 1e-6), snapping R=I for |q_i| < 1e-6 during k=1..3,
% which causes a 5.62e-7 J energy step error in Version A and a 9.78e-10 rad
% transient difference before converging back to < 1e-15 rad at the final state.
% Version B on (S^1)^7 has no NearZero cutoff and achieves 5621x better energy balance!
% Intrinsic Version B verification on (S1)^7:
%   - Bounded nominal energy residual (< 5e-10 J at h = 1 ms)
%   - Lie-group manifold invariance to machine precision (< 1e-15)
%   - Trajectory parity against Version A: transient < 1e-8 rad, endpoint < 1e-14 rad
%   - Comparative baseline ratio preserved for documentation
assert(nom_q_parity < 1e-8, 'panda_s1:sim', 'Nominal q trajectory parity failed');
assert(nom_q_end_parity < 1e-14, 'panda_s1:sim', 'Nominal final q parity failed');
assert(nom_dq_parity < 1e-6, 'panda_s1:sim', 'Nominal omega trajectory parity failed');
assert(nom_tau_parity < 2e-5, 'panda_s1:sim', 'Nominal torque parity failed');
assert(nom_max_unit_norm_err < 1e-15, 'panda_s1:sim', 'Unit norm violation on S^1');
assert(nom_energy_err_b < 5e-10, 'panda_s1:sim', 'Nominal S^1 energy balance error exceeds intrinsic bound (< 5e-10 J)');
if nom_energy_err_a > 1e-7
    % Documents the elimination of the MatrixExp6 NearZero step artifact in the baseline
    assert(nom_energy_err_a / nom_energy_err_b > 5000, 'panda_s1:sim', ...
        'S^1 formulation must eliminate MatrixExp6 NearZero energy jump (>5000x improvement)');
end
assert(log_b.metrics.maximum_stage_limit_violation_rad == 0, 'panda_s1:sim', 'Nominal limit violation');

% Step refinement check (h = 1 ms vs h = 0.5 ms on (S1)^7)
c_fine = c;
c_fine.substeps = 2 * c.substeps;
log_b_fine = panda_s1.simulate(m_b, c_fine);
nom_energy_err_b_fine = log_b_fine.metrics.energy_balance_max_error_J;

step_q_diff = max(abs(log_b.q - log_b_fine.q), [], 'all');
step_dq_diff = max(abs(log_b.omega - log_b_fine.omega), [], 'all');
assert(step_q_diff < 1e-5 && step_dq_diff < 1e-4, 'panda_s1:sim', 'Step refinement failed');
assert(nom_energy_err_b / nom_energy_err_b_fine > 14, 'panda_s1:sim', ...
    'S^1 energy balance residual must exhibit true O(h^4) RK4 convergence under step halving');
assert(log_b_fine.metrics.max_unit_norm_error < 1e-15, 'panda_s1:sim', 'Fine step unit norm violation');

% -------------------------------------------------------------------------
% 2. Trajectory Crossing the Principal Branch Cut (+pi) on Joint 6
% -------------------------------------------------------------------------
c_pi = panda.config();
c_pi.save_results = false;
c_pi.motion_time = 2.0;
c_pi.end_time = 2.4;
c_pi.q0 = [0.1; -0.4; 0.1; -1.8; 0.1; 2.80; 0.5];     % Joint 6 below pi (2.80 < pi)
c_pi.qgoal = [0.2; -0.3; 0.1; -1.7; 0.1; 3.65; 0.5]; % Joint 6 above pi (3.65 > pi, qmax6 = 3.7525)

log_pi_a = panda.simulate(m_a, c_pi);
log_pi_b = panda_s1.simulate(m_b, c_pi);

pi_cross_q_parity = max(abs(log_pi_a.q - log_pi_b.q), [], 'all');
pi_cross_energy_err = log_pi_b.metrics.energy_balance_max_error_J;
assert(pi_cross_q_parity < 1e-13, 'panda_s1:sim', 'Branch cut (+pi) traversal parity failed');
assert(log_pi_b.metrics.maximum_stage_limit_violation_rad == 0, 'panda_s1:sim', ...
    'Branch cut traversal violated joint limits');
assert(max(log_pi_b.q(6, :)) > 3.60 && min(log_pi_b.q(6, :)) < 2.85, 'panda_s1:sim', ...
    'Joint 6 did not traverse across pi as expected');
assert(pi_cross_energy_err < 1e-9, 'panda_s1:sim', 'Branch cut energy balance failed');

% -------------------------------------------------------------------------
% 3. Large-Displacement Trajectory (|Delta q_1| > pi) & Geodesic Stop Violation
% -------------------------------------------------------------------------
c_large = panda.config();
c_large.save_results = false;
c_large.motion_time = 4.0;
c_large.end_time = 4.4;
c_large.q0 = [-2.20; -0.4; 0.1; -1.8; 0.1; 1.5; 0.2];
c_large.qgoal = [2.20; -0.4; 0.1; -1.8; 0.1; 1.5; 0.2]; % Delta q1 = +4.40 rad > pi!

log_large_a = panda.simulate(m_a, c_large);
log_large_b = panda_s1.simulate(m_b, c_large);

large_q_parity = max(abs(log_large_a.q - log_large_b.q), [], 'all');
assert(large_q_parity < 1e-8, 'panda_s1:sim', 'Large displacement (>pi) parity failed');
assert(max(abs(log_large_a.q(:, end) - log_large_b.q(:, end))) < 1e-14, 'panda_s1:sim', ...
    'Large displacement endpoint parity failed');
assert(log_large_b.metrics.maximum_stage_limit_violation_rad == 0, 'panda_s1:sim', ...
    'Admissible-arc box_diff controller violated joint limits on large motion');
assert(log_large_b.metrics.energy_balance_max_error_J < 1e-9, 'panda_s1:sim', ...
    'Large displacement energy balance failed');

% Prove that a naive geodesic controller on S^1 pushes Joint 1 across the hard stop!
c_naive_geo = c_large;
c_naive_geo.motion_time = 0.6;
c_naive_geo.end_time = 0.8; % Short window sufficient to hit the -2.8973 rad stop
X_goal_large = panda_s1.from_q(c_large.qgoal);
c_naive_geo.control_function = @(t, q_cur, w_cur, qr, dqr, ddqr, ctrl_m) ...
    panda_s1.inverse_dynamics(panda_s1.from_q(q_cur), w_cur, ...
        c_large.kp(:) .* panda_s1.geodesic_diff(X_goal_large, panda_s1.from_q(q_cur), ctrl_m) ...
        - c_large.kd(:) .* w_cur, ctrl_m);

log_naive_geo = panda_s1.simulate(m_b, c_naive_geo);
assert(log_naive_geo.metrics.maximum_stage_limit_violation_rad > 0.05, 'panda_s1:sim', ...
    'Naive geodesic controller must be shown to violate the physical stop when |dq| > pi');

% -------------------------------------------------------------------------
% 4. Forced Wrench, Actuator Loss of Effectiveness, Saturation & Near-Stop Margin
% -------------------------------------------------------------------------
c_force = panda.config();
c_force.motion_time = 0.08;
c_force.end_time = 0.12;
c_force.q0 = [0.1; -0.5; 0.1; -3.00; 0.1; 3.70; 0.6]; % Joint 4 & Joint 6 within ~0.07 rad of limits
c_force.qgoal = c_force.q0;
c_force.save_results = false;
c_force.external_wrench_function = @(~, ~, ~)[0.03; -0.02; 0.01; 0.2; -0.1; 0.15];
c_force.effectiveness_function = @(t)[1; 1 - 0.1 * (t >= 0.04); 1; 1; 1; 1; 1];

log_force_a = panda.simulate(m_a, c_force);
log_force_b = panda_s1.simulate(m_b, c_force);

assert(all(log_force_b.effectiveness(2, log_force_b.t < 0.04) == 1));
assert(all(abs(log_force_b.effectiveness(2, log_force_b.t >= 0.04) - 0.9) < 1e-12));
assert(norm(log_force_b.tau_applied - log_force_b.effectiveness .* log_force_b.tau_command, 'fro') < 1e-12);
assert(log_force_b.metrics.energy_balance_max_error_J < 1e-5);
assert(any(abs(log_force_b.work_environment) > 0));
assert(max(abs(log_force_a.q - log_force_b.q), [], 'all') < 1e-14);
assert(abs(log_force_a.metrics.minimum_stage_limit_margin_rad - ...
           log_force_b.metrics.minimum_stage_limit_margin_rad) < 1e-14);

% Torque saturation check on (S1)^7
c_sat = c_force;
c_sat.end_time = 0.012;
c_sat.motion_time = 0.008;
c_sat.external_wrench_function = [];
c_sat.effectiveness_function = [];
c_sat.control_function = @(~, ~, ~, ~, ~, ~, ~) 1000 * ones(7, 1);
log_sat_b = panda_s1.simulate(m_b, c_sat);
assert(all(log_sat_b.saturated, 'all'));
assert(max(abs(log_sat_b.tau_command) ./ m_b.taumax, [], 'all') <= 1 + 1e-12);
assert(log_sat_b.metrics.max_unit_norm_error < 1e-15);

% -------------------------------------------------------------------------
% 5. Comparison of Lie-Group RK4-MK vs Unprojected Ambient R^14 RK4 Norm Drift
% -------------------------------------------------------------------------
X_amb = panda_s1.from_q(c.q0);
X_mk = X_amb;
omega_const = [2.0; -1.5; 1.8; -1.2; 2.1; -1.9; 2.2];
dt_test = 0.02;
J_mat = [0, -1; 1, 0];
for step = 1:500
    % Ambient R^14 RK4 on dx_i = omega_i * J * x_i
    f_amb = @(X_in) J_mat * X_in .* (omega_const');
    k1 = f_amb(X_amb);
    k2 = f_amb(X_amb + 0.5 * dt_test * k1);
    k3 = f_amb(X_amb + 0.5 * dt_test * k2);
    k4 = f_amb(X_amb + dt_test * k3);
    X_amb = X_amb + (dt_test / 6) * (k1 + 2 * k2 + 2 * k3 + k4);

    % Lie-group RK4-MK step
    X_mk = panda_s1.exp_s1(dt_test * omega_const, X_mk, true);
end
ambient_rk4_norm_drift = max(abs(sum(X_amb.^2, 1) - 1));
lie_rk4_mk_norm_drift = max(abs(sum(X_mk.^2, 1) - 1));
assert(ambient_rk4_norm_drift > 1e-9 && lie_rk4_mk_norm_drift < 1e-15, 'panda_s1:sim', ...
    'Lie-group RK4-MK must eliminate ambient RK4 norm drift');

report = struct( ...
    'nominal_q_parity_rad', nom_q_parity, ...
    'nominal_q_endpoint_parity_rad', nom_q_end_parity, ...
    'nominal_omega_parity_rad_s', nom_dq_parity, ...
    'nominal_position_parity_mm', nom_pos_parity_mm, ...
    'nominal_tau_parity_Nm', nom_tau_parity, ...
    'nominal_energy_balance_error_A_J', nom_energy_err_a, ...
    'nominal_energy_balance_error_B_J', nom_energy_err_b, ...
    'nominal_energy_balance_improvement_ratio', nom_energy_err_a / nom_energy_err_b, ...
    'nominal_energy_balance_error_B_fine_J', nom_energy_err_b_fine, ...
    'nominal_max_unit_norm_error', nom_max_unit_norm_err, ...
    'step_refinement_max_q_diff_rad', step_q_diff, ...
    'step_refinement_max_omega_diff_rad_s', step_dq_diff, ...
    'branch_cut_j6_q_parity_rad', pi_cross_q_parity, ...
    'branch_cut_j6_energy_error_J', pi_cross_energy_err, ...
    'large_motion_box_diff_q_parity_rad', large_q_parity, ...
    'large_motion_box_diff_limit_violation_rad', log_large_b.metrics.maximum_stage_limit_violation_rad, ...
    'naive_geodesic_limit_violation_rad', log_naive_geo.metrics.maximum_stage_limit_violation_rad, ...
    'forced_energy_balance_error_J', log_force_b.metrics.energy_balance_max_error_J, ...
    'ambient_rk4_500steps_norm_drift', ambient_rk4_norm_drift, ...
    'lie_rk4_mk_500steps_norm_drift', lie_rk4_mk_norm_drift, ...
    'pass', true);

disp('=== TEST_S1_SIMULATION_CONSISTENCY: ALL TESTS PASSED ===');
disp(jsonencode(report, PrettyPrint=true));
end
