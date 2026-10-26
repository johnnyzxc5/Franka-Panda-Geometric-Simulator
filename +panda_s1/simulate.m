function log = simulate(m, c)
%SIMULATE Munthe-Kaas Lie-group RK4 simulation on (S1)^7 manifold with ZOH control.
%
% Integrates dynamics on (S1)^7 x R^7 without storing q as an unconstrained state.
% Unit-norm constraints ||x_i|| = 1 are preserved to machine precision without
% phase distortion via Algorithm 6.1 (Abelian Munthe-Kaas RK4).
% Reference interpolation and PD tracking use the homotopy-consistent admissible
% arc operator box_diff so that trajectories never traverse forbidden stops.
%
% Inputs:
%   m: (optional) model struct from panda_s1.model()
%   c: (optional) config struct from panda.config()
% Output:
%   log: simulation record struct with X, lifted q, dq/omega, energies, and metrics

if nargin < 1 || isempty(m)
    m = panda_s1.model();
elseif ~isfield(m, 'qmid')
    m_s1_base = panda_s1.model(m.options);
    m.qmid = m_s1_base.qmid;
    m.qhalf = m_s1_base.qhalf;
    m.umid = m_s1_base.umid;
    m.cos_half = m_s1_base.cos_half;
    m.is_pure_z = m_s1_base.is_pure_z;
end
if nargin < 2 || isempty(c)
    c = panda.config();
end

assert(c.control_dt > 0 && c.substeps >= 1 && c.substeps == floor(c.substeps));
assert(c.motion_time > 0 && c.end_time >= c.motion_time);
assert(abs(c.end_time / c.control_dt - round(c.end_time / c.control_dt)) < 1e-9);

q0 = panda.joint_vector(c.q0);
goal = panda.joint_vector(c.qgoal);
assert(all(q0 >= m.qmin & q0 <= m.qmax) && all(goal >= m.qmin & goal <= m.qmax));

X0 = panda_s1.from_q(q0);
Xgoal = panda_s1.from_q(goal);
assert(all(panda_s1.check_limits(X0, m)) && all(panda_s1.check_limits(Xgoal, m)), ...
    'panda_s1:limits', 'Initial and goal configurations must lie in admissible arc A');

n = m.n;
controller = m;
if ~isempty(c.controller_model)
    controller = c.controller_model;
end

phase = ftv5.phase_plan(0, 0, 0, c.motion_time);
d_box = panda_s1.box_diff(Xgoal, X0, m);

clock = tic;
t = 0:c.control_dt:c.end_time;
N = numel(t);

% State on (S1)^7 x R^7: X (2x7), omega (7x1), and work integrals (3x1)
X = X0;
omega = zeros(n, 1);
work = zeros(3, 1); % [work_joint; friction_loss; work_env]

log = struct('t', t, 'X', zeros(2, n, N), 'q', zeros(n, N), 'dq', zeros(n, N), ...
    'omega', zeros(n, N), 'Xref', zeros(2, n, N), 'qref', zeros(n, N), ...
    'dqref', zeros(n, N), 'tau_command', zeros(n, N), 'tau_applied', zeros(n, N), ...
    'effectiveness', ones(n, N), 'saturated', false(n, N), 'H', zeros(1, N), ...
    'work_joint', zeros(1, N), 'friction_loss', zeros(1, N), ...
    'work_environment', zeros(1, N), 'position', zeros(3, N), ...
    'position_ref', zeros(3, N), 'max_unit_norm_error', zeros(1, N));

h = c.control_dt / c.substeps;
limit_violation = 0;
velocity_utilization = 0;
stage_min_margin = Inf;
max_unit_norm_ever = 0;

cm = m.umid(1, :)';
sm = m.umid(2, :)';

for k = 1:N
    q_lifted = panda_s1.to_q(X, m);
    [Xr, dqr, ddqr, qr] = panda_s1.reference(t(k), c, m, phase, X0, d_box);

    % Control computation using admissible-arc displacement box_diff
    if isempty(c.control_function)
        err_box = panda_s1.box_diff(Xr, X, m);
        virtual = ddqr + c.kp(:) .* err_box + c.kd(:) .* (dqr - omega);
        command_raw = panda_s1.inverse_dynamics(X, omega, virtual, controller);
    else
        command_raw = panda.joint_vector(c.control_function(t(k), q_lifted, omega, qr, dqr, ddqr, controller));
    end

    command = command_raw;
    if c.command_saturation
        command = max(-m.taumax, min(m.taumax, command));
    end

    [alpha, ~] = inputs(t(k), X, omega, m, c, n);
    actual = alpha .* command;

    % Diagnostics and logging
    log.X(:, :, k) = X;
    log.q(:, k) = q_lifted;
    log.dq(:, k) = omega;
    log.omega(:, k) = omega;
    log.Xref(:, :, k) = Xr;
    log.qref(:, k) = qr;
    log.dqref(:, k) = dqr;
    log.tau_command(:, k) = command;
    log.tau_applied(:, k) = actual;
    log.effectiveness(:, k) = alpha;
    log.saturated(:, k) = abs(command_raw) > m.taumax + 1e-12;
    log.H(k) = panda_s1.energy(X, omega, m);
    log.work_joint(k) = work(1);
    log.friction_loss(k) = work(2);
    log.work_environment(k) = work(3);

    norm_err_k = max(abs(sum(X.^2, 1) - 1));
    log.max_unit_norm_error(k) = norm_err_k;
    max_unit_norm_ever = max(max_unit_norm_ever, norm_err_k);

    T = panda_s1.fk(X, m);
    Tr = panda_s1.fk(Xr, m);
    log.position(:, k) = T(1:3, 4);
    log.position_ref(:, k) = Tr(1:3, 4);

    % Continuous Munthe-Kaas Lie-Group RK4 integration between control epochs
    if k < N
        for j = 1:c.substeps
            tj = t(k) + (j - 1) * h;
            X1 = X;
            omega1 = omega;

            % Stage 1: at (X1, omega1)
            [dw1, k1_w, k1_work] = mk_rhs(tj, X1, omega1, command, m, c);

            % Stage 2: at Exp(h/2 * dw1) * X1
            dtheta2 = (h / 2) * dw1;
            X2 = panda_s1.exp_s1(dtheta2, X1, true);
            omega2 = omega1 + (h / 2) * k1_w;
            [dw2, k2_w, k2_work] = mk_rhs(tj + h / 2, X2, omega2, command, m, c);

            % Stage 3: at Exp(h/2 * dw2) * X1
            dtheta3 = (h / 2) * dw2;
            X3 = panda_s1.exp_s1(dtheta3, X1, true);
            omega3 = omega1 + (h / 2) * k2_w;
            [dw3, k3_w, k3_work] = mk_rhs(tj + h / 2, X3, omega3, command, m, c);

            % Stage 4: at Exp(h * dw3) * X1
            dtheta4 = h * dw3;
            X4 = panda_s1.exp_s1(dtheta4, X1, true);
            omega4 = omega1 + h * k3_w;
            [dw4, k4_w, k4_work] = mk_rhs(tj + h, X4, omega4, command, m, c);

            % Step update via Lie group exponential action
            dtheta_step = (h / 6) * (dw1 + 2 * dw2 + 2 * dw3 + dw4);
            X = panda_s1.exp_s1(dtheta_step, X1, true);
            omega = omega1 + (h / 6) * (k1_w + 2 * k2_w + 2 * k3_w + k4_w);
            work = work + (h / 6) * (k1_work + 2 * k2_work + 2 * k3_work + k4_work);

            if c.capture_substeps
                % Vectorized stage monitoring across all 5 states (X1, X2, X3, X4, X)
                C_st = [X1(1, :)', X2(1, :)', X3(1, :)', X4(1, :)', X(1, :)'];
                S_st = [X1(2, :)', X2(2, :)', X3(2, :)', X4(2, :)', X(2, :)'];

                st_norm_err = max(abs(C_st.^2 + S_st.^2 - 1), [], 'all');
                max_unit_norm_ever = max(max_unit_norm_ever, st_norm_err);

                delta_st = atan2(S_st .* cm - C_st .* sm, C_st .* cm + S_st .* sm);
                margins_st = m.qhalf - abs(delta_st);
                stage_min_margin = min(stage_min_margin, min(margins_st, [], 'all'));
                limit_violation = max(limit_violation, max(0, -min(margins_st, [], 'all')));

                stage_omegas = [omega1, omega2, omega3, omega4, omega];
                velocity_utilization = max(velocity_utilization, ...
                    max(abs(stage_omegas) ./ m.dqmax, [], 'all'));
            end

            assert(all(isfinite(X(:))) && all(isfinite(omega)), ...
                'panda_s1:integration', 'Nonfinite state: reduce step or check model');
        end
    end
end

error_deg = (log.q - log.qref) * 180 / pi;
log.energy_residual = log.H - log.H(1) - log.work_joint - log.work_environment + log.friction_loss;

[~, rec_margins] = panda_s1.check_limits(X, m); %#ok<ASGLU>
C_rec = reshape(log.X(1, :, :), n, N);
S_rec = reshape(log.X(2, :, :), n, N);
delta_rec = atan2(S_rec .* cm - C_rec .* sm, C_rec .* cm + S_rec .* sm);
margins_rec = m.qhalf - abs(delta_rec);

log.metrics = struct('actuator_profile', m.actuator_profile, 'friction_profile', m.friction_profile, ...
    'control_dt_s', c.control_dt, 'plant_step_s', h, 'duration_s', c.end_time, ...
    'max_joint_error_deg', max(abs(error_deg), [], 'all'), ...
    'rms_joint_error_deg', sqrt(mean(error_deg.^2, 'all')), ...
    'max_position_error_mm', max(vecnorm(log.position - log.position_ref)) * 1000, ...
    'energy_balance_max_error_J', max(abs(log.energy_residual)), ...
    'minimum_recorded_limit_margin_rad', min(margins_rec, [], 'all'), ...
    'minimum_stage_limit_margin_rad', stage_min_margin, ...
    'maximum_stage_limit_violation_rad', limit_violation, ...
    'max_stage_velocity_utilization', velocity_utilization, ...
    'max_command_utilization', max(abs(log.tau_command) ./ m.taumax, [], 'all'), ...
    'saturated_joint_samples', nnz(log.saturated), ...
    'max_unit_norm_error', max_unit_norm_ever, ...
    'elapsed_seconds', toc(clock));
log.model_options = m.options;
log.urdf_revision = m.urdf_revision;
log.representation = 'S1';
end

function [dtheta_dot, domega, dwork] = mk_rhs(t, X, omega, command, m, c)
n = m.n;
[alpha, F] = inputs(t, X, omega, m, c, n);
[M, b] = panda_s1.dynamics(X, omega, m);
actual = alpha .* command;
environment = zeros(n, 1);
if any(F)
    environment = panda_s1.jacobian(X, m, 'body')' * F;
end
domega = M \ (actual + environment - b);
dtheta_dot = omega; % Left-trivialized Lie algebra velocity
f = panda.friction(omega, m);
dwork = [actual' * omega; f' * omega; environment' * omega];
end

function [alpha, F] = inputs(t, X, omega, m, c, n)
alpha = ones(n, 1);
F = zeros(6, 1);
if ~isempty(c.effectiveness_function)
    alpha = panda.joint_vector(c.effectiveness_function(t));
end
if ~isempty(c.external_wrench_function)
    q_lift = panda_s1.to_q(X, m);
    F = panda.wrench_vector(c.external_wrench_function(t, q_lift, omega));
end
assert(all(alpha >= 0 & alpha <= 1), 'panda_s1:input', 'Effectiveness must be in [0,1]');
end
