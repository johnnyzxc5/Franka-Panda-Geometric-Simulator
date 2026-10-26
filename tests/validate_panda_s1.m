function report = validate_panda_s1()
%VALIDATE_PANDA_S1 Independent mechanics, Lie identities, and Version A/B parity on (S1)^7.
%
% Verifies that the global (S1)^7 formulation (+panda_s1) satisfies all intrinsic
% manifold identities (Lee et al., Ch. 4) and achieves machine-precision equivalence
% with the original joint-coordinate formulation (+panda).

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setup_panda();

rng(13102026, 'twister');
m = panda_s1.model();
m_orig = panda.model();
n = m.n;
z = zeros(n, 1);
ep = 1e-5;

report = struct('seed', 13102026, 'states', 40, 'matlab_version', version, ...
    'mass_reference_error', 0, 'gravity_reference_error_Nm', 0, 'coriolis_reference_error_Nm', 0, ...
    'fk_chain_error', 0, 'jacobian_finite_difference_error', 0, 'mass_asymmetry', 0, ...
    'min_mass_eigenvalue', Inf, 'potential_gradient_error_Nm', 0, 'power_identity_error', 0, ...
    'version_a_fk_parity_error', 0, 'version_a_jacobian_parity_error', 0, ...
    'version_a_mass_parity_error', 0, 'version_a_inverse_dynamics_parity_error_Nm', 0, ...
    'version_a_forward_dynamics_parity_error', 0, 'version_a_energy_parity_error_J', 0, ...
    'forward_inverse_roundtrip_error', 0, 'wrench_duality_error', 0, ...
    'minimum_friction_power_W', Inf);

for k = 1:report.states
    q = m.qmin + 0.05 + (m.qmax - m.qmin - 0.1) .* rand(n, 1);
    if k == 1
        % Explicitly force Joint 6 above pi and Joint 4 near negative midpoint
        q(6) = 3.55;
        q(4) = -2.20;
    end
    omega = randn(n, 1);
    domega = randn(n, 1);
    F = randn(6, 1);

    X = panda_s1.from_q(q);

    % 1. Intrinsic (S1)^7 dynamics, kinematics, and independent Lagrange oracle
    [M, b, parts] = panda_s1.dynamics(X, omega, m);
    [ML, gL, VL] = panda_s1.lagrange_reference(X, m);
    [~, Tchain] = panda_s1.frames(X, m);
    T = panda_s1.fk(X, m);

    report.fk_chain_error = max(report.fk_chain_error, norm(T - Tchain, 'fro'));
    report.mass_reference_error = max(report.mass_reference_error, norm(M - ML, 'fro') / norm(ML, 'fro'));
    report.gravity_reference_error_Nm = max(report.gravity_reference_error_Nm, norm(parts.gravity - gL));
    report.mass_asymmetry = max(report.mass_asymmetry, norm(M - M', 'fro'));
    report.min_mass_eigenvalue = min(report.min_mass_eigenvalue, min(eig((M + M') / 2)));

    Js = panda_s1.jacobian(X, m, 'space');
    Jb = panda_s1.jacobian(X, m, 'body');

    % 2. Tangential directional derivatives D_j on (S1)^7 via Exp_S1 variations
    derivative = zeros(n, n, n);
    gradient = z;
    Jfd = zeros(6, n);
    for j = 1:n
        d = z;
        d(j) = ep;
        Xp = panda_s1.exp_s1(d, X, true);
        Xm = panda_s1.exp_s1(-d, X, true);
        [Mp, ~, Vp] = panda_s1.lagrange_reference(Xp, m);
        [Mm, ~, Vm] = panda_s1.lagrange_reference(Xm, m);
        derivative(:, :, j) = (Mp - Mm) / (2 * ep);
        gradient(j) = (Vp - Vm) / (2 * ep);

        Tp = panda_s1.fk(Xp, m);
        Tm = panda_s1.fk(Xm, m);
        twist = (Tp - Tm) / (2 * ep) * TransInv(T);
        Jfd(:, j) = [so3ToVec(twist(1:3, 1:3)); twist(1:3, 4)];
    end

    Mdot = zeros(n);
    gradK = z;
    for j = 1:n
        Mdot = Mdot + derivative(:, :, j) * omega(j);
        gradK(j) = omega' * derivative(:, :, j) * omega;
    end
    cL = Mdot * omega - 0.5 * gradK;

    report.coriolis_reference_error_Nm = max(report.coriolis_reference_error_Nm, norm(parts.coriolis - cL));
    report.potential_gradient_error_Nm = max(report.potential_gradient_error_Nm, norm(parts.gravity - gradient));
    report.power_identity_error = max(report.power_identity_error, abs(omega' * parts.coriolis - 0.5 * omega' * Mdot * omega));
    report.jacobian_finite_difference_error = max(report.jacobian_finite_difference_error, norm(Js - Jfd, 'fro'));

    % 3. Inverse/forward dynamics and wrench duality on (S1)^7
    tau_s1 = panda_s1.inverse_dynamics(X, omega, domega, m, F);
    recovered = panda_s1.forward_dynamics(X, omega, tau_s1, m, -F);
    report.forward_inverse_roundtrip_error = max(report.forward_inverse_roundtrip_error, norm(recovered - domega));
    report.wrench_duality_error = max(report.wrench_duality_error, abs(F' * (Jb * omega) - (Jb' * F)' * omega));
    report.minimum_friction_power_W = min(report.minimum_friction_power_W, omega' * parts.friction);

    % 4. Exact Parity against Version A (+panda)
    T_a = panda.fk(q, m_orig);
    Js_a = panda.jacobian(q, m_orig, 'space');
    Jb_a = panda.jacobian(q, m_orig, 'body');
    [M_a, b_a] = panda.dynamics(q, omega, m_orig);
    tau_a = panda.inverse_dynamics(q, omega, domega, m_orig, F);
    ddq_a = panda.forward_dynamics(q, omega, tau_a, m_orig, -F);
    [H_a, K_a, V_a] = panda.energy(q, omega, m_orig);
    [H_s1, K_s1, V_s1] = panda_s1.energy(X, omega, m);

    report.version_a_fk_parity_error = max(report.version_a_fk_parity_error, norm(T - T_a, 'fro'));
    report.version_a_jacobian_parity_error = max(report.version_a_jacobian_parity_error, ...
        max(norm(Js - Js_a, 'fro'), norm(Jb - Jb_a, 'fro')));
    report.version_a_mass_parity_error = max(report.version_a_mass_parity_error, norm(M - M_a, 'fro') + norm(b - b_a));
    report.version_a_inverse_dynamics_parity_error_Nm = max(report.version_a_inverse_dynamics_parity_error_Nm, norm(tau_s1 - tau_a));
    report.version_a_forward_dynamics_parity_error = max(report.version_a_forward_dynamics_parity_error, norm(recovered - ddq_a));
    report.version_a_energy_parity_error_J = max(report.version_a_energy_parity_error_J, ...
        abs(H_s1 - H_a) + abs(K_s1 - K_a) + abs(V_s1 - V_a) + abs(V_s1 - VL));
end

% 5. Physical source anchor: M(7,7) from Gaz physical tables
expected_M77 = m.Icom(3, 3, 7) + m.mass(7) * sum(m.com(1:2, 7).^2);
X0 = panda_s1.from_q(z);
[M0, ~] = panda_s1.dynamics(X0, z, m);
assert(abs(M0(7, 7) - expected_M77) < 1e-12, 'panda_s1:test', 'Physical-table M77 mismatch');
report.physical_table_M77_kg_m2 = expected_M77;

% 6. Cache invalidation and general screw fallback tests (including Joint 6 > pi)
q_test = m.qmin + 0.4 * (m.qmax - m.qmin);
q_test(6) = 3.50; % Strictly above pi to test centered lift in screw fallback
X_test = panda_s1.from_q(q_test);

rnea_shared_kinematics_S1(X_test, m.Mlist, m.Slist, 'reset');
panda_s1.dynamics(X_test, q_test, m);
panda_s1.inverse_dynamics(X_test, q_test, z, m);
stats = rnea_shared_kinematics_S1(X_test, m.Mlist, m.Slist, 'stats');
assert(stats.hits >= 1 && stats.rebuilds == 1, 'panda_s1:test', 'Bitwise S1 cache hit failed');

% Perturb relative transform Mlist
altered = m;
altered.Mlist(1, 4, 3) = altered.Mlist(1, 4, 3) + 0.01;
[Mc, ~] = panda_s1.dynamics(X_test, q_test, altered);
Mgold = MassMatrix(q_test, altered.Mlist, altered.Glist, altered.Slist) + altered.Japp;
assert(norm(Mc - Mgold, 'fro') < 1e-10, 'panda_s1:test', 'Transform cache invalidation on S1 failed');

% Perturb screw axis Slist (introduces helical pitch on Joint 2 and Joint 6)
altered = m;
altered.Slist(6, 2) = altered.Slist(6, 2) + 0.01;
altered.Slist(6, 6) = altered.Slist(6, 6) + 0.01;
[Mc, ~] = panda_s1.dynamics(X_test, q_test, altered);
Mgold = MassMatrix(q_test, altered.Mlist, altered.Glist, altered.Slist) + altered.Japp;
assert(norm(Mc - Mgold, 'fro') < 1e-10, 'panda_s1:test', 'Screw cache invalidation on S1 failed');

% Legacy mode parity check in rnea_shared_kinematics_S1
kin_cached = rnea_shared_kinematics_S1(X_test, m.Mlist, m.Slist);
kin_legacy = rnea_shared_kinematics_S1(X_test, m.Mlist, m.Slist, 'legacy');
assert(norm(kin_cached.AdTi(:) - kin_legacy.AdTi(:), inf) < 1e-14, 'panda_s1:test', 'Legacy vs cached AdTi mismatch');

% 7. Optional apparent inertia and friction profile checks
extra = (1:7)' * 0.001;
other = panda_s1.model(struct('additional_apparent_inertia', extra, 'friction_profile', 'none'));
[Mo, ~] = panda_s1.dynamics(X_test, z, other);
[M_nom, ~] = panda_s1.dynamics(X_test, z, m);
assert(norm(Mo - M_nom - diag(extra), 'fro') < 1e-10, 'panda_s1:test', 'Optional inertia added once');

report.pass = report.mass_reference_error < 1e-10 ...
    && report.gravity_reference_error_Nm < 1e-10 ...
    && report.coriolis_reference_error_Nm < 1e-6 ...
    && report.fk_chain_error < 1e-10 ...
    && report.jacobian_finite_difference_error < 1e-7 ...
    && report.mass_asymmetry < 1e-10 ...
    && report.min_mass_eigenvalue > 0 ...
    && report.potential_gradient_error_Nm < 1e-6 ...
    && report.power_identity_error < 1e-6 ...
    && report.version_a_fk_parity_error < 1e-13 ...
    && report.version_a_jacobian_parity_error < 1e-13 ...
    && report.version_a_mass_parity_error < 1e-12 ...
    && report.version_a_inverse_dynamics_parity_error_Nm < 1e-12 ...
    && report.version_a_forward_dynamics_parity_error < 1e-11 ...
    && report.version_a_energy_parity_error_J < 1e-12 ...
    && report.forward_inverse_roundtrip_error < 1e-9 ...
    && report.wrench_duality_error < 1e-12 ...
    && report.minimum_friction_power_W >= 0;

disp(jsonencode(report, PrettyPrint=true));
assert(report.pass, 'panda_s1:test', 'Model validation on (S1)^7 failed');
end
