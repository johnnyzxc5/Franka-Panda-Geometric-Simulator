function [in_limits, margins, max_norm_err, report] = check_limits(X, m, tol, tol_norm)
%CHECK_LIMITS Check joint limit arc inequalities and unit norms on (S1)^7.
%
% Mathematically rigorous admissibility condition for joint i on (S1)^7:
%   1. Manifold membership:  |x_i^T * x_i - 1| <= tol_norm
%   2. Admissible arc:       u_i^T * x_hat_i >= cos(h_i) - tol  AND  mu_i >= -tol
% where x_hat_i = x_i / ||x_i|| is the radially projected unit direction,
% u_i = [cos(m_i); sin(m_i)] is the arc center, and h_i = (qmax_i - qmin_i)/2.
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7 (or 14-element vector)
%   m: (optional) model struct from panda_s1.model()
%   tol: (optional, default 1e-10) numerical tolerance for arc inequality [rad]
%   tol_norm: (optional, default 1e-10) tolerance for unit-norm manifold condition
% Outputs:
%   in_limits: 7-by-1 logical array (true if joint satisfies both manifold & arc)
%   margins: 7-by-1 vector of signed distance to nearest limit in radians (>=0 is safe)
%   max_norm_err: maximum column norm deviation |x_i^T * x_i - 1|
%   report: struct with detailed diagnostics (on_manifold, in_arc, etc.)

if nargin < 2 || isempty(m) || ~isfield(m, 'qmid')
    m = panda_s1.model();
end
if nargin < 3 || isempty(tol)
    tol = 1e-10;
end
if nargin < 4 || isempty(tol_norm)
    tol_norm = 1e-10;
end

X = panda_s1.config_matrix(X);

c = X(1, :)';
s = X(2, :)';
cm = m.umid(1, :)';
sm = m.umid(2, :)';

% 1. Manifold condition on (S1)^7: |c_i^2 + s_i^2 - 1| <= tol_norm
norm_sq = c.^2 + s.^2;
r = sqrt(norm_sq);
unit_norm_err = abs(norm_sq - 1);
max_norm_err = max(unit_norm_err);
on_manifold = (unit_norm_err <= tol_norm);

% 2. Normalized direction for scale-invariant angular evaluation
valid_norm = (r > eps);
c_hat = zeros(7, 1);
s_hat = zeros(7, 1);
c_hat(valid_norm) = c(valid_norm) ./ r(valid_norm);
s_hat(valid_norm) = s(valid_norm) ./ r(valid_norm);

% Dot product and wedge product with arc center unit vector u_i
u_dot_x_hat = c_hat .* cm + s_hat .* sm;
u_wedge_x_hat = s_hat .* cm - c_hat .* sm;
u_dot_x_hat(~valid_norm) = -Inf;

% Exact signed offset from arc center in radians: delta_i in (-pi, pi]
delta = atan2(u_wedge_x_hat, u_dot_x_hat);
delta(~valid_norm) = NaN;

% Exact margin to closest limit in radians: mu_i = h_i - |delta_i|
margins = m.qhalf - abs(delta);
margins(~valid_norm) = -Inf;

% 3. Arc inequality evaluated on normalized unit direction
in_arc = valid_norm & (u_dot_x_hat >= (m.cos_half - tol)) & (margins >= -tol);

% 4. Strict combined admissibility: on manifold AND within arc
in_limits = on_manifold & in_arc;

if nargout > 3
    report = struct();
    report.in_limits = in_limits;
    report.all_in_limits = all(in_limits);
    report.on_manifold = on_manifold;
    report.all_on_manifold = all(on_manifold);
    report.in_arc = in_arc;
    report.all_in_arc = all(in_arc);
    report.margins_rad = margins;
    report.min_margin_rad = min(margins);
    report.max_limit_violation_rad = max(0, -min(margins));
    report.unit_norm_error = unit_norm_err;
    report.max_unit_norm_error = max_norm_err;
    report.u_dot_x = c .* cm + s .* sm;
    report.u_dot_x_normalized = u_dot_x_hat;
    report.cos_half = m.cos_half;
    report.delta_from_center_rad = delta;
end
end
