function [in_limits, margins, max_norm_err, report] = check_limits(X, m, tol)
%CHECK_LIMITS Check joint limit arc inequalities and unit norms on (S1)^7.
%
% Condition for joint i to be in physical limits [qmin_i, qmax_i]:
%   u_i^T * x_i >= cos(h_i)
% where u_i = [cos(m_i); sin(m_i)] and h_i = (qmax_i - qmin_i)/2.
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   m: (optional) model struct from panda_s1.model()
%   tol: (optional, default 1e-10) numerical tolerance
% Outputs:
%   in_limits: 7-by-1 logical array (true if joint is within admissible arc)
%   margins: 7-by-1 vector of signed distance to nearest limit in radians (>=0 is safe)
%   max_norm_err: maximum column norm deviation from 1 on S^1
%   report: struct with detailed diagnostics

if nargin < 2 || isempty(m) || ~isfield(m, 'qmid')
    m = panda_s1.model();
end
if nargin < 3 || isempty(tol)
    tol = 1e-10;
end

[X, max_norm_err] = panda_s1.config_matrix(X);

c = X(1, :)';
s = X(2, :)';
cm = m.umid(1, :)';
sm = m.umid(2, :)';

% Dot product with arc center unit vector: u_i^T * x_i = cos(delta_i)
u_dot_x = c .* cm + s .* sm;

% Arc inequality: u_dot_x >= cos_half
in_limits = (u_dot_x >= (m.cos_half - tol));

% Exact signed offset from arc center in radians: delta_i in (-pi, pi]
u_wedge_x = s .* cm - c .* sm;
delta = atan2(u_wedge_x, u_dot_x);

% Exact margin to closest limit in radians: mu_i = h_i - |delta_i|
margins = m.qhalf - abs(delta);

if nargout > 3
    report = struct();
    report.in_limits = in_limits;
    report.all_in_limits = all(in_limits);
    report.margins_rad = margins;
    report.min_margin_rad = min(margins);
    report.max_limit_violation_rad = max(0, -min(margins));
    report.max_unit_norm_error = max_norm_err;
    report.u_dot_x = u_dot_x;
    report.cos_half = m.cos_half;
    report.delta_from_center_rad = delta;
end
end
