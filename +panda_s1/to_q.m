function q = to_q(X, m)
%TO_Q Centered branch-cut-safe lift from (S1)^7 to joint angles q in [qmin, qmax].
%
% Uses the center of the physical arc m_i = (qmin_i + qmax_i)/2 to ensure that
% the branch cut of atan2 lies at m_i +/- pi (strictly in the forbidden sector
% S^1 \ A_i). This is strictly required for Joint 4 (all negative) and Joint 6
% (whose upper limit 3.7525 rad crosses +pi).
%
% Inputs:
%   X: 2-by-7 configuration matrix or 14-element vector on (S1)^7
%   m: (optional) model struct from panda_s1.model()
% Output:
%   q: 7-by-1 physical joint vector in radians

if nargin < 2 || isempty(m) || ~isfield(m, 'qmid')
    m = panda_s1.model();
end

X = panda_s1.config_matrix(X);

c = X(1, :)';
s = X(2, :)';
cm = m.umid(1, :)';
sm = m.umid(2, :)';

% delta_i = atan2(u_i^T * J^T * x_i, u_i^T * x_i) in (-pi, pi]
delta = atan2(s .* cm - c .* sm, c .* cm + s .* sm);
q = m.qmid + delta;
end
