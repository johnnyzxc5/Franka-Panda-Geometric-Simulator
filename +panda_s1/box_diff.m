function dq_box = box_diff(X2, X1, m)
%BOX_DIFF Admissible-arc displacement between configurations on (S1)^7.
%
% Computes the unique continuous displacement along the physical admissible
% arc A_i = [qmin_i, qmax_i] without wrapping across the mechanical hard stop:
%   dq_box_i = delta_i(X2) - delta_i(X1) in [-2*h_i, 2*h_i]
%
% Agreement with geodesic difference:
%   When |dq_box_i| < pi, dq_box_i == geodesic_diff_i identically.
%   When |dq_box_i| >= pi, geodesic_diff points across the forbidden stop,
%   whereas box_diff correctly guides motion along the allowed physical span.
%
% Inputs:
%   X2: 2-by-7 target configuration on (S1)^7
%   X1: 2-by-7 source configuration on (S1)^7
%   m: (optional) model struct from panda_s1.model()
% Output:
%   dq_box: 7-by-1 physical joint displacement vector [rad]

if nargin < 3 || isempty(m) || ~isfield(m, 'qmid')
    m = panda_s1.model();
end

X1 = panda_s1.config_matrix(X1);
X2 = panda_s1.config_matrix(X2);

c1 = X1(1, :); s1 = X1(2, :);
c2 = X2(1, :); s2 = X2(2, :);
cm = m.umid(1, :); sm = m.umid(2, :);

delta1 = atan2(s1 .* cm - c1 .* sm, c1 .* cm + s1 .* sm);
delta2 = atan2(s2 .* cm - c2 .* sm, c2 .* cm + s2 .* sm);

dq_box = (delta2 - delta1)';
end
