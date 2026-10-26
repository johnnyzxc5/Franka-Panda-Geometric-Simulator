function [dtheta_geo, crosses_forbidden_arc, chordal_error] = geodesic_diff(X2, X1, m)
%GEODESIC_DIFF Intrinsic SO(2) geodesic difference between two states on (S1)^7.
%
% Computes the shortest angular displacement from X1 to X2 along S^1:
%   dtheta_geo_i = atan2(x1_i^T * J^T * x2_i, x1_i^T * x2_i) in (-pi, pi]
%
% CRITICAL TOPOLOGICAL NOTE (Milestone 1, Section 5.3):
%   Because 6 of the 7 Panda joints have allowable span 2*h_i > pi,
%   two admissible configurations with |q2_i - q1_i| > pi will have a shortest
%   geodesic arc that CROSSES THE FORBIDDEN SECTOR S^1 \ A_i (the mechanical stop).
%   In that case, crosses_forbidden_arc(i) is TRUE, and dtheta_geo has the OPPOSITE
%   sign of the physically admissible motion. For control and interpolation,
%   use panda_s1.box_diff instead.
%
% Inputs:
%   X2: 2-by-7 target configuration on (S1)^7
%   X1: 2-by-7 source configuration on (S1)^7
%   m: (optional) model struct from panda_s1.model()
% Outputs:
%   dtheta_geo: 7-by-1 vector of shortest arc angles in (-pi, pi]
%   crosses_forbidden_arc: 7-by-1 logical array (true if shortest path crosses stop)
%   chordal_error: 7-by-1 vector of Lee chordal errors x1^T * J^T * x2 = sin(q2 - q1)

if nargin < 3 || isempty(m) || ~isfield(m, 'qmid')
    m = panda_s1.model();
end

X1 = panda_s1.config_matrix(X1);
X2 = panda_s1.config_matrix(X2);

c1 = X1(1, :); s1 = X1(2, :);
c2 = X2(1, :); s2 = X2(2, :);

dot_val = c1 .* c2 + s1 .* s2;
wedge_val = s2 .* c1 - c2 .* s1;

dtheta_geo = atan2(wedge_val, dot_val)';
chordal_error = wedge_val';

if nargout > 1
    cm = m.umid(1, :); sm = m.umid(2, :);
    delta1 = atan2(s1 .* cm - c1 .* sm, c1 .* cm + s1 .* sm);
    delta2 = atan2(s2 .* cm - c2 .* sm, c2 .* cm + s2 .* sm);
    crosses_forbidden_arc = (abs(delta2 - delta1)' > (pi + 1e-12));
end
end
