function [Xref, wref, dwref, qref] = reference(t, c, m, phase, X0, d_box)
%REFERENCE Intrinsic (S1)^7 reference trajectory along the admissible arc A.
%
% Given initial configuration X0 and goal configuration Xgoal in A subset (S1)^7,
% interpolates along the contractible admissible arc via the exponential map:
%   Xref(t) = Exp_{S1}( s(t) * box_diff(Xgoal, X0, m), X0 )
%   wref(t) = v(t) * box_diff(Xgoal, X0, m)
%   dwref(t) = a(t) * box_diff(Xgoal, X0, m)
%
% By convexity of the centered arc [-h_i, h_i], Xref(t) is guaranteed to remain
% strictly inside the admissible arc A for all s(t) in [0, 1], even when
% |qgoal_i - q0_i| > pi (where a geodesic on S^1 would cross the hard stop).

if nargin < 3 || isempty(m)
    m = panda_s1.model();
end
if nargin < 4 || isempty(phase)
    phase = ftv5.phase_plan(0, 0, 0, c.motion_time);
end
if nargin < 5 || isempty(X0)
    X0 = panda_s1.from_q(c.q0);
end
if nargin < 6 || isempty(d_box)
    Xgoal = panda_s1.from_q(c.qgoal);
    d_box = panda_s1.box_diff(Xgoal, X0, m);
end

[s, v, a] = ftv5.phase(phase, t);
dtheta = d_box * s;
Xref = panda_s1.exp_s1(dtheta, X0, true);
wref = d_box * v;
dwref = d_box * a;

if nargout > 3
    qref = panda.joint_vector(c.q0) + d_box * s;
end
end
