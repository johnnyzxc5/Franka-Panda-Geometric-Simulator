function T = fk(X, m)
%FK Base-to-flange forward kinematics on SE(3) directly from (S1)^7.
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   m: (optional) model struct from panda_s1.model()
% Output:
%   T: 4-by-4 homogeneous transformation of bare flange panda_link8

if nargin < 2 || isempty(m)
    m = panda_s1.model();
end

[~, T] = panda_s1.frames(X, m);
end
