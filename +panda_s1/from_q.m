function X = from_q(q)
%FROM_Q Embed joint coordinates q into the global configuration manifold (S1)^7.
%
% Input:
%   q: 7-element numeric joint vector in radians
% Output:
%   X: 2-by-7 matrix where column i is [cos(q_i); sin(q_i)]

q = panda.joint_vector(q);
X = [cos(q)'; sin(q)'];
end
