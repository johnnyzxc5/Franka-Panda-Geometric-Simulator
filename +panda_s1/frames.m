function [chain, T] = frames(X, m)
%FRAMES Successive link poses on SE(3) computed algebraically from (S1)^7.
%
% Replaces MatrixExp6 with direct planar rotation around local z-axis.
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   m: (optional) model struct from panda_s1.model()
% Outputs:
%   chain: 4-by-4-by-8 array of link poses in base frame
%   T: 4-by-4 flange pose in base frame (same as chain(:,:,8))

if nargin < 2 || isempty(m) || ~isfield(m, 'Mlist')
    m = panda_s1.model();
end

X = panda_s1.config_matrix(X);
T = eye(4);
chain = zeros(4, 4, m.n + 1);

for i = 1:m.n + 1
    T = T * m.Mlist(:, :, i);
    if i <= m.n
        ci = X(1, i);
        si = X(2, i);
        % Post-multiply by R_z(x_i): columns 1 and 2 are rotated
        T = [ ci*T(:,1) + si*T(:,2), -si*T(:,1) + ci*T(:,2), T(:,3), T(:,4) ];
    end
    chain(:, :, i) = T;
end
end
