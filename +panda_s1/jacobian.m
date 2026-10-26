function J = jacobian(X, m, frame)
%JACOBIAN Space or flange-body twist Jacobian on (S1)^7, rows [angular; linear].
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   m: (optional) model struct from panda_s1.model()
%   frame: (optional, default 'space') 'space' or 'body'
% Output:
%   J: 6-by-7 twist Jacobian matrix

if nargin < 3
    frame = 'space';
end
if nargin < 2 || isempty(m)
    m = panda_s1.model();
end

X = panda_s1.config_matrix(X);
q = panda_s1.to_q(X, m);
J = JacobianSpace(m.Slist, q);

switch char(frame)
    case 'space'
    case 'body'
        T = panda_s1.fk(X, m);
        J = Adjoint(TransInv(T)) * J;
    otherwise
        error('panda_s1:frame', 'Frame must be space or body');
end
end
