function T = fk(X, m)
%FK Space product of exponentials on (S1)^7: exp([S1],x1)...exp([S7],x7)*M_home.
%
% Evaluates the space PoE algebraically on (S1)^7 via Theorem 3.1 (screw_exp),
% completely independent of the relative body chain panda_s1.frames(X, m).
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   m: (optional) model struct from panda_s1.model()
% Output:
%   T: 4-by-4 homogeneous transformation of bare flange panda_link8

if nargin < 2 || isempty(m)
    m = panda_s1.model();
end

X = panda_s1.config_matrix(X);
T = m.home;
has_mid = isfield(m, 'umid') && isfield(m, 'qmid');

for i = m.n:-1:1
    if has_mid
        Ti = panda_s1.screw_exp(m.Slist(:, i), X(:, i), m.umid(:, i), m.qmid(i));
    else
        Ti = panda_s1.screw_exp(m.Slist(:, i), X(:, i));
    end
    T = Ti * T;
end
end
