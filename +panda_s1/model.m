function m = model(options)
%MODEL Panda seven-link model augmented for global (S1)^7 Lie-group dynamics.
%
% Wraps panda.model() and adds geometric arc limits on S^1, midpoints,
% half-spans, and precomputed local screw axis verification.

if nargin < 1
    options = struct();
end

m = panda.model(options);

% Precompute geometric limit arc parameters on S^1
m.qmid = 0.5 * (m.qmin + m.qmax);               % Arc center [rad], 7x1
m.qhalf = 0.5 * (m.qmax - m.qmin);              % Half span [rad], 7x1
m.umid = [cos(m.qmid)'; sin(m.qmid)'];          % Center unit vectors, 2x7
m.cos_half = cos(m.qhalf);                      % Limit threshold cos(h), 7x1

% Verify local screw axes A_i = Ad_{inv(M_1...M_i)} * S_i
m.is_pure_z = false(1, m.n);
Mi = eye(4);
m.Alist = zeros(6, m.n);
for i = 1:m.n
    Mi = Mi * m.Mlist(:, :, i);
    Ai = Adjoint(TransInv(Mi)) * m.Slist(:, i);
    m.Alist(:, i) = Ai;
    m.is_pure_z(i) = norm(Ai - [0; 0; 1; 0; 0; 0], inf) < 1e-10;
end

m.representation = 'S1';
m.scope = [m.scope, '; Global Lie-group formulation on (S^1)^7'];
end
