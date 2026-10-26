function kin = rnea_shared_kinematics_S1(X, Mlist, Slist, mode)
%RNEA_SHARED_KINEMATICS_S1 Exact Lie kinematics on (S1)^7 with invariant hoisting and S^1 caching.
%
% Replaces transcendental MatrixExp6 calls with direct algebraic SO(2)
% rotations for pure-z joints, with automatic fallback for general screw axes.
% Produces exact AdTi matrices consumed by InverseDynamicsSharedKinematics_MR
% and MassMatrixZeroStateSharedKinematics_MR without code duplication.
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   Mlist: 4-by-4-by-(n+1) relative home transforms
%   Slist: 6-by-n space screws
%   mode: (optional) 'cached' (default), 'reset', 'stats', 'legacy'

persistent cached_Mlist cached_Slist cached_n cached_Ai cached_invM cached_AdTi_end
persistent cached_is_pure_z cached_X_u64 cached_kin
persistent stats_calls stats_hits stats_misses stats_rebuilds

if isempty(stats_calls)
    stats_calls = uint64(0);
    stats_hits = uint64(0);
    stats_misses = uint64(0);
    stats_rebuilds = uint64(0);
end

if nargin >= 4 && strcmp(mode, 'reset')
    cached_Mlist = []; cached_Slist = []; cached_n = [];
    cached_Ai = []; cached_invM = []; cached_AdTi_end = [];
    cached_is_pure_z = []; cached_X_u64 = []; cached_kin = [];
    stats_calls = uint64(0); stats_hits = uint64(0);
    stats_misses = uint64(0); stats_rebuilds = uint64(0);
    kin = struct('Ai', zeros(6, 0), 'AdTi', zeros(6, 6, 0), 'n', 0);
    return;
end

if nargin >= 4 && strcmp(mode, 'stats')
    kin = struct('calls', stats_calls, 'hits', stats_hits, ...
                 'misses', stats_misses, 'rebuilds', stats_rebuilds);
    return;
end

X = panda_s1.config_matrix(X);
n = size(X, 2);

stats_calls = stats_calls + 1;

% Invariant Hoisting: rebuild only if structural parameters changed
need_rebuild = isempty(cached_Mlist) || isempty(cached_Slist) || ...
    ~isequal(size(cached_Mlist), size(Mlist)) || ...
    ~isequal(size(cached_Slist), size(Slist)) || ...
    ~strcmp(class(cached_Mlist), class(Mlist)) || ...
    ~strcmp(class(cached_Slist), class(Slist)) || ...
    ~isequal(cached_Mlist, Mlist) || ...
    ~isequal(cached_Slist, Slist);

if need_rebuild
    cached_n = n;
    cached_Mlist = Mlist;
    cached_Slist = Slist;
    Mi = eye(4);
    cached_Ai = zeros(6, n);
    cached_invM = zeros(4, 4, n);
    cached_is_pure_z = false(1, n);
    for i = 1:n
        Mi = Mi * Mlist(:, :, i);
        cached_Ai(:, i) = Adjoint(TransInv(Mi)) * Slist(:, i);
        cached_invM(:, :, i) = TransInv(Mlist(:, :, i));
        cached_is_pure_z(i) = norm(cached_Ai(:, i) - [0; 0; 1; 0; 0; 0], inf) < 1e-10;
    end
    cached_AdTi_end = Adjoint(TransInv(Mlist(:, :, n+1)));
    cached_X_u64 = [];
    cached_kin = [];
    stats_rebuilds = stats_rebuilds + 1;
end

% STATE CACHE LOOKUP: exact raw bitwise uint64 comparison across 14 doubles
X_u64 = typecast(X(:), 'uint64');
if ~need_rebuild && ~isempty(cached_X_u64)
    if all(cached_X_u64 == X_u64)
        stats_hits = stats_hits + 1;
        kin = cached_kin;
        return;
    end
end

stats_misses = stats_misses + 1;

AdTi = zeros(6, 6, n+1);
AdTi(:, :, n+1) = cached_AdTi_end;

for i = 1:n
    ci = X(1, i);
    si = X(2, i);
    if cached_is_pure_z(i)
        % Exact algebraic SO(2) inverse rotation R_z(-q_i) = R_z(c_i, -s_i)
        % T_inv = [c_i, s_i, 0, 0; -s_i, c_i, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1] * cached_invM
        invM = cached_invM(:, :, i);
        T_inv = [ ci * invM(1,:) + si * invM(2,:); ...
                 -si * invM(1,:) + ci * invM(2,:); ...
                  invM(3,:); ...
                  invM(4,:) ];
        AdTi(:, :, i) = Adjoint(T_inv);
    else
        % General screw fallback (e.g. artificial perturbation in tests)
        qi = atan2(si, ci);
        AdTi(:, :, i) = Adjoint(MatrixExp6(VecTose3(cached_Ai(:, i) * -qi)) * cached_invM(:, :, i));
    end
end

kin = struct('Ai', cached_Ai, 'AdTi', AdTi, 'n', n);

cached_X_u64 = X_u64;
cached_kin = kin;
end
