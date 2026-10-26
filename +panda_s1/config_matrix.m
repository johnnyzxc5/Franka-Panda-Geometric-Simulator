function [X, max_norm_err] = config_matrix(X_in, tol)
%CONFIG_MATRIX Validate and normalize Panda (S1)^7 configuration matrix.
%
% Formats accepted:
%   - 2-by-7 matrix where column i is [c_i; s_i]
%   - 14-by-1 or 1-by-14 vector (reshaped column-wise into 2-by-7)
%
% Outputs:
%   - X: 2-by-7 double matrix
%   - max_norm_err: max_i |c_i^2 + s_i^2 - 1|

if nargin < 2
    tol = [];
end

assert(isnumeric(X_in) && all(isfinite(X_in(:))), ...
    'panda_s1:config_matrix', 'Configuration X must be finite numeric');

sz = size(X_in);
if isequal(sz, [2, 7])
    X = double(X_in);
elseif isequal(sz, [14, 1]) || isequal(sz, [1, 14])
    X = reshape(double(X_in), 2, 7);
elseif isequal(sz, [7, 2])
    X = double(X_in)';
else
    error('panda_s1:config_matrix', ...
        'Configuration must be 2-by-7, 7-by-2, or 14-element vector (got %dx%d)', sz(1), sz(2));
end

if nargout > 1 || ~isempty(tol)
    max_norm_err = max(abs(sum(X.^2, 1) - 1));
    if ~isempty(tol) && max_norm_err > tol
        error('panda_s1:unit_norm', ...
            'Columns of X must have unit norm on S^1 (max error: %e > tol: %e)', ...
            max_norm_err, tol);
    end
end
end
