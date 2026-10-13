function f=wrench_vector(f)
%WRENCH_VECTOR Wrenches always have six entries [moment;force], even on a 7R arm.
assert(isnumeric(f)&&isreal(f)&&numel(f)==6&&all(isfinite(f(:))), ...
    'panda:wrench','Expected six finite real wrench entries');f=double(f(:));
end
