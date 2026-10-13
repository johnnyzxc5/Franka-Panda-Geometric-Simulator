function q=joint_vector(q)
%JOINT_VECTOR Normalize finite real seven-joint coordinates or joint efforts.
assert(isnumeric(q)&&isreal(q)&&numel(q)==7&&all(isfinite(q(:))), ...
    'panda:input','Expected seven finite real joint values');q=double(q(:));
end
