function report=test_source_geometry()
%TEST_SOURCE_GEOMETRY Modified-DH model versus manufacturer joint-origin facts.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setup_panda();
p=jsondecode(fileread(fullfile(root,'sources','joint_origins.json')));m=panda.model();
relative_error=0;pose_error=0;axis_error=0;
for i=1:8
    T=eye(4);T(1:3,1:3)=panda.rpy_rotation(p.rpy_rows(i,:)');T(1:3,4)=p.xyz_rows(i,:)';
    relative_error=max(relative_error,norm(T-m.Mlist(:,:,i),'fro'));
    if i<=7,axis_error=max(axis_error,norm(p.axis_rows(i,:)'-m.axis_local(:,i)));end
end
% Independent URDF-origin recursion, including the fixed bare flange.
q=m.qmin+.6*(m.qmax-m.qmin);T=eye(4);
for i=1:8
    origin=eye(4);origin(1:3,1:3)=panda.rpy_rotation(p.rpy_rows(i,:)');origin(1:3,4)=p.xyz_rows(i,:)';
    T=T*origin;
    if i<=7
        cq=cos(q(i));sq=sin(q(i));T=T*[cq,-sq,0,0;sq,cq,0,0;0,0,1,0;0,0,0,1];
    end
end
pose_error=norm(T-panda.fk(q,m),'fro');
report=struct('source_relative_pose_error',relative_error,'source_axis_error',axis_error, ...
    'source_flange_pose_error',pose_error,'pass',relative_error<1e-14&&axis_error<1e-14&&pose_error<1e-14);
panda.write_json(fullfile(root,'results','source_geometry_validation.json'),report);
assert(report.pass,'panda:test','Manufacturer frame cross-check');disp(report);
end
