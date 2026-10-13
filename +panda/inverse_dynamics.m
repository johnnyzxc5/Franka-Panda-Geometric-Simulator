function tau=inverse_dynamics(q,dq,ddq,m,Ftip)
%INVERSE_DYNAMICS Total joint-side effort required by the mechanical model.
% Ftip is the wrench the flange applies TO the environment, expressed in the
% flange frame; [moment;force]. Pass its negative for an applied environment wrench.
if nargin<5,Ftip=zeros(6,1);end
q=panda.joint_vector(q);dq=panda.joint_vector(dq);ddq=panda.joint_vector(ddq);
Ftip=panda.wrench_vector(Ftip);
kin=rnea_shared_kinematics_MR(q,m.Mlist,m.Slist);
tau=InverseDynamicsSharedKinematics_MR(dq,ddq,m.gravity,Ftip,m.Glist,kin) ...
    +m.Japp*ddq+panda.friction(dq,m);
end
