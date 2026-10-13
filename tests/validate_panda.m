function report=validate_panda()
%VALIDATE_PANDA Independent mechanics, Lie identities and kernel parity.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setup_panda();
rng(13102026,'twister');m=panda.model();n=m.n;z=zeros(n,1);ep=1e-5;
report=struct('seed',13102026,'states',40,'matlab_version',version, ...
    'mass_reference_error',0,'gravity_reference_error_Nm',0,'coriolis_reference_error_Nm',0, ...
    'fk_chain_error',0,'jacobian_finite_difference_error',0,'mass_asymmetry',0, ...
    'min_mass_eigenvalue',Inf,'potential_gradient_error_Nm',0,'power_identity_error',0, ...
    'stock_inverse_dynamics_error_Nm',0,'stock_mass_error',0, ...
    'forward_inverse_roundtrip_error',0,'wrench_duality_error',0, ...
    'minimum_friction_power_W',Inf,'adjoint_exponential_identity_error',0);
for k=1:report.states
    q=m.qmin+.05+(m.qmax-m.qmin-.1).*rand(n,1);
    dq=randn(n,1);ddq=randn(n,1);F=randn(6,1);
    [M,~,parts]=panda.dynamics(q,dq,m);[ML,gL]=panda.lagrange_reference(q,m);
    [~,Tchain]=panda.frames(q,m);T=panda.fk(q,m);
    report.fk_chain_error=max(report.fk_chain_error,norm(T-Tchain,'fro'));
    report.mass_reference_error=max(report.mass_reference_error,norm(M-ML,'fro')/norm(ML,'fro'));
    report.gravity_reference_error_Nm=max(report.gravity_reference_error_Nm,norm(parts.gravity-gL));
    report.mass_asymmetry=max(report.mass_asymmetry,norm(M-M','fro'));
    report.min_mass_eigenvalue=min(report.min_mass_eigenvalue,min(eig((M+M')/2)));
    Js=panda.jacobian(q,m,'space');Jb=panda.jacobian(q,m,'body');
    derivative=zeros(n,n,n);gradient=z;Jfd=zeros(6,n);
    for j=1:n
        d=z;d(j)=ep;[Mp,~,Vp]=panda.lagrange_reference(q+d,m);[Mm,~,Vm]=panda.lagrange_reference(q-d,m);
        derivative(:,:,j)=(Mp-Mm)/(2*ep);gradient(j)=(Vp-Vm)/(2*ep);
        Tp=panda.fk(q+d,m);Tm=panda.fk(q-d,m);twist=(Tp-Tm)/(2*ep)*TransInv(T);
        Jfd(:,j)=[so3ToVec(twist(1:3,1:3));twist(1:3,4)];
    end
    Mdot=zeros(n);gradK=z;
    for j=1:n,Mdot=Mdot+derivative(:,:,j)*dq(j);gradK(j)=dq'*derivative(:,:,j)*dq;end
    cL=Mdot*dq-.5*gradK;
    report.coriolis_reference_error_Nm=max(report.coriolis_reference_error_Nm,norm(parts.coriolis-cL));
    report.potential_gradient_error_Nm=max(report.potential_gradient_error_Nm,norm(parts.gravity-gradient));
    report.power_identity_error=max(report.power_identity_error,abs(dq'*parts.coriolis-.5*dq'*Mdot*dq));
    report.jacobian_finite_difference_error=max(report.jacobian_finite_difference_error,norm(Js-Jfd,'fro'));
    tr=InverseDynamics(q,dq,ddq,m.gravity,F,m.Mlist,m.Glist,m.Slist)+m.Japp*ddq+panda.friction(dq,m);
    to=panda.inverse_dynamics(q,dq,ddq,m,F);
    report.stock_inverse_dynamics_error_Nm=max(report.stock_inverse_dynamics_error_Nm,norm(tr-to));
    Mr=MassMatrix(q,m.Mlist,m.Glist,m.Slist)+m.Japp;
    report.stock_mass_error=max(report.stock_mass_error,norm(Mr-M,'fro'));
    recovered=panda.forward_dynamics(q,dq,to,m,-F);
    report.forward_inverse_roundtrip_error=max(report.forward_inverse_roundtrip_error,norm(recovered-ddq));
    report.wrench_duality_error=max(report.wrench_duality_error,abs(F'*(Jb*dq)-(Jb'*F)'*dq));
    report.minimum_friction_power_W=min(report.minimum_friction_power_W,dq'*parts.friction);
    X=randn(6,1)*.2;
    report.adjoint_exponential_identity_error=max(report.adjoint_exponential_identity_error, ...
        norm(T*MatrixExp6(VecTose3(X))*TransInv(T)-MatrixExp6(VecTose3(Adjoint(T)*X)),'fro'));
end
% Source anchor: last-axis inertia from the feasible physical tables.
% This is deliberately not the raw identified coefficient in Gaz's generated M.
expected=m.Icom(3,3,7)+m.mass(7)*sum(m.com(1:2,7).^2);
[M,~]=panda.dynamics(z,z,m);
assert(abs(M(7,7)-expected)<1e-12,'panda:test','Physical-table M77');
report.physical_table_M77_kg_m2=expected;
assert(norm(m.Japp)==0&&all(isfinite(m.qmin))&&all(isfinite(m.qmax)));
assert(m.qmax(4)<0&&m.qmax(6)>pi&&m.friction.phi1(2)==.87224);
% Cache invalidation under changed relative geometry and screw axes.
q=m.qmin+.4*(m.qmax-m.qmin);
altered=m;altered.Mlist(1,4,3)=altered.Mlist(1,4,3)+.01;
rnea_shared_kinematics_MR(q,m.Mlist,m.Slist,'reset');
panda.dynamics(q,q,m);[Mc,~]=panda.dynamics(q,q,altered);
Mgold=MassMatrix(q,altered.Mlist,altered.Glist,altered.Slist)+altered.Japp;
assert(norm(Mc-Mgold,'fro')<1e-10,'panda:test','Transform cache invalidation');
altered=m;altered.Slist(6,2)=altered.Slist(6,2)+.01;
[Mc,~]=panda.dynamics(q,q,altered);
Mgold=MassMatrix(q,altered.Mlist,altered.Glist,altered.Slist)+altered.Japp;
assert(norm(Mc-Mgold,'fro')<1e-10,'panda:test','Screw cache invalidation');
extra=(1:7)'*.001;other=panda.model(struct('additional_apparent_inertia',extra,'friction_profile','none'));
[Mo,~]=panda.dynamics(q,z,other);[M,~]=panda.dynamics(q,z,m);
assert(norm(Mo-M-diag(extra),'fro')<1e-10,'panda:test','Optional inertia added once');
assert(all(panda.friction(z,m)==0)&&all(panda.friction(q,other)==0));
for vel=[-1e6,-1,-.01,0,.01,1,1e6]
    v=vel*ones(n,1);f=panda.friction(v,m);assert(all(isfinite(f))&&v'*f>=-1e-12);
end
% Stable tanh implementation versus the published sigmoid at moderate speed.
v=(1:n)'/10-.4;
direct=m.friction.phi1.*(1./(1+exp(-m.friction.phi2.*(v+m.friction.phi3))) ...
    -1./(1+exp(-m.friction.phi2.*m.friction.phi3)));
assert(norm(panda.friction(v,m)-direct)<1e-12);
report.pass=report.mass_reference_error<1e-10&&report.gravity_reference_error_Nm<1e-10 ...
    &&report.coriolis_reference_error_Nm<1e-6&&report.fk_chain_error<1e-10 ...
    &&report.jacobian_finite_difference_error<1e-7&&report.mass_asymmetry<1e-10 ...
    &&report.min_mass_eigenvalue>0&&report.potential_gradient_error_Nm<1e-6 ...
    &&report.power_identity_error<1e-6&&report.stock_inverse_dynamics_error_Nm<1e-10 ...
    &&report.stock_mass_error<1e-10&&report.forward_inverse_roundtrip_error<1e-9 ...
    &&report.wrench_duality_error<1e-12&&report.minimum_friction_power_W>=0 ...
    &&report.adjoint_exponential_identity_error<1e-10;
panda.write_json(fullfile(root,'results','validation.json'),report);
disp(jsonencode(report,PrettyPrint=true));assert(report.pass,'panda:test','Model validation failed');
end
