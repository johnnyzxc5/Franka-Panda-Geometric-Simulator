function f=friction(dq,m)
%FRICTION Published Gaz2019 eq.25 sigmoid in a stable vectorized tanh form.
% phi1_joint2=.87224 follows the authors' erratum. This law is zero at rest
% and monotone, hence dq'*f>=0. No copied upstream GPL friction program.
dq=panda.joint_vector(dq);
f=.5*m.friction.phi1.*(tanh(.5*m.friction.phi2.*(dq+m.friction.phi3)) ...
    -tanh(.5*m.friction.phi2.*m.friction.phi3));
if strcmp(m.friction_profile,'none'),f=zeros(7,1);end
end
