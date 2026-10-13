function J=jacobian(q,m,frame)
%JACOBIAN Space or flange-body twist Jacobian, rows [angular; linear].
if nargin<3,frame='space';end
q=panda.joint_vector(q);J=JacobianSpace(m.Slist,q);
switch char(frame)
    case 'space'
    case 'body',J=Adjoint(TransInv(panda.fk(q,m)))*J;
    otherwise,error('panda:frame','Frame must be space or body');
end
end
