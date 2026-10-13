function m=model(options)
%MODEL Panda seven-link identified rigid mechanical model, Gaz et al.2019.
% Published COM inertias and friction facts; no upstream generated model code.
% Torque is total link-side generalized effort, not a direct FCI command.
if nargin<1,options=struct();end
assert(isstruct(options)&&isscalar(options),'panda:model','Options must be a scalar struct');
o=struct('gravity',[0;0;-9.80665],'friction_profile','gaz2019_sigmoid', ...
    'additional_apparent_inertia',zeros(7,1));
names=fieldnames(options);
for i=1:numel(names)
    assert(isfield(o,names{i}),'panda:model','Unknown option: %s',names{i});o.(names{i})=options.(names{i});
end
root=fileparts(fileparts(mfilename('fullpath')));
p=jsondecode(fileread(fullfile(root,'sources','parameters.json')));
m=struct('n',7,'name','Franka Emika Panda, Gaz2019 physical parameter set', ...
    'options',o,'gravity',o.gravity(:),'parameter_profile','gaz2019_physical', ...
    'actuator_profile','link_side_rigid','friction_profile',o.friction_profile, ...
    'parameter_revision','Gaz2019 revised supplement, 2019-10-15', ...
    'urdf_revision','ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98');
assert(numel(m.gravity)==3&&all(isfinite(m.gravity)));
extra=o.additional_apparent_inertia(:);
assert(numel(extra)==7&&all(isfinite(extra))&&all(extra>=0));m.Japp=diag(extra);
m.mass=p.mass(:);m.com=p.com_rows';m.Icom=zeros(3,3,7);
m.Mlist=repmat(eye(4),1,1,8);m.home_links=zeros(4,4,8);
m.Glist=zeros(6,6,7);m.Slist=zeros(6,7);m.axis_local=repmat([0;0;1],1,7);
m.a=p.dh_a(:);m.d=p.dh_d(:);m.alpha=p.dh_alpha(:);
m.qmin=p.lower(:);m.qmax=p.upper(:);m.dqmax=p.velocity(:);m.taumax=p.effort(:);
T=eye(4);
for i=1:8
    ca=cos(m.alpha(i));sa=sin(m.alpha(i));
    % Modified DH at q=0: Rx(alpha)*Tx(a)*Tz(d), relative body frames.
    Ti=[1,0,0,m.a(i);0,ca,-sa,-sa*m.d(i);0,sa,ca,ca*m.d(i);0,0,0,1];
    m.Mlist(:,:,i)=Ti;T=T*Ti;m.home_links(:,:,i)=T;
    if i<=7
        w=T(1:3,3);r=T(1:3,4);m.Slist(:,i)=[w;-cross(w,r)];
        v=p.inertia_rows(i,:);I=[v(1),v(2),v(3);v(2),v(4),v(5);v(3),v(5),v(6)];m.Icom(:,:,i)=I;
        C=VecToso3(m.com(:,i));m.Glist(:,:,i)=[I-m.mass(i)*C*C,m.mass(i)*C;-m.mass(i)*C,m.mass(i)*eye(3)];
        eigI=eig(I);assert(min(eigI)>0&&sum(eigI)-2*max(eigI)>-1e-10,'panda:inertia','Invalid physical tensor');
    end
end
m.home=T;m.Blist=Adjoint(TransInv(T))*m.Slist;
m.friction=struct('phi1',p.friction_phi1(:),'phi2',p.friction_phi2(:),'phi3',p.friction_phi3(:));
assert(all(m.friction.phi1>0)&&all(m.friction.phi2>0));
assert(any(strcmp(char(o.friction_profile),{'gaz2019_sigmoid','none'})));
m.scope='Published seven-link rigid link-side model; no separate hand, payload, electrical or elastic joint model';
end
