function log=simulate(m,c)
%SIMULATE Sampled computed effort, zero-order hold and mechanical RK4 plant.
% Reference generation is outside the plant. Controller gains are study settings.
% Limits are monitored at recorded integration stages; no collision model or
% invariance guarantee is implied. All seven Panda joints have finite limits.
if nargin<2,c=panda.config();end
assert(c.control_dt>0&&c.substeps>=1&&c.substeps==floor(c.substeps));
assert(c.motion_time>0&&c.end_time>=c.motion_time);
assert(abs(c.end_time/c.control_dt-round(c.end_time/c.control_dt))<1e-9);
q0=panda.joint_vector(c.q0);goal=panda.joint_vector(c.qgoal);
assert(all(q0>=m.qmin&q0<=m.qmax)&&all(goal>=m.qmin&goal<=m.qmax));
n=m.n;controller=m;if ~isempty(c.controller_model),controller=c.controller_model;end
phase=ftv5.phase_plan(0,0,0,c.motion_time);
clock=tic;t=0:c.control_dt:c.end_time;N=numel(t);z=zeros(2*n+3,1);z(1:n)=q0;
log=struct('t',t,'q',zeros(n,N),'dq',zeros(n,N),'qref',zeros(n,N), ...
    'dqref',zeros(n,N),'tau_command',zeros(n,N),'tau_applied',zeros(n,N), ...
    'effectiveness',ones(n,N),'saturated',false(n,N),'H',zeros(1,N), ...
    'work_joint',zeros(1,N),'friction_loss',zeros(1,N),'work_environment',zeros(1,N), ...
    'position',zeros(3,N),'position_ref',zeros(3,N));
h=c.control_dt/c.substeps;limit_violation=0;velocity_utilization=0;
stage_min_margin=Inf;
for k=1:N
    q=z(1:n);dq=z(n+1:2*n);[qr,dqr,ddqr]=panda.reference(t(k),c,phase);
    if isempty(c.control_function)
        virtual=ddqr+c.kp(:).*(qr-q)+c.kd(:).*(dqr-dq);
        command_raw=panda.inverse_dynamics(q,dq,virtual,controller);
    else
        command_raw=panda.joint_vector(c.control_function(t(k),q,dq,qr,dqr,ddqr,controller));
    end
    command=command_raw;
    if c.command_saturation,command=max(-m.taumax,min(m.taumax,command));end
    [alpha,~]=inputs(t(k),q,dq,c,n);actual=alpha.*command;
    log.q(:,k)=q;log.dq(:,k)=dq;log.qref(:,k)=qr;log.dqref(:,k)=dqr;
    log.tau_command(:,k)=command;log.tau_applied(:,k)=actual;
    log.effectiveness(:,k)=alpha;log.saturated(:,k)=abs(command_raw)>m.taumax+1e-12;
    log.H(k)=panda.energy(q,dq,m);log.work_joint(k)=z(2*n+1);
    log.friction_loss(k)=z(2*n+2);log.work_environment(k)=z(2*n+3);
    T=panda.fk(q,m);Tr=panda.fk(qr,m);
    log.position(:,k)=T(1:3,4);log.position_ref(:,k)=Tr(1:3,4);
    if k<N
        for j=1:c.substeps
            tj=t(k)+(j-1)*h;
            k1=rhs(tj,z,command,m,c);
            k2=rhs(tj+h/2,z+h*k1/2,command,m,c);
            k3=rhs(tj+h/2,z+h*k2/2,command,m,c);
            k4=rhs(tj+h,z+h*k3,command,m,c);
            if c.capture_substeps
                stages=[z,z+h*k1/2,z+h*k2/2,z+h*k3,z+h*(k1+2*k2+2*k3+k4)/6];
                margins=[stages(1:n,:)-m.qmin;m.qmax-stages(1:n,:)];
                stage_min_margin=min(stage_min_margin,min(margins,[],'all'));
                limit_violation=max(limit_violation,max(-margins,[],'all'));
                velocity_utilization=max(velocity_utilization,max(abs(stages(n+1:2*n,:))./m.dqmax,[],'all'));
            end
            z=z+h*(k1+2*k2+2*k3+k4)/6;
            assert(all(isfinite(z)),'panda:integration','Nonfinite state: reduce the step or check model/control');
        end
    end
end
error=log.q-log.qref;
log.energy_residual=log.H-log.H(1)-log.work_joint-log.work_environment+log.friction_loss;
log.metrics=struct('actuator_profile',m.actuator_profile,'friction_profile',m.friction_profile, ...
    'control_dt_s',c.control_dt,'plant_step_s',h,'duration_s',c.end_time, ...
    'max_joint_error_deg',max(abs(error),[],'all')*180/pi, ...
    'rms_joint_error_deg',sqrt(mean(error.^2,'all'))*180/pi, ...
    'max_position_error_mm',max(vecnorm(log.position-log.position_ref))*1000, ...
    'energy_balance_max_error_J',max(abs(log.energy_residual)), ...
    'minimum_recorded_limit_margin_rad',min([log.q-m.qmin;m.qmax-log.q],[],'all'), ...
    'minimum_stage_limit_margin_rad',stage_min_margin,'maximum_stage_limit_violation_rad',limit_violation, ...
    'max_stage_velocity_utilization',velocity_utilization, ...
    'max_command_utilization',max(abs(log.tau_command)./m.taumax,[],'all'), ...
    'saturated_joint_samples',nnz(log.saturated),'elapsed_seconds',toc(clock));
log.model_options=m.options;log.urdf_revision=m.urdf_revision;
end

function dz=rhs(t,z,command,m,c)
n=m.n;
q=z(1:n);dq=z(n+1:2*n);[alpha,F]=inputs(t,q,dq,c,n);
[M,b]=panda.dynamics(q,dq,m);actual=alpha.*command;environment=zeros(n,1);
if any(F),environment=panda.jacobian(q,m,'body')'*F;end
ddq=M\(actual+environment-b);f=panda.friction(dq,m);
dz=[dq;ddq;actual'*dq;f'*dq;environment'*dq];
end

function [alpha,F]=inputs(t,q,dq,c,n)
alpha=ones(n,1);F=zeros(6,1);
if ~isempty(c.effectiveness_function),alpha=panda.joint_vector(c.effectiveness_function(t));end
if ~isempty(c.external_wrench_function),F=panda.wrench_vector(c.external_wrench_function(t,q,dq));end
assert(all(alpha>=0&alpha<=1),'panda:input','Effectiveness must be in [0,1]');
end
