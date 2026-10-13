function report=test_simulation_inputs()
%TEST_SIMULATION_INPUTS Optional forcing, effectiveness and torque saturation.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setup_panda();
m=panda.model();c=panda.config();c.motion_time=.08;c.end_time=.12;
c.qgoal=c.q0;c.save_results=false;
c.external_wrench_function=@(~,~,~)[.03;-.02;.01;.2;-.1;.15];
c.effectiveness_function=@(t)[1;1-.1*(t>=.04);1;1;1;1;1];
log=panda.simulate(m,c);
assert(all(log.effectiveness(2,log.t<.04)==1));
assert(all(abs(log.effectiveness(2,log.t>=.04)-.9)<1e-12));
assert(norm(log.tau_applied-log.effectiveness.*log.tau_command,'fro')<1e-12);
assert(log.metrics.energy_balance_max_error_J<1e-5);
assert(any(abs(log.work_environment)>0));
c.end_time=.012;c.motion_time=.008;c.external_wrench_function=[];
c.effectiveness_function=[];c.control_function=@(~,~,~,~,~,~,~)1000*ones(7,1);
saturated=panda.simulate(m,c);
assert(all(saturated.saturated,'all'));
assert(max(abs(saturated.tau_command)./m.taumax,[],'all')<=1+1e-12);
states=[zeros(7,1),1e-5*ones(7,1),m.qmin,m.qmax];edge_error=0;
for i=1:size(states,2)
    q=states(:,i);[~,Tc]=panda.frames(q,m);
    edge_error=max(edge_error,norm(Tc-panda.fk(q,m),'fro'));
    assert(norm(JacobianBody(m.Blist,q)-panda.jacobian(q,m,'body'),'fro')<1e-10);
end
assert(edge_error<1e-10);
report=struct('external_work_J',log.work_environment(end), ...
    'forced_energy_balance_error_J',log.metrics.energy_balance_max_error_J, ...
    'saturation_test_max_utilization',max(abs(saturated.tau_command)./m.taumax,[],'all'), ...
    'edge_pose_error',edge_error,'pass',true);
panda.write_json(fullfile(root,'results','simulation_inputs_validation.json'),report);disp(report);
end
