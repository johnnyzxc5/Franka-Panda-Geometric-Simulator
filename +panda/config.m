function c=config()
%CONFIG Joint-flat-output tracking settings separate from identified robot data.
c.control_dt=.004;c.substeps=4;c.motion_time=3.0;c.end_time=3.6;
c.q0=[0;-.5;0;-1.8;0;1.4;.6];c.qgoal=[-.25;-.3;.35;-2.1;.1;1.6;.35];
c.kp=64*ones(7,1);c.kd=16*ones(7,1);
c.command_saturation=true;c.controller_model=[];c.control_function=[];
c.external_wrench_function=[];c.effectiveness_function=[];
c.capture_substeps=true;c.save_results=true;
end
