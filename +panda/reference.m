function [q,dq,ddq]=reference(t,c,phase)
%REFERENCE Seven joint positions as flat outputs with a smooth quintic phase.
if nargin<3,phase=ftv5.phase_plan(0,0,0,c.motion_time);end
[s,v,a]=ftv5.phase(phase,t);d=c.qgoal(:)-c.q0(:);
q=c.q0(:)+d*s;dq=d*v;ddq=d*a;
end
