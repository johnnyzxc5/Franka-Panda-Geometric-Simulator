function report=benchmark_panda()
%BENCHMARK_PANDA Matched complete dynamics work; steady-state MATLAB timings.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setup_panda();
m=panda.model();rng(13102026,'twister');
Q=m.qmin+.05+(m.qmax-m.qmin-.1).*rand(7,100);DQ=randn(7,100)*.2;
for i=1:100,panda.dynamics(Q(:,i),DQ(:,i),m);stock(Q(:,i),DQ(:,i),m);end
baseline=zeros(1,5);optimized=zeros(1,5);
for r=1:5
    tic;for i=1:100,stock(Q(:,i),DQ(:,i),m);end;baseline(r)=toc/100;
    tic;for i=1:100,panda.dynamics(Q(:,i),DQ(:,i),m);end;optimized(r)=toc/100;
end
report=struct('matlab_version',version,'states',100,'repeats',5, ...
    'stock_median_us',median(baseline)*1e6,'optimized_median_us',median(optimized)*1e6, ...
    'speedup',median(baseline)/median(optimized), ...
    'scope','M and c+g+f, changing q and dq, warmed serial CPU. No rendering or integration.');
panda.write_json(fullfile(root,'results','benchmark.json'),report);disp(report);
end
function [M,b]=stock(q,dq,m)
M=MassMatrix(q,m.Mlist,m.Glist,m.Slist)+m.Japp;
b=InverseDynamics(q,dq,zeros(7,1),m.gravity,zeros(6,1),m.Mlist,m.Glist,m.Slist)+panda.friction(dq,m);
end
