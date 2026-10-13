function animate_panda(log,m)
%ANIMATE_PANDA Interactive visual playback; dynamics have already been computed.
if nargin<2,m=panda.model();end
fig=figure('Color','w','Name','Franka Panda geometric simulation');ax=axes(fig);h=[];
for k=1:10:numel(log.t)
    if ~isgraphics(fig),break;end
    h=panda.draw_robot(log.q(:,k),m,ax,h);title(ax,sprintf('Franka Panda | t = %.2f s',log.t(k)));
    drawnow;pause(.01);
end
end
