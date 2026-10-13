function handles=draw_robot(q,m,ax,handles)
%DRAW_ROBOT Manufacturer Panda visual geometry posed by exponential kinematics.
if nargin<3||isempty(ax),ax=axes;end
chain=panda.frames(q,m);root=fileparts(fileparts(mfilename('fullpath')));
if nargin<4||isempty(handles)
    hold(ax,'on');handles=struct('patch',gobjects(1,8),'vertices',{cell(1,8)},'mesh',false);
    folder=fullfile(root,'sources','meshes');
    handles.mesh=all(arrayfun(@(i)isfile(fullfile(folder,sprintf('link%d.stl',i))),0:7));
    if handles.mesh
        for i=1:8
            [f,v]=panda.read_stl(fullfile(folder,sprintf('link%d.stl',i-1)));handles.vertices{i}=v;
            color=[.86,.88,.91];if mod(i,2)==0,color=[.26,.29,.33];end
            handles.patch(i)=patch(ax,'Faces',f,'Vertices',v,'FaceColor',color, ...
                'EdgeColor','none','FaceLighting','gouraud','AmbientStrength',.35);
        end
        camlight(ax,'headlight');lighting(ax,'gouraud');
    end
    handles.line=plot3(ax,NaN,NaN,NaN,'-o','Color',[.05,.48,.58],'LineWidth',2,'MarkerSize',4);
    handles.flange=plot3(ax,NaN,NaN,NaN,'o','MarkerFaceColor',[.95,.46,.17],'MarkerEdgeColor','none','MarkerSize',7);
    grid(ax,'on');axis(ax,'equal');view(ax,140,22);
    xlabel(ax,'x (m)');ylabel(ax,'y (m)');zlabel(ax,'z (m)');
    xlim(ax,[-.8,.8]);ylim(ax,[-.8,.8]);zlim(ax,[-.1,1.25]);
end
if handles.mesh
    for i=1:8
        T=eye(4);if i>1,T=chain(:,:,i-1);end
        v=handles.vertices{i};handles.patch(i).Vertices=v*T(1:3,1:3)'+T(1:3,4)';
    end
    handles.line.Visible='off';
else
    points=[zeros(3,1),reshape(chain(1:3,4,:),3,[])];
    set(handles.line,'XData',points(1,:),'YData',points(2,:),'ZData',points(3,:));
end
set(handles.flange,'XData',chain(1,4,8),'YData',chain(2,4,8),'ZData',chain(3,4,8));
end
