function handles = draw_robot(X, m, ax, handles)
%DRAW_ROBOT Manufacturer Panda visual geometry posed by (S1)^7 Lie kinematics.
%
% Renders the full 3D visual mesh of the Franka Emika Panda robot using
% the SE(3) link poses computed algebraically by panda_s1.frames(X, m).
%
% Inputs:
%   X: 2-by-7 configuration matrix on (S1)^7
%   m: (optional) model struct from panda_s1.model()
%   ax: (optional) target axes handle
%   handles: (optional) struct of graphic object handles from previous call
% Output:
%   handles: struct of graphic object handles for fast frame-by-frame updates

if nargin < 2 || isempty(m)
    m = panda_s1.model();
end
if nargin < 3 || isempty(ax)
    ax = gca;
end

chain = panda_s1.frames(X, m);
root = fileparts(fileparts(mfilename('fullpath')));

if nargin < 4 || isempty(handles) || ~isfield(handles, 'patch') || any(~isgraphics(handles.patch))
    hold(ax, 'on');
    handles = struct('patch', gobjects(1, 8), 'vertices', {cell(1, 8)}, 'mesh', false);
    folder = fullfile(root, 'sources', 'meshes');
    handles.mesh = all(arrayfun(@(i) isfile(fullfile(folder, sprintf('link%d.stl', i))), 0:7));

    if handles.mesh
        for i = 1:8
            [f, v] = panda.read_stl(fullfile(folder, sprintf('link%d.stl', i - 1)));
            handles.vertices{i} = v;
            color = [0.86, 0.88, 0.91];
            if mod(i, 2) == 0
                color = [0.26, 0.29, 0.33];
            end
            handles.patch(i) = patch(ax, 'Faces', f, 'Vertices', v, 'FaceColor', color, ...
                'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.35);
        end
        camlight(ax, 'headlight');
        lighting(ax, 'gouraud');
    end

    handles.line = plot3(ax, NaN, NaN, NaN, '-o', 'Color', [0.05, 0.48, 0.58], 'LineWidth', 2, 'MarkerSize', 4);
    handles.flange = plot3(ax, NaN, NaN, NaN, 'o', 'MarkerFaceColor', [0.95, 0.46, 0.17], 'MarkerEdgeColor', 'none', 'MarkerSize', 7);
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 140, 22);
    xlabel(ax, 'x (m)');
    ylabel(ax, 'y (m)');
    zlabel(ax, 'z (m)');
    xlim(ax, [-0.8, 0.8]);
    ylim(ax, [-0.8, 0.8]);
    zlim(ax, [-0.1, 1.25]);
end

if handles.mesh
    for i = 1:8
        T = eye(4);
        if i > 1
            T = chain(:, :, i - 1);
        end
        v = handles.vertices{i};
        handles.patch(i).Vertices = v * T(1:3, 1:3)' + T(1:3, 4)';
    end
    handles.line.Visible = 'off';
else
    points = [zeros(3, 1), reshape(chain(1:3, 4, :), 3, [])];
    set(handles.line, 'XData', points(1, :), 'YData', points(2, :), 'ZData', points(3, :));
end

set(handles.flange, 'XData', chain(1, 4, 8), 'YData', chain(2, 4, 8), 'ZData', chain(3, 4, 8));
end
