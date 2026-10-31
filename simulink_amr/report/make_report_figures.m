function make_report_figures()
%MAKE_REPORT_FIGURES  Renders the robot / warehouse pictures used in the project report (report/figs).
here = fileparts(mfilename('fullpath')); root = fileparts(here); addpath(root);
out = fullfile(here, 'figs'); if ~isfolder(out), mkdir(out); end
P = scout_amr_params();

% ---- robot meshes in the base_link frame (wheels at q = 0) ------------------------------------
rbt = importrobot(fullfile(root, 'scout_mini.urdf')); rbt.DataFormat = 'row';
q0 = homeConfiguration(rbt);
M.base = readMesh(fullfile(root, 'meshes', 'base_link.STL'), eye(4));
for k = 1:4
    T = getTransform(rbt, q0, sprintf('w%d', k), 'base_link');
    M.w(k) = readMesh(fullfile(root, 'meshes', sprintf('w%d.STL', k)), T);
end

% ---- Fig: robot imported by importrobot (rigidBodyTree) -------------------------------------
f = figure('Theme', 'light', 'Color', 'w', 'Position', [50 50 900 600]); ax = axes(f);
show(rbt, q0, 'Parent', ax, 'Visuals', 'on', 'Frames', 'off'); view(ax, 135, 25); axis(ax, 'equal'); grid(ax, 'on');
title(ax, 'Scout Mini imported with importrobot (rigidBodyTree: 5 bodies, 4 continuous wheel joints)');
exportgraphics(f, fullfile(out, 'robot_importrobot.png'), 'Resolution', 150); close(f);

% ---- Fig: robot render with joint axes -----------------------------------------------------------
f = figure('Theme', 'light', 'Color', 'w', 'Position', [50 50 1000 650]); ax = axes(f); hold(ax, 'on');
drawRobot(ax, M, eye(4), [0.25 0.28 0.35], [0.1 0.1 0.1]);
for k = 1:4
    T = getTransform(rbt, q0, sprintf('w%d', k), 'base_link'); c = T(1:3, 4); a = T(1:3, 1) * 0.18;
    quiver3(ax, c(1), c(2), c(3), a(1), a(2), a(3), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6);
    text(ax, c(1) + a(1) * 1.3, c(2) + a(2) * 1.3, c(3) + a(3) * 1.3, sprintf('j%d', k), 'Color', 'r', 'FontWeight', 'bold');
end
axis(ax, 'equal'); grid(ax, 'on'); view(ax, 40, 22); camlight(ax, 'headlight'); lighting(ax, 'gouraud');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
title(ax, 'Scout Mini CAD meshes (base_link + w1..w4) with wheel joint axes (red)');
exportgraphics(f, fullfile(out, 'robot_render.png'), 'Resolution', 150); close(f);

% ---- Fig: 3D warehouse with robot, workers, second AMR, truck at a point of the mission ----------
L = load(fullfile(root, 'results', 'scout_warehouse_results.mat')); r = L.R(2).r;
for tt = [17.4 29.3 40.7]
    [~, i] = min(abs(r.t - tt)); [~, j] = min(abs(r.tm - tt));
    f = figure('Theme', 'light', 'Color', 'w', 'Position', [30 30 1400 760]); ax = axes(f); hold(ax, 'on');
    drawWarehouse3D(ax, P, r.mv(j, :));
    x = r.lg(i, 1); y = r.lg(i, 2); psi = r.lg(i, 3);
    Rwf = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Tfb = [0 -1 0 P.p_fb(1); 1 0 0 P.p_fb(2); 0 0 1 P.p_fb(3)];              % base_link in footprint (yaw +90 deg)
    Tfb = [Tfb; 0 0 0 1]; Tw = [Rwf [x; y; 0]; 0 0 0 1] * Tfb;
    drawRobot(ax, M, Tw, [0.9 0.5 0.1], [0.05 0.05 0.05]);
    % LiDAR rays
    [~, k] = min(abs(r.tr - tt)); rg = r.rg(k, :); ang = linspace(-pi, pi, P.lidar.n + 1); ang(end) = [];
    for b = 1:numel(rg)
        if rg(b) < P.lidar.rmax - 0.01
            plot3(ax, [x x + rg(b) * cos(psi + ang(b))], [y y + rg(b) * sin(psi + ang(b))], [0.25 0.25], '-', 'Color', [1 0.4 0.4 0.5]);
        end
    end
    axis(ax, 'equal'); grid(ax, 'on'); view(ax, -35, 38); camlight(ax, 'headlight'); lighting(ax, 'gouraud');
    xlim(ax, [-3 29]); ylim(ax, [-2.5 12.5]); zlim(ax, [0 2.6]); xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
    title(ax, sprintf('3D warehouse with the imported Scout Mini, t = %.1f s (red lines = LiDAR returns)', tt));
    exportgraphics(f, fullfile(out, sprintf('warehouse3d_t%02.0f.png', tt)), 'Resolution', 130); close(f);
end

% ---- Simulink diagrams -------------------------------------------------------------------------------
mdl = 'scout_amr_sim'; load_system(fullfile(root, [mdl '.slx']));
set_param(mdl, 'Location', [50 50 1900 1000]);
try, print(['-s' mdl], '-dpng', '-r110', fullfile(out, 'simulink_top.png')); catch e, disp(e.message); end
for sub = {'Mission_and_Slip_Controller', 'Scout_Mini_Robot/Tyre_Ground_Contact_Model'}
    nm = regexprep(sub{1}, '.*/', '');
    try, print(['-s' mdl '/' sub{1}], '-dpng', '-r130', fullfile(out, ['simulink_' nm '.png'])); catch e, disp(e.message); end
end
close_system(mdl, 0);
disp('report figures done');
end

% -----------------------------------------------------------------------------------------------------
function m = readMesh(file, T)
    tr = stlread(file); v = tr.Points; f = tr.ConnectivityList;
    v = (T * [v'; ones(1, size(v, 1))])'; m.v = v(:, 1:3); m.f = f;
end
function drawRobot(ax, M, T, cb, cw)
    pm = @(m, c) patch(ax, 'Faces', m.f, 'Vertices', tf(m.v, T), 'FaceColor', c, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
    pm(M.base, cb); for k = 1:4, pm(M.w(k), cw); end
end
function v = tf(v, T), v = (T * [v'; ones(1, size(v, 1))])'; v = v(:, 1:3); end
function box3(ax, x0, x1, y0, y1, z0, z1, col, alpha)
    V = [x0 y0 z0; x1 y0 z0; x1 y1 z0; x0 y1 z0; x0 y0 z1; x1 y0 z1; x1 y1 z1; x0 y1 z1];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch(ax, 'Faces', F, 'Vertices', V, 'FaceColor', col, 'FaceAlpha', alpha, 'EdgeColor', [0.2 0.2 0.3], 'LineWidth', 0.5);
end
function drawWarehouse3D(ax, P, mv)
    box3(ax, -2.5, 28.5, -2.2, 12, -0.02, 0, [0.75 0.75 0.78], 1);
    for i = 1:size(P.patches, 1), q = P.patches(i, :); box3(ax, q(1), q(2), q(3), q(4), 0, 0.01, [0.45 0.7 0.95], 1); end
    pad = @(c, s, col) box3(ax, c(1) - s, c(1) + s, c(2) - s, c(2) + s, 0, 0.015, col, 1);
    pad(P.pickup, 0.8, [0.15 0.7 0.3]); pad(P.deliver, 0.8, [0.85 0.25 0.2]); pad([0 0], 0.6, [0.95 0.8 0.15]);
    for i = 1:size(P.racks, 1)
        q = P.racks(i, :); box3(ax, q(1), q(2), q(3), q(4), 0, 2.4, [0.2 0.35 0.65], 0.25);
        for s = 1:3, box3(ax, q(1) + 0.06 * (q(2) - q(1)), q(2) - 0.06 * (q(2) - q(1)), q(3) + 0.1 * (q(4) - q(3)), q(4) - 0.1 * (q(4) - q(3)), 0.2 + (s - 1) * 0.75, 0.7 + (s - 1) * 0.75, [0.78 0.58 0.38], 1); end
    end
    for i = 1:size(P.walls, 1), q = P.walls(i, :); box3(ax, q(1), q(2), q(3), q(4), 0, 1.2, [0.6 0.62 0.66], 1); end
    for i = 1:size(P.boxes, 1), q = P.boxes(i, :); box3(ax, q(1), q(2), q(3), q(4), 0, 0.8, [0.8 0.55 0.25], 1); end
    sz = P.mover.size; mv = reshape(mv, 2, []).';
    for k = 1:2    % workers
        [X, Y, Z] = cylinder(0.3, 24); surf(ax, X + mv(k, 1), Y + mv(k, 2), Z * 1.7, 'FaceColor', [0.95 0.6 0.1 + 0.2 * (k - 1)], 'EdgeColor', 'none');
    end
    h = sz(3, :) / 2; box3(ax, mv(3, 1) - h(1), mv(3, 1) + h(1), mv(3, 2) - h(2), mv(3, 2) + h(2), 0, 0.45, [0.15 0.3 0.6], 1);
    h = sz(4, :) / 2; box3(ax, mv(4, 1) - h(1), mv(4, 1) + h(1), mv(4, 2) - h(2), mv(4, 2) + h(2), 0, 0.9, [0.9 0.75 0.1], 1);
end
