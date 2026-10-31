function make_example_style_outputs(makeVideo)
%MAKE_EXAMPLE_STYLE_OUTPUTS  Presentation outputs of the Simscape warehouse mission (from results/scout_warehouse_results.mat):
%    results/trajectory_map.png               top-down map: driven path, robot footprints, LiDAR fans, obstacles
%    results/signals.png                      clearance, controller commands, wheel speed commands, chassis height / tilt
%    results/amr_warehouse_avoidance.gif/.mp4 top-down animation with LiDAR rays, planned path and moving obstacles
%  make_example_style_outputs(false) skips the animation.
if nargin < 1, makeVideo = true; end
here = fileparts(mfilename('fullpath')); cd(here); addpath(here);
L = load(fullfile('results', 'scout_warehouse_results.mat')); r = L.R(2).r; P = L.P;
if ~isfolder('results'), mkdir('results'); end
C = style();
fp = [0.58 0.67];                                           % robot footprint [length width] in m

% ---------------- trajectory map ------------------------------------------------------------------------
f = figure('Theme', 'light', 'Color', 'w', 'Position', [20 20 1500 760]); ax = axes(f); setupMap(ax, P, C);
tEnd = r.t(end); idx = @(t) find(r.t >= t, 1);
[~, jEnd] = min(abs(r.tm - tEnd));
for k = 1:4          % trails of the moving obstacles
    plot(ax, r.mv(:, 2*k-1), r.mv(:, 2*k), '.', 'Color', C.mover{k}, 'MarkerSize', 3);
end
for t = 3:3:tEnd - 1                                         % LiDAR fans along the path
    i = idx(t); [~, q] = min(abs(r.tr - t)); drawRays(ax, r.lg(i, 1), r.lg(i, 2), r.lg(i, 3), r.rg(q, :), P, C, 0.18);
end
plot(ax, r.lg(:, 1), r.lg(:, 2), '-', 'Color', C.path, 'LineWidth', 2.2);
for t = 0:2.5:tEnd - 0.5                                     % footprints
    i = idx(t); drawRobot(ax, r.lg(i, 1), r.lg(i, 2), r.lg(i, 3), fp, C.robotEdge, 'none');
end
i = numel(r.t); drawRobot(ax, r.lg(i, 1), r.lg(i, 2), r.lg(i, 3), fp, [0.1 0.6 0.1], 'none');
ev = find(diff(r.lg(:, 38)) > 0) + 1;
if ~isempty(ev), plot(ax, r.lg(ev, 1), r.lg(ev, 2), 'p', 'MarkerSize', 13, 'MarkerFaceColor', [1 0.9 0], 'MarkerEdgeColor', 'k'); end
hS = plot(ax, 0, 0, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [1 0.9 0], 'MarkerEdgeColor', 'k');
hG = plot(ax, P.deliver(1), P.deliver(2), 'p', 'MarkerSize', 20, 'MarkerFaceColor', [0.1 0.95 0.1], 'MarkerEdgeColor', 'k');
hP = plot(ax, P.pickup(1), P.pickup(2), 's', 'MarkerSize', 11, 'MarkerFaceColor', [0.2 0.8 0.3], 'MarkerEdgeColor', 'k');
hPath = plot(ax, NaN, NaN, '-', 'Color', C.path, 'LineWidth', 2.2); hRe = plot(ax, NaN, NaN, 'p', 'MarkerSize', 11, 'MarkerFaceColor', [1 0.9 0], 'MarkerEdgeColor', 'k');
legend(ax, [hPath, hS, hP, hG, hRe], {'Robot path (Simscape)', 'Start', 'Pickup', 'Delivery (goal)', 're-route event'}, 'Location', 'northeast', 'TextColor', 'w', 'Color', [0.15 0.15 0.15], 'AutoUpdate', 'off');
title(ax, sprintf('AgileX Scout Mini warehouse mission - %.1f s, path %.1f m, %d re-routes, %d collisions', tEnd, ...
      sum(hypot(diff(r.lg(:,1)), diff(r.lg(:,2)))), r.lg(end,38), r.lg(end,41)), 'Color', 'k');
exportgraphics(f, fullfile('results', 'trajectory_map.png'), 'Resolution', 130); close(f);

% ---------------- signals -------------------------------------------------------------------------------
n = size(r.Rb, 3); Rwf = zeros(3, 3, n); z = zeros(n, 1); rollD = z; pitchD = z;
T = [0 1 0; -1 0 0; 0 0 1];
for i = 1:n
    Rwf(:,:,i) = r.Rb(:,:,i) * T;
    z(i) = r.pb(i, 3) - Rwf(3,:,i) * P.p_fb;
    pitchD(i) = asind(-Rwf(3,1,i)); rollD(i) = atan2d(Rwf(3,2,i), Rwf(3,3,i));
end
f = figure('Theme', 'light', 'Color', 'w', 'Position', [20 20 1150 1100]);
ax1 = subplot(4,1,1); dark(ax1, C); plot(ax1, r.t, r.lg(:,32), 'Color', C.blue, 'LineWidth', 1.8); hold(ax1, 'on');
yline(ax1, 0, '--', 'collision', 'Color', [1 0.2 0.2], 'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'top');
ylabel(ax1, 'clearance [m]'); title(ax1, 'Distance from robot body to nearest obstacle'); xlim(ax1, [0 tEnd]);
ax2 = subplot(4,1,2); dark(ax2, C); hold(ax2, 'on');
plot(ax2, r.t, r.lg(:,7), 'Color', C.blue, 'LineWidth', 1.6); plot(ax2, r.t, r.lg(:,4), 'Color', [1 0.5 0.1], 'LineWidth', 1.6);
plot(ax2, r.t, r.lg(:,8), 'Color', [1 0.9 0.2], 'LineWidth', 1.4);
legend(ax2, {'v command [m/s]', 'v measured [m/s]', '\omega command [rad/s]'}, 'TextColor', 'w', 'Color', [0.15 0.15 0.15], 'Location', 'southeast');
ylabel(ax2, 'm/s, rad/s'); title(ax2, 'Controller commands'); xlim(ax2, [0 tEnd]);
ax3 = subplot(4,1,3); dark(ax3, C); hold(ax3, 'on');
plot(ax3, r.t, r.lg(:,9), 'Color', C.blue, 'LineWidth', 1.6); plot(ax3, r.t, r.lg(:,11), 'Color', [1 0.5 0.1], 'LineWidth', 1.6);
legend(ax3, {'left wheels', 'right wheels'}, 'TextColor', 'w', 'Color', [0.15 0.15 0.15], 'Location', 'northeast');
ylabel(ax3, '[rad/s]'); title(ax3, 'Wheel speed commands'); xlim(ax3, [0 tEnd]);
ax4 = subplot(4,1,4); dark(ax4, C); yyaxis(ax4, 'left'); plot(ax4, r.tb, 1000 * z, 'Color', C.blue, 'LineWidth', 1.6); ylabel(ax4, 'z [mm]');
ax4.YAxis(1).Color = C.blue; yyaxis(ax4, 'right'); hold(ax4, 'on');
plot(ax4, r.tb, rollD, '-', 'Color', [0.95 0.95 0.95], 'LineWidth', 0.8); plot(ax4, r.tb, pitchD, '--', 'Color', [1 0.5 0.1], 'LineWidth', 0.8);
ylabel(ax4, '[deg]'); ax4.YAxis(2).Color = [1 0.5 0.1];
legend(ax4, {'chassis height', 'roll', 'pitch'}, 'TextColor', 'w', 'Color', [0.15 0.15 0.15], 'Location', 'northeast');
title(ax4, 'Chassis stays on the ground (no floating)'); xlabel(ax4, 'time [s]'); xlim(ax4, [0 tEnd]);
exportgraphics(f, fullfile('results', 'signals.png'), 'Resolution', 130); close(f);
fprintf('chassis height %.1f..%.1f mm, roll %.2f..%.2f deg, pitch %.2f..%.2f deg\n', 1000*min(z), 1000*max(z), min(rollD), max(rollD), min(pitchD), max(pitchD));
if ~makeVideo, return; end

% ---------------- animation ------------------------------------------------------------------------------
dt = 0.2; ts = 0:dt:tEnd;
f = figure('Theme', 'light', 'Color', 'w', 'Position', [20 20 1100 560], 'Visible', 'on'); ax = axes(f, 'Position', [0.06 0.09 0.92 0.82]);
setupMap(ax, P, C);
hTrail = plot(ax, NaN, NaN, '-', 'Color', C.path, 'LineWidth', 2.4);
hPlan = plot(ax, NaN, NaN, '--', 'Color', [0.2 1 0.3], 'LineWidth', 1.4);
hRay = plot(ax, NaN, NaN, '-', 'Color', [C.ray 0.35], 'LineWidth', 0.6);
hHit = plot(ax, NaN, NaN, '.', 'Color', [0.3 0.6 1], 'MarkerSize', 7);
hBody = patch(ax, NaN, NaN, [0.15 0.3 0.6], 'EdgeColor', [0.6 0.8 1], 'FaceAlpha', 0.85, 'LineWidth', 1.2);
hHead = plot(ax, NaN, NaN, '-', 'Color', [1 1 1], 'LineWidth', 1.6);
hW = gobjects(1, 2); for k = 1:2, hW(k) = patch(ax, NaN, NaN, C.mover{k}, 'EdgeColor', 'k'); end
hA = patch(ax, NaN, NaN, C.mover{3}, 'EdgeColor', 'k'); hT = patch(ax, NaN, NaN, C.mover{4}, 'EdgeColor', 'k');
plot(ax, P.deliver(1), P.deliver(2), 'p', 'MarkerSize', 20, 'MarkerFaceColor', [0.1 0.95 0.1], 'MarkerEdgeColor', 'k');
plot(ax, P.pickup(1), P.pickup(2), 's', 'MarkerSize', 11, 'MarkerFaceColor', [0.2 0.8 0.3], 'MarkerEdgeColor', 'k');
plot(ax, 0, 0, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [1 0.9 0], 'MarkerEdgeColor', 'k');
hT_ = title(ax, '', 'Color', 'k');
stateTxt = {'driving to PICKUP', 'LOADING', 'driving to DELIVERY', 'UNLOADING', 'DONE'};
th = linspace(0, 2*pi, 25); mvSize = P.mover.size;
gifFile = fullfile('results', 'amr_warehouse_avoidance.gif'); mp4 = fullfile('results', 'amr_warehouse_avoidance.mp4');
vw = VideoWriter(mp4, 'MPEG-4'); vw.FrameRate = 10; vw.Quality = 90; open(vw);
for q = 1:numel(ts)
    t = ts(q); i = find(r.t >= t, 1); if isempty(i), i = numel(r.t); end
    x = r.lg(i,1); y = r.lg(i,2); psi = r.lg(i,3);
    [~, j] = min(abs(r.tm - t)); m = reshape(r.mv(j, :), 2, []).';
    [~, k2] = min(abs(r.tr - t)); [~, k3] = min(abs(r.tp - t));
    set(hTrail, 'XData', r.lg(1:i,1), 'YData', r.lg(1:i,2));
    [rx, ry, hx, hy] = rays(x, y, psi, r.rg(k2, :), P); set(hRay, 'XData', rx, 'YData', ry); set(hHit, 'XData', hx, 'YData', hy);
    pp = r.pp(k3, :); np = pp(1);
    if np >= 2, xy = reshape(pp(2:161), 2, []).'; xy = xy(1:np, :); set(hPlan, 'XData', xy(:,1), 'YData', xy(:,2)); else, set(hPlan, 'XData', NaN, 'YData', NaN); end
    c = [fp(1) fp(2)] / 2; B = [c(1) c(2); c(1) -c(2); -c(1) -c(2); -c(1) c(2)]; Rm = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    G = (Rm * B.').' + [x y]; set(hBody, 'XData', G(:,1), 'YData', G(:,2));
    set(hHead, 'XData', [x x + 0.45*cos(psi)], 'YData', [y y + 0.45*sin(psi)]);
    for k = 1:2, set(hW(k), 'XData', m(k,1) + 0.3*cos(th), 'YData', m(k,2) + 0.3*sin(th)); end
    hs = mvSize(3,:)/2; set(hA, 'XData', m(3,1) + hs([1 1 2 2]).*[-1 1 1 -1], 'YData', m(3,2) + hs([1 1 2 2]).*[0 0 0 0] + [-hs(2) -hs(2) hs(2) hs(2)]);
    hs = mvSize(4,:)/2; set(hT, 'XData', m(4,1) + [-hs(1) hs(1) hs(1) -hs(1)], 'YData', m(4,2) + [-hs(2) -hs(2) hs(2) hs(2)]);
    set(hA, 'XData', m(3,1) + [-mvSize(3,1) mvSize(3,1) mvSize(3,1) -mvSize(3,1)]/2);
    st = max(1, min(5, round(r.lg(i, 31))));
    set(hT_, 'String', sprintf('AgileX Scout Mini warehouse   t = %4.1f s   %s   |v| = %.2f m/s   re-routes %d', t, stateTxt{st}, abs(r.lg(i,4)), r.lg(i,38)));
    drawnow; fr = getframe(f); writeVideo(vw, fr);
    [A, map] = rgb2ind(frame2im(fr), 128);
    if q == 1, imwrite(A, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.1);
    else,      imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1); end
end
close(vw); close(f);
fprintf('saved %s and %s\n', gifFile, mp4);
end

% =========================================================================================================
function C = style()
    C.floor = [0.07 0.07 0.07]; C.wall = [0.42 0.47 0.60]; C.rack = [0.85 0.20 0.15]; C.box = [0.95 0.55 0.10];
    C.mover = {[0.95 0.55 0.10], [0.95 0.55 0.10], [0.20 0.35 0.75], [0.95 0.80 0.15]};
    C.path = [0.10 0.20 1.00]; C.blue = [0.20 0.55 1.00]; C.ray = [0.25 0.50 1.0]; C.robotEdge = [0.25 0.5 1.0];
end
function dark(ax, C)
    set(ax, 'Color', C.floor, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.35, 'XColor', 'k', 'YColor', 'k'); grid(ax, 'on'); hold(ax, 'on'); box(ax, 'on');
end
function setupMap(ax, P, C)
    dark(ax, C); axis(ax, 'equal'); xlim(ax, [-2.7 28.7]); ylim(ax, [-2.4 12.2]); xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    for i = 1:size(P.patches, 1), q = P.patches(i,:); patch(ax, q([1 2 2 1]), q([3 3 4 4]), [0.25 0.45 0.8], 'FaceAlpha', 0.25, 'EdgeColor', [0.4 0.6 1], 'LineStyle', ':'); end
    for i = 1:size(P.racks, 1), q = P.racks(i,:); patch(ax, q([1 2 2 1]), q([3 3 4 4]), C.rack, 'EdgeColor', 'k'); end
    for i = 1:size(P.boxes, 1), q = P.boxes(i,:); patch(ax, q([1 2 2 1]), q([3 3 4 4]), C.box, 'EdgeColor', 'k'); end
    for i = 1:size(P.walls, 1), q = P.walls(i,:); patch(ax, q([1 2 2 1]), q([3 3 4 4]), C.wall, 'EdgeColor', C.wall); end
    text(ax, 1.2, 5.1, 'PICKUP', 'Color', [0.6 1 0.6], 'FontWeight', 'bold', 'Clipping', 'on');
    text(ax, 24.6, 9.2, 'DELIVERY', 'Color', [0.6 1 0.6], 'FontWeight', 'bold', 'Clipping', 'on');
    text(ax, -1.0, -1.0, 'START', 'Color', [1 0.9 0.4], 'FontWeight', 'bold', 'Clipping', 'on');
end
function drawRobot(ax, x, y, psi, fp, edge, face)
    c = fp / 2; B = [c(1) c(2); c(1) -c(2); -c(1) -c(2); -c(1) c(2)]; Rm = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    G = (Rm * B.').' + [x y]; patch(ax, G(:,1), G(:,2), [0 0 0], 'FaceColor', face, 'EdgeColor', edge, 'LineWidth', 1.1);
end
function drawRays(ax, x, y, psi, rg, P, C, a)
    [rx, ry] = rays(x, y, psi, rg, P); plot(ax, rx, ry, '-', 'Color', [C.ray a], 'LineWidth', 0.5);
end
function [rx, ry, hx, hy] = rays(x, y, psi, rg, P)
    n = numel(rg); ang = psi - pi + (0:n-1) * 2*pi / n;
    ex = x + rg(:)' .* cos(ang); ey = y + rg(:)' .* sin(ang);
    rx = reshape([repmat(x, 1, n); ex; nan(1, n)], 1, []); ry = reshape([repmat(y, 1, n); ey; nan(1, n)], 1, []);
    h = rg(:)' < P.lidar.rmax - 0.01; hx = ex(h); hy = ey(h);
end
