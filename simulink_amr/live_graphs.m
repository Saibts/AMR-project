function live_graphs(showExplorer)
%LIVE_GRAPHS  Runs the warehouse mission and updates the graphs WHILE the simulation runs.
%    live_graphs          % Multibody Explorer (3D) + live graphs, slip management ON
%    live_graphs(false)   % live graphs only (no 3D window)
%  The live values are read from the output ports of the Controller block (44-value log vector,
%  mover positions, LiDAR ranges) every 0.1 s of wall time; nothing in the model is changed.
if nargin < 1, showExplorer = true; end
here = fileparts(mfilename('fullpath')); cd(here); addpath(here);
P = scout_amr_params(); mdl = 'scout_amr_sim';
if ~isfile([mdl '.slx']), build_scout_amr_model; end
load_system(mdl);
set_param(mdl, 'StopTime', num2str(P.tStop));
set_param([mdl '/Mission_and_Slip_Controller/tc_on'], 'Value', '1');
if showExplorer, set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'on');
else,            set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off'); end
blk = [mdl '/Mission_and_Slip_Controller/Controller'];

% ---------------- figure: map + three signal panels --------------------------------------------
f = figure('Name', 'Live mission', 'Theme', 'light', 'Color', 'w', 'Position', [40 40 1400 760]);
tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
axM = nexttile(tl, 1, [3 1]); hold(axM, 'on'); axis(axM, 'equal'); grid(axM, 'on');
for k = 1:size(P.racks, 1), r = P.racks(k, :); rectangle(axM, 'Position', [r(1) r(3) r(2)-r(1) r(4)-r(3)], 'FaceColor', [0.62 0.69 0.87], 'EdgeColor', [0.2 0.3 0.6]); end
for k = 1:size(P.boxes, 1), r = P.boxes(k, :); rectangle(axM, 'Position', [r(1) r(3) r(2)-r(1) r(4)-r(3)], 'FaceColor', [0.85 0.58 0.25]); end
for k = 1:size(P.patches, 1), r = P.patches(k, :); rectangle(axM, 'Position', [r(1) r(3) r(2)-r(1) r(4)-r(3)], 'FaceColor', [0.82 0.90 1.0], 'EdgeColor', [0.4 0.6 0.9], 'LineStyle', ':'); end
plot(axM, P.pickup(1), P.pickup(2), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 9); text(axM, P.pickup(1), P.pickup(2)+0.7, 'PICKUP', 'FontWeight', 'bold');
plot(axM, P.deliver(1), P.deliver(2), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 9); text(axM, P.deliver(1)-2.5, P.deliver(2)+0.7, 'DELIVERY', 'FontWeight', 'bold');
hLid = plot(axM, nan, nan, '.', 'Color', [0.9 0.2 0.2], 'MarkerSize', 6);
hTrk = plot(axM, nan, nan, 'b-', 'LineWidth', 2);
hRob = plot(axM, nan, nan, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 9);
hMv  = plot(axM, nan(4, 1), nan(4, 1), 'o', 'MarkerFaceColor', [0.95 0.75 0.1], 'MarkerEdgeColor', 'k', 'MarkerSize', 9);
hPth = plot(axM, nan, nan, 'g-', 'LineWidth', 1.8);
xlim(axM, [-2.5 28.5]); ylim(axM, [-2.2 12]); xlabel(axM, 'x [m]'); ylabel(axM, 'y [m]'); hT = title(axM, 'Live mission');

axV = nexttile(tl, 2); hold(axV, 'on'); grid(axV, 'on'); ylabel(axV, 'speed [m/s]'); title(axV, 'Forward speed');
hVm = animatedline(axV, 'Color', [0 0.45 0.74], 'LineWidth', 1.4); hVc = animatedline(axV, 'Color', [0.85 0.33 0.1], 'LineStyle', '--'); legend(axV, [hVm hVc], {'measured', 'commanded'}, 'Location', 'northeast');
axS = nexttile(tl, 4); hold(axS, 'on'); grid(axS, 'on'); ylabel(axS, 'slip ratio \kappa'); title(axS, 'Wheel slip (threshold \pm0.12)');
col = lines(4); hS = gobjects(4, 1); for k = 1:4, hS(k) = animatedline(axS, 'Color', col(k, :)); end
yline(axS, 0.12, 'k--'); yline(axS, -0.12, 'k--'); ylim(axS, [-0.5 0.5]);
axC = nexttile(tl, 6); hold(axC, 'on'); grid(axC, 'on'); ylabel(axC, 'clearance [m]'); xlabel(axC, 'time [s]'); title(axC, 'Clearance to nearest obstacle (collision = 0)');
hCl = animatedline(axC, 'Color', [0 0.45 0.74], 'LineWidth', 1.4); yline(axC, 0, 'r'); ylim(axC, [-0.2 3]);
stNames = {'drive to PICKUP', 'LOADING', 'drive to DELIVERY', 'UNLOADING', 'DONE'};

% ---------------- run the simulation and poll the Controller block ----------------------------------
set_param(mdl, 'SimulationCommand', 'start');
trk = zeros(0, 2); lastT = -1; wall = tic;
while any(strcmp(get_param(mdl, 'SimulationStatus'), {'initializing', 'updating', 'running'}))
    if strcmp(get_param(mdl, 'SimulationStatus'), 'running') && toc(wall) > 0.1
        wall = tic;
        try
            rt = get_param(blk, 'RuntimeObject'); t = get_param(mdl, 'SimulationTime');
            if ~isempty(rt) && t > lastT
                lg = double(rt.OutputPort(2).Data(:)); mv = double(rt.OutputPort(3).Data(:)); rg = double(rt.OutputPort(4).Data(:)); pth = double(rt.OutputPort(5).Data(:));
                lastT = t; trk(end+1, :) = lg(1:2).'; %#ok<AGROW>
                addpoints(hVm, t, lg(4)); addpoints(hVc, t, lg(7));
                for k = 1:4, addpoints(hS(k), t, lg(16 + k)); end
                addpoints(hCl, t, lg(32));
                a = lg(3) - pi + (0:P.lidar.n-1).' * 2*pi / P.lidar.n; hit = rg < P.lidar.rmax - 0.05;
                set(hLid, 'XData', lg(1) + rg(hit) .* cos(a(hit)), 'YData', lg(2) + rg(hit) .* sin(a(hit)));
                set(hTrk, 'XData', trk(:, 1), 'YData', trk(:, 2)); set(hRob, 'XData', lg(1), 'YData', lg(2));
                set(hMv, 'XData', mv(1:2:end), 'YData', mv(2:2:end));
                n = round(pth(1)); if n > 0, pp = reshape(pth(2:end), 2, []).'; set(hPth, 'XData', pp(1:n, 1), 'YData', pp(1:n, 2)); else, set(hPth, 'XData', nan, 'YData', nan); end
                set(hT, 'String', sprintf('t = %.1f s   %s   speed %.2f m/s   route corrections %d   collisions %d', t, stNames{max(1, min(5, round(lg(31))))}, lg(4), round(lg(38)), round(lg(41))));
                for ax = [axV axS axC], xlim(ax, [0 max(10, t + 1)]); end
                drawnow limitrate;
            end
        catch
            % the block output is not available during compilation; try again on the next poll
        end
    else
        pause(0.02);
    end
end
fprintf('Simulation finished (%s). The graphs show the whole run.\n', get_param(mdl, 'SimulationStatus'));
save_system(mdl);
end
