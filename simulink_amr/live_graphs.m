function live_graphs(showExplorer, interactive)
%LIVE_GRAPHS  Runs the warehouse simulation and updates the graphs WHILE the simulation runs.
%    live_graphs              % Multibody Explorer (3D) + live graphs, click-to-go, slip management ON
%    live_graphs(false)       % live graphs only (no 3D window)
%    live_graphs(true,false)  % scripted pickup-to-delivery mission instead of click-to-go
%  Click-to-go: click anywhere on the map (left tile of the graph figure); the robot plans a route
%  (A*, LiDAR avoidance) and drives there. The green button "Autonomous pickup and delivery" runs the
%  whole mission (pickup, loading, delivery, unloading) from the robot's current position.
%  Slip management (traction control) is always active.
%  The live values are read from the output ports of the Controller block (44-value log vector,
%  mover positions, LiDAR ranges) every 0.1 s of wall time; nothing in the model is changed.
if nargin < 1, showExplorer = true; end
if nargin < 2, interactive = true; end
here = fileparts(mfilename('fullpath')); cd(here); addpath(here);
P = scout_amr_params(); mdl = 'scout_amr_sim';
if ~isfile([mdl '.slx']), build_scout_amr_model; end
if bdIsLoaded(mdl), close_system(mdl, 0); end     % always start from the file on disk, not a stale copy in memory
load_system(mdl);
cgBlk = [mdl '/Mission_and_Slip_Controller/click_goal'];
set_param(mdl, 'StopTime', num2str(P.tStop));
if interactive, set_param(mdl, 'StopTime', '3600'); set_param(cgBlk, 'Value', '[1 0 0 0]'); else, set_param(cgBlk, 'Value', '[0 0 0 0]'); end
cleanup = onCleanup(@() resetModel(mdl, cgBlk, P.tStop));   % the saved model must stay in the scripted-mission setting (click_goal mode 0)
set_param([mdl '/Mission_and_Slip_Controller/tc_on'], 'Value', '1');
if showExplorer, set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'on');
else,            set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off'); end
blk = [mdl '/Mission_and_Slip_Controller/Controller'];

% ---------------- figure: map + three signal panels --------------------------------------------
f = figure('Name', 'Live mission', 'Theme', 'light', 'Color', 'w', 'Position', [20 40 1800 760]);
tl = tiledlayout(f, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
tl.OuterPosition = [0 0.07 1 0.93];
hMon = annotation(f, 'textbox', [0.005 0.005 0.99 0.06], 'String', 'Wheel slip monitor', 'FontSize', 13, 'FontWeight', 'bold', ...
    'VerticalAlignment', 'middle', 'BackgroundColor', [0.85 0.95 0.85], 'EdgeColor', [0.3 0.6 0.3]);
axM = nexttile(tl, 1, [3 1]); hold(axM, 'on'); axis(axM, 'equal'); grid(axM, 'on');
for k = 1:size(P.racks, 1), r = P.racks(k, :); rectangle(axM, 'Position', [r(1) r(3) r(2)-r(1) r(4)-r(3)], 'FaceColor', [0.62 0.69 0.87], 'EdgeColor', [0.2 0.3 0.6]); end
for k = 1:size(P.boxes, 1), r = P.boxes(k, :); rectangle(axM, 'Position', [r(1) r(3) r(2)-r(1) r(4)-r(3)], 'FaceColor', [0.85 0.58 0.25]); end
for k = 1:size(P.patches, 1)
    r = P.patches(k, :);
    patch(axM, r([1 2 2 1]), r([3 3 4 4]), [0.05 0.35 1.0], 'FaceAlpha', 0.35, 'EdgeColor', [0.05 0.35 1.0], 'LineWidth', 1.8, 'LineStyle', '--');
    text(axM, (r(1)+r(2))/2, (r(3)+r(4))/2, sprintf('SLIP ZONE\n\\mu = %.2f', r(5)), 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', [0 0.2 0.7]);
end
plot(axM, P.pickup(1), P.pickup(2), 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 9); text(axM, P.pickup(1), P.pickup(2)+0.7, 'PICKUP', 'FontWeight', 'bold');
plot(axM, P.deliver(1), P.deliver(2), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 9); text(axM, P.deliver(1)-2.5, P.deliver(2)+0.7, 'DELIVERY', 'FontWeight', 'bold');
hLid = plot(axM, nan, nan, '.', 'Color', [0.9 0.2 0.2], 'MarkerSize', 6);
hTrk = plot(axM, nan, nan, 'b-', 'LineWidth', 2);
hRob = plot(axM, nan, nan, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 9);
hMv  = plot(axM, nan(4, 1), nan(4, 1), 'o', 'MarkerFaceColor', [0.95 0.75 0.1], 'MarkerEdgeColor', 'k', 'MarkerSize', 9);
hPth = plot(axM, nan, nan, 'g-', 'LineWidth', 1.8);
hGoal = plot(axM, nan, nan, 'p', 'MarkerFaceColor', [0.9 0 0.9], 'MarkerEdgeColor', 'k', 'MarkerSize', 16);
gid = 0; clickMsg = '';
if interactive
    uicontrol(f, 'Style', 'pushbutton', 'String', 'Autonomous pickup and delivery', 'Units', 'normalized', 'Position', [0.01 0.90 0.20 0.07], ...
        'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', [0.2 0.65 0.3], 'ForegroundColor', 'w', 'Callback', @onMissionButton);
    f.WindowButtonDownFcn = @onMapClick;
    set(findobj(axM, 'Type', 'patch'), 'HitTest', 'off'); set(findobj(axM, 'Type', 'rectangle'), 'HitTest', 'off');
end
xlim(axM, [-2.5 28.5]); ylim(axM, [-2.2 12]); xlabel(axM, 'x [m]'); ylabel(axM, 'y [m]'); hT = title(axM, 'Live mission');

axV = nexttile(tl, 2); hold(axV, 'on'); grid(axV, 'on'); ylabel(axV, 'speed [m/s]'); title(axV, 'Forward speed');
hVm = animatedline(axV, 'Color', [0 0.45 0.74], 'LineWidth', 1.4); hVc = animatedline(axV, 'Color', [0.85 0.33 0.1], 'LineStyle', '--'); legend(axV, [hVm hVc], {'measured', 'commanded'}, 'Location', 'northeast');
axS = nexttile(tl, 5); hold(axS, 'on'); grid(axS, 'on'); ylabel(axS, 'slip ratio \kappa'); title(axS, 'Wheel slip (threshold \pm0.12)');
col = lines(4); hS = gobjects(4, 1); for k = 1:4, hS(k) = animatedline(axS, 'Color', col(k, :)); end
yline(axS, 0.12, 'k--'); yline(axS, -0.12, 'k--'); ylim(axS, [-0.5 0.5]);
axC = nexttile(tl, 8); hold(axC, 'on'); grid(axC, 'on'); ylabel(axC, 'clearance [m]'); xlabel(axC, 'time [s]'); title(axC, 'Clearance to nearest obstacle (collision = 0)');
hCl = animatedline(axC, 'Color', [0 0.45 0.74], 'LineWidth', 1.4); yline(axC, 0, 'r'); ylim(axC, [-0.2 3]);
axI = nexttile(tl, 3); hold(axI, 'on'); grid(axI, 'on'); ylabel(axI, 'current [A]'); title(axI, 'Motor current I = torque / Kt (rated black, peak red)');
hI = gobjects(4, 1); for k = 1:4, hI(k) = animatedline(axI, 'Color', col(k, :)); end
yline(axI, P.Irated, 'k--'); yline(axI, -P.Irated, 'k--'); yline(axI, P.Ipk, 'r--'); yline(axI, -P.Ipk, 'r--'); ylim(axI, [-1.15*P.Ipk 1.15*P.Ipk]);
axB = nexttile(tl, 6); hold(axB, 'on'); grid(axB, 'on'); ylabel(axB, 'voltage [V]'); hBt = title(axB, 'Battery terminal voltage (empty = 21 V)');
hVt = animatedline(axB, 'Color', [0 0.45 0.74], 'LineWidth', 1.4); yline(axB, P.Vempty, 'r'); ylim(axB, [P.Vempty - 0.5, P.Vfull + 0.3]);
axP = nexttile(tl, 9); hold(axP, 'on'); grid(axP, 'on'); ylabel(axP, 'power [W]'); xlabel(axP, 'time [s]'); title(axP, 'Electrical power from the battery (all motors + idle)');
hPw = animatedline(axP, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.4);
soc = 1; Ahused = 0; tPrev = 0;
stNames = {'drive to PICKUP', 'LOADING', 'drive to DELIVERY', 'UNLOADING', 'MISSION COMPLETE - click the map or press the button', 'GOING TO CLICKED POINT', 'READY - click the map'};

% ---------------- run the simulation and poll the Controller block ----------------------------------
set_param(mdl, 'SimulationCommand', 'start');
trk = zeros(0, 2); lastT = -1; wall = tic;
while any(strcmp(get_param(mdl, 'SimulationStatus'), {'initializing', 'updating', 'running'}))
    if ~isvalid(f), set_param(mdl, 'SimulationCommand', 'stop'); break; end      % closing the figure ends the run
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
                tau = lg(21:24); om = lg(13:16); I = tau / P.kt;
                Ptot = sum(tau .* om + P.Rm * I.^2) + P.P0;                       % same formulas as scout_amr_drive_post
                ocv = P.Vempty + (P.Vfull - P.Vempty) * soc;
                Ib = (ocv - sqrt(max(ocv^2 - 4*P.Rint*Ptot, 0))) / (2*P.Rint); Vt = ocv - P.Rint * Ib;
                Ahused = Ahused + Ib * max(t - tPrev, 0) / 3600; tPrev = t; soc = 1 - Ahused / P.Ah;
                for k = 1:4, addpoints(hI(k), t, I(k)); end
                addpoints(hVt, t, Vt); addpoints(hPw, t, Ptot);
                set(hBt, 'String', sprintf('Battery terminal voltage (empty = 21 V),  charge %.1f %%', 100*soc));
                a = lg(3) - pi + (0:P.lidar.n-1).' * 2*pi / P.lidar.n; hit = rg < P.lidar.rmax - 0.05;
                set(hLid, 'XData', lg(1) + rg(hit) .* cos(a(hit)), 'YData', lg(2) + rg(hit) .* sin(a(hit)));
                set(hTrk, 'XData', trk(:, 1), 'YData', trk(:, 2)); set(hRob, 'XData', lg(1), 'YData', lg(2));
                set(hMv, 'XData', mv(1:2:end), 'YData', mv(2:2:end));
                n = round(pth(1)); if n > 0, pp = reshape(pth(2:end), 2, []).'; set(hPth, 'XData', pp(1:n, 1), 'YData', pp(1:n, 2)); else, set(hPth, 'XData', nan, 'YData', nan); end
                set(hT, 'String', sprintf('t = %.1f s   %s   speed %.2f m/s   route corrections %d   collisions %d  %s', t, stNames{max(1, min(7, round(lg(31))))}, lg(4), round(lg(38)), round(lg(41)), clickMsg));
                % wheel slip monitor: slip %, wheel vs ground speed, traction (friction estimate), surface under the robot
                surf = 'normal floor';
                for q = 1:size(P.patches, 1)
                    r = P.patches(q, :);
                    if lg(1) >= r(1) && lg(1) <= r(2) && lg(2) >= r(3) && lg(2) <= r(4)
                        if r(5) < 0.2, surf = sprintf('OIL (mu %.2f)', r(5)); else, surf = sprintf('WATER (mu %.2f)', r(5)); end
                    end
                end
                if lg(30) > 0.5
                    txt = 'SLIP - torque and acceleration limited'; bg = [1 0.85 0.8]; ec = [0.8 0.2 0.1];
                else
                    txt = 'GRIP OK'; bg = [0.85 0.95 0.85]; ec = [0.3 0.6 0.3];
                end
                set(hMon, 'String', sprintf('Wheel slip monitor:  %s   |   slip %.0f %%   |   wheel %+.2f  ground %+.2f m/s   |   traction %.0f %%   |   surface: %s', ...
                    txt, 100 * max(abs(lg(17:20))), P.r * mean(lg(13:16)), lg(4), 100 * lg(29) / P.mu0, surf), 'BackgroundColor', bg, 'EdgeColor', ec);
                for ax = [axV axS axC axI axB axP], xlim(ax, [0 max(10, t + 1)]); end
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
tw = tic; while ~strcmp(get_param(mdl, 'SimulationStatus'), 'stopped') && toc(tw) < 10, pause(0.05); end   % let a stop request finish
resetModel(mdl, cgBlk, P.tStop);
save_system(mdl);

    function onMissionButton(~, ~)
        if ~strcmp(get_param(mdl, 'SimulationStatus'), 'running'), return; end
        gid = gid + 1; clickMsg = '';
        set_param(cgBlk, 'Value', sprintf('[2 0 0 %d]', gid));     % mode 2: pickup and delivery from where the robot is
        set(hGoal, 'XData', nan, 'YData', nan);
    end

    function onMapClick(~, ~)
        cpt = axM.CurrentPoint(1, 1:2);
        xlm = xlim(axM); ylm = ylim(axM);
        if cpt(1) < xlm(1) || cpt(1) > xlm(2) || cpt(2) < ylm(1) || cpt(2) > ylm(2), return; end
        if ~strcmp(get_param(mdl, 'SimulationStatus'), 'running'), return; end
        mrg = 0.5;                                            % planner inflation: keep the goal clear of racks and walls
        inObst = false;
        for iq = 1:size(P.racks, 1), rr = P.racks(iq, :); inObst = inObst || (cpt(1) > rr(1)-mrg && cpt(1) < rr(2)+mrg && cpt(2) > rr(3)-mrg && cpt(2) < rr(4)+mrg); end
        for iq = 1:size(P.walls, 1), rr = P.walls(iq, :); inObst = inObst || (cpt(1) > rr(1)-mrg && cpt(1) < rr(2)+mrg && cpt(2) > rr(3)-mrg && cpt(2) < rr(4)+mrg); end
        if inObst, clickMsg = '(that point is inside a rack or wall - click in an aisle)'; return; end
        gid = gid + 1; clickMsg = '';
        set_param(cgBlk, 'Value', sprintf('[1 %.3f %.3f %d]', cpt(1), cpt(2), gid));
        set(hGoal, 'XData', cpt(1), 'YData', cpt(2));
    end
end

function resetModel(mdl, cgBlk, tStop)
    try
        if bdIsLoaded(mdl)
            if ~any(strcmp(get_param(mdl, 'SimulationStatus'), {'stopped', 'terminating'})), return; end   % never edit a running model
            set_param(cgBlk, 'Value', '[0 0 0 0]'); set_param(mdl, 'StopTime', num2str(tStop));
        end
    catch
    end
end
