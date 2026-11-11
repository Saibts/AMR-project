function R = run_scout_warehouse_simulation(showExplorer, compareSlip)
%RUN_SCOUT_WAREHOUSE_SIMULATION  Runs the autonomous warehouse mission of the Scout Mini in the
%  Simulink / Simscape Multibody model, then plots and saves all results to ./results.
%    run_scout_warehouse_simulation          % 3D Mechanics Explorer on, slip management ON
%    run_scout_warehouse_simulation(false)   % headless
%    run_scout_warehouse_simulation(false, true)  % also re-run with slip management OFF (slow) and compare
if nargin < 1, showExplorer = true; end
if nargin < 2, compareSlip = false; end
here = fileparts(mfilename('fullpath')); cd(here); addpath(here);
P = scout_amr_params(); mdl = 'scout_amr_sim';
if ischar(showExplorer) && strcmp(showExplorer, 'replot')      % re-draw the figures from the saved results
    L = load(fullfile('results', 'scout_warehouse_results.mat')); R = L.R; main = R(2).r;
    plotWarehouse(main, P); plotKinematics(main, P); plotSafety(main, P); plotSlip(main, P); plotSnapshots(main, P); plotDrive(main, P);
    printSummary(main, P); return
end
if ~isfile([mdl '.slx']), build_scout_amr_model; end
load_system(mdl);
set_param(mdl, 'StopTime', num2str(P.tStop));
if showExplorer, set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'on');
else,            set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off'); end
tcBlk = [mdl '/Mission_and_Slip_Controller/tc_on'];
if ~isfolder('results'), mkdir('results'); end

modes = 1; if compareSlip, modes = [1 0]; end
R = struct();
for m = modes
    set_param(tcBlk, 'Value', num2str(m));
    fprintf('Simulating warehouse mission (slip management %s) ...\n', onoff(m)); tic
    out = sim(mdl);
    fprintf('   done in %.0f s wall time\n', toc);
    r.name = ['Slip management ' onoff(m)];
    r.t  = out.ctrl_log.Time;  r.lg = tsData(out.ctrl_log, 44);
    r.tt = out.tire_log.Time;  r.tl = tsData(out.tire_log, 44);
    r.tm = out.mover_log.Time; r.mv = tsData(out.mover_log, 8);
    r.tr = out.lidar_log.Time; r.rg = tsData(out.lidar_log, 72);
    r.tp = out.path_log.Time;  r.pp = tsData(out.path_log, 161);
    r.tb = out.base_p_log.Time; r.pb = squeeze(out.base_p_log.Data).'; if size(r.pb,1) ~= numel(r.tb), r.pb = r.pb.'; end
    r.Rb = out.base_R_log.Data;     % 3x3xN chassis rotation (for height / roll / pitch)
    R(m+1).r = r; %#ok<AGROW>
end
set_param(tcBlk, 'Value', '1'); save_system(mdl);
main = R(2).r;
save(fullfile('results', 'scout_warehouse_results.mat'), 'R', 'P');
plotWarehouse(main, P);
plotKinematics(main, P);
plotSafety(main, P);
plotSlip(main, P);
plotSnapshots(main, P);
plotDrive(main, P);
if compareSlip, plotCompare(R(2).r, R(1).r, P); end
printSummary(main, P);
scout_amr_drive_sizing;
end

function s = onoff(m), if m, s = 'ON'; else, s = 'OFF'; end, end
function d = tsData(ts, n)
    d = squeeze(ts.Data); if size(d,1) == n && size(d,2) ~= n, d = d.'; end
end
function save_fig(f, name), exportgraphics(f, fullfile('results', name), 'Resolution', 130); end

% -------------------------------------------------------------------------------------------
function drawMap(ax, P)
    hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); xlim(ax, [-2.7 28.7]); ylim(ax, [-2.4 12.2]);
    for i = 1:size(P.patches,1)
        q = P.patches(i,:);
        rectangle(ax, 'Position', [q(1) q(3) q(2)-q(1) q(4)-q(3)], 'FaceColor', [0.82 0.91 1.0], 'EdgeColor', [0.4 0.65 0.95], 'LineStyle', ':');
    end
    for i = 1:size(P.racks,1)
        q = P.racks(i,:); rectangle(ax, 'Position', [q(1) q(3) q(2)-q(1) q(4)-q(3)], 'FaceColor', [0.62 0.70 0.86], 'EdgeColor', [0.25 0.35 0.6]);
    end
    for i = 1:size(P.walls,1)
        q = P.walls(i,:); rectangle(ax, 'Position', [q(1) q(3) q(2)-q(1) q(4)-q(3)], 'FaceColor', [0.4 0.4 0.42]);
    end
    for i = 1:size(P.boxes,1)
        q = P.boxes(i,:); rectangle(ax, 'Position', [q(1) q(3) q(2)-q(1) q(4)-q(3)], 'FaceColor', [0.85 0.6 0.3]);
    end
    plot(ax, P.pickup(1),  P.pickup(2),  's', 'MarkerSize', 13, 'MarkerFaceColor', [0.2 0.75 0.3], 'MarkerEdgeColor', 'k');
    plot(ax, P.deliver(1), P.deliver(2), 's', 'MarkerSize', 13, 'MarkerFaceColor', [0.9 0.3 0.25], 'MarkerEdgeColor', 'k');
    plot(ax, 0, 0, 's', 'MarkerSize', 11, 'MarkerFaceColor', [0.95 0.8 0.15], 'MarkerEdgeColor', 'k');
    text(ax, P.pickup(1)-0.3, P.pickup(2)+0.9, 'PICKUP', 'FontWeight', 'bold', 'Clipping', 'on');
    text(ax, P.deliver(1)-1.4, P.deliver(2)+0.9, 'DELIVERY', 'FontWeight', 'bold', 'Clipping', 'on');
    text(ax, -0.9, -0.9, 'START', 'FontWeight', 'bold', 'Clipping', 'on');
    xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
end

function plotWarehouse(r, P)
    f = figure('Name', 'Warehouse mission', 'Theme', 'light', 'Color', 'w', 'Position', [30 30 1400 640]); ax = axes(f); drawMap(ax, P);
    % planned paths (every replan, faint) and the driven trajectory coloured by mission state
    hPlan = [];
    for k = 1:6:size(r.pp, 1)
        n = r.pp(k,1); if n < 2, continue; end
        xy = reshape(r.pp(k, 2:161), 2, [])'; xy = xy(1:n, :);
        hPlan = plot(ax, xy(:,1), xy(:,2), '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
    end
    cs = {[0.1 0.5 0.9], [0.2 0.7 0.3], [0.8 0.2 0.2], [0.9 0.5 0.1], [0.4 0.4 0.4]};
    hSt = gobjects(1, 4);
    for s = 1:4
        i = r.lg(:,31) == s; if ~any(i), continue; end
        x = r.lg(:,1); y = r.lg(:,2); x(~i) = NaN; y(~i) = NaN;
        hSt(s) = plot(ax, x, y, '-', 'Color', cs{s}, 'LineWidth', 2);
    end
    % moving obstacles (trails)
    mc = {[0.95 0.55 0.1], [0.9 0.75 0.05], [0.15 0.3 0.6], [0.8 0.7 0.1]}; mn = {'worker 1', 'worker 2', 'second AMR', 'pallet truck'};
    for k = 1:4
        plot(ax, r.mv(:,2*k-1), r.mv(:,2*k), '.', 'Color', mc{k}, 'MarkerSize', 3);
    end
    % re-route events
    ev = find(diff(r.lg(:,38)) > 0);
    if ~isempty(ev), plot(ax, r.lg(ev,1), r.lg(ev,2), 'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 14); end
    hv = [hPlan, hSt(1), hSt(3)]; lb = {'planned paths (every replan)', 'driving to pickup', 'loaded: driving to delivery'};
    if ~isempty(ev), hv(end+1) = findobj(ax, 'Marker', 'p', '-depth', 1); lb{end+1} = 're-route event'; end
    legend(ax, hv, lb, 'Location', 'northeast', 'AutoUpdate', 'off');
    for k = 1:4, text(ax, r.mv(end,2*k-1), r.mv(end,2*k)+0.5, mn{k}, 'FontSize', 8, 'Color', mc{k}*0.7); end
    title(ax, sprintf('Scout Mini warehouse mission: pickup -> delivery (stars = re-route events; %d re-routes, %d collisions)', r.lg(end,38), r.lg(end,41)));
    save_fig(f, '1_warehouse_trajectory.png');
end

function plotKinematics(r, P)
    f = figure('Name', 'Kinematics', 'Theme', 'light', 'Color', 'w', 'Position', [50 50 1250 780]); t = r.t; wc = lines(4);
    subplot(3,1,1); plot(t, r.lg(:,4), 'LineWidth', 1.4); hold on; plot(t, r.lg(:,7), '--'); plot(t, r.lg(:,37), ':');
    legend('measured v_x', 'commanded v', 'planner request'); ylabel('linear velocity [m/s]'); grid on; title('Robot velocity'); stateBands(gca, r);
    subplot(3,1,2); plot(t, r.lg(:,6), 'LineWidth', 1.4); hold on; plot(t, r.lg(:,8), '--'); legend('measured \omega_z', 'commanded \omega'); ylabel('angular velocity [rad/s]'); grid on; stateBands(gca, r);
    subplot(3,1,3); for k = 1:4, plot(t, r.lg(:,12+k), 'Color', wc(k,:)); hold on; end
    legend('w1 FL', 'w2 RL', 'w3 FR', 'w4 RR'); ylabel('wheel speed [rad/s]'); xlabel('time [s]'); grid on; stateBands(gca, r);
    sgtitle('Velocity, angular velocity and wheel speeds'); save_fig(f, '2_velocity_wheel_speeds.png');
end

function plotSafety(r, P)
    f = figure('Name', 'Perception and safety', 'Theme', 'light', 'Color', 'w', 'Position', [70 70 1250 780]); t = r.t;
    subplot(3,1,1); plot(t, r.lg(:,39), 'Color', [0.1 0.5 0.8]); hold on; plot(t, r.lg(:,42), 'Color', [0.85 0.4 0.1]);
    yline(0.55, 'r--', 'safety stop'); legend('closest LiDAR return', 'front sector (+/-50 deg)'); ylabel('range [m]'); ylim([0 8.2]); grid on; title('LiDAR: obstacle detection'); stateBands(gca, r);
    subplot(3,1,2); plot(t, r.lg(:,32), 'LineWidth', 1.3); yline(0, 'r-', 'collision'); ylabel('clearance to nearest obstacle [m]'); ylim([-0.5 4]); grid on; title('Collision monitoring (ground truth)'); stateBands(gca, r);
    subplot(3,1,3); stairs(t, r.lg(:,38), 'LineWidth', 1.5); hold on; stairs(t, r.lg(:,41), 'r', 'LineWidth', 1.5); stairs(t, r.lg(:,31), 'k:');
    legend('route corrections (replans around blockage)', 'collisions', 'mission state'); ylabel('count / state'); xlabel('time [s]'); grid on; title('Re-routing, collisions and mission state'); stateBands(gca, r);
    sgtitle('Obstacle detection, avoidance and collision monitoring'); save_fig(f, '3_perception_safety.png');
end

function plotSlip(r, P)
    f = figure('Name', 'Slip management', 'Theme', 'light', 'Color', 'w', 'Position', [90 90 1250 780]); t = r.t; wc = lines(4);
    subplot(2,2,1); for k = 1:4, plot(t, r.lg(:,16+k), 'Color', wc(k,:)); hold on; end
    yline(P.k_th, 'k--'); yline(-P.k_th, 'k--'); ylabel('slip ratio \kappa'); legend('w1', 'w2', 'w3', 'w4'); grid on; title('Wheel slip ratio (managed)');
    subplot(2,2,2); for k = 1:4, plot(t, r.lg(:,20+k), 'Color', wc(k,:)); hold on; end
    ylabel('wheel torque [Nm]'); grid on; title('Wheel torque after traction limiting');
    subplot(2,2,3); plot(t, r.lg(:,29), 'LineWidth', 1.5); hold on; plot(r.tt, mean(r.tl(:, [11 22 33 44]), 2), 'k:');
    legend('estimated \mu', 'true \mu under robot'); ylabel('friction coefficient'); xlabel('time [s]'); ylim([0 1]); grid on; title('Friction estimation (oil/water spills)');
    subplot(2,2,4); for k = 1:4, plot(r.tt, r.tl(:, 11*(k-1)+7), 'Color', wc(k,:)); hold on; end
    ylabel('normal load [N]'); xlabel('time [s]'); grid on; title('Tyre normal loads');
    sgtitle('Wheel-ground interaction and slip management'); save_fig(f, '4_slip_management.png');
end

function plotDrive(r, P)
    % Motor torque-speed limit with the operating points of the run, current, power and battery (report Section 8)
    D = scout_amr_drive_post(r.t, r.lg, P); S = scout_amr_drive_sizing(false); t = r.t; wc = lines(4);
    f = figure('Name', 'Motor and battery', 'Theme', 'light', 'Color', 'w', 'Position', [150 150 1250 780]);
    subplot(2,2,1); w = linspace(0, P.Vbus/P.kt, 300);
    plot(w, min(P.tmax, P.kt*(P.Vbus - P.kt*w)/P.Rm), 'k-', 'LineWidth', 1.8); hold on; yline(P.trated, 'b--', 'rated 5 N m');
    for k = 1:4, plot(abs(r.lg(:,12+k)), abs(r.lg(:,20+k)), '.', 'Color', wc(k,:), 'MarkerSize', 3); end
    for i = 1:3, yline(S.tauGrip(i), ':', sprintf('grip %s %.2f', S.muName{i}, S.tauGrip(i)), 'Color', [0.5 0.5 0.5]); end
    xlabel('wheel speed [rad/s]'); ylabel('wheel torque [N m]'); grid on; title('Torque-speed limit and operating points (four wheels)');
    legend('motor limit', '', 'w1', 'w2', 'w3', 'w4', 'Location', 'northeast');
    subplot(2,2,2); for k = 1:4, plot(t, D.I(:,k), 'Color', wc(k,:)); hold on; end
    yline(P.Irated, 'b--', 'rated 7.8 A'); yline(P.Ipk, 'r--', 'peak 23.4 A'); ylabel('motor current [A]'); grid on; title('Motor current per wheel'); stateBands(gca, r);
    subplot(2,2,3); plot(t, D.Ptot, 'LineWidth', 1.2); hold on; yline(4*150, 'r--', 'rated 4 x 150 W'); yline(D.Pmean, 'k:', sprintf('mean %.0f W', D.Pmean));
    ylabel('electrical power [W]'); xlabel('time [s]'); ylim([-100 650]); grid on; title(sprintf('Power from the battery (%.2f Wh used)', D.energyWh)); stateBands(gca, r);
    subplot(2,2,4); yyaxis left; plot(t, D.Vt, 'LineWidth', 1.4); ylabel('battery terminal voltage [V]'); yyaxis right; plot(t, 100*D.soc, 'LineWidth', 1.4); ylabel('state of charge [%]');
    xlabel('time [s]'); grid on; title(sprintf('Battery: minimum %.2f V, charge at the end %.2f %%', D.Vmin, 100*D.socEnd));
    sgtitle('Hub motors and battery (calculated from the logged torques and wheel speeds)'); save_fig(f, '7_motor_battery.png');
    plotDriveDesign(S, P);
end

function plotDriveDesign(S, P)
    % Torque-speed limit of one wheel motor with the hand-calculated design cases (report Fig. 28)
    f = figure('Name', 'Drive design cases', 'Theme', 'light', 'Color', 'w', 'Position', [170 170 1000 600]); w = linspace(0, P.Vbus/P.kt, 300);
    plot(w, min(P.tmax, P.kt*(P.Vbus - P.kt*w)/P.Rm), 'k-', 'LineWidth', 2); hold on;
    plot(w, min(P.tmax, P.kt*max(P.Vempty - P.kt*w, 0)/P.Rm), 'k--');
    yline(P.trated, 'b--', 'rated 5.0 N m'); cm = [lines(7); 0 0 0]; cm = cm(1:numel(S.cases), :); nm = {};
    for i = 1:numel(S.cases), plot(S.cases(i).omega, abs(S.cases(i).tau), 'o', 'MarkerFaceColor', cm(i,:), 'MarkerEdgeColor', 'k', 'MarkerSize', 9); nm{i} = S.cases(i).name; end %#ok<AGROW>
    for i = 1:3, yline(S.tauGrip(i), ':', sprintf('grip, %s: %.2f N m', S.muName{i}, S.tauGrip(i)), 'Color', [0.45 0.45 0.45], 'LabelHorizontalAlignment', 'right'); end
    xline(S.knee, 'g:', sprintf('knee %.1f rad/s', S.knee), 'LabelVerticalAlignment', 'bottom'); xlabel('wheel speed [rad/s]'); ylabel('wheel torque [N m]'); grid on;
    legend(['motor limit, 24 V', 'motor limit, 21 V (empty)', {'rated'}, nm], 'Location', 'northeast', 'FontSize', 8);
    title('Torque-speed limit of one hub motor, design cases and grip limits'); save_fig(f, '8_drive_design_cases.png');
end

function plotSnapshots(r, P)
    % Four demonstration moments, found from the logged data
    [~, ib] = min(hypot(r.lg(:,1) - mean(P.boxes(1,1:2)), r.lg(:,2) - mean(P.boxes(1,3:4))));               % passing the aisle box
    d1 = hypot(interp1(r.tm, r.mv(:,1), r.t) - r.lg(:,1), interp1(r.tm, r.mv(:,2), r.t) - r.lg(:,2));
    [~, ih] = min(d1);                                                                                         % closest to the crossing worker
    ir = find(diff(r.lg(:,38)) > 0, 1); if isempty(ir), [~, ir] = min(abs(r.t - r.t(end)*0.7)); end         % first route correction
    d2 = hypot(interp1(r.tm, r.mv(:,3), r.t) - r.lg(:,1), interp1(r.tm, r.mv(:,4), r.t) - r.lg(:,2));
    [~, i2] = min(d2);                                                                                         % closest to worker 2
    idx = [ib ih ir i2]; ttl = {'avoiding the floor box', 'worker crossing the aisle', 'first re-route: path blocked', 'worker in aisle 3'};
    f = figure('Name', 'LiDAR snapshots', 'Theme', 'light', 'Color', 'w', 'Position', [110 110 1400 800]);
    for s = 1:4
        ax = subplot(2,2,s); drawMap(ax, P); i = idx(s); t0 = r.t(i);
        [~, ik] = min(abs(r.tr - t0)); rg = r.rg(ik,:); a = r.lg(i,3) - pi + (0:71)*2*pi/72;
        hx = r.lg(i,1) + rg .* cos(a); hy = r.lg(i,2) + rg .* sin(a); hit = rg < 7.9;
        for q = find(hit), plot(ax, [r.lg(i,1) hx(q)], [r.lg(i,2) hy(q)], '-', 'Color', [1 0.7 0.7]); end
        plot(ax, hx(hit), hy(hit), 'r.', 'MarkerSize', 8);
        [~, im] = min(abs(r.tm - t0)); mc = {[0.95 0.55 0.1], [0.9 0.75 0.05], [0.15 0.3 0.6], [0.8 0.7 0.1]};
        for k = 1:4, plot(ax, r.mv(im,2*k-1), r.mv(im,2*k), 'o', 'MarkerFaceColor', mc{k}, 'MarkerEdgeColor', 'k', 'MarkerSize', 9); end
        [~, ip] = min(abs(r.tp - t0)); n = r.pp(ip,1);
        if n >= 2, xy = reshape(r.pp(ip, 2:161), 2, [])'; plot(ax, xy(1:n,1), xy(1:n,2), 'g-', 'LineWidth', 2); end
        plot(ax, r.lg(1:i,1), r.lg(1:i,2), 'b-', 'LineWidth', 1.5);
        plot(ax, r.lg(i,1), r.lg(i,2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 9);
        xlim(ax, r.lg(i,1) + [-6 6]); ylim(ax, r.lg(i,2) + [-4 4]);
        title(ax, sprintf('t = %.1f s: %s', t0, ttl{s}));
    end
    sgtitle('LiDAR scans (red), replanned path (green) and moving obstacles'); save_fig(f, '5_lidar_scenarios.png');
end

function plotCompare(a, b, P)
    f = figure('Name', 'Slip ON vs OFF', 'Theme', 'light', 'Color', 'w', 'Position', [130 130 1250 560]);
    subplot(1,2,1); plot(a.t, max(abs(a.lg(:,17:20)), [], 2)); hold on; plot(b.t, max(abs(b.lg(:,17:20)), [], 2));
    legend(a.name, b.name); ylabel('max |\kappa|'); xlabel('time [s]'); grid on; title('Worst-wheel slip ratio');
    subplot(1,2,2); plot(a.t, a.lg(:,4)); hold on; plot(b.t, b.lg(:,4)); ylabel('v_x [m/s]'); xlabel('time [s]'); grid on; title('Speed');
    save_fig(f, '6_slip_on_vs_off.png');
end

function stateBands(ax, r)
    % light background bands: 1 to pickup, 2 loading, 3 to delivery, 4 unloading
    hold(ax, 'on'); yl = ylim(ax); col = {[0.90 0.95 1.0], [0.88 1.0 0.88], [1.0 0.93 0.88], [1.0 0.90 0.90]};
    s = r.lg(:,31); ch = [1; find(diff(s) ~= 0) + 1; numel(s) + 1];
    for k = 1:numel(ch)-1
        st = s(ch(k)); if st < 1 || st > 4, continue; end
        patch(ax, [r.t(ch(k)) r.t(ch(k+1)-1) r.t(ch(k+1)-1) r.t(ch(k))], [yl(1) yl(1) yl(2) yl(2)], col{st}, 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');
    end
    pp_ = findobj(ax, 'Type', 'patch'); ax.Children = [setdiff(ax.Children, pp_, 'stable'); pp_]; ax.SortMethod = 'childorder'; ylim(ax, yl);
end

function printSummary(r, P)
    lg = r.lg; t = r.t; d = find(lg(:,33) > 0.5, 1);
    s = lg(:,31); tp = t(find(s >= 2, 1)); td = t(find(s >= 3, 1)); ts = t(find(s >= 4, 1));
    dist = sum(hypot(diff(lg(:,1)), diff(lg(:,2))));
    fprintf('\n========== WAREHOUSE MISSION RESULTS (%s) ==========\n', r.name);
    if isempty(d), fprintf('mission:            NOT completed within %.0f s\n', t(end)); else, fprintf('mission time:       %.1f s (done flag)\n', t(d)); end
    fprintf('start -> pickup:    reached at t = %.1f s\n', tp);
    fprintf('pickup -> delivery: left at t = %.1f s, delivered at t = %.1f s\n', tp + P.dwell, ts);
    fprintf('distance driven:    %.1f m   mean speed %.2f m/s   top speed %.2f m/s\n', dist, dist / max(t(end),1), max(lg(:,4)));
    fprintf('route corrections:  %d     collisions: %d     min clearance: %.2f m\n', lg(end,38), lg(end,41), min(lg(:,32)));
    fprintf('closest LiDAR hit:  %.2f m   peak slip ratio: %.2f   wheel energy: %.0f J\n', min(lg(:,39)), max(max(abs(lg(:,17:20)))), trapz(t, sum(abs(lg(:,21:24) .* lg(:,13:16)), 2)));
    fprintf('figures saved in:   %s\n', fullfile(pwd, 'results'));
end
