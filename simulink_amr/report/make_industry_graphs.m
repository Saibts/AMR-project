function make_industry_graphs()
%MAKE_INDUSTRY_GRAPHS  Light-theme graphs of the simulate_industry_amr_warehouse run (data from capture_industry_full).
here = fileparts(mfilename('fullpath')); F = fullfile(here, 'figs');
L = load(fullfile(F, 'industry_telemetry.mat')); D = L.D;
g = @(nm, i) D(find(strcmp([D.name], nm) & strcmp([D.yl], D(i).yl), 1)); %#ok<NASGU>
byName = @(nm) D(find([D.name] == nm, 1));
v = byName("v [m/s]"); w = byName("w [rad/s]"); rpm = byName("FL RPM/100");
slp = byName("Slip Ratio s"); clr = byName("Clearance [m]"); drf = byName("Odom Drift [m]"); imu = byName("IMU Gyro Yaw");
clr.y(1) = NaN; t = v.x; blue = [0.05 0.35 0.75]; org = [0.9 0.45 0.05]; grn = [0.1 0.55 0.25]; red = [0.8 0.1 0.1]; pur = [0.5 0.2 0.6];

f = figure('Theme', 'light', 'Color', 'w', 'Position', [20 20 1100 850]);
subplot(3,1,1); plot(t, v.y, 'Color', blue, 'LineWidth', 1.6); grid on; ylabel('v [m/s]'); title('Linear speed'); xlim([0 t(end)]); ylim([0 1]);
subplot(3,1,2); plot(t, w.y, 'Color', org, 'LineWidth', 1.6); grid on; ylabel('\omega [rad/s]'); title('Yaw rate'); xlim([0 t(end)]);
subplot(3,1,3); plot(t, 100*rpm.y, 'Color', grn, 'LineWidth', 1.6); hold on; plot(t, slp.y, 'Color', pur, 'LineWidth', 1.4);
grid on; ylabel('rpm  |  slip ratio'); xlabel('time [s]'); title('Front-left motor speed and wheel slip ratio'); legend({'FL motor [rpm]', 'slip ratio s'}, 'Location', 'southeast'); xlim([0 t(end)]);
exportgraphics(f, fullfile(F, 'industry_g1_kinematics.png'), 'Resolution', 130); close(f);

f = figure('Theme', 'light', 'Color', 'w', 'Position', [20 20 1100 700]);
subplot(2,1,1); plot(t, clr.y, 'Color', blue, 'LineWidth', 1.6); hold on;
yline(1.80, ':', 'warning field 1.80 m', 'Color', [0.3 0.3 0.3], 'LabelHorizontalAlignment', 'left'); yline(1.05, '--', 'braking field 1.05 m', 'Color', org, 'LabelHorizontalAlignment', 'left');
yline(0.65, '--', 'ISO stop 0.65 m', 'Color', red, 'LabelHorizontalAlignment', 'left'); grid on; ylabel('clearance [m]'); title('Clearance to the nearest obstacle (LiDAR) and ISO 3691-4 fields'); xlim([0 t(end)]); ylim([0 2.1]);
subplot(2,1,2); plot(t, drf.y, 'Color', red, 'LineWidth', 1.6); hold on; plot(t, abs(imu.y), 'Color', blue, 'LineWidth', 1.2);
grid on; xlabel('time [s]'); ylabel('drift [m]  |  |gyro| [rad/s]'); title('Wheel-odometry drift and IMU yaw-rate magnitude'); legend({'odometry drift [m]', '|IMU gyro yaw| [rad/s]'}, 'Location', 'northwest'); xlim([0 t(end)]);
exportgraphics(f, fullfile(F, 'industry_g2_safety_sensors.png'), 'Resolution', 130); close(f);

p = D(9); f = figure('Theme', 'light', 'Color', 'w', 'Position', [20 20 1000 560]); plot(p.x, p.y, 'Color', blue, 'LineWidth', 2); grid on; axis equal; hold on;
plot(p.x(1), p.y(1), 'o', 'MarkerFaceColor', [1 0.9 0], 'MarkerEdgeColor', 'k', 'MarkerSize', 10); plot(p.x(end), p.y(end), 'p', 'MarkerFaceColor', [0.1 0.8 0.2], 'MarkerEdgeColor', 'k', 'MarkerSize', 16);
xlabel('x [m]'); ylabel('y [m]'); title(sprintf('Driven path of the Scout Mini (%.1f m in %.1f s)', sum(hypot(diff(p.x), diff(p.y))), t(end))); legend({'path', 'start', 'dock'}, 'Location', 'southeast');
exportgraphics(f, fullfile(F, 'industry_g3_path.png'), 'Resolution', 130); close(f);
fprintf('mission %.1f s, path %.1f m, min clearance %.2f m, max drift %.2f m, max slip %.3f, vmax %.2f, wmax %.2f\n', t(end), sum(hypot(diff(p.x), diff(p.y))), min(clr.y), max(drf.y), max(abs(slp.y)), max(v.y), max(abs(w.y)));
end
