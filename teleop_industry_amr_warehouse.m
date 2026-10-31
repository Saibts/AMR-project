function teleop_industry_amr_warehouse()
%% =========================================================================
%  AGILEX SCOUT MINI 4WD: INTERACTIVE TELEOPERATION & INDUSTRIAL SIMULATION
%  Standard: ISO 3691-4 Safety Standard for Driverless Industrial Trucks
%  Platform: AgileX Scout Mini 4WD Skid-Steer Mobile Robot
%  Control : Keyboard (W/A/S/D / Arrows / Space / Q / E) & On-Screen Touch D-Pad
%  Physics : Multibody Dynamics, Surface Friction, Suspension & Sensor Drift
%  Theme   : Clean Industrial Engineering / CAD Robotics Workstation
% =========================================================================

close all; clc;

fprintf('===================================================================\n');
fprintf('  AgileX Scout Mini: Interactive Teleop & Warehouse Simulation\n');
fprintf('  Controls:\n');
fprintf('    [W / Up Arrow]    : Drive Forward\n');
fprintf('    [S / Down Arrow]  : Drive Reverse\n');
fprintf('    [A / Left Arrow]  : Steer Left\n');
fprintf('    [D / Right Arrow] : Steer Right\n');
fprintf('    [Q] / [E]         : Zero-Radius Pivot Left / Right in Place\n');
fprintf('    [Spacebar]        : Active Brake / Emergency Halt\n');
fprintf('    [C]               : Cycle Camera View (Iso -> Top -> Chase -> Dock)\n');
fprintf('    [R]               : Reset Robot Pose to Depot\n');
fprintf('===================================================================\n\n');

%% =========================================================================
%  LAYER 1: PHYSICS & ENVIRONMENT SPECIFICATIONS
%  =========================================================================
env.width   = 26.0;   % [m] Length
env.depth   = 16.0;   % [m] Width
env.height  = 5.0;    % [m] Clearance

% Surface Friction & Contact Physics
mu_floor    = 0.68;   % Polished industrial epoxy floor friction
mu_ramp     = 0.78;   % High-grip diamond-tread metal ramp friction
g_accel     = 9.81;   %#ok<NASGU> Gravity [m/s^2]

% 3D Loading Dock Ramp Geometry: [xmin, ymin, width, depth, height_start, height_end]
env.ramp = [21.5, 12.0, 3.5, 3.0, 0.0, 0.45];
rampGrade = atan2(env.ramp(6) - env.ramp(5), env.ramp(3)); % Incline slope [rad]

% Industrial Multi-Tier Storage Racks: [xmin, ymin, width, depth, height, tiers]
env.racks = [
    % Row A (West Aisle)
     3.5,  2.2,  7.0,  2.0,  3.6,  3;
     3.5,  6.2,  7.0,  2.0,  3.6,  3;
     3.5, 10.2,  7.0,  2.0,  3.6,  3;
    % Row B (East Aisle)
    15.5,  2.2,  7.0,  2.0,  3.6,  3;
    15.5,  6.2,  7.0,  2.0,  3.6,  3;
    15.5, 10.2,  7.0,  2.0,  3.6,  3
];

% Static Pallet Staging / Obstacle Blocks: [xmin, ymin, width, depth, height]
env.pallets = [
    12.2,  6.5,  1.6,  1.4,  1.2;
    12.2,  8.5,  1.6,  1.4,  1.2;
     1.5,  8.0,  1.4,  1.8,  1.0;
    23.0,  8.0,  1.4,  1.8,  1.0
];

% Dynamic Obstacles & Hazards: [xmin, ymin, width, depth, height]
dynamicObstacles = [
     1.5,  7.5,  0.8,  0.8,  1.1;   % Crate in West Corridor
    12.8, 11.0,  0.8,  0.8,  1.0    % Pallet in Central Cross-Aisle
];

% Functional Zones
env.zones = struct(...
    'depot',   [ 1.8,  2.0, 0.0], ...
    'dock',    [24.0, 13.5, 0.45], ... % Elevated Dock Station
    'charge1', [13.0,  1.5, 0.0], ...
    'charge2', [13.0, 14.5, 0.0]  ...
);

% Outer Perimeter Wall Segments
outerWalls = [
     0.0,       0.0,       env.width, 0.0;
     env.width, 0.0,       env.width, env.depth;
     env.width, env.depth, 0.0,       env.depth;
     0.0,       env.depth, 0.0,       0.0
];

% Build Complete Segment Collision Database
segments = outerWalls;
for i = 1:size(env.racks, 1)
    rx = env.racks(i,1); ry = env.racks(i,2); rw = env.racks(i,3); rd = env.racks(i,4);
    segments = [segments; ...
        rx, ry, rx+rw, ry; rx+rw, ry, rx+rw, ry+rd; ...
        rx+rw, ry+rd, rx, ry+rd; rx, ry+rd, rx, ry]; %#ok<AGROW>
end
for i = 1:size(env.pallets, 1)
    px = env.pallets(i,1); py = env.pallets(i,2); pw = env.pallets(i,3); pd = env.pallets(i,4);
    segments = [segments; ...
        px, py, px+pw, py; px+pw, py, px+pw, py+pd; ...
        px+pw, py+pd, px, py+pd; px, py+pd, px, py]; %#ok<AGROW>
end
for i = 1:size(dynamicObstacles, 1)
    dx = dynamicObstacles(i,1); dy = dynamicObstacles(i,2); dw = dynamicObstacles(i,3); dd = dynamicObstacles(i,4);
    segments = [segments; ...
        dx, dy, dx+dw, dy; dx+dw, dy, dx+dw, dy+dd; ...
        dx+dw, dy+dd, dx, dy+dd; dx, dy+dd, dx, dy]; %#ok<AGROW>
end

%% =========================================================================
%  LAYER 2: AGILEX SCOUT MINI PLATFORM DYNAMICS & KINEMATICS
%  =========================================================================
robotMass        = 30.0;     % [kg] Scout Mini base curb mass
payloadMass      = 12.0;     % [kg] Industrial payload
effectiveMass    = robotMass + payloadMass; %#ok<NASGU> % 42 kg
robotL           = 0.620;    % [m] Chassis length
robotW           = 0.550;    % [m] Chassis width
wheelR           = 0.080;    % [m] Wheel radius
trackW           = 0.612;    % [m] Wheelbase / Track width
speedPreset      = 0.80;     % [m/s] Active maximum speed preset
max_v            = speedPreset;
max_w            = 1.40;     % [rad/s] Angular rotation speed limit
accel_v          = 1.20;     % [m/s^2] Linear acceleration limit
decel_brake      = 2.20;     % [m/s^2] ISO dynamic braking limit
accel_w          = 2.80;     % [rad/s^2] Angular acceleration limit
slipFactor       = 1.18;     % Skid-steer lateral scrub resistance coefficient
max_motor_torque = 17.5;     %#ok<NASGU> % [N*m] Max brushless hub motor torque
pitch_angle      = 0.0;      % Chassis incline pitch angle [rad]

%% =========================================================================
%  LAYER 3: SENSORS & PERCEPTION STREAM
%  =========================================================================
lidarRange       = 8.0;      % [m] Useful detection range
numBeams         = 64;       % Planar LiDAR scan resolution
lidarAngles      = linspace(-pi, pi, numBeams);
lidarNoiseSigma  = 0.015;    % [m] Gaussian range noise
dropoutRate      = 0.01;     % 1% specular reflectance dropouts
lidarZ           = 0.28;     % [m] Mounting height above chassis base

% Synthetic 6-DOF IMU
imu_gyro         = [0; 0; 0]; %#ok<NASGU>
imu_gyro_bias   = 0.0025;
imu_noise_sigma = 0.012;

% Synthetic Wheel Encoder Odometry with Slip Drift
odom_pose       = [1.8; 2.0; pi/2];
odom_drift_rate = 0.020;

% ISO 3691-4 Safety Zone Radii
iso_warning_dist = 1.80;   % Warning field: progressive speed derating
iso_brake_dist   = 1.05;   %#ok<NASGU> % Braking field: alert & slowdown
iso_stop_dist    = 0.65;   % Protective stop field: mandatory forward motion halt

% Interactive Teleop State Variables
keyState = struct('up', false, 'down', false, 'left', false, 'right', false, 'pivotL', false, 'pivotR', false);
pose            = [1.8; 2.0; pi/2];
v               = 0.0;
w               = 0.0;
v_target        = 0.0;
w_target        = 0.0;
slipRatio       = 0.0; %#ok<NASGU>
safetyMode      = 'CLEAR'; %#ok<NASGU> % 'CLEAR', 'WARNING', 'ISO_ESTOP', 'MANUAL_ESTOP'
isEStopActive   = false;
isSafetyOverrideActive = true; % Toggleable ISO 3691-4 safety guard
dt              = 0.06;
t_sim           = 0.0;
running         = true;

%% =========================================================================
%  CLEAN INDUSTRIAL WORKSTATION GUI WITH ON-SCREEN D-PAD
%  =========================================================================
fig = figure('Name', 'AgileX Scout Mini - Interactive Teleop & ISO 3691-4 Safety Workstation', ...
             'NumberTitle', 'off', ...
             'Color', [0.93 0.94 0.96], ...
             'Position', [40, 30, 1480, 840], ...
             'KeyPressFcn', @onKeyDown, ...
             'KeyReleaseFcn', @onKeyUp, ...
             'CloseRequestFcn', @onClose);

% Top Real-Time Industrial Status Banner
hBanner = uicontrol('Parent', fig, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.015, 0.940, 0.970, 0.046], ...
    'BackgroundColor', [0.20 0.23 0.28], 'ForegroundColor', [0.95 0.96 0.98], ...
    'FontName', 'Segoe UI', 'FontSize', 10.5, 'FontWeight', 'bold', ...
    'String', 'AgileX Scout Mini Teleop | Ready | Use W/A/S/D or On-Screen Controls');

% Top Navigation & Camera Controls Toolbar
camState = struct('mode', 'iso');

uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Isometric', ...
    'Units', 'normalized', 'Position', [0.015, 0.892, 0.075, 0.034], ...
    'BackgroundColor', [0.32 0.36 0.42], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) setCamMode('iso'));

uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Top View', ...
    'Units', 'normalized', 'Position', [0.095, 0.892, 0.075, 0.034], ...
    'BackgroundColor', [0.32 0.36 0.42], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) setCamMode('top'));

uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Chase Cam', ...
    'Units', 'normalized', 'Position', [0.175, 0.892, 0.075, 0.034], ...
    'BackgroundColor', [0.32 0.36 0.42], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) setCamMode('chase'));

uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Dock Cam', ...
    'Units', 'normalized', 'Position', [0.255, 0.892, 0.075, 0.034], ...
    'BackgroundColor', [0.32 0.36 0.42], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) setCamMode('dock'));

% Speed Throttle Preset Toggle
uicontrol('Parent', fig, 'Style', 'text', 'String', 'Max Speed:', ...
    'Units', 'normalized', 'Position', [0.340, 0.892, 0.060, 0.030], ...
    'BackgroundColor', [0.93 0.94 0.96], 'ForegroundColor', [0.2 0.2 0.2], 'FontWeight', 'bold');

hBtnSpeedLow = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '0.4 m/s', ...
    'Units', 'normalized', 'Position', [0.405, 0.892, 0.055, 0.034], ...
    'BackgroundColor', [0.75 0.78 0.82], 'ForegroundColor', [0.1 0.1 0.1], ...
    'Callback', @(~,~) setSpeedLimit(0.40));

hBtnSpeedMed = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '0.8 m/s', ...
    'Units', 'normalized', 'Position', [0.465, 0.892, 0.055, 0.034], ...
    'BackgroundColor', [0.25 0.48 0.72], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) setSpeedLimit(0.80));

hBtnSpeedHigh = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '1.2 m/s', ...
    'Units', 'normalized', 'Position', [0.525, 0.892, 0.055, 0.034], ...
    'BackgroundColor', [0.75 0.78 0.82], 'ForegroundColor', [0.1 0.1 0.1], ...
    'Callback', @(~,~) setSpeedLimit(1.20));

% Safety Interlock Toggle Button
hBtnSafetyToggle = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'ISO Safety: ACTIVE', ...
    'Units', 'normalized', 'Position', [0.590, 0.892, 0.105, 0.034], ...
    'BackgroundColor', [0.20 0.55 0.30], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) toggleSafetyInterlock());

% Reset to Depot Button
uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Reset to Depot', ...
    'Units', 'normalized', 'Position', [0.705, 0.892, 0.090, 0.034], ...
    'BackgroundColor', [0.42 0.45 0.50], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) resetRobotPose());

% Emergency Halt Button
hBtnEStop = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'E-STOP / BRAKE', ...
    'Units', 'normalized', 'Position', [0.805, 0.892, 0.105, 0.034], ...
    'BackgroundColor', [0.80 0.18 0.18], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) toggleEmergencyStop());

% -------------------------------------------------------------------------
% 3.1 Main 3D Industrial Viewport (Left 60% Panel)
% -------------------------------------------------------------------------
ax3D = axes('Parent', fig, 'Position', [0.015, 0.200, 0.585, 0.675]);
hold(ax3D, 'on'); grid(ax3D, 'on'); box(ax3D, 'on'); axis(ax3D, 'equal');
set(ax3D, 'Color', [0.96 0.97 0.98], ...
          'XColor', [0.35 0.38 0.42], 'YColor', [0.35 0.38 0.42], 'ZColor', [0.35 0.38 0.42], ...
          'GridColor', [0.80 0.82 0.85], 'GridAlpha', 0.8);
xlim(ax3D, [-1, env.width + 1]);
ylim(ax3D, [-1, env.depth + 1]);
zlim(ax3D, [-0.1, env.height]);
xlabel(ax3D, 'X [m]', 'FontWeight', 'bold', 'Color', [0.25 0.28 0.32]);
ylabel(ax3D, 'Y [m]', 'FontWeight', 'bold', 'Color', [0.25 0.28 0.32]);
zlabel(ax3D, 'Z [m]', 'FontWeight', 'bold', 'Color', [0.25 0.28 0.32]);
title(ax3D, '3D Warehouse Environment - Interactive Robot Teleop', ...
      'Color', [0.15 0.18 0.22], 'FontSize', 10.5, 'FontWeight', 'bold');

% Polished Industrial Concrete Floor
[fx, fy] = meshgrid(0:1:env.width, 0:1:env.depth);
surf(ax3D, fx, fy, zeros(size(fx)), 'FaceColor', [0.85 0.87 0.89], ...
     'EdgeColor', [0.78 0.80 0.83], 'FaceAlpha', 1.0);

% 3D Loading Dock Ramp & Elevated Platform
drawLoadingDockRamp(ax3D, env.ramp);

% Dedicated Transit Guidance Markings (RAL 1023 Traffic Yellow)
plot3(ax3D, [1.8, 1.8], [1.1, 14.0], [0.01 0.01], 'Color', [0.90 0.70 0.10], 'LineWidth', 1.4, 'LineStyle', '--');
plot3(ax3D, [24.0, 24.0], [1.1, 11.5], [0.01 0.01], 'Color', [0.90 0.70 0.10], 'LineWidth', 1.4, 'LineStyle', '--');
plot3(ax3D, [1.8, 24.0], [14.0, 14.0], [0.01 0.01], 'Color', [0.90 0.70 0.10], 'LineWidth', 1.4, 'LineStyle', '--');
plot3(ax3D, [1.8, 24.0], [1.1, 1.1], [0.01 0.01], 'Color', [0.90 0.70 0.10], 'LineWidth', 1.4, 'LineStyle', '--');
plot3(ax3D, [14.65, 14.65], [1.1, 14.0], [0.01 0.01], 'Color', [0.90 0.70 0.10], 'LineWidth', 1.4, 'LineStyle', '--');

% Safety Hazard Striping
drawHazardStripes(ax3D, 0.3, 0.5, 3.0, 3.0);
drawHazardStripes(ax3D, 21.0, 11.5, 3.5, 3.5);

% Storage Racks, Pallets & Dynamic Obstacles
for i = 1:size(env.racks, 1)
    r = env.racks(i, :);
    drawStorageRack3D(ax3D, r(1), r(2), r(3), r(4), r(5), r(6));
end
for i = 1:size(env.pallets, 1)
    p = env.pallets(i, :);
    drawPalletStack3D(ax3D, p(1), p(2), p(3), p(4), p(5));
end
for i = 1:size(dynamicObstacles, 1)
    d = dynamicObstacles(i, :);
    drawDynamicObstacle3D(ax3D, d(1), d(2), d(3), d(4), d(5), sprintf('Obstacle %d', i));
end

% Fast Charging Bays & Station Signs
drawChargingPad(ax3D, env.zones.charge1(1), env.zones.charge1(2), 'CHARGER 1');
drawChargingPad(ax3D, env.zones.charge2(1), env.zones.charge2(2), 'CHARGER 2');
text(ax3D, env.zones.depot(1), env.zones.depot(2), 0.5, 'INBOUND DEPOT', ...
     'Color', [0.15 0.50 0.20], 'FontWeight', 'bold', 'FontSize', 8.5, 'HorizontalAlignment', 'center');
text(ax3D, env.zones.dock(1), env.zones.dock(2), 0.9, 'SHIPPING DOCK (+0.45m)', ...
     'Color', [0.70 0.20 0.20], 'FontWeight', 'bold', 'FontSize', 8.5, 'HorizontalAlignment', 'center');

% Trajectory Trails: Ground Truth (Blue solid) vs Wheel Encoder Odometry (Gray dashed)
hTrailGT   = plot3(ax3D, pose(1), pose(2), 0.05, 'Color', [0.12 0.38 0.72], 'LineWidth', 2.0);
hTrailOdom = plot3(ax3D, odom_pose(1), odom_pose(2), 0.05, 'Color', [0.55 0.58 0.62], ...
                   'LineStyle', ':', 'LineWidth', 1.5);
trailGT_X = pose(1); trailGT_Y = pose(2); trailGT_Z = 0.05;
trailOd_X = odom_pose(1); trailOd_Y = odom_pose(2);

% ISO 3691-4 Floor Safety Footprint Circle
haloTheta = linspace(0, 2*pi, 40);
hSafetyHalo = plot3(ax3D, ...
    pose(1) + iso_stop_dist * cos(haloTheta), ...
    pose(2) + iso_stop_dist * sin(haloTheta), ...
    ones(size(haloTheta)) * 0.02, ...
    'LineStyle', '--', 'Color', [0.30 0.65 0.35], 'LineWidth', 1.6);

% Planar Laser Scan Point Cloud (ROS RViz Style)
hLidarHits3D = plot3(ax3D, nan, nan, nan, '.', 'Color', [0.85 0.20 0.15], 'MarkerSize', 7);

% AgileX Scout Mini 3D CAD Robot Instance
hRobotGroup = hgtransform('Parent', ax3D);
buildAgileXScoutMiniCAD(hRobotGroup, robotL, robotW, wheelR, trackW);
set(hRobotGroup, 'Matrix', makehgtform('translate', [pose(1), pose(2), 0], 'zrotate', pose(3)));

% Lighting & View
view(ax3D, [-35, 42]);
camlight(ax3D, 'headlight');
camlight(ax3D, 'right');
lighting(ax3D, 'gouraud');

% -------------------------------------------------------------------------
% 3.2 Bottom-Left: Interactive On-Screen Touch D-Pad Panel
% -------------------------------------------------------------------------
pnlDpad = uipanel('Parent', fig, 'Title', 'Interactive Teleop Control Pad (Click or use W/A/S/D / Arrows)', ...
    'Units', 'normalized', 'Position', [0.015, 0.030, 0.585, 0.155], ...
    'BackgroundColor', [0.88 0.90 0.93], 'ForegroundColor', [0.2 0.25 0.3], 'FontWeight', 'bold');

% D-Pad Buttons
uicontrol('Parent', pnlDpad, 'Style', 'pushbutton', 'String', '▲ FORWARD [W]', ...
    'Units', 'normalized', 'Position', [0.38, 0.53, 0.24, 0.40], ...
    'BackgroundColor', [0.22 0.45 0.70], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) applyCommand('forward'));

uicontrol('Parent', pnlDpad, 'Style', 'pushbutton', 'String', '▼ REVERSE [S]', ...
    'Units', 'normalized', 'Position', [0.38, 0.07, 0.24, 0.40], ...
    'BackgroundColor', [0.22 0.45 0.70], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) applyCommand('reverse'));

uicontrol('Parent', pnlDpad, 'Style', 'pushbutton', 'String', '◀ LEFT [A]', ...
    'Units', 'normalized', 'Position', [0.15, 0.28, 0.21, 0.44], ...
    'BackgroundColor', [0.30 0.50 0.65], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) applyCommand('left'));

uicontrol('Parent', pnlDpad, 'Style', 'pushbutton', 'String', 'RIGHT [D] ▶', ...
    'Units', 'normalized', 'Position', [0.64, 0.28, 0.21, 0.44], ...
    'BackgroundColor', [0.30 0.50 0.65], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) applyCommand('right'));

uicontrol('Parent', pnlDpad, 'Style', 'pushbutton', 'String', '↺ PIVOT L [Q]', ...
    'Units', 'normalized', 'Position', [0.015, 0.28, 0.125, 0.44], ...
    'BackgroundColor', [0.45 0.40 0.60], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) applyCommand('pivotL'));

uicontrol('Parent', pnlDpad, 'Style', 'pushbutton', 'String', 'PIVOT R [E] ↻', ...
    'Units', 'normalized', 'Position', [0.860, 0.28, 0.125, 0.44], ...
    'BackgroundColor', [0.45 0.40 0.60], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) applyCommand('pivotR'));

uicontrol('Parent', pnlDpad, 'Style', 'pushbutton', 'String', '■ BRAKE', ...
    'Units', 'normalized', 'Position', [0.43, 0.30, 0.14, 0.36], ...
    'BackgroundColor', [0.75 0.20 0.20], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
    'Callback', @(~,~) applyCommand('stop'));

% -------------------------------------------------------------------------
% 3.3 Top-Right Panel: Synthetic Online LiDAR Occupancy Grid (SLAM)
% -------------------------------------------------------------------------
axOcc = axes('Parent', fig, 'Position', [0.635, 0.655, 0.345, 0.225]);
hold(axOcc, 'on'); box(axOcc, 'on');
set(axOcc, 'Color', 'w', 'XColor', [0.35 0.38 0.42], 'YColor', [0.35 0.38 0.42]);
title(axOcc, 'Perception: 2D Occupancy Grid Map', ...
      'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.15 0.18 0.22]);
xlabel(axOcc, 'x [m]', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
ylabel(axOcc, 'y [m]', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
axis(axOcc, 'equal');
xlim(axOcc, [0, env.width]); ylim(axOcc, [0, env.depth]);

gridRes = 0.25;
gridX = 0 : gridRes : env.width;
gridY = 0 : gridRes : env.depth;
occGrid = ones(length(gridY), length(gridX));

hOccImg = imagesc(axOcc, [0 env.width], [0 env.depth], occGrid);
colormap(axOcc, 'gray'); clim(axOcc, [0 1]);
set(axOcc, 'YDir', 'normal');
hOccRobot = plot(axOcc, pose(1), pose(2), 'ro', 'MarkerFaceColor', [0.85 0.20 0.15], 'MarkerSize', 5);

% -------------------------------------------------------------------------
% 3.4 Middle-Right Panel: Kinematics & Contact Dynamics
% -------------------------------------------------------------------------
axKin = axes('Parent', fig, 'Position', [0.635, 0.365, 0.345, 0.225]);
hold(axKin, 'on'); box(axKin, 'on'); grid(axKin, 'on');
set(axKin, 'Color', 'w', ...
           'XColor', [0.35 0.38 0.42], 'YColor', [0.35 0.38 0.42], ...
           'GridColor', [0.85 0.88 0.90], 'GridAlpha', 0.8);
title(axKin, 'Kinematics: Velocity, Motor RPM & Wheel Slip', ...
      'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.15 0.18 0.22]);
ylabel(axKin, 'v [m/s] | w [rad/s] | RPM/100', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
xlim(axKin, [0, 45]); ylim(axKin, [-1.8, 3.2]);

hLineV   = plot(axKin, 0, 0, 'Color', [0.12 0.44 0.72], 'LineWidth', 1.8, 'DisplayName', 'v [m/s]');
hLineW   = plot(axKin, 0, 0, 'Color', [0.85 0.50 0.10], 'LineWidth', 1.6, 'DisplayName', 'w [rad/s]');
hLineRPM = plot(axKin, 0, 0, 'Color', [0.15 0.60 0.30], 'LineWidth', 1.4, 'DisplayName', 'FL RPM/100');
hLineSlp = plot(axKin, 0, 0, 'Color', [0.55 0.25 0.65], 'LineWidth', 1.3, 'DisplayName', 'Slip Ratio s');
legKin = legend(axKin, 'Location', 'northeast', 'FontSize', 7.5);
set(legKin, 'TextColor', [0.2 0.2 0.2], 'Color', [0.98 0.98 0.98], 'EdgeColor', [0.8 0.8 0.8]);

% -------------------------------------------------------------------------
% 3.5 Bottom-Right Panel: Perception & Safety Clearance
% -------------------------------------------------------------------------
axSens = axes('Parent', fig, 'Position', [0.635, 0.050, 0.345, 0.240]);
hold(axSens, 'on'); box(axSens, 'on'); grid(axSens, 'on');
set(axSens, 'Color', 'w', ...
            'XColor', [0.35 0.38 0.42], 'YColor', [0.35 0.38 0.42], ...
            'GridColor', [0.85 0.88 0.90], 'GridAlpha', 0.8);
title(axSens, 'Safety Clearance & Sensor Telemetry', ...
      'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.15 0.18 0.22]);
xlabel(axSens, 'Time [s]', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
ylabel(axSens, 'Clearance [m] | Drift [m]', 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
xlim(axSens, [0, 45]); ylim(axSens, [0, 3.5]);

hLineClr  = plot(axSens, 0, 0, 'Color', [0.10 0.55 0.55], 'LineWidth', 1.8, 'DisplayName', 'Clearance [m]');
hLineDrf  = plot(axSens, 0, 0, 'Color', [0.75 0.20 0.20], 'LineWidth', 1.4, 'DisplayName', 'Odom Drift [m]');
hLineIMU  = plot(axSens, 0, 0, 'Color', [0.40 0.40 0.80], 'LineWidth', 1.2, 'DisplayName', 'IMU Gyro Yaw');
yline(axSens, iso_stop_dist, '--r', 'LineWidth', 1.4, 'DisplayName', 'ISO Stop Limit (0.65m)');
yline(axSens, iso_warning_dist, ':k', 'LineWidth', 1.2, 'DisplayName', 'Warning Field (1.80m)');
legSens = legend(axSens, 'Location', 'northeast', 'FontSize', 7.5);
set(legSens, 'TextColor', [0.2 0.2 0.2], 'Color', [0.98 0.98 0.98], 'EdgeColor', [0.8 0.8 0.8]);

% Data History Arrays
tHist = 0; vHist = 0; wHist = 0; rpmHist = 0; slpHist = 0;
clrHist = 0; driftHist = 0; imuHist = 0;

%% =========================================================================
%  REAL-TIME TELEOPERATION SIMULATION LOOP
%  =========================================================================
fprintf('▶ Teleoperation Loop Active (dt = %.2fs)\n', dt);

while running && ishandle(fig)
    tLoop = tic;
    
    % ---------------------------------------------------------------------
    % Surface Friction & Incline Grade Transition
    % ---------------------------------------------------------------------
    inRampZone = (pose(1) >= env.ramp(1)) && (pose(1) <= env.ramp(1) + env.ramp(3)) && ...
                 (pose(2) >= env.ramp(2)) && (pose(2) <= env.ramp(2) + env.ramp(4));
    inDockZone = (pose(1) > env.ramp(1) + env.ramp(3)) && ...
                 (pose(2) >= env.ramp(2)) && (pose(2) <= env.ramp(2) + env.ramp(4));
             
    if inRampZone
        currentMu = mu_ramp;
        targetPitch = rampGrade;
        rampProgress = (pose(1) - env.ramp(1)) / env.ramp(3);
        currentZ = env.ramp(5) + rampProgress * (env.ramp(6) - env.ramp(5));
    elseif inDockZone
        currentMu = mu_floor;
        targetPitch = 0.0;
        currentZ = env.ramp(6);
    else
        currentMu = mu_floor;
        targetPitch = 0.0;
        currentZ = 0.0;
    end
    
    pitch_angle = pitch_angle + 0.35 * (targetPitch - pitch_angle);
    
    % ---------------------------------------------------------------------
    % LAYER 3: Synthetic 2D/3D LiDAR Raycasting
    % ---------------------------------------------------------------------
    hitPoints = zeros(numBeams, 2);
    beamDistances = zeros(numBeams, 1);
    effectiveNoise = lidarNoiseSigma * (1.0 + 0.3 * rand());
    
    for b = 1:numBeams
        ang = pose(3) + lidarAngles(b);
        p1 = [pose(1), pose(2)];
        p2 = [pose(1) + lidarRange * cos(ang), pose(2) + lidarRange * sin(ang)];
        
        minD = lidarRange;
        for s = 1:size(segments, 1)
            [hit, d, ~] = lineSegIntersect(p1, p2, segments(s, 1:2), segments(s, 3:4));
            if hit && d < minD
                minD = d;
            end
        end
        
        noisyD = minD + randn() * effectiveNoise;
        noisyD = max(0.05, min(lidarRange, noisyD));
        
        if rand() < dropoutRate
            noisyD = lidarRange;
            bestHit = [pose(1) + lidarRange * cos(ang), pose(2) + lidarRange * sin(ang)];
        else
            bestHit = [pose(1) + noisyD * cos(ang), pose(2) + noisyD * sin(ang)];
        end
        
        hitPoints(b, :) = bestHit;
        beamDistances(b) = noisyD;
        
        % Online LiDAR Occupancy Grid Mapping
        if noisyD < lidarRange - 0.10
            gx = round(bestHit(1) / gridRes) + 1;
            gy = round(bestHit(2) / gridRes) + 1;
            if gx >= 1 && gx <= size(occGrid, 2) && gy >= 1 && gy <= size(occGrid, 1)
                occGrid(max(1, gy-1):min(size(occGrid,1), gy+1), ...
                        max(1, gx-1):min(size(occGrid,2), gx+1)) = 0.05;
            end
        end
    end
    
    % Minimum clearance to closest physical obstacle
    minClearance = max(0.0, min(beamDistances) - max(robotL, robotW)/2);
    
    % Longitudinal Safety Corridor Analysis (Frontal Cone):
    x_local = beamDistances .* cos(lidarAngles);
    y_local = beamDistances .* sin(lidarAngles);
    safetyEnvelopeWidth = (robotW / 2) + 0.10;
    inDrivingPath = (x_local > 0.08) & (abs(y_local) <= safetyEnvelopeWidth) & (abs(lidarAngles) <= deg2rad(35));
    if any(inDrivingPath)
        minFrontDist = min(x_local(inDrivingPath));
    else
        minFrontDist = lidarRange;
    end
    
    % ---------------------------------------------------------------------
    % TELEOPERATION INPUT ARBITRATION & ISO 3691-4 SAFETY SUPERVISOR
    % ---------------------------------------------------------------------
    % Evaluate keyboard holding states if active
    v_desired = 0.0;
    w_desired = 0.0;
    
    if keyState.up
        v_desired = max_v;
    elseif keyState.down
        v_desired = -0.50 * max_v; % Safe reverse speed
    end
    
    if keyState.left
        w_desired = max_w;
    elseif keyState.right
        w_desired = -max_w;
    end
    
    if keyState.pivotL
        v_desired = 0.0;
        w_desired = max_w;
    elseif keyState.pivotR
        v_desired = 0.0;
        w_desired = -max_w;
    end
    
    % Blend with on-screen button command target if keyboard is idle
    if ~keyState.up && ~keyState.down && ~keyState.left && ~keyState.right && ~keyState.pivotL && ~keyState.pivotR
        v_desired = v_target;
        w_desired = w_target;
    end
    
    % ISO 3691-4 Supervisory Interlock Check:
    if isEStopActive
        safetyMode = 'MANUAL_ESTOP';
        v_cmd = 0.0;
        w_cmd = 0.0;
    elseif isSafetyOverrideActive && minFrontDist <= (iso_stop_dist + 0.05) && v_desired > 0
        % Forward obstacle detected! Block forward driving but allow reverse and pivot
        safetyMode = 'ISO_ESTOP';
        v_cmd = 0.0; % Forward motion cut
        w_cmd = w_desired; % Allow rotation in place to steer away
    elseif isSafetyOverrideActive && minFrontDist <= iso_warning_dist && v_desired > 0
        % Warning zone: progressive speed derating
        safetyMode = 'WARNING';
        derateRatio = (minFrontDist - iso_stop_dist) / (iso_warning_dist - iso_stop_dist);
        v_cmd = v_desired * max(0.20, min(1.0, derateRatio));
        w_cmd = w_desired;
    else
        safetyMode = 'CLEAR';
        v_cmd = v_desired;
        w_cmd = w_desired;
    end
    
    % Smooth Acceleration & Deceleration Limiter
    if v_cmd < v
        dv = max(-decel_brake * dt, v_cmd - v);
    else
        dv = min(accel_v * dt, v_cmd - v);
    end
    dw = max(-accel_w * dt, min(accel_w * dt, w_cmd - w));
    
    v = v + dv;
    w = max(-max_w, min(max_w, w + dw));
    
    % Gradual decay when commands stop
    if abs(v_desired) < 1e-3
        v = v * 0.85;
        if abs(v) < 0.02, v = 0.0; end
    end
    if abs(w_desired) < 1e-3
        w = w * 0.80;
        if abs(w) < 0.02, w = 0.0; end
    end
    
    % ---------------------------------------------------------------------
    % 4WD Skid-Steer Kinematics, Wheel RPMs & Slip Ratio
    % ---------------------------------------------------------------------
    w_L = (v - 0.5 * slipFactor * trackW * w) / wheelR;
    w_R = (v + 0.5 * slipFactor * trackW * w) / wheelR;
    rpm_FL = w_L * (60 / (2 * pi));
    
    theoreticalSpeed = 0.5 * (w_L + w_R) * wheelR;
    slipRatio = abs(theoreticalSpeed - v) / max(0.1, max(abs(v), abs(theoreticalSpeed)));
    
    % Strict Minkowski Zero-Collision Physical Shield
    candidatePose = [
        pose(1) + v * cos(pose(3)) * dt;
        pose(2) + v * sin(pose(3)) * dt;
        angdiff_r(0, pose(3) + w * dt)
    ];
    
    if checkScoutCollision(candidatePose, robotL, robotW, segments)
        v = 0.0;
        candidatePose(1) = pose(1);
        candidatePose(2) = pose(2);
        candidatePose(3) = angdiff_r(0, pose(3) + w * dt);
    end
    pose = candidatePose;
    t_sim = t_sim + dt;
    
    % ---------------------------------------------------------------------
    % Synthetic IMU & Wheel Encoder Odometry Drift Integration
    % ---------------------------------------------------------------------
    imu_gyro = [
        randn() * imu_noise_sigma;
        (targetPitch - pitch_angle)/dt + randn() * imu_noise_sigma;
        w + imu_gyro_bias + randn() * imu_noise_sigma
    ];
    
    v_odom = v * (1.0 - odom_drift_rate * (1.0 - currentMu));
    w_odom = w * (1.0 + odom_drift_rate * 0.5);
    odom_pose(1) = odom_pose(1) + v_odom * cos(odom_pose(3)) * dt;
    odom_pose(2) = odom_pose(2) + v_odom * sin(odom_pose(3)) * dt;
    odom_pose(3) = angdiff_r(0, odom_pose(3) + w_odom * dt);
    
    driftError = hypot(pose(1) - odom_pose(1), pose(2) - odom_pose(2));
    
    % Telemetry History Buffer
    tHist(end+1)     = t_sim; %#ok<AGROW>
    vHist(end+1)     = v;     %#ok<AGROW>
    wHist(end+1)     = w;     %#ok<AGROW>
    rpmHist(end+1)   = rpm_FL / 100.0; %#ok<AGROW>
    slpHist(end+1)   = slipRatio; %#ok<AGROW>
    clrHist(end+1)   = minClearance; %#ok<AGROW>
    driftHist(end+1) = driftError; %#ok<AGROW>
    imuHist(end+1)   = imu_gyro(3); %#ok<AGROW>
    
    trailGT_X(end+1) = pose(1); trailGT_Y(end+1) = pose(2); trailGT_Z(end+1) = currentZ + 0.05; %#ok<AGROW>
    trailOd_X(end+1) = odom_pose(1); trailOd_Y(end+1) = odom_pose(2); %#ok<AGROW>
    
    % ---------------------------------------------------------------------
    % VISUAL UPDATES
    % ---------------------------------------------------------------------
    if ~ishandle(fig), break; end
    
    % Update 3D Robot Pose with Dynamic Suspension Pitch Tilt & Z-Elevation
    set(hRobotGroup, 'Matrix', ...
        makehgtform('translate', [pose(1), pose(2), currentZ], ...
                    'zrotate', pose(3), ...
                    'yrotate', pitch_angle));
    
    % Update Trails
    set(hTrailGT,   'XData', trailGT_X, 'YData', trailGT_Y, 'ZData', trailGT_Z);
    set(hTrailOdom, 'XData', trailOd_X, 'YData', trailOd_Y, 'ZData', ones(size(trailOd_X))*(currentZ + 0.04));
    
    % Update ISO Safety Footprint
    set(hSafetyHalo, 'XData', pose(1) + iso_stop_dist * cos(haloTheta), ...
                     'YData', pose(2) + iso_stop_dist * sin(haloTheta), ...
                     'ZData', ones(size(haloTheta)) * (currentZ + 0.02));
    switch safetyMode
        case {'ISO_ESTOP', 'MANUAL_ESTOP'}
            set(hSafetyHalo, 'Color', [0.85 0.20 0.15], 'LineWidth', 2.2);
        case 'WARNING'
            set(hSafetyHalo, 'Color', [0.90 0.60 0.10], 'LineWidth', 1.8);
        otherwise
            set(hSafetyHalo, 'Color', [0.30 0.65 0.35], 'LineWidth', 1.5);
    end
    
    % Update Planar Laser Scan Points
    set(hLidarHits3D, 'XData', hitPoints(:,1), 'YData', hitPoints(:,2), ...
                      'ZData', ones(numBeams,1)*(currentZ + lidarZ));
    
    % Dynamic Chase Camera
    if strcmp(camState.mode, 'chase')
        campos(ax3D, [pose(1) - 3.8*cos(pose(3)), pose(2) - 3.8*sin(pose(3)), currentZ + 3.2]);
        camtarget(ax3D, [pose(1), pose(2), currentZ + 0.5]);
    end
    
    % Occupancy Grid Update
    set(hOccImg, 'CData', occGrid);
    set(hOccRobot, 'XData', pose(1), 'YData', pose(2));
    
    % Telemetry Graphs Update
    set(hLineV,   'XData', tHist, 'YData', vHist);
    set(hLineW,   'XData', tHist, 'YData', wHist);
    set(hLineRPM, 'XData', tHist, 'YData', rpmHist);
    set(hLineSlp, 'XData', tHist, 'YData', slpHist);
    set(hLineClr, 'XData', tHist, 'YData', clrHist);
    set(hLineDrf, 'XData', tHist, 'YData', driftHist);
    set(hLineIMU, 'XData', tHist, 'YData', imuHist);
    
    if t_sim > 40
        xlim(axKin,  [0, t_sim + 5]);
        xlim(axSens, [0, t_sim + 5]);
    end
    
    % Top Industrial Status Banner
    switch safetyMode
        case 'MANUAL_ESTOP'
            set(hBanner, 'String', sprintf('AgileX Scout Mini | MANUAL EMERGENCY STOP ENGAGED | Click E-STOP or Press Space to Resume'), ...
                'ForegroundColor', [1.0 0.95 0.95], 'BackgroundColor', [0.75 0.15 0.15]);
        case 'ISO_ESTOP'
            set(hBanner, 'String', sprintf('AgileX Scout Mini | ISO 3691-4 FORWARD OVERRIDE (<0.65m) | Reverse or Pivot Allowed | Clearance: %4.2f m', ...
                minClearance), ...
                'ForegroundColor', [1.0 0.95 0.90], 'BackgroundColor', [0.70 0.25 0.15]);
        case 'WARNING'
            set(hBanner, 'String', sprintf('AgileX Scout Mini | ISO Warning Field Active (Speed Derated) | v = %+4.2f m/s | w = %+4.2f rad/s | Clearance: %4.2f m', ...
                v, w, minClearance), ...
                'ForegroundColor', [1.0 0.95 0.90], 'BackgroundColor', [0.65 0.40 0.12]);
        otherwise
            set(hBanner, 'String', sprintf('AgileX Scout Mini Teleop Active | v = %+4.2f m/s | w = %+4.2f rad/s | RPM: %+4.0f | Pitch: %+4.1f deg | Clearance: %4.2f m', ...
                v, w, rpm_FL, rad2deg(pitch_angle), minClearance), ...
                'ForegroundColor', [0.95 0.96 0.98], 'BackgroundColor', [0.20 0.23 0.28]);
    end
    
    drawnow;
    
    elapsed = toc(tLoop);
    if dt > elapsed
        pause(dt - elapsed);
    else
        pause(0.005);
    end
end

%% =========================================================================
%  NESTED INTERACTIVE CALLBACKS
%  =========================================================================
    function onKeyDown(~, event)
        switch lower(event.Key)
            case {'w', 'uparrow'}
                keyState.up = true;
            case {'s', 'downarrow'}
                keyState.down = true;
            case {'a', 'leftarrow'}
                keyState.left = true;
            case {'d', 'rightarrow'}
                keyState.right = true;
            case 'q'
                keyState.pivotL = true;
            case 'e'
                keyState.pivotR = true;
            case 'space'
                toggleEmergencyStop();
            case 'c'
                cycleCamera();
            case 'r'
                resetRobotPose();
        end
    end

    function onKeyUp(~, event)
        switch lower(event.Key)
            case {'w', 'uparrow'}
                keyState.up = false;
            case {'s', 'downarrow'}
                keyState.down = false;
            case {'a', 'leftarrow'}
                keyState.left = false;
            case {'d', 'rightarrow'}
                keyState.right = false;
            case 'q'
                keyState.pivotL = false;
            case 'e'
                keyState.pivotR = false;
        end
    end

    function applyCommand(cmd)
        switch cmd
            case 'forward'
                v_target = max_v;
                w_target = 0.0;
            case 'reverse'
                v_target = -0.50 * max_v;
                w_target = 0.0;
            case 'left'
                w_target = max_w;
            case 'right'
                w_target = -max_w;
            case 'pivotL'
                v_target = 0.0;
                w_target = max_w;
            case 'pivotR'
                v_target = 0.0;
                w_target = -max_w;
            case 'stop'
                v_target = 0.0;
                w_target = 0.0;
                v = 0.0;
                w = 0.0;
        end
    end

    function setSpeedLimit(newLimit)
        speedPreset = newLimit;
        max_v = newLimit;
        set(hBtnSpeedLow,  'BackgroundColor', [0.75 0.78 0.82], 'ForegroundColor', [0.1 0.1 0.1], 'FontWeight', 'normal');
        set(hBtnSpeedMed,  'BackgroundColor', [0.75 0.78 0.82], 'ForegroundColor', [0.1 0.1 0.1], 'FontWeight', 'normal');
        set(hBtnSpeedHigh, 'BackgroundColor', [0.75 0.78 0.82], 'ForegroundColor', [0.1 0.1 0.1], 'FontWeight', 'normal');
        if abs(newLimit - 0.40) < 1e-3
            set(hBtnSpeedLow,  'BackgroundColor', [0.25 0.48 0.72], 'ForegroundColor', 'w', 'FontWeight', 'bold');
        elseif abs(newLimit - 0.80) < 1e-3
            set(hBtnSpeedMed,  'BackgroundColor', [0.25 0.48 0.72], 'ForegroundColor', 'w', 'FontWeight', 'bold');
        else
            set(hBtnSpeedHigh, 'BackgroundColor', [0.25 0.48 0.72], 'ForegroundColor', 'w', 'FontWeight', 'bold');
        end
        fprintf('Speed preset changed to %.2f m/s\n', max_v);
    end

    function toggleSafetyInterlock()
        isSafetyOverrideActive = ~isSafetyOverrideActive;
        if isSafetyOverrideActive
            set(hBtnSafetyToggle, 'BackgroundColor', [0.20 0.55 0.30], 'String', 'ISO Safety: ACTIVE');
            fprintf('ISO 3691-4 Safety Interlock ENGAGED.\n');
        else
            set(hBtnSafetyToggle, 'BackgroundColor', [0.75 0.45 0.15], 'String', 'ISO Safety: BYPASSED');
            fprintf('ISO 3691-4 Safety Interlock BYPASSED (Manual Caution).\n');
        end
    end

    function toggleEmergencyStop()
        isEStopActive = ~isEStopActive;
        if isEStopActive
            v_target = 0.0; w_target = 0.0; v = 0.0; w = 0.0;
            keyState.up = false; keyState.down = false; keyState.left = false; keyState.right = false;
            set(hBtnEStop, 'BackgroundColor', [0.85 0.50 0.15], 'String', 'RESUME');
            fprintf('EMERGENCY BRAKE ENGAGED.\n');
        else
            set(hBtnEStop, 'BackgroundColor', [0.80 0.18 0.18], 'String', 'E-STOP / BRAKE');
            fprintf('Emergency Stop Disengaged.\n');
        end
    end

    function resetRobotPose()
        pose = [1.8; 2.0; pi/2];
        odom_pose = [1.8; 2.0; pi/2];
        v = 0.0; w = 0.0; v_target = 0.0; w_target = 0.0;
        trailGT_X = pose(1); trailGT_Y = pose(2); trailGT_Z = 0.05;
        trailOd_X = odom_pose(1); trailOd_Y = odom_pose(2);
        fprintf('Robot reset to Inbound Depot [1.8, 2.0].\n');
    end

    function setCamMode(newMode)
        camState.mode = newMode;
        switch newMode
            case 'iso'
                view(ax3D, [-35, 42]);
                campos(ax3D, 'auto');
                camtarget(ax3D, [env.width/2, env.depth/2, 1.5]);
            case 'top'
                view(ax3D, [0, 90]);
                campos(ax3D, 'auto');
                camtarget(ax3D, [env.width/2, env.depth/2, 0]);
            case 'chase'
                % Updated dynamically in simulation loop
            case 'dock'
                view(ax3D, [-22, 25]);
                campos(ax3D, [14, 6, 6]);
                camtarget(ax3D, [23.5, 13.5, 0.45]);
        end
        drawnow;
    end

    function cycleCamera()
        modes = {'iso', 'top', 'chase', 'dock'};
        currIdx = find(strcmp(camState.mode, modes));
        if isempty(currIdx), currIdx = 1; end
        nextIdx = mod(currIdx, length(modes)) + 1;
        setCamMode(modes{nextIdx});
    end

    function onClose(~, ~)
        running = false;
        delete(fig);
        fprintf('Teleop workstation closed.\n');
    end

end

%% =========================================================================
%  3D GEOMETRIC PRIMITIVES & CAD BUILDERS
%  =========================================================================
function drawLoadingDockRamp(ax, r)
    x1 = r(1); y1 = r(2); w = r(3); d = r(4); z1 = r(5); z2 = r(6);
    rampVerts = [
        x1,   y1,   z1;
        x1+w, y1,   z2;
        x1+w, y1+d, z2;
        x1,   y1+d, z1;
        x1,   y1,   0;
        x1+w, y1,   0;
        x1+w, y1+d, 0;
        x1,   y1+d, 0
    ];
    rampFaces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Parent', ax, 'Vertices', rampVerts, 'Faces', rampFaces, ...
          'FaceColor', [0.55 0.58 0.62], 'EdgeColor', [0.45 0.48 0.52], ...
          'LineWidth', 1.0, 'FaceAlpha', 0.95);
    curbW = 0.15;
    drawBox(ax, x1, y1 - curbW, 0, w, curbW, z2 + 0.1, [0.92 0.72 0.10], 1.0);
    drawBox(ax, x1, y1 + d,     0, w, curbW, z2 + 0.1, [0.92 0.72 0.10], 1.0);
    drawBox(ax, x1 + w, y1, 0, 1.5, d, z2, [0.38 0.40 0.44], 1.0);
end

function buildAgileXScoutMiniCAD(parentGroup, rL, rW, wheelR, trackW)
    chassisH = 0.18;
    drawBox(parentGroup, -rL/2, -rW/2, wheelR - 0.02, rL, rW, chassisH, [0.18 0.19 0.22], 1.0);
    drawBox(parentGroup, -rL/2 + 0.04, -rW/2 + 0.04, wheelR + chassisH - 0.03, ...
            rL - 0.08, rW - 0.08, 0.06, [0.88 0.70 0.12], 1.0);
    drawBox(parentGroup, -rL/2 + 0.08, -rW/2 + 0.08, wheelR + chassisH + 0.03, ...
            rL - 0.16, rW - 0.16, 0.015, [0.12 0.12 0.14], 1.0);
    th = linspace(0, 2*pi, 24);
    lr = 0.075;
    lh = wheelR + chassisH + 0.045;
    fill3(parentGroup, lr*cos(th), lr*sin(th), ones(size(th))*lh, ...
          [0.15 0.16 0.18], 'EdgeColor', [0.30 0.35 0.40], 'LineWidth', 1.5);
    drawBox(parentGroup, rL/2 - 0.02,  rW/4 - 0.04, wheelR + 0.05, 0.02, 0.08, 0.03, [0.90 0.92 0.95], 1.0);
    drawBox(parentGroup, rL/2 - 0.02, -rW/4 - 0.04, wheelR + 0.05, 0.02, 0.08, 0.03, [0.90 0.92 0.95], 1.0);
    drawBox(parentGroup, -rL/2,  rW/4 - 0.04, wheelR + 0.05, 0.02, 0.08, 0.03, [0.85 0.20 0.15], 1.0);
    drawBox(parentGroup, -rL/2, -rW/4 - 0.04, wheelR + 0.05, 0.02, 0.08, 0.03, [0.85 0.20 0.15], 1.0);
    wheelW = 0.075;
    halfTrack = trackW / 2;
    wheelCenters = [
         rL/3.2,  halfTrack;
         rL/3.2, -halfTrack - wheelW;
        -rL/3.2,  halfTrack;
        -rL/3.2, -halfTrack - wheelW
    ];
    for w = 1:4
        drawBox(parentGroup, wheelCenters(w,1) - wheelR, wheelCenters(w,2), 0.0, ...
                2*wheelR, wheelW, 2*wheelR, [0.12 0.12 0.14], 1.0);
        drawBox(parentGroup, wheelCenters(w,1) - wheelR/2, wheelCenters(w,2) - 0.005, wheelR/2, ...
                wheelR, wheelW + 0.01, wheelR, [0.75 0.75 0.78], 1.0);
    end
end

function inCollision = checkScoutCollision(pose, rL, rW, segments)
    inCollision = false;
    margin = 0.08;
    halfL = rL/2 + margin;
    halfW = rW/2 + margin;
    R_rot = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
    corners = [-halfL, -halfW; halfL, -halfW; halfL, halfW; -halfL, halfW]';
    rot_c = R_rot * corners;
    polyX = rot_c(1,:) + pose(1);
    polyY = rot_c(2,:) + pose(2);
    for s = 1:size(segments, 1)
        sx1 = segments(s,1); sy1 = segments(s,2);
        sx2 = segments(s,3); sy2 = segments(s,4);
        if inpolygon(sx1, sy1, polyX, polyY) || inpolygon(sx2, sy2, polyX, polyY)
            inCollision = true;
            return;
        end
        for e = 1:4
            eNext = mod(e, 4) + 1;
            [hit, ~, ~] = lineSegIntersect([polyX(e), polyY(e)], [polyX(eNext), polyY(eNext)], ...
                                           [sx1, sy1], [sx2, sy2]);
            if hit
                inCollision = true;
                return;
            end
        end
    end
end

function drawStorageRack3D(ax, x, y, w, d, h, tiers)
    blueUpright = [0.18 0.35 0.60];
    orangeBeam  = [0.88 0.45 0.12];
    postW = 0.12;
    posts = [x, y; x+w-postW, y; x+w-postW, y+d-postW; x, y+d-postW];
    for p = 1:4
        drawBox(ax, posts(p,1), posts(p,2), 0, postW, postW, h, blueUpright, 1.0);
    end
    tierHeights = linspace(h/(tiers+1), h - 0.3, tiers);
    for t = 1:tiers
        zh = tierHeights(t);
        drawBox(ax, x, y, zh, w, 0.08, 0.12, orangeBeam, 1.0);
        drawBox(ax, x, y+d-0.08, zh, w, 0.08, 0.12, orangeBeam, 1.0);
        drawBox(ax, x, y, zh, 0.08, d, 0.08, blueUpright, 0.95);
        drawBox(ax, x+w-0.08, y, zh, 0.08, d, 0.08, blueUpright, 0.95);
        palletSpacing = (w - 0.3) / 3;
        for pl = 1:3
            px = x + 0.15 + (pl-1) * palletSpacing;
            pw = palletSpacing - 0.20;
            pd = d - 0.30;
            drawBox(ax, px, y + 0.15, zh + 0.12, pw, pd, 0.14, [0.76 0.62 0.45], 1.0);
            boxColor = [0.80 0.70 0.55] * (0.90 + 0.10*rand());
            drawBox(ax, px + 0.05, y + 0.20, zh + 0.26, pw - 0.10, pd - 0.10, 0.55, boxColor, 1.0);
        end
    end
end

function drawPalletStack3D(ax, x, y, w, d, h)
    drawBox(ax, x, y, 0, w, d, 0.15, [0.74 0.60 0.44], 1.0);
    drawBox(ax, x+0.1, y+0.1, 0.15, w-0.2, d-0.2, h, [0.45 0.55 0.65], 0.90);
end

function drawDynamicObstacle3D(ax, x, y, w, d, h, labelText)
    drawBox(ax, x, y, 0, w, d, h, [0.86 0.68 0.22], 0.95);
    text(ax, x + w/2, y + d/2, h + 0.30, labelText, ...
         'Color', [0.20 0.22 0.25], 'FontWeight', 'bold', 'FontSize', 8, ...
         'HorizontalAlignment', 'center', 'BackgroundColor', [0.95 0.95 0.95]);
end

function drawChargingPad(ax, x, y, labelText)
    pw = 1.4; pd = 1.4;
    patch(ax, [x-pw/2, x+pw/2, x+pw/2, x-pw/2], ...
              [y-pd/2, y-pd/2, y+pd/2, y+pd/2], [0.01 0.01 0.01 0.01], ...
              [0.35 0.65 0.40], 'FaceAlpha', 0.85, 'EdgeColor', [0.25 0.50 0.30], 'LineWidth', 1.5);
    text(ax, x, y - 0.85, 0.05, labelText, 'Color', [0.20 0.45 0.25], ...
         'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
end

function drawHazardStripes(ax, x, y, w, h)
    plot(ax, [x x+w x+w x x], [y y y+h y+h y], 'Color', [0.90 0.70 0.10], 'LineWidth', 2.0);
    numStripes = 6;
    for s = 1:numStripes
        sx = x + (s-1) * (w/numStripes);
        plot(ax, [sx, min(x+w, sx + 0.4)], [y, min(y+h, y + 0.4)], 'Color', [0.90 0.70 0.10], 'LineWidth', 1.5);
    end
end

function drawBox(ax, x, y, z, w, d, h, faceColor, alphaVal)
    vertices = [
        x,   y,   z;
        x+w, y,   z;
        x+w, y+d, z;
        x,   y+d, z;
        x,   y,   z+h;
        x+w, y,   z+h;
        x+w, y+d, z+h;
        x,   y+d, z+h
    ];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Parent', ax, 'Vertices', vertices, 'Faces', faces, ...
          'FaceColor', faceColor, 'EdgeColor', [0.25 0.28 0.32], ...
          'LineWidth', 0.8, 'FaceAlpha', alphaVal);
end

function [hit, d, pt] = lineSegIntersect(p1, p2, p3, p4)
    hit = false; d = inf; pt = p2;
    x1 = p1(1); y1 = p1(2); x2 = p2(1); y2 = p2(2);
    x3 = p3(1); y3 = p3(2); x4 = p4(1); y4 = p4(2);
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if abs(denom) < 1e-9, return; end
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    if t >= 0 && t <= 1 && u >= 0 && u <= 1
        hit = true;
        ix = x1 + t * (x2 - x1);
        iy = y1 + t * (y2 - y1);
        pt = [ix, iy];
        d = hypot(ix - x1, iy - y1);
    end
end

function d = angdiff_r(a, b)
    d = mod(b - a + pi, 2*pi) - pi;
end
