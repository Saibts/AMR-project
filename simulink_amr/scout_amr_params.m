function P = scout_amr_params()
%SCOUT_AMR_PARAMS  Shared constants for the Scout Mini Simulink/Simscape AMR simulation.
%  World frame: robot starts at the origin facing +X. All units SI.

% --- Robot / wheel -------------------------------------------------------
P.r        = 0.085;     % [m]   wheel radius (STL mesh radius)
P.hubOff   = 0.062;     % [m]   hub centre offset from wheel frame along local +X (URDF)
P.track    = 0.612;     % [m]   effective track width
P.yWheel   = [ 0.30  0.30 -0.30 -0.30];   % lateral wheel offsets [w1 FL, w2 RL, w3 FR, w4 RR]
P.tmax     = 17.5;      % [Nm]  in-hub motor peak torque
P.p_fb     = [-0.10185; 0.16662; 0.2269]; % base_link origin expressed in base_footprint (URDF)
P.rRobot   = 0.38;      % [m]   avoidance radius of the robot

% Direction in which positive joint rotation moves the robot forward (+1/-1), from the URDF
jr = [ 2.33386 0 0; 2.56241 0 0; -2.87166 0 3.14159; -1.29916 0 3.14159];
ax = [ 1 0 0; 1 0 0; -1 0 0; -1 0 0];
Rfb = rz(pi/2);
P.sgn = zeros(1,4);
for k = 1:4
    Rbw = rz(jr(k,3)) * ry(jr(k,2)) * rx(jr(k,1));
    aj  = Rfb * Rbw * ax(k,:)';
    P.sgn(k) = sign(aj(2));          % +Y rotation = forward rolling
end

% --- Tyre / ground contact ----------------------------------------------
P.kc  = 12000;          % [N/m]   vertical contact stiffness per wheel
P.cc  = 700;            % [N s/m] vertical contact damping per wheel
P.v0  = 0.03;           % [m/s]   Coulomb-friction smoothing velocity
P.mu0 = 0.90;           % dry concrete
% Low-friction patches [xmin xmax ymin ymax mu]  (spills / polished dock)
P.patches = [ -1.5  2.5 -1.5  1.5  0.15;     % oily loading-dock floor at the start (acceleration slip)
               9.0 11.0  2.7  5.7  0.30;     % water spill in aisle 2
              15.5 18.0  6.9  9.9  0.30 ];   % water spill in aisle 3

% --- Warehouse layout (world frame, robot starts at the origin facing +X) ----
%   aisle 1 (service lane)  y -1.5 ..  1.5      rack row A  y 1.5 .. 2.7
%   aisle 2                 y  2.7 ..  5.7      rack row B  y 5.7 .. 6.9
%   aisle 3                 y  6.9 ..  9.9      rack row C  y 9.9 .. 11.1
%   cross-aisles at x = 8.6..10.2 and 15.8..17.4;  open docks at x < 3 and x > 23
segX = [3.0 8.6; 10.2 15.8; 17.4 23.0];
rowY = [1.5 2.7; 5.7 6.9; 9.9 11.1];
P.racks = zeros(9,4); n = 0;
for r = 1:3, for s = 1:3, n = n + 1; P.racks(n,:) = [segX(s,:) rowY(r,:)]; end, end
P.walls = [ -2.5 28.5 -2.2 -2.0;  -2.5 28.5 11.8 12.0;  -2.5 -2.3 -2.2 12.0;  28.3 28.5 -2.2 12.0 ];
% Floor boxes the planner does NOT know about (found by LiDAR) [xmin xmax ymin ymax]
P.boxes = [ 6.1 6.9 3.5 4.3;        % box in aisle 2 (partial block -> avoid)
           12.6 13.4 8.5 9.3 ];     % box in aisle 3
P.pickup  = [1.2 4.2];              % pickup station (dock end of aisle 2)
P.deliver = [26.0 8.4];             % delivery station (dock end of aisle 3)
P.dwell   = 3.0;                    % [s] loading / unloading time
% Moving obstacles: 1 human crossing aisle 2, 2 human walking aisle 3, 3 second AMR in aisle 1, 4 pallet truck
P.mover.size    = [0.6 0.6; 0.6 0.6; 0.9 0.6; 1.4 2.6];   % [m] x/y extent (humans are circles of d = 0.6)
P.mover.truckX0 = 27.0;  P.mover.truckX1 = 20.4;  P.mover.truckV = 1.0;  P.mover.truckTrigX = 12.0;
% LiDAR (planar, 360 deg)
P.lidar.n = 72;  P.lidar.rmax = 8.0;
% Planner grid
P.grid.res = 0.2;  P.grid.x0 = -2.5;  P.grid.y0 = -2.2;  P.grid.nx = 155;  P.grid.ny = 71;
P.grid.inflate = 0.48;  P.grid.dynHold = 2.0;  P.grid.replanT = 0.5;
P.rCol = 0.36;                      % [m] robot collision radius

% --- Controller ----------------------------------------------------------
P.Ts    = 0.005;        % [s]  control period
P.vmax  = 1.2;          % [m/s]
P.amax  = 2.5;          % [m/s^2] nominal acceleration demand
P.Kp    = 0.3;  P.Ki = 8.0;           % wheel-speed PI (stable for wheel inertia 0.005 kg m^2 at Ts)
P.mass  = 30;           % [kg] robot mass used for acceleration feed-forward
P.vSlipFloor = 0.8;     % [m/s] speed floor in the slip-ratio denominator (avoids divide-by-zero at standstill)
P.k_th  = 0.12;         % slip ratio detection threshold
P.k_tgt = 0.08;         % slip ratio the traction controller regulates to
P.k_max = 0.40;         % slip ratio at which torque is cut to minimum
P.d0    = 0.9;           % [m] obstacle influence distance
P.tStop = 120;           % [s] simulation stop time
end

function R = rx(a), R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]; end
function R = ry(a), R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]; end
function R = rz(a), R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; end
