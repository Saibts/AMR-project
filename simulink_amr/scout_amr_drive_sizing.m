function S = scout_amr_drive_sizing(verbose)
%SCOUT_AMR_DRIVE_SIZING  Wheel force, torque, current, power and traction demand of the Scout Mini drive.
%  Hand calculation of the report (Section 8): quasi-static load cases for one of four equally loaded wheels,
%  the motor torque-speed limit and the grip limit mu*m*g of the floor.  Also used by plotDrive in
%  run_scout_warehouse_simulation to draw the operating points of the real run on the same envelope.
%    S = scout_amr_drive_sizing()        prints the tables
%    S = scout_amr_drive_sizing(false)   silent
if nargin < 1, verbose = true; end
P = scout_amr_params(); g = 9.81; r = P.r; m = P.mass; W = m*g;
Ry = P.track/2; Rx = P.wheelBase/2; Rc = hypot(Rx, Ry);            % wheel offsets from the turning centre
S.m = m; S.W = W; S.Fz = W/4;
Jw = 0.005;                                                         % wheel inertia about the axle (URDF)
th = deg2rad(P.rampDeg);
Fr = @(mm) mm*g*P.Crr;                                              % rolling resistance of the whole robot

% ---- driving cases: total tractive force F at the ground, wheel speed, per-wheel torque -------------------------
%   name, force [N], linear acceleration used for the wheel-inertia term, wheel speed [rad/s]
mL = m + P.payload;
cs = {
 'Cruise 1.2 m/s',                  Fr(m),                                          0,    P.vmax/r
 'Start, a = 2.5 m/s^2',            Fr(m) + m*P.amax,                               P.amax, P.vmax/r
 'Start, a = 2.5 m/s^2, +12 kg',    Fr(mL) + mL*P.amax,                             P.amax, P.vmax/r
 'Brake, a = -4.0 m/s^2',           Fr(m) - m*1.6*P.amax,                           -1.6*P.amax, P.vmax/r
 'Dock ramp 7.3 deg, 0.8 m/s',      m*g*(sin(th) + P.Crr*cos(th)),                  0,    0.8/r
 'Dock ramp 7.3 deg, +12 kg',       mL*g*(sin(th) + P.Crr*cos(th)),                 0,    0.8/r };
k0 = struct('name', '', 'F', 0, 'tau', 0, 'omega', 0, 'I', 0, 'tauDamp', 0, 'Pmech', 0, 'Pel', 0, 'pctRated', 0, 'mass', m, 'tauAvail', 0, 'tauGround', 0, 'demand', [0 0 0]);
for i = 1:size(cs,1)
    mm = m; if contains(cs{i,1}, '12 kg'), mm = mL; end
    k = k0; k.name = cs{i,1}; k.F = cs{i,2}; k.mass = mm;
    k.tau = (k.F*r + 4*Jw*cs{i,3}/r)/4;                              % Eq. 5: F r / 4 plus the wheel inertia (joint damping added below)
    k.omega = cs{i,4};
    k.tauDamp = P.bJoint*k.omega;                                      % joint damping: the motor must also overcome b*w
    S.cases(i) = k; %#ok<AGROW>
end
% pivot in place: each wheel scrubs with muT*Fz; the component along the driving direction is driven
n = numel(S.cases);
Fpiv = P.muT*S.Fz*Ry/Rc;                                            % steady pivot, drive force per wheel [N]
k = k0; k.name = 'Pivot turn, w = 1.3 rad/s'; k.F = 4*Fpiv; k.tau = Fpiv*r; k.omega = 1.3*Ry/r; k.tauDamp = P.bJoint*k.omega; S.cases(n+1) = k;
alpha = 4;                                                          % yaw acceleration limit of the controller [rad/s^2]
Fpa = Fpiv + P.Iz*alpha/(4*Rc)*Ry/Rc;
k = k0; k.name = 'Pivot start, a = 4 rad/s^2'; k.F = 4*Fpa; k.tau = Fpa*r; k.omega = 1.3*Ry/r; k.tauDamp = P.bJoint*k.omega; S.cases(n+2) = k;
S.Mpivot = P.muT*W*Rc;                                              % steady yaw moment of the scrub [N m]
for i = 1:numel(S.cases)
    k = S.cases(i);
    k.tauGround = k.tau;                                             % torque that makes the ground force
    k.tau   = k.tauGround + k.tauDamp;                               % motor torque per wheel (sign of the motion)
    k.I     = abs(k.tau)/P.kt;                                       % Eq. 6
    k.Pmech = abs(k.tau)*k.omega;
    k.Pel   = k.Pmech + P.Rm*k.I^2;                                  % mechanical + copper loss per motor
    k.pctRated = 100*abs(k.tau)/P.trated;
    k.tauAvail = scout_amr_motor_limit(k.omega);                    % Eq. 8 at the case speed
    S.cases(i) = k;
end
% ---- grip limit ---------------------------------------------------------------------------------------------------
S.mu = [P.mu0 0.30 0.15]; S.muName = {'dry concrete', 'water', 'oil'};
S.Fmax = S.mu*W; S.aMax = S.mu*g; S.tauGrip = S.mu*S.Fz*r;           % Eqs. 11-13
for i = 1:numel(S.cases)
    S.cases(i).demand = abs(S.cases(i).F) ./ (S.mu*S.cases(i).mass*g);   % force demand / grip: >1 means the wheels slip
end
% ---- motor limits (Eqs. 9-10) -------------------------------------------------------------------------------------
S.omega0 = P.Vbus/P.kt;                                              % no-load speed
S.knee   = (P.Vbus - P.Rm*P.Ipk)/P.kt;                               % end of the current-limited range
S.tauStall = P.kt*P.Vbus/P.Rm;                                       % unlimited stall torque
S.Fpeak  = 4*P.tmax/r;                                               % peak force of the four wheels
S.wTop   = (P.vmax + 1.3*Ry)/r;                                      % fastest wheel of the mission (v 1.2 m/s, w 1.3 rad/s)
S.tauAtTop = scout_amr_motor_limit(S.wTop);
S.vNoLoad = S.omega0*r;
S.P = P;
if verbose, printSizing(S, P); end
end

function printSizing(S, P)
    fprintf('\n========== DRIVE SIZING (m = %.1f kg, W = %.1f N, r = %.3f m) ==========\n', S.m, S.W, P.r);
    fprintf('%-30s %8s %9s %7s %7s %8s %8s %9s %9s\n', 'case', 'F [N]', 'tau [Nm]', '%rated', 'I [A]', 'Pel [W]', 'avail', 'dem dry', 'dem oil');
    for k = S.cases
        fprintf('%-30s %8.1f %9.2f %7.0f %7.1f %8.1f %8.1f %9.2f %9.2f\n', k.name, k.F, k.tau, k.pctRated, k.I, k.Pel, k.tauAvail, ...
            k.demand(1), k.demand(3));
    end
    fprintf('Grip per wheel:  dry %.2f Nm (a <= %.1f m/s^2),  water %.2f Nm (%.1f),  oil %.2f Nm (%.2f)\n', ...
        S.tauGrip(1), S.aMax(1), S.tauGrip(2), S.aMax(2), S.tauGrip(3), S.aMax(3));
    fprintf('Motor: rated %.1f Nm / %.1f A, peak %.1f Nm / %.1f A, stall(unlimited) %.1f Nm, knee %.1f rad/s, no-load %.1f rad/s (%.2f m/s)\n', ...
        P.trated, P.Irated, P.tmax, P.Ipk, S.tauStall, S.knee, S.omega0, S.vNoLoad);
    fprintf('Peak force of 4 wheels %.0f N;  torque available at the fastest mission wheel speed %.1f rad/s: %.1f Nm;  pivot moment %.1f Nm\n', ...
        S.Fpeak, S.wTop, S.tauAtTop, S.Mpivot);
end
