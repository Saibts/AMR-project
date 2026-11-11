function tests = test_scout_amr_drive
%TEST_SCOUT_AMR_DRIVE  Unit tests of the drive model (no simulation needed):  runtests('test_scout_amr_drive')
tests = functiontests(localfunctions);
end

function testStallTorqueMatchesCurrentLimit(t)
    P = scout_amr_params();
    verifyEqual(t, scout_amr_motor_limit(0), P.kt*P.Ipk, 'AbsTol', 1e-9);
    verifyEqual(t, scout_amr_motor_limit(0), 15, 'AbsTol', 1e-9);
end

function testNoLoadSpeedIsVoltageLimited(t)
    P = scout_amr_params();
    verifyEqual(t, scout_amr_motor_limit(P.Vbus/P.kt), 0, 'AbsTol', 1e-9);        % 37.5 rad/s
    verifyEqual(t, scout_amr_motor_limit(1.2*P.Vbus/P.kt), 0, 'AbsTol', 1e-9);    % never negative above it
end

function testTorqueFallsAboveTheKnee(t)
    P = scout_amr_params(); knee = (P.Vbus - P.Rm*P.Ipk)/P.kt;                     % 15.5 rad/s
    verifyEqual(t, scout_amr_motor_limit(0.9*knee), P.tmax, 'AbsTol', 1e-9);       % flat below the knee
    w = linspace(knee, P.Vbus/P.kt, 20);
    verifyTrue(t, all(diff(scout_amr_motor_limit(w)) < 0));                         % strictly falling above it
    verifyEqual(t, scout_amr_motor_limit(-20), scout_amr_motor_limit(20), 'AbsTol', 1e-12);   % symmetric in the direction
end

function testCruiseAndStartTorqueMatchTable(t)
    S = scout_amr_drive_sizing(false);
    verifyEqual(t, S.cases(1).tauGround, 0.18, 'AbsTol', 0.005);   % cruise 1.2 m/s, ground part
    verifyEqual(t, S.cases(1).tau, 1.59, 'AbsTol', 0.01);          % with the joint damping 0.1*14.1
    verifyEqual(t, S.cases(2).tau, 3.24, 'AbsTol', 0.01);          % start 2.5 m/s^2
    verifyEqual(t, S.cases(1).I, S.cases(1).tau/0.64, 'AbsTol', 1e-9);
end

function testGripLimitsAreBelowMotorTorque(t)
    S = scout_amr_drive_sizing(false); P = S.P;
    verifyEqual(t, S.tauGrip, [5.31 1.77 0.88], 'AbsTol', 0.01);
    verifyTrue(t, all(S.tauGrip < P.tmax));                                         % the floor, not the motor, is the limit
    verifyGreaterThan(t, S.cases(2).demand(3), 1);                                  % 2.5 m/s^2 start on oil: wheels must slip
    verifyLessThan(t, S.cases(2).demand(1), 1);                                     % ... but not on dry concrete
end

function testBatterySagAndChargeBalance(t)
    P = scout_amr_params(); tt = (0:0.01:100).';
    lg = zeros(numel(tt), 44); lg(:, 13:16) = 10; lg(:, 21:24) = 2;                 % 2 N m at 10 rad/s on every wheel
    D = scout_amr_drive_post(tt, lg, P);
    Pel = 4*(2*10 + P.Rm*(2/P.kt)^2) + P.P0;                                        % Eq. 7
    verifyEqual(t, D.Ptot(1), Pel, 'RelTol', 1e-9);
    verifyEqual(t, D.Vt(1), P.Vfull - P.Rint*D.Ib(1), 'AbsTol', 1e-9);              % Vt = OCV - Rint I
    verifyEqual(t, D.Ib(1)*D.Vt(1), Pel, 'RelTol', 1e-9);                           % battery current delivers the power
    verifyTrue(t, all(diff(D.soc) < 0));                                            % charge falls
    verifyEqual(t, D.energyWh, Pel*100/3600, 'RelTol', 1e-6);
end
