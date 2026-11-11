function tau = scout_amr_motor_limit(w)
%SCOUT_AMR_MOTOR_LIMIT  Largest wheel torque of one hub motor at wheel speed w [rad/s] (Eq. 8 of the report).
%  Current limit: tau <= Kt*Ipeak = tmax.  Voltage limit: Kt*(V - Ke*|w|)/R, with Ke = Kt, which is zero at the no-load speed V/Ke.
P   = scout_amr_params();
tau = min(P.tmax, P.kt * max(P.Vbus - P.kt*abs(w), 0) / P.Rm);
end
