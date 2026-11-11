function [tau_j, lg, mv, rng, pth] = scout_amr_controller(pb, Rb, vb, wb, wj, tc_on)
%SCOUT_AMR_CONTROLLER  Mission navigation, wheel-speed control and traction (slip) management.
%  pb,Rb,vb,wb : base_link pose / velocity from the multibody model (world frame)
%  wj          : joint speeds of the four wheel revolute joints [rad/s]
%  tc_on       : 1 = slip management active, 0 = plain wheel-speed PI (for comparison)
P = scout_amr_params();
dt = P.Ts;
persistent init vc_prev wc_prev I mu_est vx_prev ax_f slipTimer
if isempty(init)
    init = true; vc_prev = 0; wc_prev = 0; I = zeros(4,1);
    mu_est = P.mu0; vx_prev = 0; ax_f = 0; slipTimer = 0;
end

% ---- Pose of base_footprint from base_link ------------------------------
Rwf = Rb * [0 1 0; -1 0 0; 0 0 1];
pf  = pb(:) - Rwf * P.p_fb;
psi = atan2(Rwf(2,1), Rwf(1,1));
vf  = vb(:) - cross(wb(:), Rwf * P.p_fb);
ef  = [cos(psi); sin(psi)];  el = [-sin(psi); cos(psi)];
vx  = vf(1:2).' * ef;  vy = vf(1:2).' * el;  wz = wb(3);
pos = pf(1:2);

% ---- Warehouse autonomy: mission, LiDAR, A* replanning, path following ------------------
[vref, wref, mv, rng, pth, info] = scout_wh_nav(pos(1), pos(2), psi, dt);
mv = reshape(mv.', 8, 1);          % [x1; y1; x2; y2; ...] for the 3D movers
done = info(9) > 0.5;
err = 0;

% ---- Friction-aware acceleration limit -----------------------------------
if tc_on > 0.5
    alim = min(P.amax, 0.85 * mu_est * 9.81);
else
    alim = P.amax;
end
vcmd = vc_prev + max(-1.6*alim*dt, min(alim*dt, vref - vc_prev));
acmd = (vcmd - vc_prev) / dt;
wcmd = wc_prev + max(-4*dt, min(4*dt, wref - wc_prev));
vc_prev = vcmd; wc_prev = wcmd;

wL = (vcmd - 0.5*P.track*wcmd) / P.r;
wR = (vcmd + 0.5*P.track*wcmd) / P.r;
wc = [wL; wL; wR; wR];

% ---- Slip estimation from wheel speed vs. ground speed -------------------
wfwd = P.sgn(:) .* wj(:);
vk   = vx - wz * P.yWheel(:);               % ground speed at each wheel's lateral position
den  = max(max(abs(P.r*wfwd), abs(vk)), P.vSlipFloor);   % low-speed regularisation of the slip ratio
kappa = (P.r*wfwd - vk) ./ den;
slip = any(abs(kappa) > P.k_th);

% ---- Traction management -------------------------------------------------
% motor torque envelope (Eq. 8 of the report): current limit tmax, and above the knee the back-EMF limit Kt (V - Ke w) / R
tauEnv = scout_amr_motor_limit(wfwd);
tauLim = tauEnv;
if tc_on > 0.5
    wgl = vk / P.r;                          % wheel speed that matches the ground
    dW  = (P.k_tgt*abs(vk) + 0.12) / P.r;    % allowed wheel-speed deviation (target slip)
    wc  = max(wgl - dW, min(wgl + dW, wc));  % anti-spin / anti-lock speed window
    for k = 1:4
        s = (abs(kappa(k)) - P.k_th) / (P.k_max - P.k_th);
        if s > 0, tauLim(k) = tauEnv(k) * max(0.1, 1 - s); end
    end
    if slip, mu_est = max(0.10, mu_est - 0.5*dt); else, mu_est = min(P.mu0, mu_est + 0.15*dt); end
end
if slip, slipTimer = slipTimer + dt; end

% ---- Wheel-speed PI ------------------------------------------------------
e = wc - wfwd;
I = max(-tauLim, min(tauLim, I + P.Ki*e*dt));
tau_ff = P.mass * acmd * P.r / 4;             % acceleration feed-forward
tau_d = max(-tauLim, min(tauLim, tau_ff + P.Kp*e + I));
tau_j = P.sgn(:) .* tau_d;
if tc_on < 0, tau_j(:) = 0; end   % diagnostic: motors off

ax_raw = (vx - vx_prev)/dt; vx_prev = vx; ax_f = 0.99*ax_f + 0.01*ax_raw;

lg = [pf(1); pf(2); psi; vx; vy; wz; vcmd; wcmd; wc; wfwd; kappa; tau_d; tauLim; ...
      mu_est; double(slip); info(1); info(5); double(done); err; ax_f; slipTimer; vref; ...
      info(2); info(3); info(4); info(8); info(6); info(7); info(10)];
end
