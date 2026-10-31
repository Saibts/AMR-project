function out = scout_amr_tires(pO, vO, R, w, Rb, k)
%SCOUT_AMR_TIRES  Compliant ground contact + friction (slip) model for one wheel.
%  k = wheel index 1..4.  Inputs (world frame): wheel-frame origin position/velocity, wheel rotation matrix,
%  wheel angular velocity, chassis rotation matrix.  Output (11x1):
%  [F_local(3); T_local(3); Fz; Flong; Flat; kappa; mu] -- force/torque expressed in the wheel frame.
P  = scout_amr_params();
pO = pO(:); vO = vO(:); w = w(:);
% In the imported Simscape model the joint frame's Z axis is the wheel axle; point it outboard
a   = R(:,3);
if sign(P.yWheel(k)) * a(2) < 0, a = -a; end
ph  = pO + P.hubOff * a;                    % hub centre
pc  = ph - [0; 0; P.r];                     % contact point directly below the hub
vc  = vO + cross(w, pc - pO);               % contact-point velocity

pen = P.r - ph(3);
Fz  = 0;
if pen > 0
    Fz = max(0, P.kc * pen - P.cc * vc(3));
end

mu = P.mu0;
for i = 1:size(P.patches,1)
    q = P.patches(i,:);
    if pc(1) >= q(1) && pc(1) <= q(2) && pc(2) >= q(3) && pc(2) <= q(4)
        mu = q(5);
    end
end

vs = vc(1:2);
Ft = -mu * Fz * vs / sqrt(vs.'*vs + P.v0^2);     % smoothed Coulomb friction (sliding of contact patch)
Fw = [Ft; Fz];
Tw = cross(pc - pO, Fw);

Rwf = Rb * [0 1 0; -1 0 0; 0 0 1];               % footprint rotation (base_link yaw = +90 deg)
psi = atan2(Rwf(2,1), Rwf(1,1));
ef  = [cos(psi); sin(psi)]; el = [-sin(psi); cos(psi)];
wa  = w.' * a;                                    % absolute spin about the axle
vh  = vO + cross(w, ph - pO);                     % hub velocity
vwheel = P.r * wa * (cross(a, [0;0;1]).' * [ef; 0]);   % ground speed implied by wheel spin
vx  = vh(1:2).' * ef;
den = max([abs(vwheel), abs(vx), P.vSlipFloor]);
kappa = (vwheel - vx) / den;                      % >0 wheel spin, <0 lock-up

out = [R.' * Fw; R.' * Tw; Fz; Ft.' * ef; Ft.' * el; kappa; mu];
end
