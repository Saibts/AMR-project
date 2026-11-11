function D = scout_amr_drive_post(t, lg, P)
%SCOUT_AMR_DRIVE_POST  Motor current, power and battery state from the logged wheel torques and speeds.
%  t : time [s],  lg : controller log (cols 13:16 wheel speed, 21:24 wheel torque after limiting).
%  Motor:  I = tau/Kt (Eq. 6),  Pel = tau w + R I^2 per motor (mechanical + copper loss), plus the idle power P0.
%  Battery: OCV falls linearly from 25.2 V (full) to 21.0 V (empty) with the charge, Vt = OCV - Rint I.
tau = lg(:, 21:24);  w = abs(lg(:, 13:16));
I   = tau / P.kt;                                   % [A] per motor (signed: negative = braking)
Pm  = abs(tau) .* w .* sign(tau .* lg(:, 13:16));   % mechanical power, negative while braking
Pel = Pm + P.Rm * I.^2;                             % electrical power per motor [W]
Ptot = sum(Pel, 2) + P.P0;                          % whole robot [W]
soc = ones(size(t)); Ib = zeros(size(t)); Vt = zeros(size(t));
Ah = 0;
for i = 1:numel(t)
    ocv = P.Vempty + (P.Vfull - P.Vempty) * soc(i);
    Ib(i) = (ocv - sqrt(max(ocv^2 - 4*P.Rint*Ptot(i), 0))) / (2*P.Rint);   % battery current for the power demand
    Vt(i) = ocv - P.Rint * Ib(i);
    if i < numel(t)
        Ah = Ah + Ib(i) * (t(i+1) - t(i)) / 3600;
        soc(i+1) = 1 - Ah / P.Ah;
    end
end
D.I = I; D.Pel = Pel; D.Ptot = Ptot; D.Ib = Ib; D.Vt = Vt; D.soc = soc;
D.energyWh = trapz(t, max(Ptot, 0)) / 3600;         % energy taken from the battery (no regeneration credit)
D.netWh    = trapz(t, Ptot) / 3600;
D.Imax = max(abs(I(:))); D.Irms = sqrt(mean(I(:).^2)); D.Pmax = max(Ptot); D.Pmean = mean(Ptot);
D.Vmin = min(Vt); D.socEnd = soc(end); D.Ibmax = max(Ib);
D.cuLossWh = trapz(t, sum(P.Rm * I.^2, 2)) / 3600;
D.tauRms = sqrt(mean(tau(:).^2)); D.tauMax = max(abs(tau(:)));
end
