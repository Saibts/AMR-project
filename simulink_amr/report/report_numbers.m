function report_numbers()
%REPORT_NUMBERS  Collects the figures quoted in the report from results/scout_warehouse_results.mat -> report/numbers.json
here = fileparts(mfilename('fullpath')); root = fileparts(here);
L = load(fullfile(root, 'results', 'scout_warehouse_results.mat')); R = L.R; P = L.P;
N = struct();
names = {'off', 'on'};
for m = 1:numel(R)
    if isempty(R(m).r), continue; end
    r = R(m).r; lg = r.lg; t = r.t; s = lg(:, 31);
    d = find(lg(:, 33) > 0.5, 1);
    k = struct();
    k.done = ~isempty(d);
    if k.done, k.missionTime = t(d); else, k.missionTime = NaN; end
    k.tPickup = t(find(s >= 2, 1)); k.tLeave = t(find(s >= 3, 1)); k.tDeliver = t(find(s >= 4, 1));
    k.distance = sum(hypot(diff(lg(:, 1)), diff(lg(:, 2))));
    k.topSpeed = max(lg(:, 4)); k.meanSpeed = k.distance / max(k.missionTime, 1);
    k.reroutes = lg(end, 38); k.collisions = lg(end, 41); k.minClear = min(lg(:, 32)); k.minLidar = min(lg(:, 39));
    kap = abs(lg(:, 17:20));
    k.peakSlip = max(kap(:)); k.rmsSlip = sqrt(mean(kap(:) .^ 2));
    k.fracSlip02 = mean(max(kap, [], 2) > 0.2);
    k.energy = trapz(t, sum(abs(lg(:, 21:24) .* lg(:, 13:16)), 2));
    k.rmsTorque = sqrt(mean(reshape(lg(:, 21:24), [], 1) .^ 2));
    k.maxTorque = max(max(abs(lg(:, 21:24))));
    k.muEstMin = min(lg(:, 29));
    k.wheelSpeedErrRms = sqrt(mean((lg(:, 9:12) - lg(:, 13:16)) .^ 2, 'all'));
    k.endTime = t(end); k.nSamples = numel(t);
    % cross-track / speed tracking
    k.vErrRms = sqrt(mean((lg(:, 7) - lg(:, 4)) .^ 2));
    k.maxYawRate = max(abs(lg(:, 6)));
    N.(names{m}) = k;
end
% events of the ON run
r = R(2).r; lg = r.lg; t = r.t;
ch = [1; find(diff(lg(:, 38)) > 0) + 1];
N.rerouteTimes = t(ch(2:end))';
N.params = struct('r', P.r, 'track', 0.612, 'tmax', P.tmax, 'vmax', P.vmax, 'amax', P.amax, 'Ts', P.Ts, 'kc', P.kc, 'cc', P.cc, 'mu0', P.mu0);
fid = fopen(fullfile(here, 'numbers.json'), 'w'); fwrite(fid, jsonencode(N, 'PrettyPrint', true)); fclose(fid);
disp(jsonencode(N, 'PrettyPrint', true));
end
