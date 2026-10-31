function [vref, wref, mv, rng, pth, info] = scout_wh_nav(px, py, psi, dt)
%SCOUT_WH_NAV  Warehouse autonomy: mission state machine, LiDAR perception, A* replanning,
%  pure-pursuit path following, reactive avoidance and collision monitoring.
%  Inputs : robot footprint pose (px, py, psi) and the control period dt.
%  Outputs: speed / yaw-rate request, mover positions (for the 3D scene), LiDAR ranges,
%           current path [n; x1 y1; ...] and an info vector (see bottom).
P = scout_amr_params(); G = P.grid;
persistent init t tTrig staticBlk dynT path pathN replanT nReroute state dwellT done ...
           rngHold lidarT stuckT collCount inColl lastRerouteT
if isempty(init)
    init = true; t = 0; tTrig = -1; dynT = -inf(G.nx, G.ny); path = zeros(80, 2); pathN = 0;
    replanT = 1e9; nReroute = 0; state = 1; dwellT = 0; done = false;
    rngHold = P.lidar.rmax * ones(P.lidar.n, 1); lidarT = 1e9; stuckT = 0; collCount = 0; inColl = false; lastRerouteT = -10;
    [r0, ~, ~] = scout_wh_world(0, -1, -100, -100, 0);
    staticBlk = false(G.nx, G.ny);
    for i = 1:G.nx
        for j = 1:G.ny
            cx = G.x0 + (i - 0.5) * G.res; cy = G.y0 + (j - 0.5) * G.res;
            for k = 1:13                                   % racks + walls are known a priori
                dxr = max([r0(k,1) - cx, 0, cx - r0(k,2)]); dyr = max([r0(k,3) - cy, 0, cy - r0(k,4)]);
                if sqrt(dxr^2 + dyr^2) < G.inflate, staticBlk(i, j) = true; break; end
            end
        end
    end
end
t = t + dt;
if tTrig < 0 && px > P.mover.truckTrigX, tTrig = t; end        % pallet truck is released into aisle 2
[rects, circs, mv] = scout_wh_world(t, tTrig, px, py, dt);

% ---- collision monitor (analytic ground truth, not used by the controller) ------------------
minClr = 1e3;
for k = 1:size(rects, 1)
    dxr = max([rects(k,1) - px, 0, px - rects(k,2)]); dyr = max([rects(k,3) - py, 0, py - rects(k,4)]);
    minClr = min(minClr, sqrt(dxr^2 + dyr^2) - P.rCol);
end
for k = 1:size(circs, 1)
    minClr = min(minClr, hypot(px - circs(k,1), py - circs(k,2)) - circs(k,3) - P.rCol);
end
if minClr < 0 && ~inColl, collCount = collCount + 1; end
inColl = minClr < 0;

% ---- LiDAR (10 Hz) and dynamic obstacle layer -----------------------------------------------
lidarT = lidarT + dt;
if lidarT >= 0.1
    lidarT = 0;
    rngHold = scout_wh_lidar(px, py, psi, rects, circs, P.lidar.n, P.lidar.rmax);
    for i = 1:P.lidar.n
        if rngHold(i) < P.lidar.rmax - 0.05
            a = psi - pi + (i-1) * 2*pi / P.lidar.n;
            hx = px + rngHold(i) * cos(a); hy = py + rngHold(i) * sin(a);
            ci = floor((hx - G.x0) / G.res) + 1; cj = floor((hy - G.y0) / G.res) + 1;
            if ci >= 1 && ci <= G.nx && cj >= 1 && cj <= G.ny && ~staticBlk(ci, cj)
                for di = -2:2
                    for dj = -2:2
                        ii = ci + di; jj = cj + dj;
                        if ii >= 1 && ii <= G.nx && jj >= 1 && jj <= G.ny && di*di + dj*dj <= 5
                            dynT(ii, jj) = t;
                        end
                    end
                end
            end
        end
    end
end
rng = rngHold;

% ---- mission state machine -----------------------------------------------------------------
if state == 1 || state == 2, goal = P.pickup; else, goal = P.deliver; end
if state == 5, goal = [px py]; end
gd = hypot(goal(1) - px, goal(2) - py);
if (state == 1 || state == 3) && gd < 0.40
    state = state + 1; dwellT = 0; pathN = 0;
elseif state == 2 || state == 4
    dwellT = dwellT + dt;
    if dwellT >= P.dwell
        state = state + 1; pathN = 0; replanT = 1e9;
        if state == 5, done = true; end
    end
end

vref = 0; wref = 0; rf = P.lidar.rmax; rb = P.lidar.rmax; stuck = false;
if state == 1 || state == 3
    % ---- (re)planning ------------------------------------------------------------------------
    replanT = replanT + dt;
    wasBlocked = false;
    if pathN > 0
        for q = 1:pathN-1
            for u = 0:0.25:1
                cx = path(q,1) + u * (path(q+1,1) - path(q,1)); cy = path(q,2) + u * (path(q+1,2) - path(q,2));
                ci = floor((cx - G.x0) / G.res) + 1; cj = floor((cy - G.y0) / G.res) + 1;
                if ci >= 1 && ci <= G.nx && cj >= 1 && cj <= G.ny
                    if (t - dynT(ci, cj)) < G.dynHold, wasBlocked = true; end
                end
            end
        end
    end
    if replanT >= G.replanT || pathN == 0 || wasBlocked
        replanT = 0;
        blocked = staticBlk | ((t - dynT) < G.dynHold);
        [newPath, nn] = scout_wh_plan([px py], goal, blocked, G);
        if nn > 0
            if wasBlocked && (t - lastRerouteT) > 4, nReroute = nReroute + 1; lastRerouteT = t; end
            path = newPath; pathN = nn;
        end
    end

    % ---- pure pursuit along the planned path -----------------------------------------------
    if pathN > 0
        bestD = inf; bq = 1; bu = 0;
        for q = 1:pathN-1
            ab = path(q+1,:) - path(q,:); L2 = max(ab * ab', 1e-9);
            u = max(0, min(1, (([px py] - path(q,:)) * ab') / L2));
            d = norm([px py] - (path(q,:) + u * ab));
            if d < bestD, bestD = d; bq = q; bu = u; end
        end
        look = 0.9; tgt = path(pathN,:);
        q = bq; pos = path(q,:) + bu * (path(min(q+1,pathN),:) - path(q,:)); rem = look;
        while q < pathN
            seg = norm(path(q+1,:) - pos);
            if seg >= rem, tgt = pos + (path(q+1,:) - pos) * (rem / max(seg, 1e-9)); break; end
            rem = rem - seg; pos = path(q+1,:); q = q + 1; tgt = pos;
        end
        ux = tgt(1) - px; uy = tgt(2) - py; un = max(hypot(ux, uy), 1e-6); ux = ux / un; uy = uy / un;
        % reactive layer: push away from close LiDAR returns (catches moving people / things the map lacks)
        fx = 0; fy = 0;
        for i = 1:P.lidar.n
            r = rng(i);
            if r < 0.9
                a = psi - pi + (i-1) * 2*pi / P.lidar.n; w = (0.9 - r) / 0.9;
                fx = fx - w * cos(a); fy = fy - w * sin(a);
            end
            ar = atan2(sin((i-1) * 2*pi / P.lidar.n - pi), cos((i-1) * 2*pi / P.lidar.n - pi));
            if abs(ar) < 0.9, rf = min(rf, r); end
            if abs(abs(ar) - pi) < 0.9, rb = min(rb, r); end
        end
        dx = ux + 0.25 * fx; dy = uy + 0.25 * fy;
        err = atan2(sin(atan2(dy, dx) - psi), cos(atan2(dy, dx) - psi));
        wref = max(-1.3, min(1.3, 1.8 * err));
        vref = min(P.vmax, sqrt(max(0, 2 * 0.8 * (rf - 0.55)))) * max(0, 1 - abs(err) / 1.2);   % stop-before-contact speed limit
        vref = min(vref, max(0.15, 0.8 * gd));
        if rf < 0.55, vref = 0; end                  % safety stop
        if rf < 0.68 && rb > 0.9, vref = -0.4; wref = 0; end   % something is closing in: back away if the rear is clear
        if abs(err) > 1.2 && rf >= 0.55, wref = sign(err) * 1.0; end   % turn in place
    end
end
if vref < 0.05 && (state == 1 || state == 3), stuckT = stuckT + dt; else, stuckT = 0; end
stuck = stuckT > 3;
if stuck, replanT = 1e9; end

pth = [pathN; reshape(path', [], 1)];
info = [state; nReroute; min(rng); double(inColl); minClr; rf; double(stuck); collCount; double(done); gd; tTrig];
vref = double(vref); wref = double(wref);
end
