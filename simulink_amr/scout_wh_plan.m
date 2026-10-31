function [path, n] = scout_wh_plan(s, g, blocked, G)
%SCOUT_WH_PLAN  8-connected A* on the inflated occupancy grid, followed by line-of-sight smoothing.
%  s, g    : start / goal [x y] in metres
%  blocked : nx-by-ny logical occupancy (static racks/walls + recently seen obstacles)
%  path    : 80x2 waypoints (first n rows valid), n = 0 when no route exists
NMAX = 80;
path = zeros(NMAX, 2); n = 0;
nx = G.nx; ny = G.ny;
sc = clampCell(floor(([s(1) s(2)] - [G.x0 G.y0]) / G.res) + 1, nx, ny);
gc = clampCell(floor(([g(1) g(2)] - [G.x0 G.y0]) / G.res) + 1, nx, ny);

% The robot is physically at the start cell: keep a small disc around it traversable.
for i = max(1, sc(1)-2):min(nx, sc(1)+2)
    for j = max(1, sc(2)-2):min(ny, sc(2)+2)
        blocked(i, j) = false;
    end
end
% If the goal itself is inside an inflated zone, use the closest free cell.
if blocked(gc(1), gc(2))
    bestd = inf; bc = gc;
    for i = max(1, gc(1)-6):min(nx, gc(1)+6)
        for j = max(1, gc(2)-6):min(ny, gc(2)+6)
            if ~blocked(i, j)
                d = (i-gc(1))^2 + (j-gc(2))^2;
                if d < bestd, bestd = d; bc = [i j]; end
            end
        end
    end
    gc = bc;
end

N = nx * ny;
gScore = inf(N, 1);
parent = zeros(N, 1);
closed = false(N, 1);
heapI = zeros(N + 8, 1); heapF = zeros(N + 8, 1); hs = 0;
sIdx = sc(1) + (sc(2)-1) * nx;  gIdx = gc(1) + (gc(2)-1) * nx;
gScore(sIdx) = 0;
[heapI, heapF, hs] = hpush(heapI, heapF, hs, sIdx, heur(sc, gc));
found = false;
dxs = [1 -1 0 0 1 1 -1 -1]; dys = [0 0 1 -1 1 -1 1 -1];
cst = [1 1 1 1 1.4142 1.4142 1.4142 1.4142];
while hs > 0
    [cur, heapI, heapF, hs] = hpop(heapI, heapF, hs);
    if closed(cur), continue; end
    closed(cur) = true;
    if cur == gIdx, found = true; break; end
    ci = mod(cur - 1, nx) + 1; cj = floor((cur - 1) / nx) + 1;
    for k = 1:8
        ni = ci + dxs(k); nj = cj + dys(k);
        if ni < 1 || ni > nx || nj < 1 || nj > ny, continue; end
        if blocked(ni, nj), continue; end
        if k > 4 && (blocked(ci + dxs(k), cj) || blocked(ci, cj + dys(k))), continue; end   % no corner cutting
        nIdx = ni + (nj-1) * nx;
        if closed(nIdx), continue; end
        tg = gScore(cur) + cst(k);
        if tg < gScore(nIdx)
            gScore(nIdx) = tg; parent(nIdx) = cur;
            [heapI, heapF, hs] = hpush(heapI, heapF, hs, nIdx, tg + heur([ni nj], gc));
        end
    end
end
if ~found, return; end

% Reconstruct (goal -> start)
cells = zeros(N, 2); m = 0; cur = gIdx;
while cur ~= 0 && m < 2000
    m = m + 1; cells(m, :) = [mod(cur - 1, nx) + 1, floor((cur - 1) / nx) + 1];
    cur = parent(cur);
end
cells = cells(m:-1:1, :);

% Line-of-sight smoothing (string pulling)
keep = zeros(m, 1); nk = 1; keep(1) = 1; i = 1;
while i < m
    j = m;
    while j > i + 1 && ~lineFree(cells(i,:), cells(j,:), blocked)
        j = j - 1;
    end
    nk = nk + 1; keep(nk) = j; i = j;
end
n = min(nk, NMAX);
for q = 1:n
    c = cells(keep(q), :);
    path(q, :) = [G.x0 + (c(1) - 0.5) * G.res, G.y0 + (c(2) - 0.5) * G.res];
end
path(n, :) = [g(1) g(2)];       % end exactly on the station (the grid goal may have been shifted)
if blocked(gc(1), gc(2)), path(n, :) = G.x0 + (gc - 0.5) * G.res; end
end

function c = clampCell(c, nx, ny)
    c(1) = min(max(c(1), 1), nx); c(2) = min(max(c(2), 1), ny);
end
function h = heur(a, b)
    dx = abs(a(1) - b(1)); dy = abs(a(2) - b(2));
    h = (dx + dy) + (1.4142 - 2) * min(dx, dy);     % octile distance
end
function ok = lineFree(a, b, blocked)
    L = max(abs(b(1) - a(1)), abs(b(2) - a(2))) * 2;
    ok = true;
    for q = 0:L
        u = q / max(L, 1);
        ci = round(a(1) + u * (b(1) - a(1))); cj = round(a(2) + u * (b(2) - a(2)));
        if blocked(ci, cj), ok = false; return; end
    end
end
function [hi, hf, hs] = hpush(hi, hf, hs, id, f)
    hs = hs + 1; hi(hs) = id; hf(hs) = f; k = hs;
    while k > 1
        p = floor(k / 2);
        if hf(p) <= hf(k), break; end
        ti = hi(p); hi(p) = hi(k); hi(k) = ti; tf = hf(p); hf(p) = hf(k); hf(k) = tf; k = p;
    end
end
function [id, hi, hf, hs] = hpop(hi, hf, hs)
    id = hi(1); hi(1) = hi(hs); hf(1) = hf(hs); hs = hs - 1; k = 1;
    while true
        l = 2 * k; r = l + 1; m = k;
        if l <= hs && hf(l) < hf(m), m = l; end
        if r <= hs && hf(r) < hf(m), m = r; end
        if m == k, break; end
        ti = hi(m); hi(m) = hi(k); hi(k) = ti; tf = hf(m); hf(m) = hf(k); hf(k) = tf; k = m;
    end
end
