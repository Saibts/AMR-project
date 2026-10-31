function r = scout_wh_lidar(x, y, psi, rects, circs, n, rmax)
%SCOUT_WH_LIDAR  Planar 360-degree LiDAR: ray-casts n beams against the warehouse geometry.
%  Beam i points at angle psi + (-pi + (i-1)*2*pi/n); returns ranges [m] (rmax = no return).
r = rmax * ones(n, 1);
for i = 1:n
    a = psi - pi + (i-1) * 2*pi / n;
    dx = cos(a); dy = sin(a);
    best = rmax;
    for k = 1:size(rects, 1)                       % ray vs axis-aligned box (slab method)
        if abs(dx) < 1e-9, dxs = 1e-9; else, dxs = dx; end
        if abs(dy) < 1e-9, dys = 1e-9; else, dys = dy; end
        tx1 = (rects(k,1) - x) / dxs;  tx2 = (rects(k,2) - x) / dxs;
        ty1 = (rects(k,3) - y) / dys;  ty2 = (rects(k,4) - y) / dys;
        tmin = max(min(tx1, tx2), min(ty1, ty2));
        tmax = min(max(tx1, tx2), max(ty1, ty2));
        if tmax >= max(tmin, 0) && tmin > 0 && tmin < best
            best = tmin;
        end
    end
    for k = 1:size(circs, 1)                       % ray vs circle
        fx = x - circs(k,1); fy = y - circs(k,2);
        b = fx*dx + fy*dy;
        c = fx*fx + fy*fy - circs(k,3)^2;
        disc = b*b - c;
        if disc >= 0
            tc = -b - sqrt(disc);
            if tc > 0 && tc < best, best = tc; end
        end
    end
    r(i) = round(best * 50) / 50;                  % 2 cm quantisation
end
end
