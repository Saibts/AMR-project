function [rects, circs, mv] = scout_wh_world(t, tTrig, rx, ry, dt)
%SCOUT_WH_WORLD  Warehouse obstacle geometry at time t (used by the LiDAR, collision check and the 3D movers).
%  rects : 17x4  [xmin xmax ymin ymax]  racks(9), walls(4), floor boxes(2), second AMR, pallet truck
%  circs : 2x3   [x y r]                workers walking through the aisles
%  mv    : 4x2   [x y] centres of the moving bodies (human 1, human 2, AMR 2, truck)
%  tTrig : time the truck was released (< 0 while it is still parked at the east dock)
%  rx,ry : robot position. Workers are aware of the AMR: they do not step out in front of it
%          and wait while it is close (they are still obstacles the robot has to avoid).
P = scout_amr_params();
persistent u1 ph2
if isempty(u1), u1 = 0; ph2 = 0; end
dR1 = hypot(rx - 14.0, ry - (3.0 + 2.5 * crossing(u1)));
crossingNow = (u1 - floor(u1) >= 0.30 && u1 - floor(u1) < 0.50) || (u1 - floor(u1) >= 0.80);
if (dR1 > 2.2 || crossingNow) && dR1 > 1.0, u1 = u1 + dt / 18; end   % worker 1 only starts a crossing when the lane is clear, and pauses if the AMR is right there
h2x = 20.0 + 4.5 * sin(ph2);
if hypot(rx - h2x, ry - 7.5) > 2.2, ph2 = ph2 + dt * 2*pi/26; end   % worker 2 stops when the AMR is near
h1 = [14.0, 3.0 + 2.5 * crossing(u1)];            % worker repeatedly crossing aisle 2 (pausing at the shelves)
h2 = [20.0 + 4.5 * sin(ph2), 7.5];                     % worker walking along aisle 3
a2 = [13.0 + 8.0 * tri(t/36), -0.4];                   % second AMR shuttling in aisle 1
if tTrig < 0
    tx = P.mover.truckX0;
else
    tx = max(P.mover.truckX1, P.mover.truckX0 - P.mover.truckV * (t - tTrig));
end
tk = [tx, 4.2];                                        % pallet truck driving into aisle 2
mv = [h1; h2; a2; tk];

sa = P.mover.size(3,:) / 2;  st = P.mover.size(4,:) / 2;
rects = [P.racks; P.walls; P.boxes; ...
         a2(1)-sa(1), a2(1)+sa(1), a2(2)-sa(2), a2(2)+sa(2); ...
         tk(1)-st(1), tk(1)+st(1), tk(2)-st(2), tk(2)+st(2)];
circs = [h1, P.mover.size(1,1)/2; h2, P.mover.size(2,1)/2];
end

function v = tri(u)
    v = (2/pi) * asin(sin(2*pi*u));      % triangle wave in [-1, 1]
end

function s = crossing(u)
    % 0..1 position: wait at 0, walk to 1, wait, walk back (smooth steps)
    u = u - floor(u);
    if u < 0.30,     s = 0;
    elseif u < 0.50, s = sstep((u - 0.30) / 0.20);
    elseif u < 0.80, s = 1;
    else,            s = 1 - sstep((u - 0.80) / 0.20);
    end
end
function y = sstep(x)
    y = x * x * (3 - 2 * x);
end
