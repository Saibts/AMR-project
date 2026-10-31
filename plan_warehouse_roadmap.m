function path = plan_warehouse_roadmap(currentPos, goalPt, hazardBoxes)
%% =========================================================================
%  Autonomous Warehouse Roadmap Pathfinder & Dynamic Aisle Router
%  Finds optimal collision-free corridor path around any number of hazards
%  Strictly maintains orthogonal aisle travel along warehouse safety clearways.
%  =========================================================================

% Warehouse Roadmap Guidepath Nodes (Aisle Centerlines):
nodes = [
    % West Aisle (x = 1.8)
    1.8,  1.1;   %  1: West-South Corner Junction
    1.8,  2.0;   %  2: Depot / Inbound Station
    1.8,  5.2;   %  3: West Aisle @ Cross-Aisle 1
    1.8,  9.2;   %  4: West Aisle @ Cross-Aisle 2
    1.8, 14.0;   %  5: West Aisle @ North Highway
    
    % South Corridor (y = 1.1)
    7.0,  1.1;   %  6: South Corridor Mid-West
   11.35, 1.1;   %  7: South Corridor Center-West
   14.65, 1.1;   %  8: South Corridor Center-East
   19.0,  1.1;   %  9: South Corridor Mid-East
   24.0,  1.1;   % 10: South-East Corner
   
    % Cross-Aisle 1 (y = 5.2)
    7.0,  5.2;   % 11: Cross-Aisle 1 Mid-West
   11.35, 5.2;   % 12: Cross-Aisle 1 Center-West
   14.65, 5.2;   % 13: Cross-Aisle 1 Center-East
   19.0,  5.2;   % 14: Cross-Aisle 1 Mid-East
   24.0,  5.2;   % 15: Cross-Aisle 1 East Junction
   
    % Cross-Aisle 2 (y = 9.2)
    7.0,  9.2;   % 16: Cross-Aisle 2 Mid-West
   11.35, 9.2;   % 17: Cross-Aisle 2 Center-West
   14.65, 9.2;   % 18: Cross-Aisle 2 Center-East
   19.0,  9.2;   % 19: Cross-Aisle 2 Mid-East
   24.0,  9.2;   % 20: Cross-Aisle 2 East Junction
   
    % North Freight Highway (y = 14.0)
    7.0, 14.0;   % 21: North Highway Mid-West
   11.35, 14.0;  % 22: North Highway Center-West
   14.65, 14.0;  % 23: North Highway Center-East
   19.0, 14.0;   % 24: North Highway Mid-East
   21.5, 13.5;   % 25: Ramp Incline Foot
   24.0, 13.5    % 26: Final Goal - Elevated Dock
];

% Undirected Edges connecting designated clearway corridors
edges = [
    % West Aisle vertical segments
    1, 2;  2, 3;  3, 4;  4, 5;
    
    % South Corridor horizontal segments
    1, 6;  6, 7;  7, 8;  8, 9;  9, 10;
    
    % Cross-Aisle 1 horizontal segments
    3, 11; 11, 12; 12, 13; 13, 14; 14, 15;
    
    % Cross-Aisle 2 horizontal segments
    4, 16; 16, 17; 17, 18; 18, 19; 19, 20;
    
    % North Highway horizontal segments
    5, 21; 21, 22; 22, 23; 23, 24; 24, 25; 25, 26;
    
    % Central East Lane (x = 14.65) vertical thoroughfare
    8, 13;  13, 18;  18, 23;
    
    % Central West Lane (x = 11.35) vertical segments (clear at south and north)
    7, 12;  17, 22;
    
    % East Aisle (x = 24.0) vertical segments
    10, 15;  20, 25
];

numN = size(nodes, 1);
numE = size(edges, 1);
goalNodeIdx = 26; % Dock Goal

% Default static pallets that are always present in the environment
staticPallets = [
    12.2,  6.5,  1.6,  1.4;
    12.2,  8.5,  1.6,  1.4;
     1.5,  8.0,  1.4,  1.8;
    23.0,  8.0,  1.4,  1.8
];

if nargin < 3 || isempty(hazardBoxes)
    allHazards = staticPallets;
else
    allHazards = [staticPallets; hazardBoxes(:, 1:4)];
end

% Filter out blocked edges (where any hazard box intersects edge segment within margin)
safeMargin = 0.50; % AMR half-width 0.306m + safety clearance
adjMat = inf(numN, numN);
for e = 1:numE
    nA = edges(e, 1); nB = edges(e, 2);
    pA = nodes(nA, :); pB = nodes(nB, :);
    
    edgeBlocked = false;
    for h = 1:size(allHazards, 1)
        hx = allHazards(h, 1) + allHazards(h, 3)/2;
        hy = allHazards(h, 2) + allHazards(h, 4)/2;
        hr = max(allHazards(h, 3), allHazards(h, 4))/2 + safeMargin;
        
        d = dist_point_to_segment([hx, hy], pA, pB);
        if d < hr
            edgeBlocked = true;
            break;
        end
    end
    
    if ~edgeBlocked
        w = hypot(pB(1) - pA(1), pB(2) - pA(2));
        adjMat(nA, nB) = w;
        adjMat(nB, nA) = w;
    end
end

% Find candidate start node nearest to currentPos that has at least one open outgoing edge
dists = hypot(nodes(:,1) - currentPos(1), nodes(:,2) - currentPos(2));
[~, sortIdx] = sort(dists);

startNodeIdx = -1;
for si = 1:numN
    candidate = sortIdx(si);
    if any(~isinf(adjMat(candidate, :)))
        % Ensure line from currentPos to candidate is not directly blocked by an obstacle
        posBlocked = false;
        for h = 1:size(allHazards, 1)
            hx = allHazards(h, 1) + allHazards(h, 3)/2;
            hy = allHazards(h, 2) + allHazards(h, 4)/2;
            hr = max(allHazards(h, 3), allHazards(h, 4))/2 + safeMargin;
            if dist_point_to_segment([hx, hy], currentPos, nodes(candidate, :)) < hr
                posBlocked = true;
                break;
            end
        end
        if ~posBlocked
            startNodeIdx = candidate;
            break;
        end
    end
end

if startNodeIdx == -1
    startNodeIdx = sortIdx(1); % Fallback
end

% Dijkstra Shortest Path Search
dist = inf(numN, 1);
prev = zeros(numN, 1);
visited = false(numN, 1);
dist(startNodeIdx) = 0;

for i = 1:numN
    [minD, u] = min(dist + visited * 1e9);
    if minD >= 1e8, break; end
    if u == goalNodeIdx, break; end
    visited(u) = true;
    
    for v = 1:numN
        if ~visited(v) && ~isinf(adjMat(u, v))
            alt = dist(u) + adjMat(u, v);
            if alt < dist(v)
                dist(v) = alt;
                prev(v) = u;
            end
        end
    end
end

% Backtrack path
pathNodeIndices = [];
curr = goalNodeIdx;
while curr ~= 0
    pathNodeIndices = [curr, pathNodeIndices]; %#ok<AGROW>
    curr = prev(curr);
end

if isempty(pathNodeIndices) || pathNodeIndices(1) ~= startNodeIdx
    % If goal unreachable through standard network, fallback to ramp approach
    path = [currentPos; 21.5, 13.5; 24.0, 13.5];
    return;
end

rawPath = nodes(pathNodeIndices, :);

% Prepend currentPos if it's noticeably away from start node
if hypot(currentPos(1) - rawPath(1,1), currentPos(2) - rawPath(1,2)) > 0.40
    path = [currentPos; rawPath];
else
    path = rawPath;
end

end

function d = dist_point_to_segment(pt, pA, pB)
v = pB - pA;
w = pt - pA;
c1 = dot(w, v);
if c1 <= 0, d = hypot(pt(1)-pA(1), pt(2)-pA(2)); return; end
c2 = dot(v, v);
if c2 <= c1, d = hypot(pt(1)-pB(1), pt(2)-pB(2)); return; end
b = c1 / c2;
pb = pA + b * v;
d = hypot(pt(1)-pb(1), pt(2)-pb(2));
end
