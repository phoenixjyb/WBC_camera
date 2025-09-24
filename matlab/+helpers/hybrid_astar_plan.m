function path = hybrid_astar_plan(startPose, goalPose, opts)
%HYBRID_ASTAR_PLAN Plan a SE(2) path using a minimal Hybrid A* search.
%   path = helpers.hybrid_astar_plan(startPose, goalPose, opts)
%   startPose, goalPose: [x y yaw]. opts (optional struct) can contain: 
%       StepSize, Wheelbase, MaxSteer, GoalPosTol, GoalYawTol, MaxIterations,
%       Obstacles, FootprintRadius.

if nargin < 3
    opts = struct();
end

step = getOpt(opts, 'StepSize', 0.05);
wb = getOpt(opts, 'Wheelbase', 0.5);
maxSteer = getOpt(opts, 'MaxSteer', deg2rad(30));
goalTolXY = getOpt(opts, 'GoalPosTol', 0.02);
goalTolYaw = getOpt(opts, 'GoalYawTol', deg2rad(5));
maxIter = getOpt(opts, 'MaxIterations', 5000);
footprintRadius = max(getOpt(opts, 'FootprintRadius', 0.35), 0.01);
obstacles = getOpt(opts, 'Obstacles', []);

steerSet = [-maxSteer, 0, maxSteer];
directions = [1, -1];

nodes = struct('pose', startPose, 'g', 0, 'parent', 0, 'idx', 1);
fCost = heuristic(startPose, goalPose);
openIdx = 1;
closedSet = containers.Map('KeyType','char','ValueType','double');
nodeCount = 1;

if isColliding(startPose, footprintRadius, obstacles) || ...
        isColliding(goalPose, footprintRadius, obstacles)
    path = [];
    return;
end

for iter = 1:maxIter
    if isempty(openIdx)
        break;
    end
    [~, bestPos] = min(fCost(openIdx));
    currentIdx = openIdx(bestPos);
    openIdx(bestPos) = [];
    current = nodes(currentIdx);

    key = nodeKey(current.pose);
    if isKey(closedSet, key)
        continue;
    end
    closedSet(key) = current.g;

    if reachedGoal(current.pose, goalPose, goalTolXY, goalTolYaw)
        path = backtrack(nodes, currentIdx);
        return;
    end

    for dir = directions
        for steer = steerSet
            [nextPose, cost] = propagate(current.pose, dir, steer, step, wb);
            if isempty(nextPose)
                continue;
            end
            if isColliding(nextPose, footprintRadius, obstacles)
                continue;
            end
            nextKey = nodeKey(nextPose);
            if isKey(closedSet, nextKey)
                continue;
            end
            nodeCount = nodeCount + 1;
            nodes(nodeCount).pose = nextPose;
            nodes(nodeCount).g = current.g + cost;
            nodes(nodeCount).parent = currentIdx;
            nodes(nodeCount).idx = nodeCount;
            fCost(nodeCount) = nodes(nodeCount).g + heuristic(nextPose, goalPose); %#ok<AGROW>
            openIdx(end+1) = nodeCount; %#ok<AGROW>
        end
    end
end

path = [];

    function val = getOpt(s, field, defaultVal)
        if isstruct(s) && isfield(s, field)
            val = s.(field);
        else
            val = defaultVal;
        end
    end

    function pathOut = backtrack(nodeList, idx)
        poses = nodeList(idx).pose;
        while nodeList(idx).parent ~= 0
            idx = nodeList(idx).parent;
            poses = [nodeList(idx).pose; poses]; %#ok<AGROW>
        end
        pathOut = poses;
    end

    function h = heuristic(pose, goal)
        h = hypot(goal(1) - pose(1), goal(2) - pose(2)) + 0.1 * abs(wrapToPi(goal(3) - pose(3)));
    end

    function key = nodeKey(pose)
        resXY = 0.05;
        resYaw = deg2rad(5);
        key = sprintf('%d_%d_%d', round(pose(1)/resXY), round(pose(2)/resXY), round(wrapToPi(pose(3))/resYaw));
    end

    function tf = reachedGoal(pose, goal, tolXY, tolYaw)
        tf = hypot(goal(1) - pose(1), goal(2) - pose(2)) <= tolXY && abs(wrapToPi(goal(3) - pose(3))) <= tolYaw;
    end

    function tf = isColliding(pose, radius, obsList)
        if isempty(obsList)
            tf = false;
            return;
        end
        tf = false;
        for obsIdx = 1:numel(obsList)
            obs = obsList(obsIdx);
            if isfield(obs, 'type')
                type = obs.type;
            else
                type = 'circle';
            end
            switch lower(type)
                case {'circle','disc','disk'}
                    center = obs.center;
                    rad = obs.radius;
                    tf = hypot(pose(1) - center(1), pose(2) - center(2)) <= (rad + radius);
                case {'rectangle','box','aabb'}
                    bounds = obs.bounds; % [xmin xmax ymin ymax]
                    inflate = radius;
                    tf = pose(1) >= (bounds(1) - inflate) && pose(1) <= (bounds(2) + inflate) && ...
                         pose(2) >= (bounds(3) - inflate) && pose(2) <= (bounds(4) + inflate);
                otherwise
                    tf = false;
            end
            if tf
                return;
            end
        end
    end

    function [poseOut, cost] = propagate(poseIn, dir, steer, ds, wbVal)
        cost = abs(ds) * (1 + 0.1 * abs(steer));
        if abs(steer) < 1e-6
            dx = dir * ds * cos(poseIn(3));
            dy = dir * ds * sin(poseIn(3));
            dtheta = 0;
        else
            turnRadius = wbVal / tan(steer);
            dtheta = dir * ds / turnRadius;
            dx = dir * turnRadius * (sin(poseIn(3) + dtheta) - sin(poseIn(3)));
            dy = dir * turnRadius * (-cos(poseIn(3) + dtheta) + cos(poseIn(3)));
        end
        poseOut = poseIn + [dx, dy, dtheta];
        poseOut(3) = wrapToPi(poseOut(3));
    end
end
