function [path, info] = hybrid_astar_plan(startPose, goalPose, opts)
%HYBRID_ASTAR_PLAN Plan a SE(2) path for the chassis warm-up stage.
%   [PATH, INFO] = HELPERS.HYBRID_ASTAR_PLAN(STARTPOSE, GOALPOSE, OPTS)
%   attempts to connect the provided poses using a Hybrid A* style search.
%   When Robotics System Toolbox path planners are available, an optimized
%   plannerHybridAStar instance is used; otherwise the function falls back
%   to a lightweight discrete search. INFO returns diagnostics describing
%   whichever backend produced the result.

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
disableToolbox = getOpt(opts, 'DisableToolbox', false);

infoToolbox = struct('success', false, 'status', "toolbox_not_attempted", ...
    'reason', "", 'pathLength', 0);
path = [];

if ~disableToolbox
    [pathTool, infoToolbox] = tryPlannerHybridAStar(startPose, goalPose, ...
        footprintRadius, obstacles, step, wb, maxSteer, opts);
    if infoToolbox.success
        path = pathTool;
        if nargout > 1
            info = infoToolbox;
        end
        return;
    end
end

[pathFallback, infoSearch] = fallbackSearch(startPose, goalPose, step, wb, ...
    maxSteer, goalTolXY, goalTolYaw, maxIter, footprintRadius, obstacles);
path = pathFallback;

if nargout > 1
    info = infoSearch;
    info.toolboxAttempt = infoToolbox;
end

end

%% ------------------------------------------------------------------------
function [pathOut, info] = tryPlannerHybridAStar(startPose, goalPose, ...
    footprintRadius, obstacles, step, wb, maxSteer, opts)

info = struct('success', false, 'status', "toolbox_unavailable", ...
    'reason', "no_planner", 'pathLength', 0);

if exist('plannerHybridAStar', 'class') ~= 8 || exist('stateSpaceSE2', 'class') ~= 8
    return;
end
if exist('binaryOccupancyMap', 'class') ~= 8 || exist('validatorOccupancyMap', 'class') ~= 8
    return;
end

try
    res = getOpt(opts, 'GridResolution', min(0.05, step));
    res = max(res, 1e-3);
    pad = max(footprintRadius + 0.1, 0.5);

    [bounds, occMask] = buildOccupancyMask(startPose, goalPose, obstacles, ...
        footprintRadius, res, pad);

    map = binaryOccupancyMap(occMask, 1/res);
    map.LocalOriginInWorld = [bounds.xMin, bounds.yMin];

    ss = stateSpaceSE2;
    ss.StateBounds = [bounds.xMin bounds.xMax; bounds.yMin bounds.yMax; -pi pi];
    sv = validatorOccupancyMap(ss, 'Map', map);
    sv.ValidationDistance = max(step, res);

    turnRadius = max(wb / max(tan(maxSteer), 1e-3), 0.1);
    primitiveLength = max(step, res * 2);

    planner = plannerHybridAStar(sv, 'MinTurningRadius', turnRadius, ...
        'MotionPrimitiveLength', primitiveLength);

    if any(checkOccupancy(map, startPose(1:2))) || any(checkOccupancy(map, goalPose(1:2)))
        info.status = "toolbox_start_or_goal_blocked";
        info.reason = "start_goal_in_collision";
        pathOut = [];
        return;
    end

    [refPath, solInfo] = plan(planner, startPose(:)', goalPose(:)');
    info.raw = solInfo;
    if isfield(solInfo, 'IsPathFound') && ~solInfo.IsPathFound
        info.status = "toolbox_no_path";
        info.reason = "planner_failed";
        pathOut = [];
        return;
    end
    states = refPath.States;
    if isempty(states)
        info.status = "toolbox_empty";
        info.reason = "planner_returned_empty";
        pathOut = [];
        return;
    end
    pathOut = states;
    info.success = true;
    info.status = "toolbox";
    info.reason = "plannerHybridAStar";
    info.pathLength = size(pathOut, 1);
catch ME
    info.status = "toolbox_exception";
    info.reason = ME.identifier;
    info.exception = ME;
    pathOut = [];
end
end

function [bounds, occ] = buildOccupancyMask(startPose, goalPose, obstacles, ...
    footprintRadius, res, pad)

if isempty(obstacles)
    obsCenters = zeros(0,2);
    obsRadius = zeros(0,1);
else
    obsCenters = reshape([obstacles.center], 2, []).';
    radField = arrayfun(@(o) getfieldwithdefault(o, 'radius', 0), obstacles);
    obsRadius = radField(:);
end

xVals = [startPose(1); goalPose(1); obsCenters(:,1)];
yVals = [startPose(2); goalPose(2); obsCenters(:,2)];
if isempty(xVals)
    xVals = [startPose(1); goalPose(1)];
    yVals = [startPose(2); goalPose(2)];
end

if isempty(obsRadius)
    maxObsRad = 0;
else
    maxObsRad = max(obsRadius);
end

xMin = min(xVals) - (maxObsRad + footprintRadius + pad);
xMax = max(xVals) + (maxObsRad + footprintRadius + pad);
yMin = min(yVals) - (maxObsRad + footprintRadius + pad);
yMax = max(yVals) + (maxObsRad + footprintRadius + pad);

if ~isfinite(xMin) || ~isfinite(xMax)
    xMin = min([startPose(1), goalPose(1)]) - pad;
    xMax = max([startPose(1), goalPose(1)]) + pad;
end
if ~isfinite(yMin) || ~isfinite(yMax)
    yMin = min([startPose(2), goalPose(2)]) - pad;
    yMax = max([startPose(2), goalPose(2)]) + pad;
end

cols = max(1, ceil((xMax - xMin) / res));
rows = max(1, ceil((yMax - yMin) / res));
xGrid = xMin + ((0:cols-1) + 0.5) * res;
yGrid = yMin + ((0:rows-1) + 0.5) * res;
[XX, YY] = meshgrid(xGrid, yGrid);
occ = false(rows, cols);

inflateR = footprintRadius;
for idx = 1:numel(obstacles)
    obs = obstacles(idx);
    if ~isfield(obs, 'type') || isempty(obs.type)
        obs.type = 'circle';
    end
    switch lower(obs.type)
        case {'circle','disc','disk'}
            center = obs.center;
            radius = getfieldwithdefault(obs, 'radius', 0);
            mask = (XX - center(1)).^2 + (YY - center(2)).^2 <= (radius + inflateR)^2;
            occ = occ | mask;
        case {'rectangle','box','aabb'}
            boundsObs = obs.bounds; % [xmin xmax ymin ymax]
            inflate = inflateR;
            mask = XX >= (boundsObs(1) - inflate) & XX <= (boundsObs(2) + inflate) & ...
                   YY >= (boundsObs(3) - inflate) & YY <= (boundsObs(4) + inflate);
            occ = occ | mask;
        otherwise
            % ignore unknown types
    end
end

bounds = struct('xMin', xMin, 'xMax', xMin + cols * res, ...
                'yMin', yMin, 'yMax', yMin + rows * res);
end

%% ------------------------------------------------------------------------
function [path, info] = fallbackSearch(startPose, goalPose, step, wb, maxSteer, ...
    goalTolXY, goalTolYaw, maxIter, footprintRadius, obstacles)

steerSet = [-maxSteer, 0, maxSteer];
directions = [1, -1];

nodes = struct('pose', startPose, 'g', 0, 'parent', 0, 'idx', 1);
fCost = heuristic(startPose, goalPose);
openIdx = 1;
closedSet = containers.Map('KeyType', 'char', 'ValueType', 'double');
nodeCount = 1;

info = struct('success', false, 'status', "search_exhausted", 'iterations', 0, ...
    'expanded', 0, 'queued', 0, 'maxOpen', 1, 'reason', "open_list_empty", ...
    'pathLength', 0);

if isColliding(startPose, footprintRadius, obstacles) || ...
        isColliding(goalPose, footprintRadius, obstacles)
    path = [];
    info.status = "invalid_pose";
    info.reason = "start_or_goal_in_collision";
    return;
end

for iter = 1:maxIter
    info.iterations = iter - 1;
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
        info.success = true;
        info.status = "goal_reached";
        info.reason = "goal_satisfied";
        info.pathLength = size(path, 1);
        info.iterations = iter;
        info.maxOpen = max(info.maxOpen, numel(openIdx));
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
            info.queued = info.queued + 1;
            info.maxOpen = max(info.maxOpen, numel(openIdx));
        end
    end
    info.expanded = info.expanded + 1;
end

if isempty(openIdx)
    info.status = "search_exhausted";
    info.reason = "open_list_empty";
else
    info.status = "max_iter";
    info.reason = "iteration_limit";
end
info.maxOpen = max(info.maxOpen, numel(openIdx));
path = [];
end

%% ------------------------------------------------------------------------
function val = getOpt(s, field, defaultVal)
if isstruct(s) && isfield(s, field)
    val = s.(field);
else
    val = defaultVal;
end
end

function val = getfieldwithdefault(s, field, defaultVal)
if isstruct(s) && isfield(s, field) && ~isempty(s.(field))
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
tf = hypot(goal(1) - pose(1), goal(2) - pose(2)) <= tolXY && ...
     abs(wrapToPi(goal(3) - pose(3))) <= tolYaw;
end

function tf = isColliding(pose, radius, obsList)
if isempty(obsList)
    tf = false;
    return;
end
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
            rad = getfieldwithdefault(obs, 'radius', 0);
            if hypot(pose(1) - center(1), pose(2) - center(2)) <= (rad + radius)
                tf = true;
                return;
            end
        case {'rectangle','box','aabb'}
            bounds = obs.bounds; % [xmin xmax ymin ymax]
            inflate = radius;
            if pose(1) >= (bounds(1) - inflate) && pose(1) <= (bounds(2) + inflate) && ...
               pose(2) >= (bounds(3) - inflate) && pose(2) <= (bounds(4) + inflate)
                tf = true;
                return;
            end
        otherwise
            % ignore unknown types
    end
end
tf = false;
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
