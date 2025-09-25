function [baseWaypointsOut, thetaRefOut, rawBaseWaypointsOut, refPositionsOut, refRPYOut, refTimesOut, rampInfo] = ...
    plan_ramp_segment(baseWaypointsIn, thetaRefIn, rawBaseWaypointsIn, refPositionsIn, refRPYIn, refTimesIn, sampleDt, robot, armJointNames, homeEEPos, homeEERPY, homeBasePose, rampBaseSpeedMax, rampBaseYawRateMax, desiredYawTarget, collisionAvoidanceDefs)
%PLAN_RAMP_SEGMENT Generate arm/chassis warm-up segment preceding tracking phase.
%   Mirrors the legacy prepend_ramp_segment helper, returning augmented
%   waypoints, reference poses, and detailed ramp metadata.

if nargin < 10 || isempty(robot)
    error('prepend_ramp_segment requires robot model input.');
end
if nargin < 11 || isempty(armJointNames)
    error('prepend_ramp_segment requires armJointNames.');
end
if nargin < 12 || isempty(homeEERPY)
    homeEERPY = [0 0 0];
end
if nargin < 13 || isempty(homeBasePose)
    homeBasePose = wbc.compute_home_base_pose();
end
if nargin < 14 || isempty(rampBaseSpeedMax)
    rampBaseSpeedMax = 0.2;
end
if nargin < 15 || isempty(rampBaseYawRateMax)
    rampBaseYawRateMax = 0.5;
end
if nargin < 16 || isempty(desiredYawTarget)
    desiredYawTarget = thetaRefIn(1);
end
if nargin < 17 || isempty(collisionAvoidanceDefs)
    collisionAvoidanceDefs = {};
end

homePose = homeBasePose(:)';
if numel(homePose) ~= 3
    error('Home base pose must be a 1x3 vector [x y yaw].');
end

% Solve IK once for the arm ramp goal (match height/orientation, free XY)
eeNameLocal = 'left_gripper_link';
armGoal = compute_arm_ramp_goal(homeEEPos, refPositionsIn(1,:), refRPYIn, homeEERPY);
armGoal.initialPosition = armGoal.position;
armGoal.initialRPY = armGoal.rpy;
desiredZ = armGoal.position(3);
desiredRPY = armGoal.rpy;

configHome = homeConfiguration(robot);
vectorNames = {configHome.JointName};
armIdxLocal = zeros(1, numel(armJointNames));
for j = 1:numel(armJointNames)
    idx = find(strcmp(vectorNames, armJointNames{j}), 1);
    if isempty(idx)
        error('prepend_ramp_segment:JointMissing', 'Joint %s not present in robot model.', armJointNames{j});
    end
    armIdxLocal(j) = idx;
end

qStart = zeros(1, numel(armIdxLocal));
for j = 1:numel(armIdxLocal)
    qStart(j) = configHome(armIdxLocal(j)).JointPosition;
end

armPhaseSteps = 20;
armSamplePeriod = sampleDt;
if armSamplePeriod <= 0
    armSamplePeriod = 0.1;
end
armTotalTime = (max(armPhaseSteps, 1) - 1) * armSamplePeriod;
timeSamples = linspace(0, armTotalTime, max(armPhaseSteps, 1));

gikSolver = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'cartesian','orientation'});
cartBound = constraintCartesianBounds(eeNameLocal);
cartBound.ReferenceBody = robot.BaseName;
cartBound.Bounds = [-Inf Inf; -Inf Inf; desiredZ desiredZ];
cartBound.Weights = [0.1 0.1 1];
orientTarget = constraintOrientationTarget(eeNameLocal);
orientTarget.ReferenceBody = robot.BaseName;
orientTarget.TargetOrientation = eul2quat(desiredRPY, 'XYZ');

jointBoundsConstraint = constraintJointBounds(robot);
[collisionConstraints, collisionInputs] = instantiate_collision_avoidance(robot, collisionAvoidanceDefs);

constraintObjects = {cartBound, orientTarget, jointBoundsConstraint};
constraintInputs = {'cartesian','orientation','jointbounds'};
if ~isempty(collisionConstraints)
    constraintObjects = [constraintObjects, collisionConstraints]; %#ok<AGROW>
    constraintInputs = [constraintInputs, collisionInputs]; %#ok<AGROW>
end

gikSolver = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', constraintInputs);

[seedConfigs, seedLabels, seedJointVectors] = build_arm_ramp_seed_set(configHome, armIdxLocal, qStart, robot, armJointNames);
[configGoal, qEnd, candidateSolutions, selectedSeedIdx] = solve_arm_ramp_gik(gikSolver, constraintObjects, seedConfigs, armIdxLocal, qStart);

TeeGoal = getTransform(robot, configGoal, eeNameLocal);
virtualPos = TeeGoal(1:3,4)';
firstRPY = rotm2eul(TeeGoal(1:3,1:3), 'XYZ');

armGoal.position = virtualPos;
armGoal.rpy = firstRPY;
armGoal.jointSolution = qEnd;
armGoal.candidateJointSolutions = candidateSolutions;
armGoal.seedSelection = struct('index', selectedSeedIdx, 'labels', {seedLabels});

if armTotalTime <= 0
    armRampJointTraj = repmat(qStart, max(armPhaseSteps, 1), 1);
    armRampJointVel = zeros(size(armRampJointTraj));
    armRampJointAcc = zeros(size(armRampJointTraj));
else
    [qTraj, qdTraj, qddTraj] = quinticpolytraj([qStart; qEnd]', [0 armTotalTime], timeSamples);
    armRampJointTraj = qTraj';
    armRampJointVel = qdTraj';
    armRampJointAcc = qddTraj';
end

targetPose = compute_chassis_ramp_goal(homePose, baseWaypointsIn, thetaRefIn, desiredYawTarget);

posError = norm(targetPose(1:2) - homePose(1:2));
yawError = abs(wrapToPi(targetPose(3) - homePose(3)));

rampInfo = struct('enabled', false, 'duration', 0, 'steps', 0, ...
    'startPose', homePose, 'targetPose', targetPose, 'timeAdded', 0, ...
    'armSteps', 0, 'baseSteps', 0, ...
    'eeStart', homeEEPos, 'eeGoal', virtualPos, ...
    'baseStart', homePose(1:2), 'baseGoal', targetPose(1:2), ...
    'virtualEEPose', armGoal, 'baseSpeedLimit', rampBaseSpeedMax, ...
    'baseYawRateLimit', rampBaseYawRateMax, ...
    'armGoalCandidates', candidateSolutions, 'armSeedLabels', {seedLabels}, ...
    'armSelectedSeed', selectedSeedIdx, 'armSeedVectors', seedJointVectors, ...
    'armJointVelocity', [], 'armJointAcceleration', [], 'armSamplePeriod', armSamplePeriod, ...
    'positionError', posError, 'yawError', yawError, ...
    'planner', struct(), 'plannedPath', [], 'obstacles', []);
rampInfo.collisionAvoidance = collisionAvoidanceDefs;

armPhaseSteps = max(armPhaseSteps, size(armRampJointTraj,1));
armRampJointTraj = resize_array(armRampJointTraj, armPhaseSteps, qStart);
armRampJointVel = resize_array(armRampJointVel, armPhaseSteps, zeros(1,size(armRampJointTraj,2)));
armRampJointAcc = resize_array(armRampJointAcc, armPhaseSteps, zeros(1,size(armRampJointTraj,2)));

rampInfo.armSteps = armPhaseSteps;
armSampleTimes = (0:armPhaseSteps-1)' * armSamplePeriod;
rampInfo.armJointTrajectory = armRampJointTraj;
rampInfo.armJointVelocity = armRampJointVel;
rampInfo.armJointAcceleration = armRampJointAcc;
rampInfo.armTimeVector = armSampleTimes;

plannerOpts = struct('StepSize', 0.05, 'Wheelbase', 0.5, 'MaxSteer', deg2rad(30), ...
    'GoalPosTol', 0.02, 'GoalYawTol', deg2rad(5), 'MaxIterations', 5000, ...
    'FootprintRadius', 0.35, 'MaxYawRate', rampBaseYawRateMax, 'SpeedLimit', rampBaseSpeedMax);

obstacles = [];
if exist('chassis_obstacles', 'file') == 2
    try
        obstacles = chassis_obstacles();
    catch ME
        warning('prepend_ramp_segment:ObstacleConfig', ...
            'Failed to load chassis obstacles (%s). Continuing without obstacles.', ME.message);
    end
end
plannerOpts.Obstacles = obstacles;

[path, plannerInfo] = helpers.hybrid_astar_plan(homePose, targetPose, plannerOpts);
if isempty(path)
    baseInterp = zeros(0, 2);
    thetaInterpRes = zeros(0, 1);
    basePhaseSteps = 0;
    pathLength = 0;
else
    baseInterp = path(:,1:2);
    baseYaw = path(:,3);
    pathLength = sum(sqrt(sum(diff(path(:,1:2)).^2,2)));
    if size(path,1) > 1
        sPath = [0; cumsum(sqrt(sum(diff(path(:,1:2)).^2,2)))];
        totalLen = sPath(end);
        basePhaseSteps = max(1, ceil(totalLen / max(rampBaseSpeedMax * sampleDt, 1e-6)));
        if basePhaseSteps < 2
            basePhaseSteps = 2;
        end
        sRes = linspace(0, totalLen, basePhaseSteps);
        baseInterp = interp1(sPath, path(:,1:2), sRes, 'linear', 'extrap');
        yawPath = unwrap(baseYaw);
        thetaInterpRes = interp1(sPath, yawPath, sRes, 'linear', 'extrap');
    else
        basePhaseSteps = 1;
        thetaInterpRes = wrapToPi(baseYaw);
    end
    thetaInterpRes = wrapToPi(thetaInterpRes(:));
end

rampInfo.enabled = true;
rampInfo.planner = plannerInfo;
rampInfo.plannedPath = path;
rampInfo.obstacles = obstacles;

basePhaseSteps = size(baseInterp,1);
rampInfo.baseSteps = basePhaseSteps;
rampInfo.steps = armPhaseSteps + basePhaseSteps;
rampInfo.duration = rampInfo.steps * sampleDt;
rampInfo.armDuration = armPhaseSteps * sampleDt;
rampInfo.baseDuration = basePhaseSteps * sampleDt;
rampInfo.sampleDt = sampleDt;
rampInfo.basePathLength = pathLength;

baseRamp = zeros(rampInfo.steps, 2);
thetaRamp = zeros(rampInfo.steps, 1);
if armPhaseSteps > 0
    baseRamp(1:armPhaseSteps, :) = repmat(homePose(1:2), armPhaseSteps, 1);
    thetaRamp(1:armPhaseSteps) = homePose(3);
end
if basePhaseSteps > 0
    baseRamp(armPhaseSteps+1:end, :) = baseInterp;
    thetaRamp(armPhaseSteps+1:end) = linspace(homePose(3), targetPose(3), basePhaseSteps)';
end

if rampInfo.steps > 0
    rampEndPos = baseRamp(end, :);
else
    rampEndPos = homePose(1:2);
end

refTimesRamp = refTimesIn(1) + (-rampInfo.steps:-1)' * sampleDt;

refPositionsRamp = zeros(rampInfo.steps, size(refPositionsIn,2));
refRPYRamp = zeros(rampInfo.steps, size(refRPYIn,2));

if armPhaseSteps > 0
    configFK = homeConfiguration(robot);
    for k = 1:armPhaseSteps
        for j = 1:numel(armIdxLocal)
            configFK(armIdxLocal(j)).JointPosition = armRampJointTraj(k, j);
        end
        TeeSample = getTransform(robot, configFK, eeNameLocal);
        refPositionsRamp(k, :) = TeeSample(1:3,4)';
        if size(refRPYIn,2) >= 3
            refRPYRamp(k, :) = rotm2eul(TeeSample(1:3,1:3), 'XYZ');
        end
    end
end

if basePhaseSteps > 0
    configGoal = homeConfiguration(robot);
    for j = 1:numel(armIdxLocal)
        configGoal(armIdxLocal(j)).JointPosition = armRampJointTraj(end, j);
    end
    TeeGoalRel = getTransform(robot, configGoal, eeNameLocal);
    for k = 1:basePhaseSteps
        Tbase = trvec2tform([baseInterp(k,:), 0]) * axang2tform([0 0 1 thetaInterpRes(k)]);
        TeeWorld = Tbase * TeeGoalRel;
        refPositionsRamp(armPhaseSteps + k, :) = TeeWorld(1:3,4)';
        if size(refRPYIn,2) >= 3
            refRPYRamp(armPhaseSteps + k, :) = rotm2eul(TeeWorld(1:3,1:3), 'XYZ');
        end
    end
end

if rampInfo.steps == 0
    refPositionsRamp = zeros(0, size(refPositionsIn,2));
    refRPYRamp = zeros(0, size(refRPYIn,2));
end

trackWaypoints = baseWaypointsIn;
trackRaw = rawBaseWaypointsIn;

if isempty(trackWaypoints)
    baseWaypointsOut = baseRamp;
    thetaRefOut = thetaRamp;
    rawBaseWaypointsOut = baseRamp;
    refPositionsOut = [refPositionsRamp; refPositionsIn];
    refRPYOut = [refRPYRamp; refRPYIn];
    refTimesOut = [refTimesRamp; refTimesIn];
    rampInfo.timeAdded = rampInfo.duration;
    return;
end

baseShift = rampEndPos - trackWaypoints(1,:);
baseWaypointsAligned = trackWaypoints + baseShift;
rawBaseWaypointsAligned = trackRaw + baseShift;

thetaRefOut = [thetaRamp; thetaRefIn];
baseWaypointsOut = [baseRamp; baseWaypointsAligned];
rawBaseWaypointsOut = [baseRamp; rawBaseWaypointsAligned];
refPositionsOut = [refPositionsRamp; refPositionsIn];
refRPYOut = [refRPYRamp; refRPYIn];
refTimesOut = [refTimesRamp; refTimesIn];

rampInfo.timeAdded = rampInfo.duration;
end

%% ------------------------------------------------------------------------
function armGoal = compute_arm_ramp_goal(homeEEPos, firstEEPos, refRPYIn, homeEERPY)
armGoal = struct();
armGoal.position = [homeEEPos(1:2), firstEEPos(3)];
if isempty(refRPYIn)
    armGoal.rpy = homeEERPY;
else
    armGoal.rpy = refRPYIn(1,:);
end
armGoal.homeRPY = homeEERPY;
end

function targetPose = compute_chassis_ramp_goal(homePose, baseWaypointsIn, thetaRefIn, desiredYaw)
if isempty(baseWaypointsIn)
    targetPose = homePose;
else
    if nargin >= 4 && ~isempty(desiredYaw)
        yawVal = desiredYaw;
    elseif ~isempty(thetaRefIn)
        yawVal = thetaRefIn(1);
    else
        yawVal = homePose(3);
    end
    targetPose = [baseWaypointsIn(1,:), yawVal];
end
end

function arrOut = resize_array(arrIn, targetRows, fillRow)
if size(arrIn,1) == targetRows
    arrOut = arrIn;
elseif isempty(arrIn)
    arrOut = repmat(fillRow, targetRows, 1);
else
    arrOut = arrIn;
    arrOut(end+1:targetRows, :) = fillRow; %#ok<AGROW>
end
end

function [seedConfigs, seedLabels, seedJointVectors] = build_arm_ramp_seed_set(configTemplate, armIdxLocal, qHome, robot, armJointNames)
numJoints = numel(armIdxLocal);
seedConfigs = {};
seedLabels = {};
seedJointVectors = zeros(0, numJoints);

jointLimits = get_joint_position_limits(robot, armJointNames);

add_seed(qHome, 'home');

defaultAmplitude = deg2rad(40);
for j = 1:numJoints
    bounds = jointLimits(j,:);
    amplitude = defaultAmplitude;
    lowerBound = bounds(1);
    upperBound = bounds(2);
    if isfinite(lowerBound) && isfinite(upperBound) && upperBound > lowerBound
        amplitude = min(amplitude, 0.45 * (upperBound - lowerBound));
    end
    if amplitude < 1e-4
        continue;
    end

    lowerVec = qHome;
    lowerVec(j) = clamp_joint_value(qHome(j) - amplitude, bounds);
    if abs(lowerVec(j) - qHome(j)) > 1e-5
        add_seed(lowerVec, sprintf('%s_lower', armJointNames{j}));
    end

    upperVec = qHome;
    upperVec(j) = clamp_joint_value(qHome(j) + amplitude, bounds);
    if abs(upperVec(j) - qHome(j)) > 1e-5
        add_seed(upperVec, sprintf('%s_upper', armJointNames{j}));
    end
end

pairCandidates = [2 3; 2 4; 3 4];
pairOffset = deg2rad(30);
for row = 1:size(pairCandidates, 1)
    idxPair = pairCandidates(row, :);
    if any(idxPair > numJoints)
        continue;
    end
    for signVal = [-1, 1]
        pairVec = qHome;
        pairVec(idxPair(1)) = clamp_joint_value(qHome(idxPair(1)) + signVal * pairOffset, jointLimits(idxPair(1), :));
        pairVec(idxPair(2)) = clamp_joint_value(qHome(idxPair(2)) - signVal * pairOffset, jointLimits(idxPair(2), :));
        if any(~isfinite(pairVec(idxPair)))
            continue;
        end
        if norm(pairVec - qHome) > 1e-4
            label = sprintf('combo_%s_%s', armJointNames{idxPair(1)}, armJointNames{idxPair(2)});
            add_seed(pairVec, label);
        end
    end
end

if isempty(seedConfigs)
    seedConfigs = {set_config_positions(configTemplate, armIdxLocal, qHome)};
    seedLabels = {'home'};
    seedJointVectors = qHome;
end

    function add_seed(qVec, label)
        if any(~isfinite(qVec))
            return;
        end
        if ~isempty(seedJointVectors)
            diffs = seedJointVectors - qVec;
            if any(vecnorm(diffs, 2, 2) < 1e-5)
                return;
            end
        end
        seedJointVectors(end+1, :) = qVec; %#ok<AGROW>
        seedConfigs{end+1} = set_config_positions(configTemplate, armIdxLocal, qVec); %#ok<AGROW>
        seedLabels{end+1} = label; %#ok<AGROW>
    end
end

function value = clamp_joint_value(value, bounds)
lowerBound = bounds(1);
upperBound = bounds(2);
if ~isfinite(lowerBound)
    lowerBound = -inf;
end
if ~isfinite(upperBound)
    upperBound = inf;
end
if lowerBound > upperBound
    lowerBound = upperBound;
end
value = min(max(value, lowerBound), upperBound);
end

function limits = get_joint_position_limits(robot, armJointNames)
limits = nan(numel(armJointNames), 2);
for bodyIdx = 1:numel(robot.Bodies)
    jointObj = robot.Bodies{bodyIdx}.Joint;
    matchIdx = find(strcmp(jointObj.Name, armJointNames), 1);
    if ~isempty(matchIdx)
        limits(matchIdx, :) = jointObj.PositionLimits;
    end
end
for idx = 1:size(limits, 1)
    if any(isnan(limits(idx, :)))
        limits(idx, :) = [-inf, inf];
    end
end
end

function configOut = set_config_positions(configTemplate, armIdxLocal, qVec)
configOut = configTemplate;
for idx = 1:numel(armIdxLocal)
    configOut(armIdxLocal(idx)).JointPosition = qVec(idx);
end
end

function [configBest, qBest, qCandidates, seedSelection] = solve_arm_ramp_gik(gikSolver, constraintObjects, seedConfigs, armIdxLocal, qStart)
configs = cell(0, 1);
qCandidates = zeros(0, numel(armIdxLocal));
seedIndices = zeros(0, 1);

for sIdx = 1:numel(seedConfigs)
    try
        solStruct = gikSolver(seedConfigs{sIdx}, constraintObjects{:});
    catch
        continue;
    end
    if isempty(solStruct)
        continue;
    end
    qVec = zeros(1, numel(armIdxLocal));
    valid = true;
    for j = 1:numel(armIdxLocal)
        posVal = solStruct(armIdxLocal(j)).JointPosition;
        if ~isfinite(posVal)
            valid = false;
            break;
        end
        qVec(j) = posVal;
    end
    if ~valid
        continue;
    end
    if ~isempty(qCandidates)
        if any(vecnorm(qCandidates - qVec, 2, 2) < 1e-5)
            continue;
        end
    end
    configs{end+1} = solStruct; %#ok<AGROW>
    qCandidates(end+1, :) = qVec; %#ok<AGROW>
    seedIndices(end+1, 1) = sIdx; %#ok<AGROW>
end

if isempty(configs)
    error('prepend_ramp_segment:IKFailure', 'Failed to compute an arm warm-up IK solution.');
end

if size(qCandidates, 1) > 1
    deltas = qCandidates - qStart;
    distances = vecnorm(deltas, 2, 2);
    [~, bestIdx] = min(distances);
else
    bestIdx = 1;
end

configBest = configs{bestIdx};
qBest = qCandidates(bestIdx, :);
seedSelection = seedIndices(bestIdx);
end

function [constraints, inputNames] = instantiate_collision_avoidance(robot, entries)
constraints = {};
inputNames = {};

if exist('constraintCollisionAvoidance', 'class') ~= 8 && exist('constraintCollisionAvoidance', 'file') ~= 2
    return;
end

if nargin < 2 || isempty(entries)
    return;
end

if ~iscell(entries)
    entries = {entries};
end

for idx = 1:numel(entries)
    entry = entries{idx};
    if isa(entry, 'constraintCollisionAvoidance')
        collObj = entry;
        collObj.SelfCollisions = true;
    elseif isstruct(entry)
        collObj = constraintCollisionAvoidance(robot);
        if isfield(entry, 'Environment') && ~isempty(entry.Environment)
            collObj.Environment = entry.Environment;
        end
        if isfield(entry, 'CollisionPairs') && ~isempty(entry.CollisionPairs)
            collObj.CollisionPairs = entry.CollisionPairs;
        end
        if isfield(entry, 'Weights') && ~isempty(entry.Weights)
            collObj.Weights = entry.Weights;
        end
        if isfield(entry, 'NumSamples') && ~isempty(entry.NumSamples)
            collObj.NumSamples = entry.NumSamples;
        end
        if isfield(entry, 'SelfCollisions') && ~isempty(entry.SelfCollisions)
            collObj.SelfCollisions = logical(entry.SelfCollisions);
        else
            collObj.SelfCollisions = true;
        end
    else
        error('prepend_ramp_segment:InvalidCollisionEntry', ...
            'Unsupported CollisionAvoidance entry of type %s.', class(entry));
    end

    constraints{end+1} = collObj; %#ok<AGROW>
    inputNames{end+1} = 'collisionavoidance'; %#ok<AGROW>
end
end
