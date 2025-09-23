% rt_whole_body_controller.m
% Robotics System Toolbox-based whole-body controller prototype.
% Loads a reference trajectory, synchronizes the chassis and arm on a
% shared timeline, respects velocity limits, and optionally records an
% animation.

clc;

%% Parse external overrides (set from workspace or batch invocations)
if exist('traj_source', 'var') && ~isempty(traj_source)
    trajSourceLocal = string(traj_source);
else
    trajSourceLocal = "demo_arc";
end

if exist('traj_duration', 'var') && ~isempty(traj_duration)
    trajDurationLocal = traj_duration;
else
    trajDurationLocal = 8.0;
end

if exist('traj_scale', 'var') && ~isempty(traj_scale)
    trajScaleLocal = traj_scale;
else
    trajScaleLocal = 1.0;
end

if exist('traj_file', 'var') && ~isempty(traj_file)
    trajFileLocal = string(traj_file);
else
    trajFileLocal = "";
end

if exist('enable_animation', 'var') && ~isempty(enable_animation)
    animateRobot = logical(enable_animation);
else
    animateRobot = true;
end

if exist('animation_video_file', 'var') && ~isempty(animation_video_file)
    videoFile = string(animation_video_file);
else
    videoFile = "";
end

if exist('animation_frame_rate', 'var') && ~isempty(animation_frame_rate)
    videoFrameRate = animation_frame_rate;
else
    videoFrameRate = 30;
end

if exist('animation_playback_speed', 'var') && ~isempty(animation_playback_speed)
    playbackSpeed = animation_playback_speed;
else
    playbackSpeed = 2.0;
end

if exist('output_dir', 'var') && ~isempty(output_dir)
    outputDir = string(output_dir);
else
    outputDir = string(fullfile(pwd, 'outputs'));
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

if exist('use_gik', 'var') && ~isempty(use_gik)
    useGeneralizedIK = logical(use_gik);
else
    useGeneralizedIK = false;
end

gikOptions = struct('PositionTolerance', 0.005, ...
                    'OrientationTolerance', deg2rad(3), ...
                    'ExtraConstraints', {{}}, ...
                    'UseWarmStart', true, ...
                    'DistanceConstraints', {{}}, ...
                    'CartesianBounds', {{}}, ...
                    'CollisionAvoidance', {{}}, ...
                    'JointBounds', struct([]));
if exist('gik_options', 'var') && ~isempty(gik_options) && isstruct(gik_options)
    optFields = fieldnames(gik_options);
    for fIdx = 1:numel(optFields)
        gikOptions.(optFields{fIdx}) = gik_options.(optFields{fIdx});
    end
end

if strlength(trajFileLocal) > 0
    [~, outputBase, ~] = fileparts(trajFileLocal);
else
    outputBase = char(trajSourceLocal);
end

if strlength(videoFile) == 0
    videoFile = fullfile(outputDir, outputBase + "_animation.mp4");
elseif ~startsWith(videoFile, "/") && ~contains(videoFile, ":/")
    videoFile = fullfile(outputDir, videoFile);
end

%% Load robot model
thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir); % ensure helper packages/functions are reachable
urdfPath = fullfile(thisDir, '..', 'mobile_arm_whole_body', 'urdf', 'arm_on_car_center_rotZ_neg90.urdf');
robot = importrobot(urdfPath, 'DataFormat', 'struct');
robot.Gravity = [0 0 -9.81];
armJointNames = {'left_arm_joint1','left_arm_joint2','left_arm_joint3', ...
                 'left_arm_joint4','left_arm_joint5','left_arm_joint6'};
eeName = 'left_gripper_link';

homeConfig = homeConfiguration(robot);
TeeHome = getTransform(robot, homeConfig, eeName);
homeEEPosition = TeeHome(1:3,4)';
homeEERPY = rotm2eul(TeeHome(1:3,1:3), 'XYZ');

homeBasePose = compute_home_base_pose();
rampBaseSpeedMax = 0.2;
trackingBaseSpeedMax = 0.6;

if useGeneralizedIK
    if isempty(gikOptions.DistanceConstraints)
        defaultDist = struct('EndEffector', eeName, ...
                             'ReferenceBody', 'left_arm_base_link', ...
                             'Bounds', [0.30, 5.0], ...
                             'Weights', 1.0);
        gikOptions.DistanceConstraints = {defaultDist};
    end
    if isempty(gikOptions.CartesianBounds)
        defaultCart = struct('EndEffector', eeName, ...
                             'ReferenceBody', robot.BaseName, ...
                             'Bounds', [-Inf, Inf; -Inf, Inf; 0.30, Inf], ...
                             'TargetTransform', eye(4), ...
                             'Weights', [1 1 1]);
        gikOptions.CartesianBounds = {defaultCart};
    end

    if isempty(gikOptions.CollisionAvoidance)
        chassisMeshPath = fullfile(thisDir, '..', 'mobile_arm_whole_body', 'meshes', 'cr_no_V.stl');
        if exist(chassisMeshPath, 'file') == 2
            try
                chassisMesh = helpers.load_collision_mesh(chassisMeshPath);
                collisionStruct = struct('Environment', {{chassisMesh}}, ...
                                         'SelfCollisions', false, ...
                                         'Weights', 1.0, ...
                                         'NumSamples', 6);
                gikOptions.CollisionAvoidance = {collisionStruct};
            catch ME
                warning('rt_whole_body_controller:CollisionMeshLoad', ...
                    'Failed to load collision mesh %s (%s). Continuing without collision avoidance.', chassisMeshPath, ME.message);
            end
        else
            warning('rt_whole_body_controller:CollisionMeshMissing', ...
                'Collision mesh %s not found. Continuing without collision avoidance.', chassisMeshPath);
        end
    end
end

%% Load reference trajectory (camera/world space)
trajOpts = struct('source', char(trajSourceLocal), ...
                  'duration', trajDurationLocal, ...
                  'scale', trajScaleLocal, ...
                  'file', char(trajFileLocal), ...
                  'sample_dt', 0.1, ...
                  'max_speed', 3.0);
traj = generate_external_trajectory(trajOpts);

refPositionsWorld = traj.eePoses(:,1:3);
if size(traj.eePoses,2) >= 6
    refRPYWorld = traj.eePoses(:,4:6);
else
    refRPYWorld = zeros(size(refPositionsWorld));
end

yawDesiredAll = [];
if size(refRPYWorld,2) >= 3
    yawDesiredAll = wrapToPi(refRPYWorld(:,3));
end
numRefSamples = size(refPositionsWorld,1);

refTimes = traj.timestamps;
if isempty(refTimes)
    refTimes = (0:numRefSamples-1)' * trajOpts.sample_dt;
else
    refTimes = refTimes(:);
end

rawBaseWaypoints = traj.baseWaypoints;
if size(rawBaseWaypoints,1) ~= numRefSamples
    tOrig = linspace(0, 1, size(rawBaseWaypoints,1));
    tNew = linspace(0, 1, numRefSamples);
    rawBaseWaypoints = interp1(tOrig, rawBaseWaypoints, tNew, 'linear', 'extrap');
end

baseWaypointsNominal = rawBaseWaypoints;
if size(baseWaypointsNominal,1) > 4
    win = max(5, ceil(size(baseWaypointsNominal,1) / 10));
    baseWaypointsNominal(:,1) = smoothdata(baseWaypointsNominal(:,1), 'movmean', win);
    baseWaypointsNominal(:,2) = smoothdata(baseWaypointsNominal(:,2), 'movmean', win);
    baseWaypointsNominal(1,:) = rawBaseWaypoints(1,:);
    baseWaypointsNominal(end,:) = rawBaseWaypoints(end,:);
end

[thetaGeomNominal, ~, ~] = compute_base_heading(baseWaypointsNominal);
if ~isempty(yawDesiredAll)
    thetaRefNominal = yawDesiredAll;
else
    thetaRefNominal = thetaGeomNominal;
end

[baseWaypointsAug, thetaRefAug, rawBaseWaypointsAug, refPositionsAug, refRPYAug, refTimesAug, rampInfo] = ...
    prepend_ramp_segment(baseWaypointsNominal, thetaRefNominal, rawBaseWaypoints, refPositionsWorld, refRPYWorld, refTimes, trajOpts.sample_dt, robot, armJointNames, homeEEPosition, homeEERPY, homeBasePose, rampBaseSpeedMax, thetaRefNominal(1));

baseWaypointsNominalFull = baseWaypointsAug;
rawBaseWaypointsFull = rawBaseWaypointsAug;
[thetaRefAug, ~, ~] = compute_base_heading(baseWaypointsAug);
thetaRefNominalFull = thetaRefAug;

rampSamples = rampInfo.steps;
armRampSamples = rampInfo.armSteps;
baseRampSamples = rampInfo.baseSteps;
if rampSamples > 0
    rampIdx = 1:rampSamples;
else
    rampIdx = [];
end
trackIdx = (rampSamples + 1):size(baseWaypointsAug,1);
if isempty(trackIdx)
    error('prepend_ramp_segment:NoTrackingData', 'Ramp generation consumed all samples; no tracking segment remains.');
end

refTimesRamp = refTimesAug(rampIdx);
baseWaypointsRamp = baseWaypointsAug(rampIdx, :);
thetaRampNominal = thetaRefAug(rampIdx);
rawBaseWaypointsRamp = rawBaseWaypointsAug(rampIdx, :);

poseTformsNominal = build_base_to_ee_targets(baseWaypointsAug, thetaRefAug, refPositionsAug, refRPYAug);

refPositionsWorld = refPositionsAug(trackIdx, :);
refRPYWorld = refRPYAug(trackIdx, :);
refTimes = refTimesAug(trackIdx);
baseWaypointsNominal = baseWaypointsAug(trackIdx, :);
rawBaseWaypoints = rawBaseWaypointsAug(trackIdx, :);

[qMatrixNominal, jointNamesNominal, ikInfosNominal] = deal([]);
if useGeneralizedIK
    gikOptsNominal = gikOptions;
    gikOptsNominal.InitialGuess = homeConfig;
    [qMatrixNominal, jointNamesNominal, ikInfosNominal] = rt_compute_arm_gik(robot, eeName, poseTformsNominal, gikOptsNominal);
else
    [qMatrixNominal, jointNamesNominal, ikInfosNominal] = rt_compute_arm_ik(robot, eeName, poseTformsNominal);
end
[foundJoints, armIdx] = ismember(armJointNames, jointNamesNominal);
if any(~foundJoints)
    error('One or more arm joints missing from imported robot model.');
end
armTrajectoryNominal = qMatrixNominal(:, armIdx);
armRampTrajectoryIK = zeros(0, numel(armIdx));
ikInfosRamp = {};
if isfield(rampInfo, 'armJointTrajectory') && ~isempty(rampInfo.armJointTrajectory)
    armRampTrajectoryIK = rampInfo.armJointTrajectory;
    armRampSamples = size(armRampTrajectoryIK, 1);
elseif armRampSamples > 0
    armRampTrajectoryIK = armTrajectoryNominal(1:armRampSamples, :);
end
if armRampSamples > 0 && ~isempty(ikInfosNominal)
    upper = min(armRampSamples, numel(ikInfosNominal));
    ikInfosRamp = ikInfosNominal(1:upper);
end

homeVec = zeros(1, numel(armIdx));
for j = 1:numel(armIdx)
    homeVec(j) = homeConfig(armIdx(j)).JointPosition;
end

baseWaypointsRef = baseWaypointsNominal;
baseRefineInfo = struct('applied', false, 'reason', "disabled", 'maxShift', 0, 'meanShift', 0);

[thetaGeomRef, arcLen, segmentDist] = compute_base_heading(baseWaypointsRef);
thetaRef = thetaGeomRef;
if size(refRPYWorld,2) >= 3
    thetaRef = wrapToPi(refRPYWorld(:,3));
end
poseTformsBase = build_base_to_ee_targets(baseWaypointsRef, thetaRef, refPositionsWorld, refRPYWorld);

if useGeneralizedIK
    seedConfigRef = homeConfiguration(robot);
    for cfgIdx = 1:numel(seedConfigRef)
        name = seedConfigRef(cfgIdx).JointName;
        matchIdx = find(strcmp(jointNamesNominal, name), 1);
        if ~isempty(matchIdx)
            seedConfigRef(cfgIdx).JointPosition = armTrajectoryNominal(1, matchIdx);
        end
    end
    gikOptsRef = gikOptions;
    gikOptsRef.InitialGuess = seedConfigRef;
    [qMatrixRefined, jointNamesRefined, ikInfosRefined] = rt_compute_arm_gik(robot, eeName, poseTformsBase, gikOptsRef);
else
    [qMatrixRefined, jointNamesRefined, ikInfosRefined] = rt_compute_arm_ik(robot, eeName, poseTformsBase);
end
[foundJoints, armIdx] = ismember(armJointNames, jointNamesRefined);
if any(~foundJoints)
    error('One or more arm joints missing from imported robot model after refinement.');
end
armTrajectoryRef = qMatrixRefined(:, armIdx);

%% Retiming with joint limits (tracking segment only)
armLimitCfg = arm_joint_limits();
armVelLimits = armLimitCfg.velocity;
armAccLimits = armLimitCfg.acceleration;
[armTimesTrack, armTrajectoryTimedTrack, armVelTimedTrack, retimeInfo] = helpers.retime_joint_trajectory(armTrajectoryRef, ...
    MaxVelocity=armVelLimits, MaxAcceleration=armAccLimits, ...
    TimeStep=0.05, MinSegmentTime=0.2);
armTimesTrack = armTimesTrack(:);
armTrajectoryTrack = armTrajectoryTimedTrack;
armTrajectoryInitial = armTrajectoryTrack;

%% Synchronize chassis motion with retimed tracking trajectory
baseLimits = struct('v_max', trackingBaseSpeedMax, 'omega_max', 1.0, 'lat_acc_max', 0.6);
maxCurvature = estimate_max_curvature(segmentDist, thetaGeomRef);
if maxCurvature < 1e-6
    baseSpeedLimit = baseLimits.v_max;
else
    baseSpeedLimit = min([baseLimits.v_max, baseLimits.omega_max / maxCurvature, ...
                          sqrt(baseLimits.lat_acc_max / maxCurvature)]);
    baseSpeedLimit = max(baseSpeedLimit, 0.05);
end

[baseXTrack, baseYTrack, thetaTrack, vBaseTrack, omegaBaseTrack, scaleFactor, armTimesTrack, armVelTimedTrack, retimeInfo, syncDiag] = ...
    synchronize_base_trajectory(baseWaypointsRef, thetaRef, arcLen, refTimes, ...
                                armTimesTrack, armVelTimedTrack, retimeInfo, baseLimits, baseSpeedLimit);
poseHistoryTrack = [baseXTrack, baseYTrack, thetaTrack];
thetaRefSyncTrack = syncDiag.thetaRefTimeline;

% Interpolate desired end-effector positions to synchronized tracking timeline
if numel(armTimesTrack) > 1
    timeParamTrack = (armTimesTrack - armTimesTrack(1)) / (armTimesTrack(end) - armTimesTrack(1));
else
    timeParamTrack = zeros(size(armTimesTrack));
end
if numel(refTimes) > 1
    timeParamRef = (refTimes - refTimes(1)) / (refTimes(end) - refTimes(1));
else
    timeParamRef = zeros(size(refTimes));
end

desiredEETrack = interp1(timeParamRef, refPositionsWorld, timeParamTrack, 'pchip', 'extrap');
desiredRPYTrack = zeros(numel(armTimesTrack), 3);
for idx = 1:3
    desiredRPYTrack(:, idx) = wrapToPi(interp1(timeParamRef, unwrap(refRPYWorld(:, idx)), timeParamTrack, 'pchip', 'extrap'));
end

poseTformsFinal = cell(1, numel(armTimesTrack));
for k = 1:numel(armTimesTrack)
    Tbase = trvec2tform([baseXTrack(k), baseYTrack(k), 0]) * axang2tform([0 0 1 thetaTrack(k)]);
    Tworld = trvec2tform(desiredEETrack(k,:)) * eul2tform(desiredRPYTrack(k,:), 'XYZ');
    poseTformsFinal{k} = Tbase \ Tworld;
end

initialConfig = homeConfiguration(robot);
for j = 1:numel(initialConfig)
    matchIdx = find(strcmp(armJointNames, initialConfig(j).JointName), 1);
    if ~isempty(matchIdx)
        initialConfig(j).JointPosition = armTrajectoryInitial(1, matchIdx);
    end
end

[qMatrixFinal, jointNamesFinal, ikInfosFinal] = deal([]);
if useGeneralizedIK
    gikOptsFinal = gikOptions;
    gikOptsFinal.InitialGuess = initialConfig;
    [qMatrixFinal, jointNamesFinal, ikInfosFinal] = rt_compute_arm_gik(robot, eeName, poseTformsFinal, gikOptsFinal);
else
    [qMatrixFinal, jointNamesFinal, ikInfosFinal] = rt_compute_arm_ik(robot, eeName, poseTformsFinal, [0.5 0.5 0.5 1 1 1], initialConfig);
end
[foundJointsFinal, armIdxFinal] = ismember(armJointNames, jointNamesFinal);
if any(~foundJointsFinal)
    error('One or more arm joints missing from final IK solve.');
end
armTrajectoryTrack = qMatrixFinal(:, armIdxFinal);

ikError = evaluate_ik_pose_error(robot, armJointNames, armTrajectoryTrack, poseTformsFinal, eeName);
ikPoseTol = struct('position', 0.02, 'orientation', deg2rad(5));
badPos = ikError.translation > ikPoseTol.position;
badOri = ikError.orientation > ikPoseTol.orientation;
if any(badPos)
    warning('rt_whole_body_controller:IKPositionError', ...
        '%d IK samples exceed %.1f mm translation error (max %.1f mm).', ...
        nnz(badPos), ikPoseTol.position*1e3, max(ikError.translation)*1e3);
end
if any(badOri)
    warning('rt_whole_body_controller:IKOrientationError', ...
        '%d IK samples exceed %.1f deg orientation error (max %.1f deg).', ...
        nnz(badOri), rad2deg(ikPoseTol.orientation), rad2deg(max(ikError.orientation)));
end

ikJointLimitFlags = false(numel(ikInfosFinal),1);
ikConvergedFlags = true(numel(ikInfosFinal),1);
for kInfo = 1:numel(ikInfosFinal)
    info = ikInfosFinal{kInfo};
    if isfield(info, 'Status')
        statusStr = lower(string(info.Status));
        if contains(statusStr, 'joint limit')
            ikJointLimitFlags(kInfo) = true;
        end
    end
    if isfield(info, 'ExitFlag')
        ikConvergedFlags(kInfo) = info.ExitFlag > 0;
    end
end
if any(ikJointLimitFlags)
    warning('rt_whole_body_controller:IKJointLimits', ...
        '%d IK samples reported joint limit activity.', nnz(ikJointLimitFlags));
end
if any(~ikConvergedFlags)
    warning('rt_whole_body_controller:IKConvergence', ...
        '%d IK samples did not report successful convergence.', nnz(~ikConvergedFlags));
end

%% Assemble staged trajectory and compute metrics
if isempty(armTrajectoryTrack)
    readyPose = homeVec;
else
    readyPose = armTrajectoryTrack(1,:);
end

armWarmupTraj = zeros(0, numel(homeVec));
if armRampSamples > 0
    if size(armRampTrajectoryIK,1) == armRampSamples
        armWarmupTraj = armRampTrajectoryIK;
        readyPose = armWarmupTraj(end,:);
    else
        tauArm = linspace(0, 1, armRampSamples)';
        armWarmupTraj = (1 - tauArm) .* homeVec + tauArm .* readyPose;
    end
end

if baseRampSamples > 0
    armBaseHoldTraj = repmat(readyPose, baseRampSamples, 1);
else
    armBaseHoldTraj = zeros(0, numel(homeVec));
end

armTrajectory = [armWarmupTraj; armBaseHoldTraj; armTrajectoryTrack];
rampSamplesTotal = armRampSamples + baseRampSamples;

if isempty(armVelTimedTrack)
    armVelTimedTrack = zeros(size(armTrajectoryTrack));
end
armVelTimed = [zeros(rampSamplesTotal, size(armTrajectoryTrack,2)); armVelTimedTrack];

sampleDt = trajOpts.sample_dt;
if ~isempty(armTimesTrack)
    armTimesTrack = armTimesTrack - armTimesTrack(1);
end
tArmWarmup = ((0:armRampSamples-1)') * sampleDt;
tBaseWarmup = ((armRampSamples:(armRampSamples + baseRampSamples - 1))') * sampleDt;
tTracking = armTimesTrack + rampSamplesTotal * sampleDt;
tVec = [tArmWarmup; tBaseWarmup; tTracking];
armTimes = tVec;

poseHistoryArm = zeros(armRampSamples, 3);
if armRampSamples > 0
    poseHistoryArm(:,1:2) = baseWaypointsRamp(1:armRampSamples, :);
    poseHistoryArm(:,3) = thetaRampNominal(1:armRampSamples);
end
poseHistoryBase = zeros(baseRampSamples, 3);
if baseRampSamples > 0
    idxStart = armRampSamples + 1;
    idxEnd = idxStart + baseRampSamples - 1;
    poseHistoryBase(:,1:2) = baseWaypointsRamp(idxStart:idxEnd, :);
    poseHistoryBase(:,3) = thetaRampNominal(idxStart:idxEnd);
end
poseHistory = [poseHistoryArm; poseHistoryBase; poseHistoryTrack];
baseX = poseHistory(:,1);
baseY = poseHistory(:,2);
theta = poseHistory(:,3);

thetaRefRamp = thetaRampNominal;
thetaRefSync = [thetaRefRamp; thetaRefSyncTrack];
thetaRefSync = thetaRefSync(:);
thetaDeviation = wrapToPi(theta - thetaRefSync);

if rampSamplesTotal > 0
    desiredEERamp = repmat(refPositionsWorld(1,:), rampSamplesTotal, 1);
    desiredRPYRamp = repmat(refRPYWorld(1,:), rampSamplesTotal, 1);
else
    desiredEERamp = zeros(0, size(refPositionsWorld,2));
    desiredRPYRamp = zeros(0, 3);
end
desiredEEInterp = [desiredEERamp; desiredEETrack];
desiredRPYInterp = [desiredRPYRamp; desiredRPYTrack];

if isempty(refTimes)
    refTimesGlobal = ((0:rampSamplesTotal-1)') * sampleDt;
else
    refTimesGlobal = [((0:rampSamplesTotal-1)') * sampleDt; refTimes + rampSamplesTotal * sampleDt];
end
refTimesGlobal = refTimesGlobal(:);

trackingStartIdx = max(1, min(rampSamplesTotal + 1, numel(tVec)));

baseWaypointsRefFull = [baseWaypointsRamp; baseWaypointsRef];
[thetaRefFull, ~, ~] = compute_base_heading(baseWaypointsRefFull);

if isfield(syncDiag, 'directionSign')
    directionSignFull = [ones(rampSamplesTotal,1); syncDiag.directionSign];
else
    directionSignFull = [];
end

eeMetrics = helpers.compute_ee_metrics(robot, armJointNames, armTrajectory, poseHistory, tVec, eeName);

vx_world = gradient(baseX, tVec);
vy_world = gradient(baseY, tVec);
omegaBase = gradient(unwrap(theta), tVec);
if numel(vx_world) > 1
    vx_world([1 end]) = vx_world([2 end-1]);
    vy_world([1 end]) = vy_world([2 end-1]);
    omegaBase([1 end]) = omegaBase([2 end-1]);
end

vBaseWorldMag = sqrt(vx_world.^2 + vy_world.^2);
v_long = vx_world .* cos(theta) + vy_world .* sin(theta);
v_lat  = -vx_world .* sin(theta) + vy_world .* cos(theta);
cmdHistory = [v_long(:), omegaBase(:)];

eeErrorVec = desiredEEInterp - eeMetrics.positions;
eeErrorNorm = sqrt(sum(eeErrorVec.^2, 2));
trackingIdx = trackingStartIdx:numel(eeErrorNorm);
eeErrorVecTracking = eeErrorVec(trackingIdx, :);
eeErrorNormTracking = eeErrorNorm(trackingIdx);
if isempty(eeErrorNormTracking)
    maxEEErrorTracking = 0;
    meanEEErrorTracking = 0;
else
    maxEEErrorTracking = max(eeErrorNormTracking);
    meanEEErrorTracking = mean(eeErrorNormTracking);
end
maxEEErrorTotal = max(eeErrorNorm);
meanEEErrorTotal = mean(eeErrorNorm);

if maxEEErrorTracking > 0.05
    warning('rt_whole_body_controller:TrackingError', ...
        'Max EE position error %.3f m (tracking phase) exceeds tolerance of 0.05 m.', maxEEErrorTracking);
end

eeSpeed = eeMetrics.speed;
eeAccelMag = eeMetrics.accel_mag;
eeJerkMag = eeMetrics.jerk_mag;

%% Visualize planar motion
figPlanar = figure('Name', 'Planar Base Path'); hold on; grid on; axis equal;
plot(rawBaseWaypointsFull(:,1), rawBaseWaypointsFull(:,2), 'k--o', 'DisplayName', 'Raw waypoints');
plot(baseWaypointsRefFull(:,1), baseWaypointsRefFull(:,2), 'c-.', 'LineWidth', 1.0, 'DisplayName', 'Refined base (IK)');
plot(baseX, baseY, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Synchronized path');
scatter(baseX(1), baseY(1), 70, 'filled', 'MarkerFaceColor', [0.1 0.7 0.2], 'DisplayName', 'Start');
scatter(baseX(end), baseY(end), 70, 'filled', 'MarkerFaceColor', [0.8 0.2 0.2], 'DisplayName', 'End');
quiver(baseX(1:10:end), baseY(1:10:end), cos(theta(1:10:end))*0.1, sin(theta(1:10:end))*0.1, 0, ...
       'Color', [0 0.4 0.8], 'DisplayName', 'Heading');
xlabel('X (m)'); ylabel('Y (m)'); legend('Location','bestoutside');
title('Chassis trajectory synchronized with arm timeline');

%% Summaries
if useGeneralizedIK
    ikLabel = 'GIK';
else
    ikLabel = 'IK';
end
fprintf('Generated %d arm samples via %s.\n', size(armTrajectory,1), ikLabel);
fprintf('Timeline duration: %.2f s across %d synchronized samples.\n', tVec(end), numel(tVec));
fprintf('Base timeline scale factor: %.2f (>=1 implies slowing due to limits).\n', scaleFactor);
fprintf('Max base speed: %.3f m/s, Max base yaw rate: %.3f rad/s.\n', max(vBaseWorldMag), max(abs(omegaBase)));
fprintf('Max base yaw deviation from reference: %.2f deg.\n', rad2deg(max(abs(thetaDeviation))));
fprintf('Ramp-up samples: %d (arm %d, base %d). Tracking begins at index %d.\n', rampInfo.steps, rampInfo.armSteps, rampInfo.baseSteps, trackingStartIdx);
fprintf('Max EE tracking error: %.3f m (mean %.3f m).\n', maxEEErrorTracking, meanEEErrorTracking);
fprintf('Overall EE error including ramp: max %.3f m (mean %.3f m).\n', maxEEErrorTotal, meanEEErrorTotal);

%% Store results for downstream use
results.armTrajectory = armTrajectory;
results.armVelocities = armVelTimed;
results.armJointNames = armJointNames;
results.armTimes = armTimes;
results.armLimits = armLimitCfg;
results.basePose = poseHistory;
results.baseTimes = tVec;
results.baseCmd = cmdHistory;
results.baseWaypoints = baseWaypointsRefFull;
results.baseWaypointsRaw = rawBaseWaypointsFull;
results.baseWaypointsNominal = baseWaypointsNominalFull;
results.baseHeadingNominal = thetaRefNominalFull;
results.baseHeadingRefined = thetaRefFull;
results.baseYawReferenceTimeline = thetaRefSync;
results.baseYawError = thetaDeviation;
results.baseSyncScale = scaleFactor;
results.baseSyncScaleHistory = syncDiag.scaleHistory;
results.baseRefinement = baseRefineInfo;
results.baseInitialization = rampInfo;
if ~isempty(directionSignFull)
    results.baseDirectionSign = directionSignFull;
end
results.baseVelocityLong = v_long;
results.baseVelocityLat = v_lat;
results.baseVelocityYaw = omegaBase;
results.baseVelocityMag = vBaseWorldMag;
results.eeDesired = desiredEEInterp;
results.eeDesiredRPY = desiredRPYInterp;
results.eeActual = eeMetrics;
results.eeErrorNorm = eeErrorNorm;
results.eeErrorVec = eeErrorVec;
results.eeErrorNormTracking = eeErrorNormTracking;
results.eeErrorVecTracking = eeErrorVecTracking;
results.maxTrackingError = maxEEErrorTracking;
results.meanTrackingError = meanEEErrorTracking;
results.maxTrackingErrorTotal = maxEEErrorTotal;
results.meanTrackingErrorTotal = meanEEErrorTotal;
results.eeSpeed = eeSpeed;
results.eeAccelMag = eeAccelMag;
results.eeJerkMag = eeJerkMag;
results.referenceTimes = refTimesGlobal;
results.retime = retimeInfo;
results.trackingStartIndex = trackingStartIdx;
results.ikPoseError = ikError;
results.ikJointLimitFlags = ikJointLimitFlags;
results.ikConverged = ikConvergedFlags;
stageBoundaries = [armRampSamples, armRampSamples + baseRampSamples, size(armTrajectory,1)];
stageBoundaries = unique(max(stageBoundaries, 1));
if stageBoundaries(end) ~= size(armTrajectory,1)
    stageBoundaries(end+1) = size(armTrajectory,1);
end
stageLabels = ["Arm ramp", "Chassis ramp", "Tracking"];
if numel(stageLabels) > numel(stageBoundaries)
    stageLabels = stageLabels(1:numel(stageBoundaries));
elif numel(stageLabels) < numel(stageBoundaries)
    stageLabels(end+1:numel(stageBoundaries)) = stageLabels(end);
end

results.stageBoundaries = stageBoundaries;
results.stageLabels = stageLabels;
results.ikPoseTolerance = ikPoseTol;
results.ikInfos = struct('rampWarmup', {ikInfosRamp}, ...
                         'nominal', {ikInfosNominal}, ...
                         'refined', {ikInfosRefined}, ...
                         'final', {ikInfosFinal});
if useGeneralizedIK
    results.ikMode = "gik";
    results.gikSettings = struct('PositionTolerance', gikOptions.PositionTolerance, ...
                                 'OrientationTolerance', gikOptions.OrientationTolerance, ...
                                 'UseWarmStart', gikOptions.UseWarmStart, ...
                                 'ExtraConstraintCount', numel(gikOptions.ExtraConstraints), ...
                                 'DistanceConstraintCount', numel(gikOptions.DistanceConstraints), ...
                                 'CartesianBoundsCount', numel(gikOptions.CartesianBounds), ...
                                 'CollisionConstraintCount', numel(gikOptions.CollisionAvoidance));
else
    results.ikMode = "ik";
end
ikInfos = results.ikInfos;
assignin('base', 'rt_results', results);
assignin('base', 'rt_ikInfos', ikInfos);

%% Plots & animation
figEECompare = helpers.plot_ee_comparison(desiredEEInterp, eeMetrics.positions, poseHistory);
perfFigs = helpers.plot_performance_metrics(tVec, struct( ...
    'errorNorm', eeErrorNorm, ...
    'base', struct('x', baseX, 'y', baseY, 'yaw', theta, ...
                   'yaw_ref', thetaRefSync, 'yaw_err', thetaDeviation, ...
                   'v_long', v_long, 'v_lat', v_lat, 'omega', omegaBase), ...
    'armAngles', armTrajectory, ...
    'armJointNames', {armJointNames}, ...
    'eeSpeed', eeSpeed, 'eeAccel', eeAccelMag, 'eeJerk', eeJerkMag));
figJoint = helpers.plot_joint_trajectories(armTimes, armTrajectory, armJointNames);
figChassis = helpers.plot_chassis_profile(tVec, poseHistory);
if animateRobot
    args = {'EndEffectorName', string(eeName), 'PlaybackSpeed', playbackSpeed};
    if strlength(videoFile) > 0
        args = [args, {'VideoFile', string(videoFile)}, {'VideoFrameRate', videoFrameRate}]; %#ok<AGROW>
    end
    helpers.animate_whole_body(robot, armJointNames, armTrajectory, armTimes, ...
        poseHistory, tVec, desiredEEInterp, args{:}, ...
        'StageBoundaries', stageBoundaries, 'StageLabels', stageLabels);
end

resultsFile = fullfile(outputDir, outputBase + "_results.mat");
save(char(resultsFile), 'results', 'ikInfos');

exportgraphics(figPlanar, char(fullfile(outputDir, outputBase + "_planar_path.png")), 'Resolution', 200);
exportgraphics(figEECompare, char(fullfile(outputDir, outputBase + "_ee_compare.png")), 'Resolution', 200);
exportgraphics(figJoint, char(fullfile(outputDir, outputBase + "_arm_joints.png")), 'Resolution', 200);
exportgraphics(figChassis, char(fullfile(outputDir, outputBase + "_chassis_profile.png")), 'Resolution', 200);
exportgraphics(perfFigs.error, char(fullfile(outputDir, outputBase + "_error_norm.png")), 'Resolution', 200);
exportgraphics(perfFigs.base, char(fullfile(outputDir, outputBase + "_base_states.png")), 'Resolution', 200);
exportgraphics(perfFigs.arm, char(fullfile(outputDir, outputBase + "_arm_angles_grid.png")), 'Resolution', 200);
exportgraphics(perfFigs.ee, char(fullfile(outputDir, outputBase + "_ee_kinematics.png")), 'Resolution', 200);

%% Helper functions =====================================================
function [thetaRef, arcLen, segmentDist] = compute_base_heading(baseWaypoints)
numPts = size(baseWaypoints,1);
if numPts < 2
    thetaRef = zeros(numPts,1);
    arcLen = zeros(numPts,1);
    segmentDist = zeros(max(numPts-1,1),1);
    return;
end
segmentDiff = diff(baseWaypoints,1,1);
segmentDist = sqrt(sum(segmentDiff.^2, 2));
arcLen = [0; cumsum(segmentDist)];
thetaRef = zeros(numPts,1);
thetaRef(1:end-1) = atan2(segmentDiff(:,2), segmentDiff(:,1));
thetaRef(end) = thetaRef(end-1);
thetaRef = unwrap(thetaRef);
end

function poseTforms = build_base_to_ee_targets(baseWaypoints, thetaRef, refPositionsWorld, refRPYWorld)
numSamples = size(refPositionsWorld,1);
poseTforms = cell(1, numSamples);
for i = 1:numSamples
    idxBase = min(i, size(baseWaypoints,1));
    Tbase = eye(4);
    Tbase(1:3,1:3) = axang2rotm([0 0 1 thetaRef(min(i, numel(thetaRef)))]);
    Tbase(1:3,4) = [baseWaypoints(idxBase,:), 0];
    Tworld = trvec2tform(refPositionsWorld(i,:)) * eul2tform(refRPYWorld(i,:));
    poseTforms{i} = Tbase \ Tworld;
end
end

function [baseWaypointsOut, thetaRefOut, rawBaseWaypointsOut, refPositionsOut, refRPYOut, refTimesOut, rampInfo] = ...
    prepend_ramp_segment(baseWaypointsIn, thetaRefIn, rawBaseWaypointsIn, refPositionsIn, refRPYIn, refTimesIn, sampleDt, robot, armJointNames, homeEEPos, homeEERPY, homeBasePose, rampBaseSpeedMax, desiredYawTarget)

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
    homeBasePose = compute_home_base_pose();
end
if nargin < 14 || isempty(rampBaseSpeedMax)
    rampBaseSpeedMax = 0.2;
end
if nargin < 15 || isempty(desiredYawTarget)
    desiredYawTarget = thetaRefIn(1);
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

[seedConfigs, seedLabels, seedJointVectors] = build_arm_ramp_seed_set(configHome, armIdxLocal, qStart, robot, armJointNames);
[configGoal, qEnd, candidateSolutions, selectedSeedIdx] = solve_arm_ramp_gik(gikSolver, cartBound, orientTarget, seedConfigs, armIdxLocal, qStart);

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
    'armGoalCandidates', candidateSolutions, 'armSeedLabels', {seedLabels}, ...
    'armSelectedSeed', selectedSeedIdx, 'armSeedVectors', seedJointVectors, ...
    'armJointVelocity', [], 'armJointAcceleration', [], 'armSamplePeriod', armSamplePeriod);

if posError < 1e-4 && yawError < deg2rad(0.5)
    baseWaypointsOut = baseWaypointsIn;
    thetaRefOut = thetaRefIn;
    rawBaseWaypointsOut = rawBaseWaypointsIn;
    refPositionsOut = refPositionsIn;
    refRPYOut = refRPYIn;
    refTimesOut = refTimesIn;
    return;
end

basePhaseDuration = max(1.5, max(posError / 0.25, yawError / deg2rad(45)));
basePhaseStepsMin = max(ceil(basePhaseDuration / max(sampleDt, 1e-3)), 20);

rampInfo.enabled = true;
rampInfo.armSteps = armPhaseSteps;
rampInfo.armJointTrajectory = armRampJointTraj;
rampInfo.armJointVelocity = armRampJointVel;
rampInfo.armJointAcceleration = armRampJointAcc;
rampInfo.armTimeVector = timeSamples(:);
rampInfo.armGoalConfig = qEnd;

plannerOpts = struct('StepSize', 0.05, 'Wheelbase', 0.5, 'MaxSteer', deg2rad(30), ...
    'GoalPosTol', 0.01, 'GoalYawTol', deg2rad(5), 'MaxIterations', 5000);
path = helpers.hybrid_astar_plan(homePose, targetPose, plannerOpts);
if isempty(path)
    path = [homePose; targetPose];
end
path(1,:) = homePose;
path(end,:) = targetPose;

if size(path,1) == 1
    basePhaseSteps = max(basePhaseStepsMin, 0);
    baseInterp = zeros(basePhaseSteps, 2);
    thetaInterpRes = zeros(basePhaseSteps, 1);
    pathLength = 0;
else
    diffs = diff(path(:,1:2));
    segmentDist = sqrt(sum(diffs.^2, 2));
    arc = [0; cumsum(segmentDist)];
    pathLength = arc(end);
    if pathLength < 1e-9
        basePhaseSteps = max(basePhaseStepsMin, 0);
        baseInterp = zeros(basePhaseSteps, 2);
        thetaInterpRes = zeros(basePhaseSteps, 1);
    else
        maxStep = max(rampBaseSpeedMax * sampleDt, 1e-3);
        minSamplesSpeed = max(1, ceil(pathLength / maxStep));
        basePhaseSteps = max(basePhaseStepsMin, minSamplesSpeed);
        sPath = arc / pathLength;
        sRes = linspace(0, 1, basePhaseSteps + 1)';
        baseInterp = interp1(sPath, path(:,1:2), sRes(2:end), 'linear', 'extrap');
        yawPath = unwrap(path(:,3));
        thetaInterpRes = interp1(sPath, yawPath, sRes(2:end), 'linear', 'extrap');
    end
end
if isempty(thetaInterpRes)
    thetaInterpRes = zeros(0,1);
else
    thetaInterpRes = wrapToPi(thetaInterpRes);
end

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
    thetaRamp(armPhaseSteps+1:end) = thetaInterpRes(:);
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

    function quat = slerp_quat_local(q0, q1, tau)
        q0 = q0(:)'/max(norm(q0), 1e-12);
        q1 = q1(:)'/max(norm(q1), 1e-12);
        dotProd = dot(q0, q1);
        if dotProd < 0
            q1 = -q1;
            dotProd = -dotProd;
        end
        if dotProd > 0.9995
            quat = (1 - tau) * q0 + tau * q1;
            quat = quat / max(norm(quat), 1e-12);
            return;
        end
        theta = acos(max(min(dotProd, 1), -1));
        sinTheta = sin(theta);
        quat = (sin((1 - tau) * theta) / sinTheta) * q0 + (sin(tau * theta) / sinTheta) * q1;
        quat = quat / max(norm(quat), 1e-12);
    end
end

function [baseWaypointsRefined, info] = refine_base_waypoints(robot, armJointNames, armTrajectory, thetaRef, refPositionsWorld, baseWaypointsNominal, eeName)
info = struct('applied', false, 'reason', "", 'maxShift', 0, 'meanShift', 0);
if isempty(armTrajectory)
    info.reason = "empty_arm_trajectory";
    baseWaypointsRefined = [];
    return;
end
numSamples = size(armTrajectory,1);
if numSamples ~= size(refPositionsWorld,1)
    info.reason = "size_mismatch";
    baseWaypointsRefined = [];
    return;
end

configTemplate = homeConfiguration(robot);
vectorNames = {configTemplate.JointName};
armIdx = zeros(1, numel(armJointNames));
for j = 1:numel(armJointNames)
    idx = find(strcmp(vectorNames, armJointNames{j}), 1);
    if isempty(idx)
        info.reason = sprintf('joint_%s_missing', armJointNames{j});
        baseWaypointsRefined = [];
        return;
    end
    armIdx(j) = idx;
end

config = configTemplate;
baseCandidates = zeros(numSamples, 2);
for k = 1:numSamples
    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = armTrajectory(k, j);
    end
    Tarm = getTransform(robot, config, eeName);
    offsetBase = Tarm(1:3,4);
    Rz = axang2rotm([0 0 1 thetaRef(min(k, numel(thetaRef)))]);
    offsetWorld = Rz * offsetBase;
    baseWorld = refPositionsWorld(k,:)' - offsetWorld;
    baseCandidates(k, :) = baseWorld(1:2)';
end

delta = baseCandidates - baseWaypointsNominal;
shiftNorm = sqrt(sum(delta.^2, 2));
info.maxShift = max(shiftNorm);
info.meanShift = mean(shiftNorm);
if info.maxShift < 1e-4
    info.reason = "no_significant_change";
    baseWaypointsRefined = [];
    return;
end

baseWaypointsRefined = baseCandidates;
if size(baseWaypointsRefined,1) > 4
    win = max(5, ceil(size(baseWaypointsRefined,1) / 12));
    baseWaypointsRefined(:,1) = smoothdata(baseWaypointsRefined(:,1), 'movmean', win);
    baseWaypointsRefined(:,2) = smoothdata(baseWaypointsRefined(:,2), 'movmean', win);
    baseWaypointsRefined(1,:) = baseCandidates(1,:);
    baseWaypointsRefined(end,:) = baseCandidates(end,:);
end

info.reason = "ik_projection";
end

function ikError = evaluate_ik_pose_error(robot, armJointNames, armTrajectory, poseTformsTarget, eeName)
numSamples = size(armTrajectory,1);
configTemplate = homeConfiguration(robot);
vectorNames = {configTemplate.JointName};
armIdx = zeros(1, numel(armJointNames));
for j = 1:numel(armJointNames)
    idx = find(strcmp(vectorNames, armJointNames{j}), 1);
    if isempty(idx)
        error('evaluate_ik_pose_error:JointMissing', 'Joint %s not found in robot model.', armJointNames{j});
    end
    armIdx(j) = idx;
end

translationErr = zeros(numSamples,1);
orientationErr = zeros(numSamples,1);
config = configTemplate;
for k = 1:numSamples
    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = armTrajectory(k, j);
    end
    Tactual = getTransform(robot, config, eeName);
    Ttarget = poseTformsTarget{k};
    translationErr(k) = norm(Tactual(1:3,4) - Ttarget(1:3,4));
    Rrel = Ttarget(1:3,1:3)' * Tactual(1:3,1:3);
    axang = rotm2axang(Rrel);
    orientationErr(k) = abs(wrapToPi(axang(4)));
end

ikError.translation = translationErr;
ikError.orientation = orientationErr;
ikError.orientation_deg = rad2deg(orientationErr);
end

function maxCurvature = estimate_max_curvature(segmentDist, thetaRef)
if numel(thetaRef) <= 2
    maxCurvature = 0;
    return;
end
curv = 0;
for k = 2:numel(thetaRef)-1
    ds = max(segmentDist(k), 1e-6);
    dtheta = wrapToPi(thetaRef(k+1) - thetaRef(k));
    curv = max(curv, abs(dtheta) / ds);
end
maxCurvature = curv;
end

function [baseX, baseY, theta, vBase, omegaBase, scaleAccum, armTimes, armVel, retimeInfo, diagOut] = ...
    synchronize_base_trajectory(baseWaypointsRef, thetaRef, arcLen, refTimes, ...
                                armTimes, armVel, retimeInfo, baseLimits, baseSpeedLimit)
scaleAccum = 1;
scaleHistory = scaleAccum;
baseTimeMin = arcLen(end) / max(baseSpeedLimit, 1e-6);
scaleInit = max(1, baseTimeMin / max(armTimes(end), 1e-6));
if scaleInit > 1.0001
    scaleAccum = scaleInit;
    scaleHistory(end) = scaleAccum;
    armTimes = armTimes * scaleInit;
    armVel = armVel / scaleInit;
    retimeInfo.arrivalTimes = retimeInfo.arrivalTimes * scaleInit;
    retimeInfo.segmentTimes = retimeInfo.segmentTimes * scaleInit;
end

timeParamRef = zeros(size(refTimes));
if refTimes(end) > refTimes(1)
    timeParamRef = (refTimes - refTimes(1)) / (refTimes(end) - refTimes(1));
end

maxIter = 6;
dirSign = ones(numel(armTimes), 1);
for iter = 1:maxIter
    if armTimes(end) > armTimes(1)
        timeParam = (armTimes - armTimes(1)) / (armTimes(end) - armTimes(1));
    else
        timeParam = zeros(size(armTimes));
    end

    baseX = interp1(timeParamRef, baseWaypointsRef(:,1), timeParam, 'pchip', 'extrap');
    baseY = interp1(timeParamRef, baseWaypointsRef(:,2), timeParam, 'pchip', 'extrap');
    thetaInterp = interp1(timeParamRef, unwrap(thetaRef), timeParam, 'pchip', 'extrap');

    baseX = baseX(:);
    baseY = baseY(:);
    thetaNominal = unwrap(thetaInterp(:));

    heading = zeros(size(thetaNominal));
    dirLocal = ones(size(thetaNominal));
    heading(1) = thetaNominal(1);
    for k = 2:numel(thetaNominal)
        dx = baseX(k) - baseX(k-1);
        dy = baseY(k) - baseY(k-1);
        if hypot(dx, dy) < 1e-6
            heading(k) = heading(k-1);
            dirLocal(k) = dirLocal(k-1);
            continue;
        end
        desiredYaw = atan2(dy, dx);
        forwardDelta = wrapToPi(desiredYaw - heading(k-1));
        backwardDelta = wrapToPi(desiredYaw + pi - heading(k-1));
        if abs(backwardDelta) < abs(forwardDelta)
            dirLocal(k) = -1;
            heading(k) = heading(k-1) + backwardDelta;
        else
            dirLocal(k) = 1;
            heading(k) = heading(k-1) + forwardDelta;
        end
    end
    theta = unwrap(heading(:));

    vx_world = gradient(baseX, armTimes);
    vy_world = gradient(baseY, armTimes);
    omegaBase = gradient(theta, armTimes);
    vBase = vx_world .* cos(theta) + vy_world .* sin(theta);

    if numel(vBase) > 1
        vBase([1 end]) = vBase([2 end-1]);
        omegaBase([1 end]) = omegaBase([2 end-1]);
    end

    limitFactor = max([1, max(abs(vBase))/max(baseSpeedLimit,1e-6), max(abs(omegaBase))/baseLimits.omega_max]);
    if limitFactor <= 1.0001 || iter == maxIter
        dirSign = dirLocal;
        break;
    end

    scaleAccum = scaleAccum * limitFactor;
    scaleHistory(end+1,1) = scaleAccum; %#ok<AGROW>
    armTimes = armTimes * limitFactor;
    armVel = armVel / limitFactor;
    retimeInfo.arrivalTimes = retimeInfo.arrivalTimes * limitFactor;
    retimeInfo.segmentTimes = retimeInfo.segmentTimes * limitFactor;
end

theta = wrapToPi(theta);
omegaBase = gradient(unwrap(theta), armTimes);
if numel(omegaBase) > 1
    omegaBase([1 end]) = omegaBase([2 end-1]);
end

diagOut = struct('thetaRefTimeline', wrapToPi(thetaInterp(:)), ...
                 'scaleHistory', scaleHistory, ...
                 'directionSign', dirSign);
end

function homePose = compute_home_base_pose()
%COMPUTE_HOME_BASE_POSE Return the default world-frame chassis home pose.
homePose = [-2, -2, 0];
end

function armGoal = compute_arm_ramp_goal(homeEEPos, firstEEPos, refRPYIn, homeEERPY)
%COMPUTE_ARM_RAMP_GOAL Define the virtual EE target used during arm warm-up.
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
%COMPUTE_CHASSIS_RAMP_GOAL Determine the base target pose for the chassis ramp.
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

function [seedConfigs, seedLabels, seedJointVectors] = build_arm_ramp_seed_set(configTemplate, armIdxLocal, qHome, robot, armJointNames)
%BUILD_ARM_RAMP_SEED_SET Generate initial guesses for the warm-up IK solve.
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
    for sign = [-1, 1]
        pairVec = qHome;
        pairVec(idxPair(1)) = clamp_joint_value(qHome(idxPair(1)) + sign * pairOffset, jointLimits(idxPair(1), :));
        pairVec(idxPair(2)) = clamp_joint_value(qHome(idxPair(2)) - sign * pairOffset, jointLimits(idxPair(2), :));
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
%CLAMP_JOINT_VALUE Apply finite position limits if available.
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
%GET_JOINT_POSITION_LIMITS Collect joint position limits for the arm.
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
%SET_CONFIG_POSITIONS Populate a configuration struct with provided joint values.
configOut = configTemplate;
for idx = 1:numel(armIdxLocal)
    configOut(armIdxLocal(idx)).JointPosition = qVec(idx);
end
end

function [configBest, qBest, qCandidates, seedSelection] = solve_arm_ramp_gik(gikSolver, cartBound, orientTarget, seedConfigs, armIdxLocal, qStart)
%SOLVE_ARM_RAMP_GIK Evaluate IK across seeds and pick the closest to qStart.
configs = cell(0, 1);
qCandidates = zeros(0, numel(armIdxLocal));
seedIndices = zeros(0, 1);

for sIdx = 1:numel(seedConfigs)
    try
        solStruct = gikSolver(seedConfigs{sIdx}, cartBound, orientTarget);
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
