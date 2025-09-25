function [results, diagnostics] = rt_whole_body_controller(opts)
%RT_WHOLE_BODY_CONTROLLER Robotics System Toolbox-based whole-body controller prototype.
%   RESULTS = RT_WHOLE_BODY_CONTROLLER(OPTS) executes the end-to-end MATLAB
%   workflow using the provided options and returns the computed trajectories
%   and diagnostics. The original script overrides (traj_source, traj_file,
%   etc.) are still honored when running inside MATLAB by checking the base
%   workspace if they are not supplied in OPTS. This preserves existing
%   workflows while enabling eventual MATLAB Coder compatibility.

arguments
    opts struct = struct();
end

function value = localGetOption(opts, name, defaultValue)
%LOCALGETOPTION Fetch option value from opts struct or base workspace fallback.
if isfield(opts, name) && ~isempty(opts.(name))
    value = opts.(name);
    return;
end

value = defaultValue;
if ~coder.target('MATLAB')
    return;
end

try
    varName = char(string(name));
    query = sprintf('exist(''%s'', ''var'') && ~isempty(%s)', varName, varName);
    if evalin('base', query)
        value = evalin('base', varName);
    end
catch
    % fall through to default on any base-workspace error
end
end

results = struct();
diagnostics = struct();

fetchOpt = @(name, defaultValue) localGetOption(opts, name, defaultValue);

trajSourceLocal = string(fetchOpt('traj_source', "demo_arc"));
trajDurationLocal = fetchOpt('traj_duration', 8.0);
trajScaleLocal = fetchOpt('traj_scale', 1.0);
trajFileLocal = string(fetchOpt('traj_file', ""));

animateRobot = logical(fetchOpt('enable_animation', true));
enableVisualization = logical(fetchOpt('enable_visualization', true));
doVisualization = enableVisualization && coder.target('MATLAB');
videoFile = string(fetchOpt('animation_video_file', ""));
videoFrameRate = fetchOpt('animation_frame_rate', 30);
playbackSpeed = fetchOpt('animation_playback_speed', 2.0);

outputDir = string(fetchOpt('output_dir', fullfile(pwd, 'outputs')));
if ~isfolder(outputDir)
    mkdir(outputDir);
end

useGeneralizedIK = logical(fetchOpt('use_gik', false));

gikOptions = struct('PositionTolerance', 0.005, ...
                    'OrientationTolerance', deg2rad(3), ...
                    'ExtraConstraints', {{}}, ...
                    'UseWarmStart', true, ...
                    'DistanceConstraints', {{}}, ...
                    'CartesianBounds', {{}}, ...
                    'CollisionAvoidance', {{}}, ...
                    'JointBounds', struct([]));
gikOverrides = fetchOpt('gik_options', struct());
if ~isempty(gikOverrides) && isstruct(gikOverrides)
    optFields = fieldnames(gikOverrides);
    for fIdx = 1:numel(optFields)
        gikOptions.(optFields{fIdx}) = gikOverrides.(optFields{fIdx});
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

homeBasePose = wbc.compute_home_base_pose();
rampBaseSpeedMax = 0.2;
rampBaseYawRateMax = 0.5; % rad/s yaw limit during chassis warm-up
trackingBaseSpeedMax = 0.6;
trackingBaseYawRateMax = 0.5; % rad/s yaw limit during tracking phase

collisionAvoidanceDefault = wbc.configure_arm_collision_avoidance(robot, thisDir);
if ~isfield(gikOptions, 'CollisionAvoidance') || isempty(gikOptions.CollisionAvoidance)
    gikOptions.CollisionAvoidance = collisionAvoidanceDefault;
else
    gikOptions.CollisionAvoidance = wbc.normalize_collision_avoidance_entries(gikOptions.CollisionAvoidance);
end
rampCollisionDefs = gikOptions.CollisionAvoidance;

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

end

%% Load reference trajectory (camera/world space)
trajOpts = struct('source', char(trajSourceLocal), ...
                  'duration', trajDurationLocal, ...
                  'scale', trajScaleLocal, ...
                  'file', char(trajFileLocal), ...
                  'sample_dt', 0.1, ...
                  'max_speed', 3.0);
refData = wbc.prepare_reference_data(trajOpts);
refPositionsWorld = refData.refPositionsWorld;
refRPYWorld = refData.refRPYWorld;
refTimes = refData.refTimes;
rawBaseWaypoints = refData.rawBaseWaypoints;
baseWaypointsNominal = refData.baseWaypointsNominal;
thetaGeomNominal = refData.baseWaypointsGeomYaw;
thetaRefNominal = refData.baseYawReference;
yawDesiredAll = refData.yawDesiredAll;

[baseWaypointsAug, thetaRefAug, rawBaseWaypointsAug, refPositionsAug, refRPYAug, refTimesAug, rampInfo] = ...
    wbc.plan_ramp_segment(baseWaypointsNominal, thetaRefNominal, rawBaseWaypoints, refPositionsWorld, refRPYWorld, refTimes, trajOpts.sample_dt, robot, armJointNames, homeEEPosition, homeEERPY, homeBasePose, rampBaseSpeedMax, rampBaseYawRateMax, thetaRefNominal(1), rampCollisionDefs);

baseWaypointsNominalFull = baseWaypointsAug;
rawBaseWaypointsFull = rawBaseWaypointsAug;
thetaRefAugOriginal = thetaRefAug;
[thetaRefGeom, ~, ~] = wbc.compute_base_heading(baseWaypointsAug);
thetaRefAug = thetaRefGeom;
if rampInfo.steps > 0
    thetaRefAug(1:rampInfo.steps) = thetaRefAugOriginal(1:rampInfo.steps);
end
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

poseTformsNominal = wbc.build_base_to_ee_targets(baseWaypointsAug, thetaRefAug, refPositionsAug, refRPYAug);

refPositionsWorld = refPositionsAug(trackIdx, :);
refRPYWorld = refRPYAug(trackIdx, :);
refTimes = refTimesAug(trackIdx);
baseWaypointsNominal = baseWaypointsAug(trackIdx, :);
rawBaseWaypoints = rawBaseWaypointsAug(trackIdx, :);

[qMatrixNominal, jointNamesNominal, ikInfosNominal] = deal([]);
if useGeneralizedIK
    gikOptsNominal = gikOptions;
    gikOptsNominal.InitialGuess = homeConfig;
    [qMatrixNominal, jointNamesNominal, ikInfosNominal] = wbc.solve_arm_ik(robot, eeName, poseTformsNominal, ...
        UseGeneralizedIK=true, GIKOptions=gikOptsNominal);
else
    [qMatrixNominal, jointNamesNominal, ikInfosNominal] = wbc.solve_arm_ik(robot, eeName, poseTformsNominal, ...
        InitialGuess=homeConfig);
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

[thetaGeomRef, arcLen, segmentDist] = wbc.compute_base_heading(baseWaypointsRef);
thetaRef = thetaGeomRef;
if size(refRPYWorld,2) >= 3
    thetaRef = wrapToPi(refRPYWorld(:,3));
end
poseTformsBase = wbc.build_base_to_ee_targets(baseWaypointsRef, thetaRef, refPositionsWorld, refRPYWorld);

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
    [qMatrixRefined, jointNamesRefined, ikInfosRefined] = wbc.solve_arm_ik(robot, eeName, poseTformsBase, ...
        UseGeneralizedIK=true, GIKOptions=gikOptsRef);
else
    [qMatrixRefined, jointNamesRefined, ikInfosRefined] = wbc.solve_arm_ik(robot, eeName, poseTformsBase);
end
[foundJoints, armIdx] = ismember(armJointNames, jointNamesRefined);
if any(~foundJoints)
    error('One or more arm joints missing from imported robot model after refinement.');
end
armTrajectoryRef = qMatrixRefined(:, armIdx);

%% Retiming with joint limits and synchronization (tracking segment only)
armLimitCfg = arm_joint_limits();
baseLimits = struct('v_max', trackingBaseSpeedMax, 'omega_max', trackingBaseYawRateMax, 'lat_acc_max', 0.6);
maxCurvature = wbc.estimate_max_curvature(segmentDist, thetaGeomRef);
if maxCurvature < 1e-6
    baseSpeedLimit = baseLimits.v_max;
else
    baseSpeedLimit = min([baseLimits.v_max, baseLimits.omega_max / maxCurvature, ...
                          sqrt(baseLimits.lat_acc_max / maxCurvature)]);
    baseSpeedLimit = max(baseSpeedLimit, 0.05);
end

thetaRampEnd = homeBasePose(3);
if ~isempty(thetaRampNominal)
    thetaRampEnd = thetaRampNominal(end);
end

syncOut = wbc.sync_base_and_arm(baseWaypointsRef, thetaRef, arcLen, refTimes, refPositionsWorld, refRPYWorld, ...
    armTrajectoryRef, baseLimits, baseSpeedLimit, thetaRampEnd);

armTimesTrack = syncOut.armTimes;
armTrajectoryTrack = syncOut.armTrajectoryInitial;
armTrajectoryInitial = armTrajectoryTrack;
armVelTimedTrack = syncOut.armVelocities;
retimeInfo = syncOut.retimeInfo;
poseHistoryTrack = syncOut.basePoseTrack;
thetaRefSyncTrack = syncOut.thetaRefTimeline;
scaleFactor = syncOut.scaleFactor;
syncDiag = syncOut.syncDiag;
vBaseTrack = syncOut.baseVelocity.v;
omegaBaseTrack = syncOut.baseVelocity.omega;
desiredEETrack = syncOut.desiredEETrack;
desiredRPYTrack = syncOut.desiredRPYTrack;
poseTformsFinal = syncOut.poseTformsFinal;
if ndims(poseTformsFinal) == 3
    numFinal = size(poseTformsFinal, 3);
    poseTformsFinalFlat = zeros(numFinal, 16);
    for k = 1:numFinal
        poseTformsFinalFlat(k, :) = reshape(poseTformsFinal(:,:,k)', 1, 16);
    end
    poseTformsFinal = poseTformsFinalFlat;
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
    [qMatrixFinal, jointNamesFinal, ikInfosFinal] = wbc.solve_arm_ik(robot, eeName, poseTformsFinal, ...
        UseGeneralizedIK=true, GIKOptions=gikOptsFinal);
else
    [qMatrixFinal, jointNamesFinal, ikInfosFinal] = wbc.solve_arm_ik(robot, eeName, poseTformsFinal, ...
        IKWeights=[0.5 0.5 0.5 1 1 1], InitialGuess=initialConfig);
end
[foundJointsFinal, armIdxFinal] = ismember(armJointNames, jointNamesFinal);
if any(~foundJointsFinal)
    error('One or more arm joints missing from final IK solve.');
end
armTrajectoryTrack = qMatrixFinal(:, armIdxFinal);

ikError = wbc.evaluate_ik_pose_error(robot, armJointNames, armTrajectoryTrack, poseTformsFinal, eeName);
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
metricsInput = struct('rampInfo', rampInfo, ...
    'armRampSamples', armRampSamples, 'baseRampSamples', baseRampSamples, ...
    'armRampTrajectory', armRampTrajectoryIK, 'homeVec', homeVec, ...
    'armTrajectoryTrack', armTrajectoryTrack, 'armVelTrack', armVelTimedTrack, ...
    'armTimesTrack', armTimesTrack, 'baseWaypointsRamp', baseWaypointsRamp, ...
    'thetaRampNominal', thetaRampNominal, 'baseWaypointsRef', baseWaypointsRef, ...
    'thetaRef', thetaRef, 'poseHistoryTrack', poseHistoryTrack, ...
    'thetaRefSyncTrack', thetaRefSyncTrack, 'desiredEETrack', desiredEETrack, ...
    'desiredRPYTrack', desiredRPYTrack, 'scaleFactor', scaleFactor, ...
    'syncDiag', syncDiag, 'baseVelocityTrack', struct('v', vBaseTrack, 'omega', omegaBaseTrack), ...
    'sampleDt', trajOpts.sample_dt, 'refPositionsWorld', refPositionsWorld, ...
    'refRPYWorld', refRPYWorld, 'refTimes', refTimes, ...
    'rawBaseWaypointsFull', rawBaseWaypointsFull, ...
    'baseWaypointsNominalFull', baseWaypointsNominalFull, ...
    'thetaRefNominalFull', thetaRefNominalFull, ...
    'robot', robot, 'armJointNames', {armJointNames}, 'eeName', eeName);

metrics = wbc.compute_metrics(metricsInput);

armTrajectory = metrics.armTrajectory;
armVelTimed = metrics.armVelTimed;
armTimes = metrics.armTimes;
tVec = metrics.tVec;
poseHistory = metrics.poseHistory;
baseX = metrics.baseX;
baseY = metrics.baseY;
theta = metrics.theta;
thetaRefSync = metrics.thetaRefSync;
thetaDeviation = metrics.thetaDeviation;
baseWaypointsRefFull = metrics.baseWaypointsRefFull;
directionSignFull = metrics.directionSignFull;
desiredEEInterp = metrics.desiredEEInterp;
desiredRPYInterp = metrics.desiredRPYInterp;
refTimesGlobal = metrics.refTimesGlobal;
trackingStartIdx = metrics.trackingStartIdx;
eeMetrics = metrics.eeMetrics;
v_long = metrics.v_long;
v_lat = metrics.v_lat;
omegaBase = metrics.omegaBase;
vBaseWorldMag = metrics.vBaseWorldMag;
cmdHistory = metrics.cmdHistory;
eeErrorVec = metrics.eeErrorVec;
eeErrorNorm = metrics.eeErrorNorm;
eeErrorVecTracking = metrics.eeErrorVecTracking;
eeErrorNormTracking = metrics.eeErrorNormTracking;
maxEEErrorTracking = metrics.maxEEErrorTracking;
meanEEErrorTracking = metrics.meanEEErrorTracking;
maxEEErrorTotal = metrics.maxEEErrorTotal;
meanEEErrorTotal = metrics.meanEEErrorTotal;
eeSpeed = metrics.eeSpeed;
eeAccelMag = metrics.eeAccelMag;
eeJerkMag = metrics.eeJerkMag;
scaleFactor = metrics.scaleFactor;
syncDiag = metrics.syncDiag;
if ~isempty(metrics.baseVelocityTrack)
    vBaseTrack = metrics.baseVelocityTrack.v;
    omegaBaseTrack = metrics.baseVelocityTrack.omega;
end

if maxEEErrorTracking > 0.05
    warning('rt_whole_body_controller:TrackingError', ...
        'Max EE position error %.3f m (tracking phase) exceeds tolerance of 0.05 m.', maxEEErrorTracking);
end

%% Assemble results and diagnostics
[thetaRefFull, ~, ~] = wbc.compute_base_heading(baseWaypointsRefFull);

stageBoundaries = [armRampSamples, armRampSamples + baseRampSamples, size(armTrajectory,1)];
stageBoundaries = unique(max(stageBoundaries, 1));
if stageBoundaries(end) ~= size(armTrajectory,1)
    stageBoundaries(end+1) = size(armTrajectory,1);
end
stageLabels = ["Arm ramp", "Chassis ramp", "Tracking"];
if numel(stageLabels) > numel(stageBoundaries)
    stageLabels = stageLabels(1:numel(stageBoundaries));
elseif numel(stageLabels) < numel(stageBoundaries)
    stageLabels(end+1:numel(stageBoundaries)) = stageLabels(end);
end

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
if isfield(syncDiag, 'scaleHistory')
    results.baseSyncScaleHistory = syncDiag.scaleHistory;
else
    results.baseSyncScaleHistory = scaleFactor;
end
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

diagnostics = struct();
diagnostics.rampInfo = rampInfo;
diagnostics.baseRefinement = baseRefineInfo;
scaleHistoryDiag = [];
if isfield(syncDiag, 'scaleHistory')
    scaleHistoryDiag = syncDiag.scaleHistory;
end
diagnostics.sync = struct('scaleFactor', scaleFactor, ...
                          'basePoseTrack', poseHistoryTrack, ...
                          'armTimes', armTimesTrack, ...
                          'thetaRefTimeline', thetaRefSyncTrack, ...
                          'baseVelocity', struct('v', vBaseTrack, 'omega', omegaBaseTrack), ...
                          'directionSign', directionSignFull, ...
                          'scaleHistory', scaleHistoryDiag);
diagnostics.sync.desiredEETrack = desiredEETrack;
diagnostics.sync.desiredRPYTrack = desiredRPYTrack;
diagnostics.retime = retimeInfo;
diagnostics.metrics = metrics;
diagnostics.ikError = ikError;
diagnostics.ikJointLimitFlags = ikJointLimitFlags;
diagnostics.ikConverged = ikConvergedFlags;
diagnostics.ikInfos = results.ikInfos;
diagnostics.flags = struct('trackingError', maxEEErrorTracking > 0.05, ...
                           'jointLimitActivity', any(ikJointLimitFlags), ...
                           'ikNonConverged', any(~ikConvergedFlags));

if coder.target('MATLAB')
    assignin('base', 'rt_results', results);
    assignin('base', 'rt_ikInfos', diagnostics.ikInfos);
end

%% Summaries (MATLAB only)
if coder.target('MATLAB')
    if useGeneralizedIK
        ikLabel = 'GIK';
    else
        ikLabel = 'IK';
    end
    fprintf('Generated %d arm samples via %s.\n', size(armTrajectory,1), ikLabel);
    if ~isempty(tVec)
        fprintf('Timeline duration: %.2f s across %d synchronized samples.\n', tVec(end), numel(tVec));
    end
    fprintf('Base timeline scale factor: %.2f (>=1 implies slowing due to limits).\n', scaleFactor);
    fprintf('Max base speed: %.3f m/s, Max base yaw rate: %.3f rad/s.\n', max(vBaseWorldMag), max(abs(omegaBase)));
    fprintf('Max base yaw deviation from reference: %.2f deg.\n', rad2deg(max(abs(thetaDeviation))));
    fprintf('Ramp-up samples: %d (arm %d, base %d). Tracking begins at index %d.\n', ...
        rampInfo.steps, rampInfo.armSteps, rampInfo.baseSteps, trackingStartIdx);
    fprintf('Max EE tracking error: %.3f m (mean %.3f m).\n', maxEEErrorTracking, meanEEErrorTracking);
    fprintf('Overall EE error including ramp: max %.3f m (mean %.3f m).\n', maxEEErrorTotal, meanEEErrorTotal);
end

%% Visualization and artifact export
figPlanar = [];
figEECompare = [];
perfFigs = struct();
figJoint = [];
figChassis = [];
if doVisualization
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
        animArgs = {'EndEffectorName', string(eeName), 'PlaybackSpeed', playbackSpeed};
        if strlength(videoFile) > 0
            animArgs = [animArgs, {'VideoFile', string(videoFile)}, {'VideoFrameRate', videoFrameRate}]; %#ok<AGROW>
        end
        if isfield(rampInfo, 'obstacles') && ~isempty(rampInfo.obstacles)
            animArgs = [animArgs, {'Obstacles', rampInfo.obstacles}]; %#ok<AGROW>
        end
        if isfield(rampInfo, 'plannedPath') && ~isempty(rampInfo.plannedPath)
            animArgs = [animArgs, {'TargetPath', rampInfo.plannedPath}]; %#ok<AGROW>
        end
        helpers.animate_whole_body(robot, armJointNames, armTrajectory, armTimes, ...
            poseHistory, tVec, desiredEEInterp, animArgs{:}, ...
            'StageBoundaries', stageBoundaries, 'StageLabels', stageLabels);
    end
end

if coder.target('MATLAB')
    resultsFile = fullfile(outputDir, outputBase + "_results.mat");
    ikInfos = diagnostics.ikInfos;
    save(char(resultsFile), 'results', 'ikInfos');

    if doVisualization
        exportgraphics(figPlanar, char(fullfile(outputDir, outputBase + "_planar_path.png")), 'Resolution', 200);
        exportgraphics(figEECompare, char(fullfile(outputDir, outputBase + "_ee_compare.png")), 'Resolution', 200);
        exportgraphics(figJoint, char(fullfile(outputDir, outputBase + "_arm_joints.png")), 'Resolution', 200);
        exportgraphics(figChassis, char(fullfile(outputDir, outputBase + "_chassis_profile.png")), 'Resolution', 200);
        if isfield(perfFigs, 'error') && ~isempty(perfFigs)
            exportgraphics(perfFigs.error, char(fullfile(outputDir, outputBase + "_error_norm.png")), 'Resolution', 200);
            exportgraphics(perfFigs.base, char(fullfile(outputDir, outputBase + "_base_states.png")), 'Resolution', 200);
            exportgraphics(perfFigs.arm, char(fullfile(outputDir, outputBase + "_arm_angles_grid.png")), 'Resolution', 200);
            exportgraphics(perfFigs.ee, char(fullfile(outputDir, outputBase + "_ee_kinematics.png")), 'Resolution', 200);
        end
    end
end

end
