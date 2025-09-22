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

[thetaRefNominal, ~, ~] = compute_base_heading(baseWaypointsNominal);

[baseWaypointsNominal, thetaRefNominal, rawBaseWaypoints, refPositionsWorld, refRPYWorld, refTimes, rampInfo] = ...
    prepend_ramp_segment(baseWaypointsNominal, thetaRefNominal, rawBaseWaypoints, refPositionsWorld, refRPYWorld, refTimes, trajOpts.sample_dt, homeEEPosition, homeEERPY);

[thetaRefNominal, ~, ~] = compute_base_heading(baseWaypointsNominal);

poseTformsNominal = build_base_to_ee_targets(baseWaypointsNominal, thetaRefNominal, refPositionsWorld, refRPYWorld);

[qMatrixNominal, jointNamesNominal, ikInfosNominal] = rt_compute_arm_ik(robot, eeName, poseTformsNominal);
[foundJoints, armIdx] = ismember(armJointNames, jointNamesNominal);
if any(~foundJoints)
    error('One or more arm joints missing from imported robot model.');
end
armTrajectoryNominal = qMatrixNominal(:, armIdx);

[baseWaypointsRef, baseRefineInfo] = refine_base_waypoints(robot, armJointNames, armTrajectoryNominal, thetaRefNominal, refPositionsWorld, baseWaypointsNominal, eeName);
if isempty(baseWaypointsRef)
    baseWaypointsRef = baseWaypointsNominal;
    baseRefineInfo.applied = false;
else
    baseRefineInfo.applied = true;
    baseWaypointsRef(1,:) = rawBaseWaypoints(1,:);
    baseWaypointsRef(end,:) = rawBaseWaypoints(end,:);
    if rampInfo.steps > 0
        keepIdx = 1:min(rampInfo.steps, size(baseWaypointsRef,1));
        baseWaypointsRef(keepIdx, :) = baseWaypointsNominal(keepIdx, :);
    end
end

[thetaRef, arcLen, segmentDist] = compute_base_heading(baseWaypointsRef);
poseTformsBase = build_base_to_ee_targets(baseWaypointsRef, thetaRef, refPositionsWorld, refRPYWorld);

[qMatrixRefined, jointNamesRefined, ikInfosRefined] = rt_compute_arm_ik(robot, eeName, poseTformsBase);
[foundJoints, armIdx] = ismember(armJointNames, jointNamesRefined);
if any(~foundJoints)
    error('One or more arm joints missing from imported robot model after refinement.');
end
armTrajectoryRef = qMatrixRefined(:, armIdx);

%% Retiming with joint limits
armLimitCfg = arm_joint_limits();
armVelLimits = armLimitCfg.velocity;
armAccLimits = armLimitCfg.acceleration;
[armTimes, armTrajectoryTimed, armVelTimed, retimeInfo] = helpers.retime_joint_trajectory(armTrajectoryRef, ...
    MaxVelocity=armVelLimits, MaxAcceleration=armAccLimits, ...
    TimeStep=0.05, MinSegmentTime=0.2);
armTimes = armTimes(:);
armTrajectory = armTrajectoryTimed;
armTrajectoryInitial = armTrajectory;

%% Synchronize chassis motion with retimed trajectory
baseLimits = struct('v_max', 0.6, 'omega_max', 1.0, 'lat_acc_max', 0.6);
maxCurvature = estimate_max_curvature(segmentDist, thetaRef);
if maxCurvature < 1e-6
    baseSpeedLimit = baseLimits.v_max;
else
    baseSpeedLimit = min([baseLimits.v_max, baseLimits.omega_max / maxCurvature, ...
                          sqrt(baseLimits.lat_acc_max / maxCurvature)]);
    baseSpeedLimit = max(baseSpeedLimit, 0.05);
end

[baseX, baseY, theta, vBase, omegaBase, scaleFactor, armTimes, armVelTimed, retimeInfo, syncDiag] = ...
    synchronize_base_trajectory(baseWaypointsRef, thetaRef, arcLen, refTimes, ...
                                armTimes, armVelTimed, retimeInfo, baseLimits, baseSpeedLimit);
poseHistory = [baseX, baseY, theta];
cmdHistory = [vBase(:), omegaBase(:)];
tVec = armTimes;
thetaRefSync = syncDiag.thetaRefTimeline;
thetaDeviation = wrapToPi(theta - thetaRefSync);
trackingStartIdx = max(1, min(rampInfo.steps + 1, numel(tVec)));

% Interpolate desired end-effector positions to synchronized timeline
if armTimes(end) > armTimes(1)
    timeParam = (armTimes - armTimes(1)) / (armTimes(end) - armTimes(1));
else
    timeParam = zeros(size(armTimes));
end
if refTimes(end) > refTimes(1)
    timeParamRef = (refTimes - refTimes(1)) / (refTimes(end) - refTimes(1));
else
    timeParamRef = zeros(size(refTimes));
end

desiredEEInterp = interp1(timeParamRef, refPositionsWorld, timeParam, 'pchip', 'extrap');
desiredRPYInterp = zeros(numel(armTimes), 3);
for idx = 1:3
    desiredRPYInterp(:, idx) = wrapToPi(interp1(timeParamRef, unwrap(refRPYWorld(:, idx)), timeParam, 'pchip', 'extrap'));
end

poseTformsFinal = cell(1, numel(armTimes));
for k = 1:numel(armTimes)
    Tbase = trvec2tform([baseX(k), baseY(k), 0]) * axang2tform([0 0 1 theta(k)]);
    Tworld = trvec2tform(desiredEEInterp(k,:)) * eul2tform(desiredRPYInterp(k,:), 'XYZ');
    poseTformsFinal{k} = Tbase \ Tworld;
end

initialConfig = homeConfiguration(robot);
for j = 1:numel(initialConfig)
    matchIdx = find(strcmp(armJointNames, initialConfig(j).JointName), 1);
    if ~isempty(matchIdx)
        initialConfig(j).JointPosition = armTrajectoryInitial(1, matchIdx);
    end
end

[qMatrixFinal, jointNamesFinal, ikInfosFinal] = rt_compute_arm_ik(robot, eeName, poseTformsFinal, [0.5 0.5 0.5 1 1 1], initialConfig);
[foundJointsFinal, armIdxFinal] = ismember(armJointNames, jointNamesFinal);
if any(~foundJointsFinal)
    error('One or more arm joints missing from final IK solve.');
end
armTrajectory = qMatrixFinal(:, armIdxFinal);

ikError = evaluate_ik_pose_error(robot, armJointNames, armTrajectory, poseTformsFinal, eeName);
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

% Compute actual end-effector metrics in world frame
eeMetrics = helpers.compute_ee_metrics(robot, armJointNames, armTrajectory, poseHistory, tVec, eeName);

% Base velocities in longitudinal/lateral components
vx_world = gradient(baseX, tVec);
vy_world = gradient(baseY, tVec);
v_long = vx_world .* cos(theta) + vy_world .* sin(theta);
v_lat  = -vx_world .* sin(theta) + vy_world .* cos(theta);

% End-effector error metrics
eeErrorVec = desiredEEInterp - eeMetrics.positions;
eeErrorNorm = sqrt(sum(eeErrorVec.^2, 2));
trackingIdx = trackingStartIdx:numel(eeErrorNorm);
eeErrorVecTracking = eeErrorVec(trackingIdx, :);
eeErrorNormTracking = eeErrorNorm(trackingIdx);
maxEEErrorTracking = max(eeErrorNormTracking);
meanEEErrorTracking = mean(eeErrorNormTracking);
maxEEErrorTotal = max(eeErrorNorm);
meanEEErrorTotal = mean(eeErrorNorm);

if maxEEErrorTracking > 0.05
    warning('rt_whole_body_controller:TrackingError', ...
        'Max EE position error %.3f m (tracking phase) exceeds tolerance of 0.05 m.', maxEEErrorTracking);
end

% End-effector kinematic magnitudes
eeSpeed = eeMetrics.speed;
eeAccelMag = eeMetrics.accel_mag;
eeJerkMag = eeMetrics.jerk_mag;

%% Visualize planar motion
figPlanar = figure('Name', 'Planar Base Path'); hold on; grid on; axis equal;
plot(rawBaseWaypoints(:,1), rawBaseWaypoints(:,2), 'k--o', 'DisplayName', 'Raw waypoints');
plot(baseWaypointsRef(:,1), baseWaypointsRef(:,2), 'c-.', 'LineWidth', 1.0, 'DisplayName', 'Refined base (IK)');
plot(baseX, baseY, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Synchronized path');
scatter(baseX(1), baseY(1), 70, 'filled', 'MarkerFaceColor', [0.1 0.7 0.2], 'DisplayName', 'Start');
scatter(baseX(end), baseY(end), 70, 'filled', 'MarkerFaceColor', [0.8 0.2 0.2], 'DisplayName', 'End');
quiver(baseX(1:10:end), baseY(1:10:end), cos(theta(1:10:end))*0.1, sin(theta(1:10:end))*0.1, 0, ...
       'Color', [0 0.4 0.8], 'DisplayName', 'Heading');
xlabel('X (m)'); ylabel('Y (m)'); legend('Location','bestoutside');
title('Chassis trajectory synchronized with arm timeline');

%% Summaries
fprintf('Generated %d arm samples via IK.\n', size(armTrajectory,1));
fprintf('Timeline duration: %.2f s across %d synchronized samples.\n', tVec(end), numel(tVec));
fprintf('Base timeline scale factor: %.2f (>=1 implies slowing due to limits).\n', scaleFactor);
fprintf('Max base speed: %.3f m/s, Max base yaw rate: %.3f rad/s.\n', max(abs(vBase)), max(abs(omegaBase)));
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
results.baseWaypoints = baseWaypointsRef;
results.baseWaypointsRaw = rawBaseWaypoints;
results.baseWaypointsNominal = baseWaypointsNominal;
results.baseHeadingNominal = thetaRefNominal;
results.baseHeadingRefined = thetaRef;
results.baseYawReferenceTimeline = thetaRefSync;
results.baseYawError = thetaDeviation;
results.baseSyncScale = scaleFactor;
results.baseSyncScaleHistory = syncDiag.scaleHistory;
results.baseRefinement = baseRefineInfo;
results.baseInitialization = rampInfo;
if isfield(syncDiag, 'directionSign')
    results.baseDirectionSign = syncDiag.directionSign;
end
results.baseVelocityLong = v_long;
results.baseVelocityLat = v_lat;
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
results.referenceTimes = refTimes;
results.retime = retimeInfo;
results.trackingStartIndex = trackingStartIdx;
results.ikPoseError = ikError;
results.ikJointLimitFlags = ikJointLimitFlags;
results.ikConverged = ikConvergedFlags;
results.ikPoseTolerance = ikPoseTol;
results.ikInfos = struct('nominal', {ikInfosNominal}, 'refined', {ikInfosRefined}, 'final', {ikInfosFinal});
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
        'StageBreakIndex', trackingStartIdx, 'StageLabels', ["Ramp-up", "Tracking"]);
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
    prepend_ramp_segment(baseWaypointsIn, thetaRefIn, rawBaseWaypointsIn, refPositionsIn, refRPYIn, refTimesIn, sampleDt, homeEEPos, homeEERPY)

homePose = [0, 0, 0];
targetPose = [baseWaypointsIn(1,:), thetaRefIn(1)];

posError = norm(targetPose(1:2) - homePose(1:2));
yawError = abs(wrapToPi(targetPose(3) - homePose(3)));

rampInfo = struct('enabled', false, 'duration', 0, 'steps', 0, ...
    'startPose', homePose, 'targetPose', targetPose, 'timeAdded', 0, ...
    'armSteps', 0, 'baseSteps', 0);

if posError < 1e-4 && yawError < deg2rad(0.5)
    baseWaypointsOut = baseWaypointsIn;
    thetaRefOut = thetaRefIn;
    rawBaseWaypointsOut = rawBaseWaypointsIn;
    refPositionsOut = refPositionsIn;
    refRPYOut = refRPYIn;
    refTimesOut = refTimesIn;
    return;
end

armPhaseSteps = max(ceil(2.0 / max(sampleDt, 1e-3)), 20);
basePhaseDuration = max(1.5, max(posError / 0.25, yawError / deg2rad(45)));
basePhaseSteps = max(ceil(basePhaseDuration / max(sampleDt, 1e-3)), 20);

rampInfo.enabled = true;
rampInfo.armSteps = armPhaseSteps;
rampInfo.baseSteps = basePhaseSteps;
rampInfo.steps = armPhaseSteps + basePhaseSteps;
rampInfo.duration = rampInfo.steps * sampleDt;

baseRamp = zeros(rampInfo.steps, 2);
thetaRamp = zeros(rampInfo.steps, 1);
if armPhaseSteps > 0
    baseRamp(1:armPhaseSteps, :) = repmat(homePose(1:2), armPhaseSteps, 1);
    thetaRamp(1:armPhaseSteps) = homePose(3);
end
if basePhaseSteps > 0
    interpX = linspace(homePose(1), targetPose(1), basePhaseSteps + 1)';
    interpY = linspace(homePose(2), targetPose(2), basePhaseSteps + 1)';
    interpTheta = linspace(homePose(3), targetPose(3), basePhaseSteps + 1)';
    baseRamp(armPhaseSteps+1:end, :) = [interpX(2:end), interpY(2:end)];
    thetaRamp(armPhaseSteps+1:end) = unwrap(interpTheta(2:end));
end

refTimesRamp = refTimesIn(1) + (-rampInfo.steps:-1)' * sampleDt;

firstPos = refPositionsIn(1,:);
posRamp = repmat(refPositionsIn(1,:), rampInfo.steps, 1);
refRPYRamp = repmat(refRPYIn(1,:), rampInfo.steps, 1);
refPositionsRamp = posRamp;

baseWaypointsOut = [baseRamp; baseWaypointsIn];
thetaRefOut = [thetaRamp; thetaRefIn];
rawBaseWaypointsOut = [baseRamp; rawBaseWaypointsIn];
refPositionsOut = [refPositionsRamp; refPositionsIn];
refRPYOut = [refRPYRamp; refRPYIn];
refTimesOut = [refTimesRamp; refTimesIn];

rampInfo.timeAdded = refTimesOut(1);

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
