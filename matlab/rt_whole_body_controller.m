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

baseWaypointsRef = rawBaseWaypoints;
if size(rawBaseWaypoints,1) > 4
    win = max(5, ceil(size(rawBaseWaypoints,1) / 10));
    baseWaypointsRef(:,1) = smoothdata(rawBaseWaypoints(:,1), 'movmean', win);
    baseWaypointsRef(:,2) = smoothdata(rawBaseWaypoints(:,2), 'movmean', win);
    baseWaypointsRef(1,:) = rawBaseWaypoints(1,:);
    baseWaypointsRef(end,:) = rawBaseWaypoints(end,:);
end

% Orientation of the chassis along the reference XY path
segmentDiff = diff(baseWaypointsRef,1,1);
segmentDist = sqrt(sum(segmentDiff.^2, 2));
arcLen = [0; cumsum(segmentDist)];

thetaRef = zeros(numRefSamples,1);
if numRefSamples > 1
    thetaRef(1:end-1) = atan2(segmentDiff(:,2), segmentDiff(:,1));
    thetaRef(end) = thetaRef(end-1);
end
thetaRef = unwrap(thetaRef);

% Convert world poses to chassis frame for IK
poseTformsBase = cell(1, numRefSamples);
for i = 1:numRefSamples
    Rbase = axang2rotm([0 0 1 thetaRef(i)]);
    Tbase = eye(4);
    Tbase(1:3,1:3) = Rbase;
    Tbase(1:3,4) = [baseWaypointsRef(i,:), 0];
    Tworld = trvec2tform(refPositionsWorld(i,:)) * eul2tform(refRPYWorld(i,:));
    poseTformsBase{i} = Tbase \ Tworld;
end

%% Inverse kinematics at reference samples
[qMatrix, jointNames, ikInfos] = rt_compute_arm_ik(robot, eeName, poseTformsBase);
[~, armIdx] = ismember(armJointNames, jointNames);
if any(armIdx == 0)
    error('One or more arm joints missing from imported robot model.');
end
armTrajectoryRef = qMatrix(:, armIdx);

%% Retiming with joint limits
armLimitCfg = arm_joint_limits();
armVelLimits = armLimitCfg.velocity;
armAccLimits = armLimitCfg.acceleration;
[armTimes, armTrajectoryTimed, armVelTimed, retimeInfo] = helpers.retime_joint_trajectory(armTrajectoryRef, ...
    MaxVelocity=armVelLimits, MaxAcceleration=armAccLimits, ...
    TimeStep=0.05, MinSegmentTime=0.2);
armTimes = armTimes(:);
armTrajectory = armTrajectoryTimed;

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

[baseX, baseY, theta, vBase, omegaBase, scaleFactor, armTimes, armVelTimed, retimeInfo] = ...
    synchronize_base_trajectory(baseWaypointsRef, thetaRef, arcLen, refTimes, ...
                                armTimes, armVelTimed, retimeInfo, baseLimits, baseSpeedLimit);
poseHistory = [baseX, baseY, theta];
cmdHistory = [vBase(:), omegaBase(:)];
tVec = armTimes;

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

% End-effector kinematic magnitudes
eeSpeed = eeMetrics.speed;
eeAccelMag = eeMetrics.accel_mag;
eeJerkMag = eeMetrics.jerk_mag;

%% Visualize planar motion
figPlanar = figure('Name', 'Planar Base Path'); hold on; grid on; axis equal;
plot(rawBaseWaypoints(:,1), rawBaseWaypoints(:,2), 'k--o', 'DisplayName', 'Raw waypoints');
plot(baseWaypointsRef(:,1), baseWaypointsRef(:,2), 'c-.', 'LineWidth', 1.0, 'DisplayName', 'Smoothed ref');
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
fprintf('Max base speed: %.3f m/s, Max base yaw rate: %.3f rad/s.\n', max(abs(vBase)), max(abs(omegaBase)));

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
results.baseSyncScale = scaleFactor;
results.baseVelocityLong = v_long;
results.baseVelocityLat = v_lat;
results.eeDesired = desiredEEInterp;
results.eeActual = eeMetrics;
results.eeErrorNorm = eeErrorNorm;
results.eeErrorVec = eeErrorVec;
results.eeSpeed = eeSpeed;
results.eeAccelMag = eeAccelMag;
results.eeJerkMag = eeJerkMag;
results.referenceTimes = refTimes;
results.retime = retimeInfo;
assignin('base', 'rt_results', results);
assignin('base', 'rt_ikInfos', ikInfos);

%% Plots & animation
figEECompare = helpers.plot_ee_comparison(desiredEEInterp, eeMetrics.positions, poseHistory);
perfFigs = helpers.plot_performance_metrics(tVec, struct( ...
    'errorNorm', eeErrorNorm, ...
    'base', struct('x', baseX, 'y', baseY, 'yaw', theta, ...
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
        poseHistory, tVec, desiredEEInterp, args{:});
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

function thetaOut = enforce_yaw_rate(thetaIn, timeVec, omegaMax)
thetaOut = thetaIn;
for k = 2:numel(thetaIn)
    dt = max(timeVec(k) - timeVec(k-1), 1e-6);
    maxDelta = omegaMax * dt;
    delta = wrapToPi(thetaOut(k) - thetaOut(k-1));
    if abs(delta) > maxDelta
        thetaOut(k) = thetaOut(k-1) + sign(delta) * maxDelta;
    end
end
end

function [baseX, baseY, theta, vBase, omegaBase, scaleAccum, armTimes, armVel, retimeInfo] = ...
    synchronize_base_trajectory(baseWaypointsRef, thetaRef, arcLen, refTimes, ...
                                armTimes, armVel, retimeInfo, baseLimits, baseSpeedLimit)
scaleAccum = 1;
baseTimeMin = arcLen(end) / max(baseSpeedLimit, 1e-6);
scaleInit = max(1, baseTimeMin / max(armTimes(end), 1e-6));
if scaleInit > 1.0001
    scaleAccum = scaleInit;
    armTimes = armTimes * scaleInit;
    armVel = armVel / scaleInit;
    retimeInfo.arrivalTimes = retimeInfo.arrivalTimes * scaleInit;
    retimeInfo.segmentTimes = retimeInfo.segmentTimes * scaleInit;
end

timeParamRef = zeros(size(refTimes));
if refTimes(end) > refTimes(1)
    timeParamRef = (refTimes - refTimes(1)) / (refTimes(end) - refTimes(1));
end

for iter = 1:5
    if armTimes(end) > armTimes(1)
        timeParam = (armTimes - armTimes(1)) / (armTimes(end) - armTimes(1));
    else
        timeParam = zeros(size(armTimes));
    end

    baseX = interp1(timeParamRef, baseWaypointsRef(:,1), timeParam, 'pchip', 'extrap');
    baseY = interp1(timeParamRef, baseWaypointsRef(:,2), timeParam, 'pchip', 'extrap');
    thetaInterp = interp1(timeParamRef, unwrap(thetaRef), timeParam, 'linear', 'extrap');

    baseX = baseX(:);
    baseY = baseY(:);
    theta = enforce_yaw_rate(thetaInterp(:), armTimes, baseLimits.omega_max);
    theta = unwrap(theta);

    dsInterp = sqrt(diff(baseX).^2 + diff(baseY).^2);
    sInterp = [0; cumsum(dsInterp)];
    vBase = gradient(sInterp, armTimes);
    omegaBase = gradient(theta, armTimes);

    if numel(vBase) > 1
        vBase(1) = vBase(2);
        vBase(end) = vBase(end-1);
        omegaBase(1) = omegaBase(2);
        omegaBase(end) = omegaBase(end-1);
    end

    limitFactor = max([1, max(vBase)/max(baseSpeedLimit,1e-6), max(abs(omegaBase))/baseLimits.omega_max]);
    if limitFactor <= 1.0001 || iter == 5
        break;
    end

    scaleAccum = scaleAccum * limitFactor;
    armTimes = armTimes * limitFactor;
    armVel = armVel / limitFactor;
    retimeInfo.arrivalTimes = retimeInfo.arrivalTimes * limitFactor;
    retimeInfo.segmentTimes = retimeInfo.segmentTimes * limitFactor;
end
end
