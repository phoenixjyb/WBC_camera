function metrics = compute_metrics(args)
%COMPUTE_METRICS Assemble warm-up, trajectory, and tracking metrics.
%   METRICS = WBC.COMPUTE_METRICS(ARGS) reproduces the legacy post-processing
%   sequence in rt_whole_body_controller: it stitches ramp + tracking arm
%   trajectories, builds time series for the chassis, computes end-effector
%   error metrics, and returns a struct used to populate the final results.
%
%   Required fields in ARGS:
%     rampInfo               - struct from plan_ramp_segment
%     armRampSamples         - scalar count of arm ramp samples
%     baseRampSamples        - scalar count of base ramp samples
%     armRampTrajectory      - warm-up joint trajectory (may be empty)
%     homeVec                - 1xN home joint positions
%     armTrajectoryTrack     - tracking joint trajectory (NxN)
%     armVelTrack            - tracking joint velocities
%     armTimesTrack          - tracking time vector
%     baseWaypointsRamp      - ramp base waypoints
%     thetaRampNominal       - ramp yaw references
%     baseWaypointsRef       - tracking base waypoints
%     thetaRef               - tracking yaw references
%     poseHistoryTrack       - tracking base pose history
%     thetaRefSyncTrack      - tracking yaw reference timeline
%     desiredEETrack         - tracking EE positions
%     desiredRPYTrack        - tracking EE orientations
%     scaleFactor            - scalar sync scale factor
%     syncDiag               - struct from sync_base_and_arm
%     baseVelocityTrack      - struct with fields v, omega
%     sampleDt               - scalar sample period
%     refPositionsWorld      - tracking EE positions (world)
%     refRPYWorld            - tracking EE RPY
%     refTimes               - tracking timestamps
%     rawBaseWaypointsFull   - ramp+tracking raw waypoints (precomputed)
%     baseWaypointsNominalFull - ramp+tracking nominal waypoints
%     thetaRefNominalFull    - nominal yaw timeline (ramp+tracking)
%     robot                  - rigidBodyTree
%     armJointNames          - cell array of joint names
%     eeName                 - end-effector name
%
%   Outputs bundle all time-series and metrics required by the top-level
%   controller (arm/base trajectories, EE errors, command histories, etc.).

arguments
    args struct
end

rampInfo = args.rampInfo;
armRampSamples = args.armRampSamples;
baseRampSamples = args.baseRampSamples;
armRampTrajectory = args.armRampTrajectory;
homeVec = args.homeVec;
armTrajectoryTrack = args.armTrajectoryTrack;
armVelTrack = args.armVelTrack;
armTimesTrack = args.armTimesTrack;
baseWaypointsRamp = args.baseWaypointsRamp;
thetaRampNominal = args.thetaRampNominal;
baseWaypointsRef = args.baseWaypointsRef;
thetaRef = args.thetaRef;
poseHistoryTrack = args.poseHistoryTrack;
thetaRefSyncTrack = args.thetaRefSyncTrack;
desiredEETrack = args.desiredEETrack;
desiredRPYTrack = args.desiredRPYTrack;
scaleFactor = args.scaleFactor;
syncDiag = args.syncDiag;
baseVelocityTrack = args.baseVelocityTrack;
sampleDt = args.sampleDt;
refPositionsWorld = args.refPositionsWorld;
refRPYWorld = args.refRPYWorld;
refTimes = args.refTimes;
rawBaseWaypointsFull = args.rawBaseWaypointsFull;
baseWaypointsNominalFull = args.baseWaypointsNominalFull;
thetaRefNominalFull = args.thetaRefNominalFull;
robot = args.robot;
armJointNames = args.armJointNames;
eeName = args.eeName;

if isempty(armTrajectoryTrack)
    armTrajectoryTrack = zeros(0, numel(homeVec));
end
if isempty(armVelTrack)
    armVelTrack = zeros(size(armTrajectoryTrack));
end
if isempty(armTimesTrack)
    armTimesTrack = zeros(0,1);
end

% Construct warm-up arm trajectories
if armRampSamples > 0
    if ~isempty(armRampTrajectory) && size(armRampTrajectory,1) == armRampSamples
        armWarmupTraj = armRampTrajectory;
        readyPose = armWarmupTraj(end,:);
    else
        tauArm = linspace(0, 1, armRampSamples)';
        if isempty(armTrajectoryTrack)
            readyPose = homeVec;
        else
            readyPose = armTrajectoryTrack(1,:);
        end
        armWarmupTraj = (1 - tauArm) .* homeVec + tauArm .* readyPose;
    end
else
    armWarmupTraj = zeros(0, numel(homeVec));
    if isempty(armTrajectoryTrack)
        readyPose = homeVec;
    else
        readyPose = armTrajectoryTrack(1,:);
    end
end

if baseRampSamples > 0
    armBaseHoldTraj = repmat(readyPose, baseRampSamples, 1);
else
    armBaseHoldTraj = zeros(0, numel(homeVec));
end

armTrajectory = [armWarmupTraj; armBaseHoldTraj; armTrajectoryTrack];
rampSamplesTotal = armRampSamples + baseRampSamples;

if isempty(armVelTrack)
    armVelTrack = zeros(size(armTrajectoryTrack));
end
armVelTimed = [zeros(rampSamplesTotal, size(armVelTrack,2)); armVelTrack];

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

baseWaypointsRefFull = [baseWaypointsRamp; baseWaypointsRef];

thetaRefRamp = thetaRampNominal;
thetaRefSync = [thetaRefRamp; thetaRefSyncTrack];
thetaRefSync = thetaRefSync(:);

thetaDeviation = wrapToPi(theta - thetaRefSync);

% Desired EE interpolation across entire timeline
if rampSamplesTotal > 0
    if ~isempty(refPositionsWorld)
        firstPos = refPositionsWorld(1,:);
    else
        firstPos = zeros(1, size(desiredEETrack,2));
    end
    desiredEERamp = repmat(firstPos, rampSamplesTotal, 1);
    if ~isempty(refRPYWorld)
        firstRPY = refRPYWorld(1,:);
    else
        firstRPY = zeros(1, size(desiredRPYTrack,2));
    end
    desiredRPYRamp = repmat(firstRPY, rampSamplesTotal, 1);
else
    desiredEERamp = zeros(0, size(desiredEETrack,2));
    desiredRPYRamp = zeros(0, size(desiredRPYTrack,2));
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

if isfield(syncDiag, 'directionSign') && ~isempty(syncDiag.directionSign)
    directionSignFull = [ones(rampSamplesTotal,1); syncDiag.directionSign];
else
    directionSignFull = [];
end

% Compute EE metrics
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

if size(desiredEEInterp,2) ~= size(eeMetrics.positions,2)
    % pad/truncate to match dimensions
    cols = size(eeMetrics.positions,2);
    desiredEEInterp = desiredEEInterp(:,1:cols);
end

 eeErrorVec = desiredEEInterp - eeMetrics.positions;
 eeErrorNorm = sqrt(sum(eeErrorVec.^2, 2));
trackingIdx = trackingStartIdx:numel(eeErrorNorm);
eoVecTrack = eeErrorVec(trackingIdx, :);
eoNormTrack = eeErrorNorm(trackingIdx);
if isempty(eoNormTrack)
    maxEEErrorTracking = 0;
    meanEEErrorTracking = 0;
else
    maxEEErrorTracking = max(eoNormTrack);
    meanEEErrorTracking = mean(eoNormTrack);
end
maxEEErrorTotal = max(eeErrorNorm);
meanEEErrorTotal = mean(eeErrorNorm);

eeSpeed = eeMetrics.speed;
eeAccelMag = eeMetrics.accel_mag;
eeJerkMag = eeMetrics.jerk_mag;

metrics = struct();
metrics.armTrajectory = armTrajectory;
metrics.armVelTimed = armVelTimed;
metrics.armTimes = armTimes;
metrics.tVec = tVec;
metrics.poseHistory = poseHistory;
metrics.baseX = baseX;
metrics.baseY = baseY;
metrics.theta = theta;
metrics.thetaRefSync = thetaRefSync;
metrics.thetaDeviation = thetaDeviation;
metrics.baseWaypointsRefFull = baseWaypointsRefFull;
metrics.rawBaseWaypointsFull = rawBaseWaypointsFull;
metrics.baseWaypointsNominalFull = baseWaypointsNominalFull;
metrics.thetaRefNominalFull = thetaRefNominalFull;
metrics.desiredEEInterp = desiredEEInterp;
metrics.desiredRPYInterp = desiredRPYInterp;
metrics.refTimesGlobal = refTimesGlobal;
metrics.trackingStartIdx = trackingStartIdx;
metrics.directionSignFull = directionSignFull;
metrics.v_long = v_long;
metrics.v_lat = v_lat;
metrics.omegaBase = omegaBase;
metrics.vBaseWorldMag = vBaseWorldMag;
metrics.cmdHistory = cmdHistory;
metrics.eeMetrics = eeMetrics;
metrics.eeErrorVec = eeErrorVec;
metrics.eeErrorNorm = eeErrorNorm;
metrics.eeErrorVecTracking = eoVecTrack;
metrics.eeErrorNormTracking = eoNormTrack;
metrics.maxEEErrorTracking = maxEEErrorTracking;
metrics.meanEEErrorTracking = meanEEErrorTracking;
metrics.maxEEErrorTotal = maxEEErrorTotal;
metrics.meanEEErrorTotal = meanEEErrorTotal;
metrics.eeSpeed = eeSpeed;
metrics.eeAccelMag = eeAccelMag;
metrics.eeJerkMag = eeJerkMag;
metrics.scaleFactor = scaleFactor;
metrics.syncDiag = syncDiag;
metrics.baseVelocityTrack = baseVelocityTrack;
end
