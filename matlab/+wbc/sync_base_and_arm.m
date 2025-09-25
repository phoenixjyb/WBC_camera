function syncOut = sync_base_and_arm(baseWaypointsRef, thetaRef, arcLen, refTimes, refPositionsWorld, refRPYWorld, armTrajectoryRef, baseLimits, baseSpeedLimit, thetaRampEnd) %#codegen
%SYNC_BASE_AND_ARM Retimes the arm trajectory and synchronizes the chassis.
%   SYNCOUT = WBC.SYNC_BASE_AND_ARM(...) wraps the legacy combination of
%   helpers.retime_joint_trajectory and synchronize_base_trajectory, and
%   reconstructs desired EE poses for downstream IK.
%
%   Required inputs mirror the values computed in rt_whole_body_controller
%   just before synchronization.

arguments
    baseWaypointsRef double
    thetaRef double
    arcLen double
    refTimes double
    refPositionsWorld double
    refRPYWorld double
    armTrajectoryRef double
    baseLimits struct
    baseSpeedLimit double
    thetaRampEnd double = thetaRef(1)
end

armLimitCfg = arm_joint_limits();
armVelLimits = armLimitCfg.velocity;
armAccLimits = armLimitCfg.acceleration;

[armTimesTrack, armTrajectoryTimedTrack, armVelTimedTrack, retimeInfo] = helpers.retime_joint_trajectory(armTrajectoryRef, ...
    MaxVelocity=armVelLimits, MaxAcceleration=armAccLimits, TimeStep=0.05, MinSegmentTime=0.2);
armTimesTrack = armTimesTrack(:);
armTrajectoryTimedTrack = armTrajectoryTimedTrack;
armVelTimedTrack = armVelTimedTrack;

[baseXTrack, baseYTrack, thetaTrack, vBaseTrack, omegaBaseTrack, scaleFactor, armTimesTrack, armVelTimedTrack, retimeInfo, syncDiag] = ...
    synchronize_base_trajectory(baseWaypointsRef, thetaRef, arcLen, refTimes, armTimesTrack, armVelTimedTrack, retimeInfo, baseLimits, baseSpeedLimit, thetaRampEnd);
poseHistoryTrack = [baseXTrack, baseYTrack, thetaTrack];
thetaRefSyncTrack = syncDiag.thetaRefTimeline;

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
for idx = 1:min(3, size(refRPYWorld,2))
    desiredRPYTrack(:, idx) = wrapToPi(interp1(timeParamRef, unwrap(refRPYWorld(:, idx)), timeParamTrack, 'pchip', 'extrap'));
end

if isempty(refTimes)
    refTimesAligned = zeros(0,1);
else
    refTimesAligned = zeros(size(refTimes));
    if numel(armTimesTrack) > 1
        refTimesAligned = interp1(timeParamTrack, armTimesTrack, timeParamRef, 'pchip', 'extrap');
    else
        refTimesAligned(:) = armTimesTrack(1);
    end
end

poseTformsFinal = zeros(4,4,numel(armTimesTrack));
for k = 1:numel(armTimesTrack)
    Tbase = trvec2tform([baseXTrack(k), baseYTrack(k), 0]) * axang2tform([0 0 1 thetaTrack(k)]);
    Tworld = trvec2tform(desiredEETrack(k,:)) * eul2tform(desiredRPYTrack(k,:), 'XYZ');
    poseTformsFinal(:,:,k) = Tbase \ Tworld;
end

syncOut = struct();
syncOut.armTimes = armTimesTrack(:);
syncOut.armTrajectoryInitial = armTrajectoryTimedTrack;
syncOut.armVelocities = armVelTimedTrack;
syncOut.retimeInfo = retimeInfo;
syncOut.basePoseTrack = poseHistoryTrack;
syncOut.baseVelocity = struct('v', vBaseTrack, 'omega', omegaBaseTrack);
syncOut.scaleFactor = scaleFactor;
syncOut.syncDiag = syncDiag;
syncOut.thetaRefTimeline = thetaRefSyncTrack;
syncOut.desiredEETrack = desiredEETrack;
syncOut.desiredRPYTrack = desiredRPYTrack;
syncOut.poseTformsFinal = poseTformsFinal;
syncOut.refTimesAligned = refTimesAligned(:);
end

function [baseX, baseY, theta, vBase, omegaBase, scaleAccum, armTimes, armVel, retimeInfo, diagOut] = ...
    synchronize_base_trajectory(baseWaypointsRef, thetaRef, arcLen, refTimes, ...
                                armTimes, armVel, retimeInfo, baseLimits, baseSpeedLimit, initialHeading)
if nargin < 10 || isempty(initialHeading)
    initialHeading = thetaRef(1);
end
initialHeading = wrapToPi(initialHeading);
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
    thetaNominal(1) = initialHeading;

    heading = zeros(size(thetaNominal));
    dirLocal = ones(size(thetaNominal));
    heading(1) = initialHeading;
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
            deltaTarget = backwardDelta;
        else
            dirLocal(k) = 1;
            deltaTarget = forwardDelta;
        end

        dtStep = max(armTimes(k) - armTimes(k-1), 1e-6);
        maxDelta = baseLimits.omega_max * dtStep;
        deltaTarget = wrapToPi(deltaTarget);
        if abs(deltaTarget) > maxDelta
            deltaTarget = sign(deltaTarget) * maxDelta;
        end
        heading(k) = heading(k-1) + deltaTarget;
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

thetaUnwrapped = theta;
for k = 2:numel(thetaUnwrapped)
    dtStep = max(armTimes(k) - armTimes(k-1), 1e-6);
    maxDelta = baseLimits.omega_max * dtStep;
    delta = thetaUnwrapped(k) - thetaUnwrapped(k-1);
    if abs(delta) > maxDelta
        thetaUnwrapped(k) = thetaUnwrapped(k-1) + sign(delta) * maxDelta;
    end
end
theta = wrapToPi(thetaUnwrapped);
omegaBase = gradient(thetaUnwrapped, armTimes);
if numel(omegaBase) > 1
    omegaBase([1 end]) = omegaBase([2 end-1]);
end

diagOut = struct('thetaRefTimeline', wrapToPi(thetaInterp(:)), ...
                 'scaleHistory', scaleHistory, ...
                 'directionSign', dirSign);
end
