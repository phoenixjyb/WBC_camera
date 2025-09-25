function refData = prepare_reference_data(trajOpts)
%PREPARE_REFERENCE_DATA Load EE/base trajectories and compute nominal references.
%   REFDATA = WBC.PREPARE_REFERENCE_DATA(TRAJOPTS) wraps trajectory loading and
%   preprocessing performed at the top of rt_whole_body_controller.
%
%   Expected TRAJOPTS fields (same as legacy script):
%     source, duration, scale, file, sample_dt, max_speed.
%
%   Output fields:
%     - trajectory            : raw struct returned by generate_external_trajectory
%     - refPositionsWorld     : Nx3 EE positions in world frame
%     - refRPYWorld           : Nx3 EE roll/pitch/yaw (zeros if missing)
%     - refTimes              : Nx1 timestamps (seconds)
%     - rawBaseWaypoints      : Mx2 chassis waypoints prior to smoothing
%     - baseWaypointsNominal  : Nx2 smoothed/synchronized chassis waypoints
%     - baseWaypointsGeomYaw  : Nx1 geometric heading derived from waypoints
%     - baseYawReference      : Nx1 heading after fusing desired EE yaw (if available)
%     - yawDesiredAll         : Nx1 desired yaw extracted from EE poses (can be empty)
%
%   The helper keeps behaviour identical to the pre-refactor script so tests
%   comparing baseline results remain valid.

arguments
    trajOpts struct
end

traj = generate_external_trajectory(trajOpts);

refPositionsWorld = traj.eePoses(:,1:3);
if size(traj.eePoses,2) >= 6
    refRPYWorld = traj.eePoses(:,4:6);
else
    refRPYWorld = zeros(size(refPositionsWorld));
end

numRefSamples = size(refPositionsWorld, 1);
refTimes = traj.timestamps;
if isempty(refTimes)
    sampleDt = getfieldwithdefault(trajOpts, 'sample_dt', 0.1);
    refTimes = (0:numRefSamples-1)' * sampleDt;
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

[thetaGeomNominal, ~, ~] = compute_base_heading_local(baseWaypointsNominal);

yawDesiredAll = [];
if size(refRPYWorld,2) >= 3
    yawDesiredAll = wrapToPi(refRPYWorld(:,3));
end

if ~isempty(yawDesiredAll)
    thetaRefNominal = yawDesiredAll;
else
    thetaRefNominal = thetaGeomNominal;
end

refData = struct();
refData.trajectory = traj;
refData.refPositionsWorld = refPositionsWorld;
refData.refRPYWorld = refRPYWorld;
refData.refTimes = refTimes;
refData.rawBaseWaypoints = rawBaseWaypoints;
refData.baseWaypointsNominal = baseWaypointsNominal;
refData.baseWaypointsGeomYaw = thetaGeomNominal;
refData.baseYawReference = thetaRefNominal;
refData.yawDesiredAll = yawDesiredAll;
end

function [thetaRef, arcLen, segmentDist] = compute_base_heading_local(baseWaypoints)
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

function val = getfieldwithdefault(s, fieldName, defaultVal)
if isfield(s, fieldName) && ~isempty(s.(fieldName))
    val = s.(fieldName);
else
    val = defaultVal;
end
end
