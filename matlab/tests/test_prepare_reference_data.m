function tests = test_prepare_reference_data
%TEST_PREPARE_REFERENCE_DATA Validate the reference preprocessing helper.
%   Ensures wbc.prepare_reference_data reproduces the legacy trajectory
%   preparation logic used inside rt_whole_body_controller.

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
baseDir = fileparts(mfilename('fullpath'));
matlabRoot = fileparts(baseDir);
addpath(matlabRoot); %#ok<NASGU>

opts = struct('source', "demo_arc", ...
              'duration', 8.0, ...
              'scale', 1.0, ...
              'file', "", ...
              'sample_dt', 0.1, ...
              'max_speed', 3.0);

testCase.TestData.Options = opts;
end

function teardownOnce(~)
% restore path implicitly on MATLAB exit
end

function testMatchesLegacyImplementation(testCase)
opts = testCase.TestData.Options;
refData = wbc.prepare_reference_data(opts);

% Legacy inline computation copied from pre-refactor script
traj = generate_external_trajectory(opts);
refPositionsWorld = traj.eePoses(:,1:3);
if size(traj.eePoses,2) >= 6
    refRPYWorld = traj.eePoses(:,4:6);
else
    refRPYWorld = zeros(size(refPositionsWorld));
end

numRefSamples = size(refPositionsWorld,1);
refTimes = traj.timestamps;
if isempty(refTimes)
    refTimes = (0:numRefSamples-1)' * opts.sample_dt;
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

[thetaGeomNominal, ~, ~] = compute_base_heading_legacy(baseWaypointsNominal);
if size(refRPYWorld,2) >= 3
    yawDesiredAll = wrapToPi(refRPYWorld(:,3));
else
    yawDesiredAll = [];
end
if ~isempty(yawDesiredAll)
    thetaRefNominal = yawDesiredAll;
else
    thetaRefNominal = thetaGeomNominal;
end

% Assertions
verifyEqual(testCase, refData.refPositionsWorld, refPositionsWorld, 'AbsTol', 1e-12);
verifyEqual(testCase, refData.refRPYWorld, refRPYWorld, 'AbsTol', 1e-12);
verifyEqual(testCase, refData.refTimes, refTimes, 'AbsTol', 1e-12);
verifyEqual(testCase, refData.rawBaseWaypoints, rawBaseWaypoints, 'AbsTol', 1e-12);
verifyEqual(testCase, refData.baseWaypointsNominal, baseWaypointsNominal, 'AbsTol', 1e-12);
verifyEqual(testCase, refData.baseWaypointsGeomYaw, thetaGeomNominal, 'AbsTol', 1e-12);
verifyEqual(testCase, refData.baseYawReference, thetaRefNominal, 'AbsTol', 1e-12);
verifyEqual(testCase, refData.yawDesiredAll, yawDesiredAll, 'AbsTol', 1e-12);
end

function [thetaRef, arcLen, segmentDist] = compute_base_heading_legacy(baseWaypoints)
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
