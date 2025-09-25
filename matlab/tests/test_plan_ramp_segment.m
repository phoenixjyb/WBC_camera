function tests = test_plan_ramp_segment
%TEST_PLAN_RAMP_SEGMENT Validate ramp planner integration via top-level controller.
%   Ensures the ramp metadata and augmented waypoints follow expected
%   structure for the demo trajectory, guarding against regressions in the
%   new wbc.plan_ramp_segment helper.

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
baseDir = fileparts(mfilename('fullpath'));
matlabRoot = fileparts(baseDir);
addpath(matlabRoot);

opts = struct('traj_source', 'demo_arc', ...
              'traj_duration', 8.0, ...
              'traj_scale', 1.0, ...
              'enable_animation', false, ...
              'enable_visualization', false);

[results, diagnostics] = rt_whole_body_controller(opts);

testCase.TestData.Results = results;
testCase.TestData.Diagnostics = diagnostics;
testCase.TestData.HomePose = wbc.compute_home_base_pose();
end

function teardownOnce(~)
% nothing to clean up
end

function testRampMetadata(testCase)
info = testCase.TestData.Diagnostics.rampInfo;
verifyTrue(testCase, isstruct(info));
verifyTrue(testCase, info.enabled, 'Ramp should be enabled for demo trajectory');
verifyGreaterThan(testCase, info.armSteps, 0);
verifyGreaterThan(testCase, info.baseSteps, 0);
verifyEqual(testCase, info.steps, info.armSteps + info.baseSteps);
verifyBetween(testCase, info.baseYawRateLimit, 0.4, 0.6);
end

function testRampAlignment(testCase)
results = testCase.TestData.Results;
info = testCase.TestData.Diagnostics.rampInfo;
homePose = testCase.TestData.HomePose;

% First waypoint should match home base pose
verifyEqual(testCase, results.baseWaypoints(1, :), homePose(1:2), 'AbsTol', 1e-8);

% Ramp end should align with start of tracking segment (after arm steps)
endIdx = info.armSteps + info.baseSteps;
verifyEqual(testCase, results.baseWaypoints(endIdx, :), info.baseGoal, 'AbsTol', 1e-8);

% Confirm yaw reference timeline covers entire base waypoint sequence
verifyGreaterThanOrEqual(testCase, numel(results.baseYawReferenceTimeline), size(results.baseWaypoints, 1));
end

function testStageLabels(testCase)
results = testCase.TestData.Results;
verifyEqual(testCase, string(results.stageLabels(1)), "Arm ramp");
verifyEqual(testCase, string(results.stageLabels(2)), "Chassis ramp");
verifyEqual(testCase, string(results.stageLabels(end)), "Tracking");
end

function testSyncDiagnostics(testCase)
results = testCase.TestData.Results;
diag = testCase.TestData.Diagnostics;
info = diag.rampInfo;
syncData = diag.sync;

verifyEqual(testCase, syncData.scaleFactor, results.baseSyncScale, 'AbsTol', 1e-12);
verifyEqual(testCase, size(syncData.basePoseTrack,1), numel(syncData.armTimes));
trackStart = info.steps + 1;
verifyEqual(testCase, syncData.basePoseTrack(1,:), results.basePose(trackStart,:), 'AbsTol', 1e-10);
verifyEqual(testCase, syncData.thetaRefTimeline(1), results.baseYawReferenceTimeline(trackStart), 'AbsTol', 1e-10);
end

function verifyBetween(testCase, value, lowerBound, upperBound)
verifyGreaterThanOrEqual(testCase, value, lowerBound);
verifyLessThanOrEqual(testCase, value, upperBound);
end
