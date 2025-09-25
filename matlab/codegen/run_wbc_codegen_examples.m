function run_wbc_codegen_examples()
%RUN_WBC_CODEGEN_EXAMPLES Demonstrate MATLAB Coder invocations for core modules.
%   This script configures representative argument shapes and invokes
%   CODEGEN to produce C++ from the wbc_codegen helpers. Adjust the upper
%   bounds as needed to match application-specific horizons.

%% sync_base_and_arm
N_MAX = 1000;
M_MAX = 6;

baseWaypointsRef = coder.typeof(0, [N_MAX, 2], [true, false]);
thetaRef = coder.typeof(0, [N_MAX, 1], true);
arcLen = coder.typeof(0, [N_MAX, 1], true);
refTimes = coder.typeof(0, [N_MAX, 1], true);
refPositionsWorld = coder.typeof(0, [N_MAX, 3], [true, false]);
refRPYWorld = coder.typeof(0, [N_MAX, 3], [true, false]);
armTrajectoryRef = coder.typeof(0, [N_MAX, M_MAX], [true, false]);
baseLimitsProto = struct('v_max', 0, 'omega_max', 0, 'lat_acc_max', 0);
baseLimitsType = coder.typeof(baseLimitsProto);
baseSpeedLimit = coder.typeof(0);
thetaRampEnd = coder.typeof(0);

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
codegen('-config', cfg, ...
    'wbc_codegen.sync_base_and_arm', ...
    '-args', {baseWaypointsRef, thetaRef, arcLen, refTimes, ...
              refPositionsWorld, refRPYWorld, armTrajectoryRef, ...
              baseLimitsType, baseSpeedLimit, thetaRampEnd});

%% retime_joint_trajectory
waypoints = coder.typeof(0, [N_MAX, M_MAX], [true, false]);
maxVel = coder.typeof(0, [1, M_MAX], false);
maxAcc = coder.typeof(0, [1, M_MAX], false);
codegen('-config', cfg, ...
    'wbc_codegen.retime_joint_trajectory', ...
    '-args', {waypoints, maxVel, maxAcc, coder.typeof(0), coder.typeof(0)});

%% compute_tracking_metrics
metrics_desired = coder.typeof(0, [N_MAX, 3], [true, false]);
metrics_actual = coder.typeof(0, [N_MAX, 3], [true, false]);
basePose = coder.typeof(0, [N_MAX, 3], [true, false]);
tVec = coder.typeof(0, [N_MAX, 1], true);
directionSign = coder.typeof(0, [N_MAX, 1], true);
codegen('-config', cfg, ...
    'wbc_codegen.compute_tracking_metrics', ...
    '-args', {metrics_desired, metrics_actual, basePose, tVec, directionSign});

end

