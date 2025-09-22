# Time History Construction

This document captures the current approach for unfolding the 50-sample reference trajectory (0.1 s spacing) into the timeline used by the whole-body controller.

## Inputs
- Reference EE poses (`traj.eePoses`)
- Reference base waypoints (`traj.baseWaypoints`)
- Optional timestamps (defaults to uniform 0.1 s)

## Ramp-Up Stages
1. **Arm Ramp (base fixed)**
   - Duration: `armPhaseSteps = max(ceil(2.0/sample_dt), 20)` (≥ 2 s)
   - Base pose held at URDF home `(x=0, y=0, yaw=0)`
   - Desired EE pose fixed at the first waypoint
   - Arm interpolates toward that pose (current implementation is linear interpolation, planned upgrade to collision-aware GIK)

2. **Chassis Ramp (EE fixed)**
   - Duration: `basePhaseSteps = max(ceil(max(posError/0.25, yawError/45°)/sample_dt), 20)` (≥ 1.5 s)
   - Base interpolates from home to the first waypoint (will be replaced by planner to follow EE ground-plane offset)
   - Desired EE pose remains the first waypoint

Ramp metadata is stored in `results.baseInitialization` (armSteps, baseSteps, duration). The full reference timeline is prepended with these ramp samples, shifting the original timestamps backward.

## Tracking Phase
- IK computed at each reference sample (after the ramp) to produce the nominal arm trajectory relative to base
- `helpers.retime_joint_trajectory` retimes the arm under joint velocity/acceleration limits, producing ~285 samples in the current runs
- `synchronize_base_trajectory` now chooses the minimal yaw change per step so the chassis can roll backward (v_long < 0) instead of spinning 180°, logging direction signs in `syncDiag.directionSign`
- `trackingStartIndex = rampSteps + 1` marks the first tracking sample; metrics and animation captions use this index to distinguish ramp vs tracking

## Logged Outputs
- `eeErrorNormTracking`, `eeErrorVecTracking`, `maxTrackingError`, `meanTrackingError`
- `baseDirectionSign` (forward/backward indicator)
- `baseInitialization` (ramp metadata) and `baseSyncScaleHistory`

## Planned Improvements
- Replace the chassis ramp interpolation with a planner (e.g., hybrid A*) that moves the base to the EE ground-plane offset while respecting the chassis/column geometry and collision constraints
- Switch ramp and tracking solvers to `generalizedInverseKinematics` with collision avoidance to eliminate large IK residuals
- After planner/GIK integration, revisit tracking error (~0.13 m max) and tune the synchronized path accordingly
