# Time History Construction

This document captures the current approach for unfolding the 50-sample reference trajectory (0.1 s spacing) into the timeline used by the whole-body controller.

## Inputs
- Reference EE poses (`traj.eePoses`)
- Reference base waypoints (`traj.baseWaypoints`)
- Optional timestamps (defaults to uniform 0.1 s)

## Ramp-Up Stages
1. **Arm Ramp (base fixed)**
   - Duration: `armPhaseSteps = max(ceil(2.0/sample_dt), 20)` (≥ 2 s)
   - Base pose held at the fixed home `(x=-2, y=-2, yaw=0)`
   - Desired EE pose fixed at the first waypoint
   - Arm follows a GIK-generated warm-up trajectory toward a virtual target that matches the first waypoint orientation and height while the base remains fixed at home

2. **Chassis Ramp (EE fixed)**
   - Duration: `basePhaseSteps = max(ceil(max(posError/0.25, yawError/45°)/sample_dt), 20)` (≥ 1.5 s)
   - Base follows a Hybrid A* path from the home pose to the first waypoint while holding the desired EE pose fixed
   - Ramp motion capped at 0.2 m/s; the tracking phase later restores the 0.6 m/s limit
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
- Benchmark the new GIK/Hybrid A* ramp against legacy runs and tighten solver tolerances or collision bounds if residual IK errors appear
- Refine tracking-stage tuning (velocity limits, synchronized base gains) to bring the ~0.13 m error spike inside tolerance
- Extend the Hybrid A* planner with obstacle awareness once the chassis footprint and scene geometry are finalized
