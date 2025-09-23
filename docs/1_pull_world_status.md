# 1_pull_world Ramp & Tracking Status

## Arm Ramp Evaluation
- Current arm ramp keeps the chassis fixed at home `[-2, -2, 0]` while lifting the end-effector to the first waypoint's height and orientation.
- XY remains at the solver-selected "virtual" position (`[0.390, -0.153]`), which is still ~0.25 m away from the first desired waypoint `[0.550, 0.080]`.
- Because the chassis does not move during this stage, the arm ramp alone cannot reach the first waypoint; large offsets remain when tracking begins.

## Chassis Ramp & Tracking Metrics (latest run)
- Arm ramp samples: 20 (2.0 s); chassis ramp samples: 142 (14.2 s).
- Chassis ramp goal: `[0, 0, -1.571]`; base travels ~2.83 m before tracking starts.
- Tracking error with ramp blended but without chassis realignment: max 0.195 m (mean 0.075 m); including ramp: max 2.481 m, mean 0.440 m.
- Repeated joint-limit warnings from GIK indicate the remaining XY gap exceeds the arm's comfortable workspace.

## Visualization
- `plot_ramp_overview` now renders the robot, chassis mesh, arm ramp path, and desired first waypoint for inspection (`matlab/outputs/ramp_overview_file.{png,fig}`).

## Next Steps
1. Allow the chassis ramp (Hybrid A*) to move the base near the first waypoint before tracking; this should eliminate the remaining XY offset.
2. Once the chassis ramp closes the gap, rerun `rt_whole_body_controller` to confirm tracking errors drop below tolerance.
3. Revisit joint-limit tuning and solver tolerances after the ramp/planner alignment is corrected.
