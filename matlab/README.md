# MATLAB Whole-Body Controller Toolkit

This folder outlines a MATLAB-based control workflow that mirrors the ROS 2 whole-body execution stack. Use these scripts to prototype controller behavior, tune parameters, and validate trajectories before wiring into ROS.

## Contents
- `moveit_joint_trajectory_playback.m` — top-level script coordinating trajectory ingestion, splitting, and command generation for MoveIt-generated joint paths.
- `split_trajectory.m` — function that separates MoveIt joint trajectories into planar and arm components.
- `diff_drive_tracker.m` — pure-pursuit-like tracker that converts planar paths into linear/angular velocity commands with curvature and acceleration limits.
- `arm_joint_controller.m` — stub illustrating how to forward arm joint trajectories to a low-level driver or simulator.
- `arm_joint_limits.m` — central definition of adjustable arm joint velocity/acceleration limits.
- `+helpers/plot_planar_path.m` — visualization helper for planar trajectories and tracker pose.
- `+helpers/retime_joint_trajectory.m` — time-parameterizes arm waypoints with velocity/acceleration limits.
- `+helpers/load_collision_mesh.m` — loads chassis/column STL geometry into a `collisionMesh` for GIK collision avoidance.

## Usage Workflow
1. Export or record a MoveIt `JointTrajectory` (JSON or MATLAB struct) containing the world joint and arm joints.
2. Load the trajectory into MATLAB (see examples in `whole_body_controller.m`).
3. Call `split_trajectory` to obtain the base and arm segments.
4. Use `helpers.retime_joint_trajectory` (powered by `arm_joint_limits.m`) to re-sample the arm waypoints so joint velocities stay within limits.
5. Feed the planar segment through `diff_drive_tracker` to produce commanded `v` and `omega` respecting configured limits (use `lat_error_gain` to penalize lateral error if desired).
6. Forward the arm segment to your arm simulator/controller (after retiming) via `arm_joint_controller`.
7. Use the plotting or animation helpers to verify base motion and end-effector tracking.

Tune `limits` and tracker gains in MATLAB and port the validated values back into the ROS configs (`config/base_limits.yaml`, `config/tracker.yaml`).

## Robotics System Toolbox Workflow
- `rt_whole_body_controller.m` — end-to-end example that leverages Robotics System Toolbox:
  - Imports the URDF into a `rigidBodyTree` via `importrobot`.
  - Uses `inverseKinematics` to realize arm joint trajectories from camera pose waypoints.
  - Simulates base motion with `controllerPurePursuit` and `differentialDriveKinematics` (built-in diff-drive tracker).
  - Produces command histories you can map directly into ROS trajectories.
- `rt_compute_arm_ik.m` — helper that wraps repeated IK calls and returns joint arrays aligned with the robot’s joint list.
- `generate_external_trajectory.m` — factory for demo and file-based trajectories (MAT/JSON/CSV) that feed both EE and base paths.
  Built-in presets: `demo_arc`, `demo_line`, `dolly_forward`, `orbit_left`, `crane_up`, `pan_tilt`.
  When loading JSON poses (e.g. `1_pull_world.json`), the script interpolates/smooths the chassis path, retimes the arm and base on a shared timeline, and enforces base velocity/yaw limits before generating commands.
  JSON pose sequences are assumed to be sampled at 100 ms; the loader densifies any segment that would exceed the configurable camera speed limit (default 3 m/s) to keep position deltas realistic. All analysis plots, synchronized trajectories, and animation videos are exported to `matlab/outputs/` (or a custom `output_dir`) for easier inspection.
- `run_camera_motion_tests.m` — batch runner that exercises the presets (no animation) and prints joint/base speed summaries.
- Sample external trajectory: drop a JSON with a `poses` array (position + quaternion) into this folder and run
  ```matlab
  traj_source = 'file';
  traj_file = '1_pull_world.json';
  traj_duration = 12;        % resample duration (seconds)
  enable_animation = false;  % set true for visuals
  rt_whole_body_controller
  ```
  Base waypoints will be derived by preserving the initial EE offset unless the file supplies `baseWaypoints`.
- `+helpers/plot_joint_trajectories.m` — plots arm joint angles vs time.
- `+helpers/plot_chassis_profile.m` — plots chassis x/y/yaw time profiles and planar path.
- `+helpers/animate_whole_body.m` — animates the arm and base motion with camera trajectory overlay.

Prefer this toolbox-backed flow when you have access to Robotics System Toolbox—it cuts down on custom control code and keeps MATLAB simulation behavior aligned with MathWorks robotics examples.
