# Whole-Body Camera Tracking Architecture

## Overview
- Hybrid ROS 2 + MoveIt stack delivering arm-and-base whole-body motion to follow streamed camera poses at 10 Hz.
- Supervisor orchestrates staged execution: arm ramp alignment, base ramp positioning, and synchronized tracking while maintaining collision-free motion.
- Dedicated planners provide arm trajectories (MoveIt) and planar base paths (Hybrid A*), with execution nodes publishing joint commands and diff-drive velocity setpoints.
- Obstacle manager seeds the planning scene and base occupancy grid with static/dynamic obstacles to emulate perception inputs.
- Tooling includes plotting, RViz launch files, and regression tests to validate planners and state-machine behaviour.

## Packages & Roles
- `mobile_arm_whole_body_interfaces`: Message/service definitions for camera targets, phase state, base ramp/arm ramp/tracking solvers.
- `mobile_arm_whole_body_control`: Runtime nodes, planners, supervisor, and shared trajectory utilities.
- `mobile_arm_whole_body_bringup`: Launch files, RViz configuration, and orchestration scripts.
- `docs`, `analysis`, `tools`: Documentation, logs, and offline plotting scripts for validation.

## Flow Overview

```mermaid
flowchart LR
  source["`camera_path/target`
CameraPoseTarget] -->|queue| ingest{{Trajectory Ingestor}}
  ingest -->|`camera_path/active`| supervisor{{Whole-Body Supervisor}}
  ingest -->|`camera_path/status`| monitor[Status Consumers]
  supervisor -->|`/camera_path/request_next` (Trigger)| ingest

  supervisor -->|Arm ramp request| armDecision{"Arm ramp needed?"}
  armDecision -- No --> baseStage
  armDecision -- Yes --> armPlan[/`/whole_body/plan_arm_ramp`
Whole-Body Planner/]
  armPlan --> armPub[/`/whole_body/arm_plan`
Arm Trajectory Publisher/]
  armPub --> armExec["`whole_body/arm_command`
Controller HW/SIM"]

  armPlan -. planning scene .-> planner[MoveIt Planning Scene]
  obstacles[/Obstacle Manager/] -->|`/whole_body/obstacle_grid`
  | basePlanner[/Base Planner/]
  obstacles -. collision objects .-> planner

  supervisor -->|Base ramp request| baseStage{"Base ramp needed?"}
  baseStage -- No --> trackStart
  baseStage -- Yes --> basePlanner[/`/whole_body/plan_base_ramp`
Hybrid A*/]
  basePlanner -->|`/whole_body/base_plan`| baseCmd[/Base Motion Commander/]
  baseCmd --> cmdOut["`whole_body/cmd_vel`
Chassis Controller"]
  baseCmd <-- `whole_body/odom` --> odom[Diff-Drive Odometry]

  trackStart{{Tracking stage}}
  baseStage --> trackStart
  armDecision --> trackStart
  supervisor -->|Tracking request| trackingPlan[/`/whole_body/plan_tracking_segment`
Whole-Body Planner/]
  trackingPlan -->|Split arm/base| armPub
  trackingPlan -->|`/whole_body/base_plan`| baseCmd
  trackingPlan -. collision checks .-> planner

  basePlanner -->|Nominal speeds| supervisor
  armPlan -->|Estimated duration| supervisor
  trackingPlan -->|Phase state| supervisor
```

## Core Nodes
### Trajectory Ingestor (`mobile_arm_whole_body_control/src/trajectory_ingestor_node.cpp`)
- Receives `camera_path/target` queue, assigns IDs, and exposes the current active target on `camera_path/active`.
- Publishes status transitions (`NEW`, `ACTIVE`, `COMPLETE`) and responds to `/camera_path/request_next` to advance streaming playback.

### Whole-Body Supervisor (`mobile_arm_whole_body_control/src/whole_body_supervisor_node.cpp`)
- State machine covering modes `IDLE`, `ARM_RAMP`, `BASE_RAMP`, `TRACKING` with 100 ms update cadence.
- Tracks configurable initial base and EE poses (`initial_base_pose`, `initial_ee_pose`, `reference_frame`) plus a `base_goal_offset` heuristic for chassis placement.
- Calls planners sequentially:
  - `/whole_body/plan_arm_ramp` (MoveIt arm solver) and republishes returned trajectory to `/whole_body/arm_plan`.
  - `/whole_body/plan_base_ramp` (Hybrid A*) using current base pose and computed goal offset, publishing the resulting path to `/whole_body/base_plan`.
  - `/whole_body/plan_tracking_segment` for whole-body MoveIt planning, splitting results into arm/base outputs before publishing.
- Maintains latest base/EE pose state for successive planning requests; normalizes quaternions via shared utilities.
- Coordinates completion with the ingestor by calling `/camera_path/request_next` once tracking planners succeed.

### Whole-Body Planner (`mobile_arm_whole_body_control/src/whole_body_planner_node.cpp`)
- Wraps MoveIt MoveGroup for Manipulator and Whole-Body groups.
- Services:
  - `/whole_body/plan_arm_ramp`: pose-target MoveIt plan with tolerance controls.
  - `/whole_body/plan_tracking_segment`: whole-body plan, split into arm trajectory and base path via utility library.
- Collision validation for synchronized samples using the planning scene; detailed diagnostics include colliding links, penetration depth, and offending base pose.

### Base Planner (`mobile_arm_whole_body_control/src/base_planner_node.cpp`)
- Hybrid A*-style grid search over static/dynamic occupancy grids supplied via parameter or `/whole_body/obstacle_grid` (50% threshold by default).
- Service `/whole_body/plan_base_ramp` returns `nav_msgs/Path` plus nominal velocity/yaw-rate hints.
- Supports transient obstacle grids, static disc obstacles, and straight-line fallback when no map is available.

### Arm Trajectory Publisher (`mobile_arm_whole_body_control/src/arm_trajectory_publisher_node.cpp`)
- Listens to `/whole_body/arm_plan`, resamples the trajectory at `control_period` (default 50 ms), interpolates between waypoints, and publishes `/whole_body/arm_command` for downstream controllers.

### Base Motion Commander (`mobile_arm_whole_body_control/src/base_motion_commander_node.cpp`)
- Pure-pursuit style tracker consuming `/whole_body/base_plan` and `/whole_body/odom`.
- Parameters: `lookahead_distance`, `slowdown_distance`, `control_period`, PID gains, goal tolerances.
- Publishes `/whole_body/cmd_vel`, clamps linear/angular velocities, and logs completion once inside tolerance.

### Obstacle Manager (`mobile_arm_whole_body_control/src/obstacle_manager_node.cpp`)
- Generates occupancy grid and MoveIt collision objects from configured disc obstacles (including latest 14th waypoint disc and ramp-stage blocker).
- Publishes `/whole_body/obstacle_grid` (latched) and applies collision objects via PlanningSceneInterface.

### Planner Stub (`mobile_arm_whole_body_control/src/planner_stub_node.cpp`)
- Lightweight service set for test harnesses when full planners are offline.

## Shared Trajectory Utilities (`mobile_arm_whole_body_control/src/trajectory_utils.cpp`)
- `split_whole_body_plan`: separates world joint variables from manipulator joints, returning arm-only trajectory and base SE(2) path.
- `trajectory_requires_motion`: joint tolerance check used for ramp skip logic.
- `compute_base_goal_pose`: heuristically offsets the chassis backward along EE yaw while matching desired Z.
- `normalize_quaternion` and `quaternion_yaw`: geometry helpers shared by planner/supervisor.

## Data Flow by Stage
1. **Ingestion**: `camera_path/target` → queue → `camera_path/active` (first target) with status tracking.
2. **Arm Ramp**: Supervisor seeds MoveIt with current state, receives `trajectory_msgs/JointTrajectory`, applies skip logic, and republishes on `/whole_body/arm_plan` for execution.
3. **Base Ramp**: Supervisor computes SE(2) goal (offset heuristic), calls Hybrid A* planner, publishes `/whole_body/base_plan`, and updates internal base pose.
4. **Tracking**: Whole-body MoveIt plan split into arm/base; supervisor publishes results while the commander and arm publisher stream commands.
5. **Completion**: Upon tracking success, supervisor calls `/camera_path/request_next`; ingestor activates the next queued target.

## Topics & Services (Primary)
- Topics: `camera_path/target`, `camera_path/active`, `camera_path/status`, `/whole_body/arm_plan`, `/whole_body/arm_command`, `/whole_body/base_plan`, `/whole_body/cmd_vel`, `/whole_body/state`, `/whole_body/obstacle_grid`, `/whole_body/odom`.
- Services: `/camera_path/request_next`, `/whole_body/plan_arm_ramp`, `/whole_body/plan_base_ramp`, `/whole_body/plan_tracking_segment`.

## Configuration Highlights
- Supervisor parameters: `arm_joint_skip_tolerance`, `base_path_skip_tolerance`, `initial_base_pose`, `initial_ee_pose`, `reference_frame`, `base_goal_offset`.
- Base commander: `control_period`, `lookahead_distance`, `slowdown_distance`, `max_speed`, `max_yaw_rate`, gain values, goal tolerances.
- Base planner: map resolution/size/origin, static disc obstacles, obstacle threshold.
- Obstacle manager: disc obstacle list, map origin, frames (`map_frame`, `world_frame`).

## Testing & Tooling
- Unit tests (`test/test_trajectory_utils.cpp`) cover plan splitting, motion detection, base goal heuristics, and quaternion normalization (`colcon test --packages-select mobile_arm_whole_body_control --ctest-args -R test_trajectory_utils`).
- Launch test scaffold (currently disabled in sandbox) for supervisor/ingestor handshake.
- RViz & rosbag workflow documented in `docs/visualization.md` with launch entry point `mobile_arm_whole_body_bringup/launch/whole_body_streaming.launch.py`.
- Plotting script `tools/plot_whole_body_log.py` accepts CSV exports from rosbag topics for visual diagnostics (ramp/tracking, obstacles, errors).

## Execution Snapshot
- Bringup launch spins: obstacle manager, MoveIt planning stack, base planner, supervisor, trajectory ingestor, arm publisher, base commander, RViz configuration.
- Typical run sequence: stream camera target, supervisor plans arm ramp → base ramp → tracking, execution nodes follow `/whole_body/arm_plan` and `/whole_body/base_plan`, rosbag/RViz capture dynamics.

## Open Issues & Next Steps
- Tune `base_goal_offset` against actual chassis footprint and obstacle geometry; consider perception-driven offsets or full pose constraints.
- Replace planner stub in integration tests once sandbox permits launch-based testing.
- Extend base commander with odom feedback filters, velocity smoothing, and saturation-aware logging.
- Integrate perception pipeline for dynamic obstacles beyond scripted discs.
- Validate tracking behaviour in RViz/in simulation, capturing stage-by-stage metrics for regression.
