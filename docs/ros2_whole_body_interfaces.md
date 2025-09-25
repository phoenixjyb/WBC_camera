# ROS 2 Whole-Body Coordinator Interfaces

Proposed communication contracts for the C++ port of the MATLAB whole-body controller targeting ROS 2 Humble on Jetson AGX Orin. The design keeps parity with the MATLAB planning stages (arm ramp, chassis ramp, trajectory tracking) while respecting MoveIt 2.5.9 conventions.

## State Machine Overview
- **Node:** `whole_body_coordinator` (`rclcpp::Node`)
- **Primary states:** `Idle` → `ArmRamp` → `ChassisRamp` → `Tracking`
- **Transitions:**
  - `Idle → ArmRamp` when an EE trajectory goal is accepted.
  - `ArmRamp → ChassisRamp` after the arm reaches ramp target and reports convergence.
  - `ChassisRamp → Tracking` once base warm-up finishes and synchronization is established.
  - Any state → `Idle` on abort/interrupt or completion.

The node keeps a rolling `CoordinatorStatus` message (below) so external supervisors—Nav2, task planners, GUIs—can monitor progress.

## Topics
| Topic | Direction | Type | Notes |
|-------|-----------|------|-------|
| `/whole_body/ee_goal` | Subscribe | `trajectory_msgs/msg/JointTrajectory` or `moveit_msgs/msg/MotionPlanRequest` | MoveIt client posts whole-body plan inputs. Trajectory form preferred for direct replay; request form used when planning inside coordinator. |
| `/whole_body/trajectory_cmd` | Publish | `mobile_arm_whole_body/msg/WholeBodyCommand` | Bundles arm trajectory, base profile, EE references, synchronization factors, and rich metrics for logging or mirroring. |
| `/whole_body/arm_cmd` | Publish | `trajectory_msgs/msg/JointTrajectory` | Mirrors ramp + tracking arm segments; fed to arm controller when direct streaming required. |
| `/whole_body/base_cmd` | Publish | `geometry_msgs/msg/TwistStamped` | Base velocity command synchronized to arm timeline; replaceable with diff-drive interface. |
| `/whole_body/state` | Publish | `mobile_arm_whole_body/msg/CoordinatorStatus` | Contains current mode, time indices, EE errors, retimer scale factor, warnings. |
| `/diagnostics` | Publish | `diagnostic_msgs/msg/DiagnosticArray` | Aggregates IK convergence, joint limit, tracking errors. |
| `/whole_body/visualization` | Publish | `nav_msgs/msg/Path` + `sensor_msgs/msg/JointState` | Optional RViz overlays for base path and synchronized arm trajectory. |

## Services & Actions
| Name | Type | Purpose |
|------|------|---------|
| `/whole_body/start` | `mobile_arm_whole_body/action/ExecuteWholeBody` (goal phase) | Accepts EE trajectory (either MoveIt request or pre-timed arm/base) and kicks off coordinator. Returns acceptance result and tracking identifiers. |
| `/whole_body/cancel` | `std_srvs/srv/Trigger` | Aborts the current execution; coordinator transitions to Idle and publishes stop commands. |
| `/whole_body/pause` | `std_srvs/srv/SetBool` | Optional pause/resume hook for higher-level supervisors. |
| `/whole_body/reseed_home` | `std_srvs/srv/Trigger` | Forces the ramp planner to recompute warm-up targets (used after manual repositioning). |
| `/whole_body/ExecuteWholeBody` | Action (`mobile_arm_whole_body_msgs/action/ExecuteWholeBody`) | Long-running action variant combining start/cancel/feedback; feedback stream includes stage transitions and EE error statistics. |

## Message Sketches
`mobile_arm_whole_body/msg/WholeBodyCommand` (see `msg/WholeBodyCommand.msg`):
```text
std_msgs/Header header
string source_id
trajectory_msgs/JointTrajectory arm_trajectory
mobile_arm_whole_body/BaseProfile base_profile
mobile_arm_whole_body/StageInfo[] stages
mobile_arm_whole_body/EndEffectorReference ee_reference
mobile_arm_whole_body/SyncStatus sync
mobile_arm_whole_body/WholeBodyMetrics metrics
```

`mobile_arm_whole_body/msg/BaseProfile`:
```text
std_msgs/Header header
builtin_interfaces/Duration[] time_from_start
geometry_msgs/Pose[] pose
geometry_msgs/Twist[] twist
float32[] direction_sign
float32[] curvature
float32[] yaw_reference
float32[] yaw_actual
float32[] arc_length
float32[] velocity_scale
```

`mobile_arm_whole_body/msg/StageInfo`:
```text
uint8 stage_id  # enumerated ramp/tracking identifiers
string label
uint32 start_index
uint32 end_index
builtin_interfaces/Duration start_time_from_start
builtin_interfaces/Duration end_time_from_start
builtin_interfaces/Duration duration
float32 progress
```

`mobile_arm_whole_body/msg/EndEffectorReference`:
```text
builtin_interfaces/Duration[] time_from_start
geometry_msgs/Pose[] desired
geometry_msgs/Pose[] actual
float32[] position_error_norm
float32[] orientation_error_norm
float32 max_error_position
float32 mean_error_position
float32 max_error_position_tracking
float32 mean_error_position_tracking
```

`mobile_arm_whole_body/msg/SyncStatus`:
```text
float32 scale_factor
float32[] scale_history
float32[] direction_sign
float32 base_speed_limit
float32 base_yaw_rate_limit
float32 ramp_base_speed_limit
float32 ramp_base_yaw_rate_limit
float32 theta_ramp_end
```

`mobile_arm_whole_body/msg/WholeBodyMetrics`:
```text
float32 base_speed_max
float32 base_yaw_rate_max
float32 base_yaw_deviation_max
float32 base_yaw_deviation_mean
float32 ee_error_max
float32 ee_error_mean
float32 ee_error_max_total
float32 ee_error_mean_total
float32[] ee_speed
float32[] ee_accel
float32[] ee_jerk
float32[] arm_velocity
float32[] arm_acceleration
float32[] cmd_velocity_longitudinal
float32[] cmd_velocity_lateral
float32[] cmd_yaw_rate
```

`mobile_arm_whole_body/msg/CoordinatorStatus`:
```text
std_msgs/Header header
uint8 state
float32 state_progress
builtin_interfaces/Duration state_elapsed
builtin_interfaces/Duration state_remaining
string active_request_id
string last_event
mobile_arm_whole_body/StageInfo current_stage
mobile_arm_whole_body/WholeBodyMetrics metrics
mobile_arm_whole_body/SyncStatus sync
bool tracking_warning
bool ik_warning
bool joint_limit_warning
bool base_limit_warning
bool collision_warning
```

`mobile_arm_whole_body/action/ExecuteWholeBody` couples rich goal, result, and feedback payloads:

```text
# Goal
string request_id
geometry_msgs/PoseStamped[] ee_waypoints
trajectory_msgs/JointTrajectory seed_arm
mobile_arm_whole_body/BaseProfile seed_base
mobile_arm_whole_body/WholeBodyCommand preset_command
bool allow_replan
bool record_diagnostics
---
# Result
bool success
string message
mobile_arm_whole_body/WholeBodyCommand executed_command
mobile_arm_whole_body/WholeBodyMetrics final_metrics
---
# Feedback
mobile_arm_whole_body/CoordinatorStatus status
```

## Parameterization
`whole_body_coordinator` retrieves limits and planner knobs from YAML loaded under `mobile_arm_whole_body/config/`. Key groups:
- `arm_limits`: velocity, acceleration, jerk limits (mirrors MATLAB `arm_joint_limits`).
- `base_limits`: `v_max`, `omega_max`, `lat_acc_max`, curvature caps.
- `ramp`: warm-up durations, IK tolerances, seed strategies.
- `retimer`: sampling resolution, minimum segment time, scale-factor clamp.
- `collision`: MoveIt planning scene links, allowed collisions, environment meshes (matching `wbc.configure_arm_collision_avoidance`).

## Node Integration Checklist
1. **MoveIt Interface:** use the Planning Scene monitor and `move_group` action to request whole-body trajectories when only EE poses are supplied. Maintain parity with MATLAB IK warm-start by caching solution seeds.
2. **Trajectory Synchronization:** port `wbc.sync_base_and_arm` to C++; expose diagnostics identical to MATLAB (scale history, direction sign, base path).
3. **Metrics Publishing:** compute EE error, base yaw deviation, and stage boundaries in C++; publish both in `CoordinatorStatus` and latched `/diagnostics` updates.
4. **Baseline Comparison:** drop the `.mat` outputs from `run_capture_baseline` into a ROS 2 test fixture (e.g., gtest + yaml) so tracked errors stay within tolerances.
5. **Threading:** keep arm command streaming and base command streaming in separate timers to respect controller update rates (e.g., 100 Hz base, 250 Hz arm) while referencing shared synchronized timeline.

## Mapping to MATLAB Helpers
| MATLAB helper | C++ equivalent responsibility |
|---------------|--------------------------------|
| `wbc.prepare_reference_data` | Trajectory ingestion / smoothing component feeding MoveIt client. |
| `wbc.plan_ramp_segment` | Ramp planner nodelet (uses MoveIt IK + SE(2) path for base). |
| `wbc.solve_arm_ik` | MoveIt `RobotState::setFromIK` with warm-start caching. |
| `wbc.sync_base_and_arm` | Trajectory retimer + synchronized base profiler. |
| `wbc.compute_metrics` | Diagnostics module populating ROS status messages and logger output. |

## Testing Hooks
- Launch test: `ros2 launch mobile_arm_whole_body whole_body_coordinator.launch.py` with FakeController ensures command topics publish nominal data.
- Regression test: `launch_testing` harness loads MATLAB baseline (CSV/JSON) and asserts EE error, base scale factor, and arm joint seeds within tolerances.
- Hardware shim: optional `chair_mode` parameter bypasses base commands and publishes EE-only diagnostics for lab benches.

These contracts will guide the `wbc_core` C++ library API and the surrounding ROS 2 nodes while keeping compatibility with existing MATLAB datasets.
