# Real-Time Whole-Body Execution on AGX Orin

Design for deploying the controller as a streaming, low-latency ROS 2 stack that meets a 10 Hz end-effector setpoint rate while issuing high-rate arm/chassis commands.

## High-Level Data Flow
1. **Reference Ingress (`whole_body_input`)**
   - Subscribes to `/camera/ee_reference` (`geometry_msgs/PoseStamped`) at 10 Hz.
   - Optional velocity hints on `/camera/ee_velocity` (`geometry_msgs/TwistStamped`).
   - Maintains a sliding buffer (≥2 s) of time-stamped setpoints; oldest data flushed once consumed.
   - Detects discontinuities (pose jump, timestamp gap) and raises `ReplanRequest` to coordinator.

2. **Coordinator (`whole_body_coordinator`)**
   - Owns the mode state machine (`Idle`, `ArmRamp`, `ChassisRamp`, `Tracking`).
   - Consumes buffered reference windows, calls ramp planner when entering ramp states or on replan events.
   - Generates synchronized arm/base trajectories (mirroring MATLAB helpers) and streams commands.
   - Publishes rich diagnostics on `/whole_body/trajectory_cmd` and `/whole_body/state` using the new message set.

3. **Ramp Planner (`whole_body_ramp_planner`)**
   - Runs Hybrid A* in SE(2) against dynamic obstacle map.
   - Accepts ramp goals from coordinator, occupancy grids/obstacle arrays from perception.
   - Returns `mobile_arm_whole_body/WholeBodyCommand` segments ready for execution.

4. **Controller Outputs**
   - Arm joint commands: `/whole_body/arm_cmd` (`trajectory_msgs/JointTrajectoryPoint`) at 250 Hz into the arm controller.
   - Base commands: `/whole_body/base_cmd` (`geometry_msgs/TwistStamped`) at 100 Hz into chassis interface.
   - Diagnostics: `/whole_body/trajectory_cmd`, `/whole_body/state`, `/diagnostics`, `/whole_body/visualization`.

## Timing & Threading
- **Servo Clock:** Coordinator ticks at 10 Hz (`rclcpp::WallTimer` on `RCL_STEADY_TIME`) to ingest new reference points and resample trajectories.
- **High-Rate Publishers:** Separate callback groups + timers (100 Hz base, 250 Hz arm) using precomputed trajectories; each timer fetches interpolated samples for current time.
- **Ramp Planner Thread:** `rclcpp::executors::MultiThreadedExecutor` with `MutuallyExclusive` callback groups. Planner runs in dedicated thread to avoid blocking servo loop; results handed off via lock-free queue or `std::atomic<std::shared_ptr<WholeBodyCommand>>`.
- **CPU Affinity:** Pin servo + high-rate publishers to isolated cores using `systemd` or `taskset`; keep planner/perception threads on separate cores.

## QoS Profiles
| Topic | QoS | Notes |
|-------|-----|-------|
| `/camera/ee_reference` | Reliable, keep_last=5 | Ensure no lost setpoints; small depth prevents stale pile-up. |
| `/whole_body/trajectory_cmd` | Reliable, keep_last=1 | Always publish freshest aggregated data. |
| `/whole_body/arm_cmd` | Best effort, keep_last=1 | Timely delivery prioritized; controller holds last command. |
| `/whole_body/base_cmd` | Best effort, keep_last=1 | Similar to arm commands to avoid latency. |
| `/whole_body/state` | Reliable, keep_last=10 | Diagnostic subscribers get full history. |
| `/obstacles` | Reliable, keep_last=3 | Planner needs consistent view of dynamic obstacles. |

## Mode Management
- **Idle:** Zero commands, waiting for new goal. Buffer must accumulate ≥0.5 s of data before ramping.
- **ArmRamp:** Executes warm-up seeds via `wbc.plan_ramp_segment`; base holds home pose. Planner ensures IK feasibility before transitioning.
- **ChassisRamp:** Hybrid A* output drives base toward track segment start; dynamic obstacles trigger incremental replans (bounded horizon, e.g., 2 s lookahead).
- **Tracking:** Continuous retiming via `wbc.sync_base_and_arm`; at each 10 Hz servo tick, coordinator resamples latest buffer and updates high-rate publishers.
- **Replan Handling:** On obstacle/event, ramp planner recomputes; coordinator atomically swaps plan, logs `last_event`, and, if necessary, slows base via scale factor.

## Integration With MATLAB Helpers
| MATLAB Helper | C++ Component | Notes |
|---------------|---------------|-------|
| `wbc.prepare_reference_data` | Reference buffer + smoothing | Replace file-based input with streaming buffer but re-use smoothing logic. |
| `wbc.plan_ramp_segment` | Ramp planner node | Hybrid A* warm-up computed in C++; still returns structured `WholeBodyCommand`. |
| `wbc.solve_arm_ik` | MoveIt IK wrapper | Use cached seeds per servo tick; warm-start from latest solution. |
| `wbc.sync_base_and_arm` | Coordinator retimer | Applies same scale-factor logic for base limits. |
| `wbc.compute_metrics` | Metrics publisher | Populate metrics sections of `WholeBodyCommand` & `CoordinatorStatus`. |

## Dynamic Obstacle Handling
- Subscribe to `/obstacles/dynamic` (`mobile_arm_whole_body/ObstacleArray` TBD) and `/costmap` (`nav_msgs/OccupancyGrid`).
- Planner maintains short horizon (≈5 m, 3 s) and reuses partial solutions between calls.
- Use incremental Hybrid A* (reuse open list) for quick updates; set maximum plan time (e.g., 40 ms). If exceeded, keep current path and flag warning.

## Safety & Faults
- Command watchdogs on arm/base interfaces (timeout = 100 ms) to halt on missed updates.
- Coordinator monitors EE error; exceeding threshold triggers `STATE_ERROR` and `ExecuteWholeBody` result `success=false`.
- Collision warnings from perception propagate to `CoordinatorStatus.collision_warning` and slow base via scale factor reduction.

## Implementation Checklist
1. Scaffold new ROS 2 nodes (`whole_body_input`, `whole_body_coordinator`, `whole_body_ramp_planner`) within `mobile_arm_whole_body` package. ✅
2. Implement reference buffer with time-sorted deque and interpolation hooks. ✅ (`include/mobile_arm_whole_body/reference_buffer.hpp`)
3. Port MATLAB ramp/retime logic to C++ library (`wbc_core`) using Eigen & MoveIt utilities, leveraging the `+wbc_codegen/` entry points as the starting interface.
4. Integrate the MoveIt IK bridge (`mobile_arm_whole_body::MoveItIKSolver`) to replace MATLAB/GIK solves with `setFromIK` calls that honour collision checks in the MoveIt planning scene.
4. Integrate Hybrid A* with dynamic obstacles; validate planning latency on AGX Orin.
5. Build unit/integration tests: replay MATLAB baselines, simulate dynamic obstacles, measure timing.
6. Profile command latencies; adjust timer priorities and QoS as needed.

The `launch/whole_body_realtime.launch.py` file starts the three scaffolded nodes with default parameters, providing a convenient entry point for replaying recorded trajectories during development.

This layout satisfies the 10 Hz inflow requirement while ensuring high-frequency actuation, dynamic obstacle handling, and comprehensive telemetry for monitoring and debugging.
