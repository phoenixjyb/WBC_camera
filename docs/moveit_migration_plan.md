# MoveIt Migration Plan

## User Prompts
> hi, there, please have a detailed read of this project, and i would like to develop moveit based framework, and mirror all my matlab code functionality. we can sketch out a detailed migration and development plan. please have a go. then we conquer it step by step.  the communications between modules are via ros2 humble.
>
> the whole from work shall look like this:
>
> 1, system received desired trajectory data from a ros2 topic, the trajectory shall be a series of camera poses (in world cooridinate system), and it can be a format that we release data point one by one, and we move onto the next data to track only if the previous one is marked tracked. this is different compared with matlab simulation that we have a whole suite of data points one time.
>
> 2, the robot(chassis+arm) shall move from its designated location to the desired EE trajectory's 1st way point, with a stage called ramp up, and in which two stages are imposed, one for robotic arm to arrive at the same height (z direction), and keep the same orientation (roll yaw pitch) as the 1st desired waypoint, but the x and y can be relaxed. as what we have done in matlab, we can use IK to determine a desired location with x and y, and intropolate a series of points (essentially a series of joint positions for each of the joint on arm), for the arm to maintain a smooth action series to arrive that ramp up goal position.
>
> 3, then we will have the chassis to carry the arm and move together, to the desired start position, which is the goal position for chassis ramp up, and when chassis arrives, this natually enables arm to get to the 1st waypoint of desired EE trajoctory. along the way, there might be obstacles to avoid by chassis, and chassis shall plan a path get around any obstacles along the way. the planning shall be refreshed all the time, given the fact that obstacles can appear any time on its originally planned path and path needs to be replaned.
>
> 4, arm ramp up shall be planned by moveit, and it cannot hit chassis and column and arm itself (which are expressed by stl and urdf files), chassis ramp up shall be powered by hybrid a star like alrogithms, as we moves planarly.
>
> 5, for the next stage that arm and chassis moves together to track all the desired waypoints, we shall make sure that they won't hit stuff around it either. collision avoidance shall always be considered in path or trajectory planning. and in IK solving.
>
> 6, there shall be a state machine control which mode we are in, like arm ramp, chassis ramp, and EE tracking.
>
> 7, arm joint commands are issued out via ROS2 message, and chassis motion in x, and y, speed, and yaw rate shall be issued out via ros2 message.
>
> 8, we shall support both open loop and  closed loop control of the chassis. for this program, we don't need to worry about wheel differential speed control, instead, we just release linear speed into and chassis itself will do the differential.
>
> 9, the control freq. can be set at 10hz.
>
> 10, the target embedded sysstem to deploy to is nvidia orin agx, with ubuntu 2204, ros2 humble and moveit 2.5.9 on it.

## Objectives
- Mirror MATLAB whole-body workflow (ramp+tracking, IK/GIK, Hybrid A*) inside a ROS 2 Humble + MoveIt 2.5.9 stack running on NVIDIA Orin AGX.
- Support streaming camera pose goals, stateful progression, and 10 Hz control loops for both arm and chassis.
- Guarantee collision-aware planning and execution across all phases, leveraging existing URDF/SRDF and STL assets.

## High-Level Architecture
- **Trajectory Ingestion Node**: subscribes to a streaming topic (e.g., `camera_path/goal_pose`) and buffers queued waypoints until each is acknowledged as tracked. Publishes active target to the state machine.
- **Whole-Body Supervisor**: ROS 2 node implementing the state machine (`ARM_RAMP`, `BASE_RAMP`, `TRACKING`, `IDLE`, `PAUSED`). Tracks goal completion flags, handles retries, and orchestrates restarts on obstacle updates.
- **MoveIt Planning Adapter**: wraps `moveit_cpp` or `move_group_interface` for planning/IK. Provides services for arm ramp trajectory generation, streaming IK for tracking, and collision-world updates.
- **Arm Trajectory Splitter/Publisher**: converts MoveIt trajectories into `trajectory_msgs/JointTrajectory` commands, handles retiming (10 Hz output) and interfaces with controller manager.
- **Base Planner & Tracker**: Hybrid A* planner (or Nav2 plugin) producing SE(2) paths with dynamic obstacle refresh; diff-drive tracker converts synchronized planar trajectories into `geometry_msgs/Twist` commands. Supports open-loop (publish `Twist`) and closed-loop (subscribe to odometry and run feedback).
- **Monitoring & Logging**: collects metrics analogous to MATLAB `rt_results` (EE error, base speed, constraint flags) for validation; outputs to rosbag2 or diagnostics topics.
- Robot description: all MoveIt integration points load `urdf/arm_on_car_center_rotZ_n90_center.urdf` to keep the chassis-centered mount consistent with MATLAB baselines.

## ROS 2 Package Layout & Interfaces (Draft)
- **Packages**
  - `mobile_arm_whole_body_control`: core logic (supervisor, planners, trackers, utilities).
  - `mobile_arm_whole_body_interfaces`: custom ROS 2 interface types shared between nodes (`CameraPoseTarget`, `CameraPoseStatus`, `TrackingPhaseState`).
  - `mobile_arm_whole_body_bringup`: launch files, parameters, and configs stitching the system together.

- **Nodes (initial scope)**
  - `trajectory_ingestor` (`rclcpp::Node` or `rclpy`): subscribes to `camera_path/target` (`CameraPoseTarget`) and exposes `/camera_path/request_next` (`std_srvs/Trigger`) service acknowledgement once tracking completes. Publishes the active target on `camera_path/active` (`CameraPoseTarget`) and status feedback on `camera_path/status` (`CameraPoseStatus`).
  - `whole_body_supervisor` (`rclcpp::LifecycleNode`): owns finite-state machine, consumes `camera_path/active`, emits `/whole_body/state` (`TrackingPhaseState`), `/whole_body/events` (diagnostics), and orchestrates arm/base planners via services.
  - `arm_ramp_planner` (MoveIt integration, C++): service `/whole_body/plan_arm_ramp` accepting `PlanArmRamp.srv` (target height/orientation, relaxed XY bounds) and returning `trajectory_msgs/JointTrajectory`.
  - `base_ramp_planner` (Hybrid A*, C++/Python): service `/whole_body/plan_base_ramp` accepting `PlanBaseRamp.srv` (start, goal, obstacles) and returning `nav_msgs/Path` plus timing metadata.
  - `tracking_planner` (MoveIt whole-body): service `/whole_body/plan_tracking_segment` returning synchronized joint + base trajectories and collision diagnostics.
  - `arm_trajectory_publisher`: subscribes to planned arm trajectories, retimes to 10 Hz, publishes `trajectory_msgs/JointTrajectory` to controller.
  - `base_motion_commander`: converts SE(2) trajectories to `geometry_msgs/Twist` at 10 Hz; optional closed-loop feedback using `nav_msgs/Odometry`.

- **Messages / Services / Actions (to be defined)**
  - `CameraPoseTarget.msg`: `uint32 id`, `geometry_msgs/Pose pose`, `builtin_interfaces/Time stamp`, `float32 tolerance_position`, `float32 tolerance_orientation`.
  - `CameraPoseStatus.msg`: `uint32 id`, `uint8 status` (enum: NEW=0, TRACKING=1, COMPLETE=2, FAILED=3), `string message`.
  - `TrackingPhaseState.msg`: `uint8 mode` (IDLE=0, ARM_RAMP=1, BASE_RAMP=2, TRACKING=3, HOLD=4), `uint32 active_target_id`.
  - `PlanArmRamp.srv`: request contains target pose, relaxed XY bounds, timeout; response returns `trajectory_msgs/JointTrajectory`, success flag, diagnostics.
  - `PlanBaseRamp.srv`: request contains start pose, goal pose, obstacle set (array of primitives), dynamic replan hints; response returns `nav_msgs/Path`, velocity profile, diagnostics.
  - `PlanTrackingSegment.srv`: request with queue of poses, current state, and smoothing options; response returns arm/base trajectories and synchronization metadata.
  - Reuse standard topics: `trajectory_msgs/JointTrajectory` for arm commands, `geometry_msgs/Twist` for base open-loop, `nav_msgs/Odometry` for feedback, `moveit_msgs/PlanningScene` for collision updates.

- **Launch & Parameters**
  - `bringup/whole_body_streaming.launch.py`: starts MoveIt stack, supervisor, trajectory ingestor, planners, command publishers.
  - YAML configs for gains/limits (arm retiming, diff-drive tracker), state machine timeouts, replan thresholds, interface QoS (10 Hz reliability).

## Development Phases
1. **Foundation & Infrastructure**
   - Finalize ROS 2 package layout mirroring `mobile_arm_whole_body` and set up colcon build/testing on Orin.
   - Extend URDF/SRDF with collision meshes as required; verify in RViz with MoveIt Planning Scene updates.
   - Define ROS 2 interfaces (topics, services, actions) and message contracts for trajectory streaming, acknowledgements, and phase control.

2. **Trajectory Streaming Interface**
   - Implement ingestion node with stateful queue, “goal achieved” handshake, timeout/failure policies, and optional prefetching.
   - Provide tools to load MATLAB sample JSONs and re-publish them as streaming topics for regression.

3. **Arm Ramp Planning via MoveIt**
   - Port MATLAB ramp generation: create MoveIt planning request targeting relaxed XY with enforced Z+orientation constraints (via path constraints or GIK plugin).
   - Integrate collision objects (chassis, column) and verify solver respects them; log diagnostics (exit status, constraint violations).
   - Retime ramp trajectory to 10 Hz, ensuring smooth joint velocities.

4. **Chassis Ramp with Hybrid A***
   - Implement or wrap Hybrid A* planner (consider `nav2_smac_planner` or custom C++ node) using current obstacle map (OccupancyGrid).
   - Sync plan with arm ramp completion; feed path to base tracker, issue `Twist` commands with curvature/accel limits matching MATLAB.
   - Add continuous replanning triggered by obstacle updates or divergence detection.

5. **Coordinated Tracking Phase**
   - Build combined planner that maintains synchronized arm/base motion: MoveIt generates whole-body trajectory, splitter publishes arm commands while base tracker follows planar projection.
   - Ensure collision checking across dynamic environment updates; integrate MoveIt Planning Scene diff updates from perception stack.
   - Support streaming waypoint advancement: upon “tracked” ack, shift queue and request next goal.

6. **State Machine & Supervisory Control**
   - Implement lifecycle-managed node to coordinate transitions, monitor controllers, handle overrides (pause/resume), and expose diagnostics.
   - Add safeguards for fault states (e.g., IK failure, planner timeout) with recovery behaviors.

7. **Execution Interfaces & Control Modes**
   - Produce ROS 2 controller configurations for open-loop (`Twist` publisher) and closed-loop (feedback using odometry to adjust commands or replan) base control.
   - Verify arm controllers (FollowJointTrajectory) integrate with existing hardware drivers; add command smoothing/resampling for 10 Hz output.

8. **Validation & Regression**
   - Mirror MATLAB `run_camera_motion_tests` via ROS 2 launch/test harness that replays sample trajectories and asserts velocity/error bounds.
   - Capture metrics in rosbag2; create analysis scripts (Python notebook) to compare against MATLAB baselines.

9. **Deployment & Performance Tuning**
   - Containerize or create Ansible scripts for Orin AGX setup (ROS 2 Humble + MoveIt 2.5.9, GPU drivers, dependencies).
   - Profile CPU/GPU usage at 10 Hz, tune thread priorities, and validate real-time performance.
   - Document operational runbook, including startup sequence, monitoring, and fallback procedures.

## Key Technical Considerations
- Continuous collision monitoring: keep MoveIt Planning Scene synced with perception updates; ensure base planner consumes same obstacle map for consistency.
- IK/GIK strategy: evaluate generalized IK plugins vs. MoveIt Servo or Task Constructor for constrained arm motions; maintain parity with MATLAB solver tolerances.
- Time synchronization: align arm joint trajectory and base tracker via shared timeline (matching MATLAB `synchronize_base_trajectory`) and enforce curvature/lat accel bounds.
- Feedback integration: support odometry-based corrections (closed loop) while keeping deterministic open-loop behavior for simulation/regression.
- **Ramp selection logic**: before planning, compare the initial chassis/arm pose to the first EE waypoint. If the arm can reach the waypoint (height + orientation) without motion, skip the arm ramp. If the first waypoint is also reachable without moving the chassis, skip the base ramp. Arm/base ramps only run when necessary, and chassis decisions are derived from EE reachability (not a fixed base pose).

## Next Steps
1. Baseline the ROS 2 package: add launch skeleton for new nodes and define message/service contracts.
2. Prototype trajectory ingestion and state machine scaffolding with mocked planners to exercise mode transitions.
3. Incrementally port ramp planners (arm then base) and validate against MATLAB scenarios before tackling full tracking synchronization.

## Planner Implementation Roadmap

### Stage A — MoveIt Arm Planning Adapter
- Create a dedicated planner node (C++) that instantiates `moveit_cpp::MoveItCpp` using the existing `mobile_arm_whole_body` configuration.
- **Service `/whole_body/plan_arm_ramp`**
  - Translate `PlanArmRamp` targets into MoveIt `MotionPlanRequest` objects with path constraints: enforce Z/Orientation, relax X/Y via bounding box or cost term.
  - Inject collision objects for chassis/column and environment updates (subscribe to `/planning_scene` diffs or a perception topic producing `moveit_msgs::CollisionObject`).
  - Use MoveIt’s Constrained Planning or Task Constructor to generate a collision-free trajectory; if planning fails, fall back to generalized IK sampling while respecting collision scene.
  - Post-process trajectory with MoveIt time-parameterization (Iterative Parabolic Time Parameterization) and package as `JointTrajectory` with 10 Hz sampling metadata.
  - Expose a reachability check helper so the supervisor can query whether the first waypoint is feasible without ramp motion (reused for skip logic).
- **Service `/whole_body/plan_tracking_segment`**
  - Accept streaming EE targets, seed with current joint/base state, and plan for the `whole_body` group (planar world joint + arm) so the resulting path respects collisions throughout.
  - Support incremental re-planning by allowing partial plans (e.g., plan N seconds ahead) and exposing diagnostics (planning time, collision checks, constraint residuals).
  - Provide hooks to load dynamic obstacles (e.g., obstacle array/point cloud) and throttle updates to MoveIt Planning Scene to meet 10 Hz operation.

### Stage B — Chassis Hybrid A* Planner with Obstacle Projection
- Build a dedicated `base_hybrid_astar_node` (C++) that consumes a rolling occupancy grid and chassis footprint spec, then serves `/whole_body/plan_base_ramp`.
- Expose parameters for static test obstacles (e.g., disc obstacles specified as `[x, y, radius, height]`) so simulation runs can inject known blockers along the projected EE path before perception is online.
- Maintain a 2D occupancy grid:
  - Fuse perceived obstacles (e.g., detected objects along EE path) by projecting 3D bounding boxes to the ground plane, inflating by chassis half-width + safety margin.
  - Accept static map layers (URDF collision geometry, studio fixtures) and dynamic obstacle updates; support clearing via `OccupancyGrid` headers or explicit “clear” service.
- Hybrid A* search details:
  - Discretize SE(2) space (x, y, yaw) with forward/reverse motion primitives respecting wheelbase and max steer.
  - Incorporate cost heuristics (distance-to-goal, heading alignment) and forbid states intersecting inflated obstacles.
  - Emit diagnostics (path length, min turning radius, collision checks) alongside the planned `nav_msgs/Path`.
- Service response enrichments:
  - Attach nominal velocity/yaw-rate limits derived from curvature to guide the diff-drive tracker.
  - Surface clear error codes when no collision-free path exists or obstacle data is stale.
- Provide an optional `/whole_body/replan_base` trigger so the supervisor can request immediate replans when tracking detects divergence.

### Stage C — Coordinated Tracking & Collision Assurance
- Extend tracking planner to align MoveIt whole-body trajectory with Hybrid A* path:
  - Split MoveIt plan into arm/base components, respecting timing constraints derived from arm retiming and base curvature bounds.
  - Run collision verification across the combined trajectory (arm links vs. environment, chassis footprint vs. occupancy grid) and reject plans that violate clearance margins.
- Implement feedback hooks:
  - Subscribe to odometry and joint states; if deviation exceeds thresholds, request replans from both planners.
  - Optionally integrate MoveIt Servo or online smoothing for small tracking corrections without full replan.
- Surface detailed diagnostics to the supervisor (per-stage success/failure, error codes) so state machine can retry, skip, or abort depending on failure cause.

### Stage D — Testing & Validation
- Unit/launch tests for each planner service using synthetic obstacle scenarios (e.g., block on ground, overhead obstacle near arm path).
- Regression harness replaying MATLAB JSON trajectories while injecting obstacles to confirm the base detours and the arm end-effector remains on the desired path.
- RViz/ros2bag workflows to visualize collision geometry, planned paths, and executed commands (include instructions in docs).

### Stage E — Deployment Prep
- Parameterize perception inputs (topics, obstacle inflation radii) for Orin deployment.
- Profile planner latencies at 10 Hz control loop; optimize MoveIt/Hybrid A* parameters or leverage multithreading as needed.
- Document failure-handling sequences (e.g., planner timeout, collision detected mid-execution) and tie into supervisor recovery states.

## Progress Log
- **Stage 1 – Interface design (this update)**: Added package layout draft, interface definitions, and lifecycle roles. *Tests*: none (documentation-only change).
- **Stage 2 – ROS 2 scaffolding (this update)**: Created `mobile_arm_whole_body_interfaces`, `mobile_arm_whole_body_control`, and `mobile_arm_whole_body_bringup` packages with placeholder nodes, messages, services, and launch file. *Tests*: `colcon build` (workspace).
- **Stage 3 – Streaming handshake scaffolding (this update)**: Implemented trajectory ingestion queue with auto-ID, activation, and completion advancement via `/camera_path/request_next`; supervisor now transitions IDLE → ARM_RAMP and auto-completes targets after a placeholder duration while calling back into the ingestor. *Tests*: `colcon build` (workspace).
- **Stage 4 – Interface package compliance (this update)**: Added explicit `rosidl_interface_packages` group membership in `package.xml` and removed the temporary skip flag so interface builds rely on standard checks. *Tests*: `colcon build --packages-select mobile_arm_whole_body_interfaces`; `colcon build` (workspace).
- **Stage 5 – Handshake harness (this update)**: Added launch test exercising the trajectory ingestor and supervisor handshake with synthetic targets; test auto-skips when UDP sockets are unavailable in constrained environments. *Tests*: `colcon test --packages-select mobile_arm_whole_body_control` (launch test skipped under sandbox).
- **Stage 6 – Planner stubs (this update)**: Introduced stub services for arm/base ramp and tracking plans, wired the supervisor to call them sequentially, and extended bringup/test harness to include the stub. *Tests*: `colcon build` (workspace); `colcon test --packages-select mobile_arm_whole_body_control` (launch test skipped under sandbox).
- **Stage 7 – MoveIt planner integration (this update)**: Added `whole_body_planner_node` backed by MoveIt’s MoveGroup interface to serve `/whole_body/plan_arm_ramp` and `/whole_body/plan_tracking_segment`; bringup now launches the MoveIt planner while tests still rely on the stub. Base ramp service remains a stub until the Hybrid A* planner lands. *Tests*: `colcon build`; `colcon test --packages-select mobile_arm_whole_body_control` (launch test skipped under sandbox).
- **Stage 8 – Base ramp planner (this update)**: Implemented `base_planner_node` with grid-based Hybrid-A*-style search over supplied occupancy grids, returning obstacle-aware `nav_msgs/Path` plans and nominal velocity hints. Bringup now launches both MoveIt and base planners; test harness continues to use the stubbed services. *Tests*: `colcon build`; `colcon test --packages-select mobile_arm_whole_body_control` (launch test skipped under sandbox).
- **Stage 9 – Ramp skip logic (this update)**: Supervisor now analyzes MoveIt trajectories and base paths to skip arm/base ramps when motion is unnecessary, using configurable tolerances. *Tests*: `colcon build`; `colcon test --packages-select mobile_arm_whole_body_control` (launch test skipped under sandbox).
- **Stage 9.1 – Ramp state tracking (this update)**: Supervisor maintains configurable `initial_base_pose`/`initial_ee_pose`, `reference_frame`, and a `base_goal_offset` heuristic, feeds those into ramp/tracking requests, and republishes every plan on `/whole_body/arm_plan` and `/whole_body/base_plan` so downstream executors follow the latest state. *Tests*: `colcon build --packages-select mobile_arm_whole_body_control`; `colcon test --packages-select mobile_arm_whole_body_control --ctest-args -R test_trajectory_utils`.
- **Stage 10 – Whole-body path extraction (this update)**: MoveIt tracking plans are now split into arm trajectories and planar base paths, enabling synchronized execution. Added `obstacle_manager_node` to publish synthetic obstacle grids/collision objects, letting planners detour around the 14th waypoint disc and the ramp-stage obstacle. *Tests*: `colcon build`; `colcon test --packages-select mobile_arm_whole_body_control` (launch test skipped under sandbox).
- **Stage 11 – Base tracking scaffold (this update)**: Base commander subscribes to `/whole_body/base_plan` & `/whole_body/odom`, outputting placeholder diff-drive commands; base planner unit test ensures detours around scripted disc obstacles. *Tests*: `colcon build`; `colcon test --packages-select mobile_arm_whole_body_control` (launch tests skip under sandbox).
- **Stage 12 – Collision vetting (this update)**: Whole-body planner now splits trajectories and verifies each synchronized sample against MoveIt's planning scene, rejecting plans that collide with published obstacles; RViz/rosbag workflow documented alongside plotting utilities. *Tests*: `colcon build`; `colcon test --packages-select mobile_arm_whole_body_control` (launch tests skip under sandbox).
