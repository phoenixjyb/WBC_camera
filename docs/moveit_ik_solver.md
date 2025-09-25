# MoveIt-Based IK/GIK Replacement

To mirror the MATLAB generalized IK stage inside MoveIt 2.5.9, the package now
ships a reusable helper class:

```
mobile_arm_whole_body::MoveItIKSolver
```

## Features
- Loads the robot model from `robot_description` and constructs a
dedicated `planning_scene::PlanningScene` for collision queries.
- Solves pose sequences via `robot_state::RobotState::setFromIK`, with
warm-starts, configurable timeouts, and collision checking.
- Publishes `IKSolution` diagnostics (success flag + MoveIt error code) for
integration with the coordinator pipeline.
- Supports external scene updates through `update_planning_scene` so
obstacle layers or perception modules can inject the latest geometry.

## Usage Example
```cpp
auto node = rclcpp::Node::make_shared("ik_test");
mobile_arm_whole_body::MoveItIKSolver solver(node, "arm_group", "left_gripper_link");

std::vector<geometry_msgs::msg::Pose> targets(1);
// ... fill targets[0]
mobile_arm_whole_body::IKOptions opts;
opts.timeout = 0.1;
opts.max_attempts = 10;

std::vector<mobile_arm_whole_body::IKSolution> solutions;
if (solver.solve(targets, solutions, opts)) {
  RCLCPP_INFO(node->get_logger(), "IK succeeded");
}
```

### Planning Scene Diff
To honour collision avoidance, feed the planner with scene updates:

```cpp
moveit_msgs::msg::PlanningScene scene_msg;
scene_msg.is_diff = true;
// populate robot state / world objects
solver.update_planning_scene(scene_msg);
```

## Parity Considerations
- **Position/Orientation tolerances:** MoveIt’s IK plugins typically apply
strict tolerances. `IKOptions::orientation_tolerance` is used to derive
consistency limits when sampling nearby solutions; adjust the plugin
settings if a wider tolerance is required.
- **Collision avoidance:** Enabled by default. Disable by setting
`opts.check_collisions = false` for benchmarking, but prefer to keep it on
to match MATLAB’s `constraintCollisionAvoidance` behaviour.
- **Warm-starts:** Provide the previous joint solution (or home pose) via
the `seed` argument. The solver automatically reuses the last valid
solution between poses.

## Integration Roadmap
1. Replace MATLAB GIK calls inside the coordinator with this helper.
2. Map MATLAB constraint sets (distance, Cartesian bounds) to appropriate
MoveIt constraints or planning-scene updates.
3. Extend the helper to accept per-axis tolerances and constraint lists as
additional parameters if needed.

This bridge completes the migration path from MATLAB’s GIK stage to MoveIt’s
kinematics stack and keeps the codegen-friendly numeric helpers for the rest
of the pipeline.

