# mobile_arm_whole_body

Whole-body (base + arm) MoveIt2 demo for a diff-drive chassis with a 6-DoF arm and a planar virtual joint.

## Contents
- `urdf/arm_on_car_center_rotZ_neg90.urdf` — Arm mounted at the car center frame with a -90° Z rotation.
- `srdf/arm_on_car_center_whole_body.srdf` — Adds a planar `virtual_joint` (odom→chassis_center_link) and groups: `manipulator`, `mobile_base`, `whole_body`.
- `config/` — Kinematics, OMPL pipeline, and MoveIt FakeController config.
- `launch/whole_body_demo.launch.py` — Starts robot_state_publisher, move_group, RViz (MotionPlanning), and a static TF `odom→chassis_center_link` (identity for demo).
- `scripts/world_joint_to_cmd_vel.py` — Example bridge (toy) to turn planar joint trajectories into `/cmd_vel` (not wired in the demo launch).

## Requirements
- ROS 2 (Humble or newer)
- MoveIt 2 with `moveit_configs_utils`
- `robot_state_publisher`, `rviz2`, `joint_state_publisher_gui`

## Build
```bash
cd ~/ws/src
# copy this folder here as mobile_arm_whole_body
cd ~/ws
colcon build --symlink-install
source install/setup.bash
```

## Run (planning-only with FakeController)
```bash
ros2 launch mobile_arm_whole_body whole_body_demo.launch.py
```
Then in RViz → MotionPlanning:
- Set **Planning Group = whole_body**
- Set an end-effector pose goal
- **Plan** → **Execute** (simulated with FakeController)

## Wire to real robot (outline)
- Replace `config/moveit_controllers.yaml` with your real controllers (arm: FollowJointTrajectory; base: diff_drive or a custom controller that consumes `world_joint` goals).
- Publish real TF `odom→chassis_center_link` from your localization/odometry stack.
- Optionally add a chassis collision shape to the URDF so whole-body planning is collision-aware.

## Trajectory-Following Assumptions
- The demo focuses on pure trajectory tracking; obstacle avoidance is disabled.
- Arm joint positions obey the limits encoded in the URDF. Use the MATLAB retiming helper (`matlab/+helpers/retime_joint_trajectory.m`) to keep joint velocities/accelerations within bounds before execution.
- Base motion is always generated through a diff-drive tracker, so lateral commands are suppressed by design—yaw plus forward motion produces the necessary lateral displacement.
